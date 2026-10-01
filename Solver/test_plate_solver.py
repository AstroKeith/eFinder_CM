"""
test_plate_solver.py
----------------------
A synthetic, IMU-aware stand-in for your real plate-solve function --
for bench-testing the pointing-estimator wiring indoors, without a
camera or a clear sky.

WHY YOUR OLD "SOLVE ONE STOCK IMAGE" TEST MODE DOESN'T WORK HERE ANYMORE
==========================================================================
Your previous test mode returned the same fixed RA/Dec every time (from
one stock image), which was fine when the plate-solver was the only
input. Now the estimator also has a real BNO085 attached, and it expects
each "solve" to be at least roughly where the IMU currently says the
scope is pointing -- a fixed RA/Dec that never moves, while you wave the
scope around by hand, immediately produces a nonsensical multi-degree
"correction" and defeats the point of the test.

WHAT THIS DOES INSTEAD
========================
`make_synthetic_plate_solve()` builds a function with the exact same
shape as your real plate-solve function -- call it with no arguments,
get back `(ra_deg, dec_deg, roll_deg, solved)` -- but instead of
processing a camera image, it:

  1. Uses the REAL BNO085's most recent reading -- so it tracks wherever
     you actually point the telescope by hand.
  2. Applies a fixed, "hidden" test bias to that raw reading -- standing
     in for the real-world miscalibration (magnetic heading error,
     mounting offset, etc.) your real offset-calibration is meant to
     discover and cancel out. Without this, the test would be trivial:
     the estimator would have nothing real to calibrate, and a sign
     error or similar bug in the offset math could hide undetected.
  3. Adds a small amount of random noise, to stand in for real
     plate-solve imprecision.
  4. Optionally simulates realistic processing delay and occasional
     solve failure, if you want to exercise those code paths too.

This is a drop-in replacement for `your_plate_solve()` in
main_loop_example.py -- see the `TEST_MODE` toggle there.

WHAT TO EXPECT WHEN YOU RUN IT
================================
- On the very first call, the estimator has nothing to compare against
  yet, so it just calibrates from whatever this returns and reports that
  straight back as the "plate_solve" estimate.
- After that, if you wave the telescope around slowly (below your
  `max_rate_deg_s`), every call re-solves and re-calibrates -- the offset
  should settle down and RA/Dec should track your hand movements smoothly.
- If you sweep it quickly, `get_pointing_estimate()` should skip calling
  this function entirely and fall back to the IMU-corrected estimate
  instead -- watch for `[imu]` vs `[solved]` in main_loop_example.py's
  output to confirm the rate gating is actually kicking in.
- The RA/Dec numbers themselves are NOT meaningful astronomically (the
  scope isn't really looking at that patch of sky) -- this only proves
  the code paths (calibration, rate-gating, offset application) are
  wired up correctly, not that the resulting sky coordinates are real.
"""
from __future__ import annotations

import random
import time
from typing import Callable, Optional, Tuple

from scipy.spatial.transform import Rotation

from astro_time import ObserverLocation, altaz_to_radec
from attitude import altaz_pa_from_attitude, celestial_north_reference_enu
from telescope_pointing import IMUSample


class LastSampleCache:
    """Remembers the most recent IMUSample -- used to give the synthetic
    solver access to "the IMU reading this tick already took" instead of
    triggering a second, independent hardware read of its own.

    Why this matters: `get_pointing_estimate()` reads the IMU once, then
    (if not moving too fast) calls your capture_and_solve function. If
    that function did its *own* fresh IMU read, and you're moving the
    scope by hand while `processing_delay_s` elapses, the two readings
    could disagree -- and the offset would then get calibrated from a
    "true" position that doesn't actually correspond to the raw IMU
    reading it's paired with. Routing both through the same cached sample
    keeps them consistent, exactly like a real camera: the image content
    is fixed at capture time, even though *processing* it takes longer.

    Usage: wrap your real `read_imu` function so it updates the cache on
    every call (see main_loop_example.py), and pass the cache itself
    (not `read_imu`) to `make_synthetic_plate_solve`.
    """

    def __init__(self) -> None:
        self._last: Optional[IMUSample] = None

    def update(self, sample: IMUSample) -> IMUSample:
        self._last = sample
        return sample

    def __call__(self) -> IMUSample:
        if self._last is None:
            raise RuntimeError(
                "LastSampleCache has no reading yet -- read_imu() must be "
                "called at least once (which get_pointing_estimate() always "
                "does before it would call the synthetic solver) before this "
                "can be used."
            )
        return self._last


DEFAULT_HIDDEN_BIAS_DEG: Tuple[float, float, float] = (10.0, 2.0, -3.0)


def true_radec_pa_from_raw(
    imu_sample: IMUSample,
    site: ObserverLocation,
    hidden_bias_deg: Tuple[float, float, float] = DEFAULT_HIDDEN_BIAS_DEG,
    position_angle_convention: str = "equatorial",
) -> Tuple[float, float, float]:
    """The noiseless ground truth `make_synthetic_plate_solve()` is built
    on: what a perfect solve of this raw IMU reading would report, given
    the fixed `hidden_bias_deg` this module applies. Exposed standalone
    (not just inside the closure below) so other code -- e.g.
    check_dec_direction.py -- can compare an estimator's live output
    against the same ground truth the fake solver itself uses, with no
    noise, rather than relying on your own sense of which way a
    coordinate "should" move -- which, per the module docstring above,
    isn't something TEST_MODE's synthetic RA/Dec preserves.

    position_angle_convention: matches
    `TelescopePointingEstimator.position_angle_convention` -- default
    "equatorial" (roll measured from the North Celestial Pole direction,
    like virtually every real plate solver) since that's now this
    estimator's own default too; pass "altaz" only if you've also set
    the estimator to that convention.
    """
    hidden_bias = Rotation.from_euler("xyz", hidden_bias_deg, degrees=True)
    true_rot = hidden_bias.inv() * imu_sample.rotation()
    # alt/az from the boresight first (unaffected by PA convention) so
    # ra/dec -- and the matching up_reference -- are available before
    # extracting roll in the right convention below.
    alt, az, _ = altaz_pa_from_attitude(true_rot)
    ra, dec = altaz_to_radec(alt, az, site, imu_sample.timestamp)
    up_ref = (
        celestial_north_reference_enu(ra, dec, site, imu_sample.timestamp)
        if position_angle_convention == "equatorial"
        else None
    )
    _, _, roll = altaz_pa_from_attitude(true_rot, up_reference=up_ref)
    return ra, dec, roll


def make_synthetic_plate_solve(
    get_reference_imu_sample: Callable[[], IMUSample],
    site: ObserverLocation,
    hidden_bias_deg: Tuple[float, float, float] = DEFAULT_HIDDEN_BIAS_DEG,
    noise_deg: float = 0.02,
    fail_probability: float = 0.0,
    processing_delay_s: float = 0.2,
    seed: Optional[int] = None,
    position_angle_convention: str = "equatorial",
) -> Callable[[], Tuple[float, float, float, bool]]:
    """Build a synthetic capture_and_solve() function for bench testing.

    get_reference_imu_sample: a zero-argument function returning the
        IMUSample to base the fake solve on. Pass a `LastSampleCache`
        instance here (see above and main_loop_example.py) -- NOT your
        raw hardware `read_imu` function -- so the fake solve is based on
        the exact same reading `get_pointing_estimate()` already took
        this tick, rather than triggering a second, separately-timed
        hardware read.
    site: your ObserverLocation (same one the estimator uses).
    hidden_bias_deg: a fixed (yaw, pitch, roll) rotation, degrees, standing
        in for "the real-world error your calibration should discover and
        cancel". The default is arbitrary but non-trivial -- change it if
        you want, but there's no need to match anything real; it's not
        telling you about your actual hardware; it's just there so the
        test has something genuine for the offset math to do.
    noise_deg: random noise added to each solved RA/Dec/roll, standing in
        for real plate-solve imprecision (default ~1 arcmin).
    fail_probability: chance (0-1) that a given call reports solved=False
        instead, to let you exercise the fallback path without needing an
        actual fast slew. 0 (default) means it always succeeds.
    processing_delay_s: artificial delay before returning, standing in for
        camera exposure + solve time (default 0.2s, matching your typical
        real solve time).
    seed: optional RNG seed, for reproducible test runs.
    position_angle_convention: must match the estimator's own
        `position_angle_convention` (default "equatorial" on both) --
        otherwise this fake solver's roll and the estimator's
        interpretation of it disagree, which corrupts calibration in
        exactly the way a real solver/estimator convention mismatch
        would (see telescope_pointing.py's docstring for that).
    """
    rng = random.Random(seed)

    def synthetic_plate_solve() -> Tuple[float, float, float, bool]:
        if processing_delay_s:
            time.sleep(processing_delay_s)

        if rng.random() < fail_probability:
            return 0.0, 0.0, 0.0, False

        imu_sample = get_reference_imu_sample()
        ra, dec, roll = true_radec_pa_from_raw(imu_sample, site, hidden_bias_deg, position_angle_convention)

        ra_noisy = (ra + rng.gauss(0.0, noise_deg)) % 360.0
        dec_noisy = max(-90.0, min(90.0, dec + rng.gauss(0.0, noise_deg)))
        roll_noisy = (roll + rng.gauss(0.0, noise_deg)) % 360.0

        return ra_noisy, dec_noisy, roll_noisy, True

    return synthetic_plate_solve
