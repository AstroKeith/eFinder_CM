"""
rig_plate_solve.py
--------------------
A synthetic "plate solve" driven by an independent Alt-Az encoder rig
instead of a camera -- for full daytime end-to-end testing of the mount
calibration and pointing pipeline with your REAL BNO085 hardware, using a
ground truth source that is completely independent of the IMU (unlike
test_plate_solver.py, whose "truth" is derived from the very same IMU
reading it's later compared against).

THE IDEA
=========
Your Alt-Az test rig reports RA, Dec, Altitude, and Azimuth directly from
its own shaft encoders as you move the telescope by hand -- genuine,
independent ground truth for *pointing direction*. What it can't give you
is roll/position angle, because it has no camera. This module fills that
gap by CALCULATING the position angle a real plate-solving camera would
report, from first principles, using one physical fact about rigid Alt-Az
mounts:

  A camera bolted directly to the OTA (no field derotator) holds a FIXED,
  constant roll relative to the local VERTICAL CIRCLE -- i.e. relative to
  the zenith direction, projected perpendicular to the boresight -- at
  every pointing. That constant is `camera_clocking_deg` below -- pick
  any value (0.0 is a perfectly good placeholder) since only its
  CONSTANCY across every pointing matters for this test, not its absolute
  numeric value.

  IMPORTANT -- and an earlier version of this module got it wrong: the
  fixed reference is the ZENITH direction, not local horizon NORTH. Only
  the azimuth axis is the local vertical (zenith) itself, so only
  zenith-referenced roll survives an azimuth rotation exactly; a rotation
  about the altitude axis (horizontal, and different at every azimuth)
  does NOT leave a north-referenced roll invariant except by coincidence
  at specific azimuths (due east/west). This was confirmed by directly
  simulating a genuine 2-axis gimbal (azimuth about world Z, then
  altitude about the co-rotating horizontal axis) independently of this
  module's own machinery: roll-relative-to-zenith stayed at the same
  value across every alt/az combination tried, while roll-relative-to-
  north swung across the full 0-360 range. Using local north as the fixed
  reference (the original, buggy version) doesn't just add noise -- it
  feeds a subtly WRONG "true_rot" into mount_calibration.py at every
  calibration point, one that grows more wrong the further apart your
  test points are (exactly the "residual gets worse with bigger, more
  spread-out test angles" symptom a user with a real encoder rig actually
  hit) -- see mount_calibration.py's module docstring for why a
  position-dependent error corrupts the FIT itself, not just the output,
  the same failure shape as the equatorial-vs-Alt-Az position-angle-
  convention bug this package's real-solver path hit earlier.

  What VARIES with sky position, given a fixed zenith-referenced roll, is
  the difference between that and the equatorial (North-Celestial-Pole-
  referenced) roll a real solver reports -- this is NOT simply the
  textbook parallactic angle (which is defined relative to local NORTH,
  not zenith) but a related, computable quantity; it's exactly why
  Alt-Az-mounted cameras without a derotator show "field rotation" over a
  tracking run: the camera's roll relative to the vertical circle never
  moves, but its roll relative to the sky keeps drifting.

So: given RA/Dec/Alt/Az from the rig (all four, though only one pair is
strictly independent information -- both are used directly here since
your rig hands them both over already) plus the site and timestamp, the
equatorial-convention roll is fully determined -- see
`true_radec_pa_from_rig()` below. It's computed via the SAME
`attitude_from_altaz_pa` / `altaz_pa_from_attitude` /
`celestial_north_reference_enu` machinery the rest of this package
already uses (not a freshly hand-derived formula), to stay consistent and
avoid a new sign-error risk -- the ZENITH-vs-NORTH mistake above was in
which reference vector got fed into that machinery, not in the machinery
itself.

WHY THIS IS A BETTER DAYTIME TEST THAN test_plate_solver.py's TEST_MODE
==========================================================================
TEST_MODE's synthetic solver reads the SAME IMU sample it's later
compared against, and derives its "truth" by undoing a fixed hidden bias
from that one reading -- fine for exercising the wiring (rate-gating,
offset math, the calibration code paths), but it can never prove the
*geometry* is right end-to-end, because the "truth" and the "test
subject" both trace back to one IMU reading -- a sign error in, say,
`enu_vector_to_altaz` would happily cancel itself out on both sides and
never show up as a test failure.

This module instead sources RA/Dec/Alt/Az from your rig's own encoders --
a measurement of where the telescope is actually pointing that has
nothing to do with the IMU at all. Combined with your real BNO085
attached to the same telescope, this exercises the full real pipeline
(rate gating, offset computation, AND automatic mount calibration, since
this gives genuine, non-degenerate position-angle data as you move to
different parts of the sky) against real independent ground truth,
indoors, without needing clear skies -- about as close to a true
end-to-end test as you can get without starlight.

USAGE SKETCH
=============
Wire this in place of `your_plate_solve` / `make_synthetic_plate_solve` in
main_loop_example.py:

    from rig_plate_solve import make_rig_plate_solve

    def read_rig():
        # Replace with your actual rig-reading code.
        ra, dec, alt, az = my_rig_driver.read()
        return ra, dec, alt, az, datetime.now(timezone.utc)

    plate_solve = make_rig_plate_solve(read_rig, site)

Everything else (get_pointing_estimate, the main loop, the "mount
calibrated" reporting) is unchanged -- from the estimator's point of view
this looks exactly like a real solver, because as far as the geometry is
concerned, it is one.

A NOTE ON "CURRENT EPOCH"
===========================
Same requirement as your real plate solver (see main_loop_example.py's
"WHY CURRENT EPOCH..." section): whatever RA/Dec your rig reports needs
to be in the same (no precession/nutation correction) frame this
package's astro_time.py assumes. If your rig's RA/Dec output is J2000 or
some other fixed epoch instead, either convert it before passing it in
here, or just use its Alt/Az output plus `astro_time.altaz_to_radec()` to
derive current-epoch RA/Dec yourself -- the roll calculation below only
actually needs Alt/Az plus a current-epoch RA/Dec for the
`celestial_north_reference_enu` step.
"""
from __future__ import annotations

import random
from datetime import datetime
from typing import Callable, Optional, Tuple

import numpy as np

from astro_time import ObserverLocation
from attitude import attitude_from_altaz_pa, altaz_pa_from_attitude, celestial_north_reference_enu


ZENITH_ENU = np.array([0.0, 0.0, 1.0])  # local "Up" -- the true fixed reference for a rigid 2-axis gimbal


def true_radec_pa_from_rig(
    ra_deg: float,
    dec_deg: float,
    alt_deg: float,
    az_deg: float,
    timestamp: datetime,
    site: ObserverLocation,
    camera_clocking_deg: float = 0.0,
) -> float:
    """The equatorial-convention position angle a real plate-solving
    camera -- rigidly bolted to this Alt-Az OTA, no field derotator --
    WOULD report at this pointing. See the module docstring for the
    physics this relies on.

    camera_clocking_deg: the (arbitrary but constant) roll the camera
        holds relative to the local VERTICAL CIRCLE (zenith direction,
        projected perpendicular to the boresight) -- see the module
        docstring for why zenith, not north, is the correct fixed
        reference for a rigid 2-axis gimbal. 0.0 is fine for testing --
        only its constancy across calls matters here, not its particular
        numeric value.

    Returns just the roll (pa_deg) -- ra_deg/dec_deg themselves are
    already known exactly, straight from your rig's encoders, so there's
    no need to round-trip them through this function too.
    """
    # Step 1: the attitude this camera has at this Alt/Az, if its roll
    # relative to the local ZENITH direction (not north -- see module
    # docstring) were exactly camera_clocking_deg -- which it always is,
    # everywhere, for a rigidly-mounted derotator-free camera on a 2-axis
    # gimbal.
    fixed_roll_attitude = attitude_from_altaz_pa(alt_deg, az_deg, camera_clocking_deg, up_reference=ZENITH_ENU)

    # Step 2: re-read the roll of that SAME attitude, but this time in the
    # equatorial (North-Celestial-Pole-referenced) convention a real
    # solver reports -- the difference that pops out here is the net
    # effect of both the zenith-vs-north geometry AND the parallactic
    # angle at this sky position/time.
    up_ref = celestial_north_reference_enu(ra_deg, dec_deg, site, timestamp)
    _, _, roll_deg = altaz_pa_from_attitude(fixed_roll_attitude, up_reference=up_ref)
    return roll_deg


def make_rig_plate_solve(
    get_rig_reading: Callable[[], Tuple[float, float, float, float, datetime]],
    site: ObserverLocation,
    camera_clocking_deg: float = 0.0,
    noise_deg: float = 0.0,
    seed: Optional[int] = None,
) -> Callable[[], Tuple[float, float, float, bool]]:
    """Build a capture_and_solve()-shaped function driven by your Alt-Az
    rig instead of a camera -- see module docstring.

    get_rig_reading: zero-argument function returning
        (ra_deg, dec_deg, alt_deg, az_deg, timestamp) from your rig's
        encoders -- current-epoch RA/Dec (see the module docstring's note
        on this) and a UTC timestamp.
    site: your ObserverLocation (same one the estimator uses).
    camera_clocking_deg: see `true_radec_pa_from_rig` -- leave at the
        default unless you have a specific reason to change it.
    noise_deg: optional random noise added to ra/dec/roll, standing in
        for real solve imprecision. 0.0 (default, exact) is usually more
        useful here, since the whole point of this rig is clean ground
        truth -- add a little only if you specifically want to test noise
        robustness.
    seed: optional RNG seed, for reproducible runs (only matters if
        noise_deg > 0).
    """
    rng = random.Random(seed)

    def rig_plate_solve() -> Tuple[float, float, float, bool]:
        ra, dec, alt, az, timestamp = get_rig_reading()
        roll = true_radec_pa_from_rig(ra, dec, alt, az, timestamp, site, camera_clocking_deg)

        if noise_deg:
            ra = (ra + rng.gauss(0.0, noise_deg)) % 360.0
            dec = max(-90.0, min(90.0, dec + rng.gauss(0.0, noise_deg)))
            roll = (roll + rng.gauss(0.0, noise_deg)) % 360.0

        return ra, dec, roll, True

    return rig_plate_solve


def _independent_gimbal_ground_truth(alt_deg, az_deg, camera_clocking_deg, ra_deg, dec_deg, site, timestamp):
    """Ground truth built WITHOUT using true_radec_pa_from_rig, attitude_from_altaz_pa,
    or altaz_pa_from_attitude -- a genuinely independent re-derivation, used only by
    the self-test below to guard against the exact class of bug this module already
    shipped once (a plausible-sounding but wrong physical assumption that the earlier
    self-test, built from the SAME machinery it was testing, couldn't catch).

    Composes a real 2-axis gimbal directly (azimuth rotation about world Z, THEN
    altitude rotation about the co-rotating horizontal tipping axis -- exactly how a
    real Alt-Az mount's two axes compose), attaches a camera with a fixed body-frame
    "up" at `camera_clocking_deg` from the ZENITH direction (see module docstring for
    why zenith, not north, is the correct invariant), and reads off the roll of that
    same camera relative to celestial (equatorial) north -- via plain vector algebra,
    not this module's own helper functions.
    """
    E = np.array([1.0, 0.0, 0.0])
    Z = np.array([0.0, 0.0, 1.0])

    def _rotate(v, axis, angle_deg):
        # Rodrigues' rotation formula -- no scipy Rotation object, to keep this
        # genuinely independent of the rest of the package's rotation machinery.
        axis = axis / np.linalg.norm(axis)
        th = np.radians(angle_deg)
        return (v * np.cos(th) + np.cross(axis, v) * np.sin(th) + axis * np.dot(axis, v) * (1 - np.cos(th)))

    # Reference pose (alt=0, az=0): boresight North, camera-up towards Zenith.
    bore_ref = np.array([0.0, 1.0, 0.0])
    up_ref_body = np.array([0.0, 0.0, 1.0])
    # Apply camera_clocking_deg about the reference boresight (this rotates the
    # camera's "up" away from zenith by the fixed clocking angle, in the body frame).
    up_ref_body = _rotate(up_ref_body, bore_ref, camera_clocking_deg)

    bore = _rotate(bore_ref, E, alt_deg)          # tip in altitude about the az=0 horizontal axis
    cam_up = _rotate(up_ref_body, E, alt_deg)
    # Azimuth increases clockwise as seen from above (compass bearing: North
    # -> East -> South -> West), which is a NEGATIVE-angle rotation about
    # +Z in the standard right-handed math sense -- matches
    # altaz_to_enu_vector's convention (az=90 -> East -> +X).
    bore = _rotate(bore, Z, -az_deg)
    cam_up = _rotate(cam_up, Z, -az_deg)

    def _perp(ref, bore):
        n = ref - np.dot(ref, bore) * bore
        return n / np.linalg.norm(n)

    up_ref_eq = celestial_north_reference_enu(ra_deg, dec_deg, site, timestamp)
    n_eq = _perp(up_ref_eq, bore)
    e_eq = np.cross(bore, n_eq)
    e_eq = e_eq / np.linalg.norm(e_eq)
    roll_eq = np.degrees(np.arctan2(np.dot(cam_up, e_eq), np.dot(cam_up, n_eq))) % 360.0
    return roll_eq


if __name__ == "__main__":
    # Self-test / demonstration -- no rig or hardware needed.
    from datetime import timezone
    from astro_time import radec_to_altaz, altaz_to_radec

    site = ObserverLocation(latitude_deg=51.5074, longitude_deg=-0.1278, elevation_m=25.0)
    t = datetime(2026, 9, 8, 21, 0, 0, tzinfo=timezone.utc)

    print("=== Sanity check 1: agrees with an INDEPENDENTLY built 2-axis gimbal model ===")
    print("    (this is the check that would have caught the zenith-vs-north bug)")
    worst_diff = 0.0
    for alt in (10.0, 35.0, 60.0, 80.0):
        for az in (10.0, 80.0, 170.0, 260.0, 340.0):
            ra, dec = altaz_to_radec(alt, az, site, t)
            roll_module = true_radec_pa_from_rig(ra, dec, alt, az, t, site, camera_clocking_deg=17.0)
            roll_independent = _independent_gimbal_ground_truth(alt, az, 17.0, ra, dec, site, t)
            diff = abs(((roll_module - roll_independent) + 180.0) % 360.0 - 180.0)
            worst_diff = max(worst_diff, diff)
    print(f"  worst disagreement across {4*5} alt/az points: {worst_diff:.6f} deg (expect near 0)")
    assert worst_diff < 1e-6, f"true_radec_pa_from_rig disagrees with the independent gimbal model by {worst_diff} deg"

    print("\n=== Sanity check 2: well off the meridian, roll should differ substantially from camera_clocking_deg ===")
    ra_e, dec_e = altaz_to_radec(30.0, 90.0, site, t)  # due east, well off the meridian
    roll_e = true_radec_pa_from_rig(ra_e, dec_e, 30.0, 90.0, t, site, camera_clocking_deg=17.0)
    print(f"  camera_clocking_deg=17.0 -> roll={roll_e:.4f} deg (expect well away from 17.0)")
    assert abs(((roll_e - 17.0) + 180.0) % 360.0 - 180.0) > 5.0, "off the meridian, roll should diverge from camera_clocking_deg"

    print("\n=== Sanity check 3: end-to-end -- mount calibration converges using ONLY rig-derived truth ===")
    # Simulate a rig session: point at several genuinely different sky
    # positions, synthesize what the REAL IMU would read at each (with a
    # deliberately large, arbitrary hidden mount misalignment -- exactly
    # like test_mount_calibration.py does), and check the estimator
    # recovers accurate pointing using only rig_plate_solve()'s output.
    import numpy as np
    from scipy.spatial.transform import Rotation
    from attitude import quat_xyzw
    from telescope_pointing import TelescopePointingEstimator, IMUSample, combine_estimate, PlateSolveResult

    world_bias = Rotation.from_euler("z", 9.0, degrees=True)
    body_mount = Rotation.from_euler("xyz", [12.0, -30.0, 55.0], degrees=True)  # deliberately large, arbitrary

    def true_radec_pa_from_rig_wrap(alt, az, tt):
        ra, dec = altaz_to_radec(alt, az, site, tt)
        roll = true_radec_pa_from_rig(ra, dec, alt, az, tt, site, camera_clocking_deg=0.0)
        return ra, dec, roll

    def simulated_real_imu(alt, az, roll, tt):
        # What the real BNO085 would read given the hidden mount/world bias.
        up_ref = celestial_north_reference_enu(*altaz_to_radec(alt, az, site, tt), site, tt)
        true_world_rot = attitude_from_altaz_pa(alt, az, roll, up_reference=up_ref)
        raw_rot = world_bias * true_world_rot * body_mount
        x, y, z, w = quat_xyzw(raw_rot)
        return IMUSample(x=x, y=y, z=z, w=w, timestamp=tt)

    estimator = TelescopePointingEstimator(site)  # defaults: require_mount_calibration=True, "equatorial"
    targets = [(60.0, 30.0), (40.0, 170.0), (25.0, 260.0), (70.0, 320.0), (35.0, 90.0)]
    for alt, az in targets:
        tt = t
        ra, dec, roll = true_radec_pa_from_rig_wrap(alt, az, tt)
        imu = simulated_real_imu(alt, az, roll, tt)
        solve = PlateSolveResult(ra_deg=ra, dec_deg=dec, timestamp=tt, position_angle_deg=roll)
        combine_estimate(estimator, solve, imu)

    print(f"  mount_calibrated={estimator.mount_calibrated}")
    print(f"  {estimator.mount_calibration_status.describe()}")
    assert estimator.mount_calibrated, "should have calibrated from 5 spread-out rig-driven solves"

    # Now check a fresh, unvisited direction -- IMU-only, no solve there.
    alt_q, az_q, tt = 55.0, 210.0, t
    ra_q, dec_q, roll_q = true_radec_pa_from_rig_wrap(alt_q, az_q, tt)
    imu_q = simulated_real_imu(alt_q, az_q, roll_q, tt)
    est = estimator.estimate(imu_q)
    sep_arcsec = np.degrees(np.arccos(np.clip(
        np.sin(np.radians(dec_q)) * np.sin(np.radians(est.dec_deg))
        + np.cos(np.radians(dec_q)) * np.cos(np.radians(est.dec_deg)) * np.cos(np.radians(ra_q - est.ra_deg)),
        -1.0, 1.0,
    ))) * 3600.0
    print(f"  query point sky error: {sep_arcsec:.1f} arcsec")
    assert sep_arcsec < 5.0, "should be accurate to a few arcsec at an unvisited direction"

    print("\nAll checks passed.")
