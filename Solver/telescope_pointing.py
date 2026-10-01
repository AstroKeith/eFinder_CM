"""
telescope_pointing.py
----------------------
Combine a camera plate-solver with a BNO085 IMU (Rotation Vector /
quaternion output) to keep a continuous, accurate estimate of where a
telescope is pointing (RA/Dec), including during fast slews when the
plate-solver cannot solve because the frame is motion-blurred.

THE IDEA
========
1. While the telescope is stationary or sidereally tracking, the plate
   solver gives ground-truth RA/Dec (optionally + position angle) for
   the current instant.
2. At that same instant we also have a raw BNO085 attitude quaternion.
   Both describe "where the telescope is pointing" in a topocentric
   (Alt/Az / local-horizon) reference frame -- the plate solve is
   converted from RA/Dec into Alt/Az using the observer's location and
   the solve's timestamp (this is what makes the frames comparable: the
   IMU's fused reference frame is Earth-fixed, i.e. it does *not* rotate
   with the sky, exactly like Alt/Az).
3. The rotation needed to turn the *raw IMU attitude* into the *true
   (plate-solved) attitude* is computed and stored as a single
   "correction" quaternion -- the `offset`.
4. During a fast slew (no plate solve available), each new raw IMU
   quaternion has the same offset applied to it, is converted back from
   Alt/Az to RA/Dec using *that reading's own timestamp* (because Alt/Az
   -> RA/Dec is time-dependent -- the sky moves under the Earth-fixed
   IMU frame), and that becomes the best pointing estimate.
5. The offset is refreshed every time a new plate solve succeeds, so it
   never has to remain valid for longer than one slew.

HOW THE OFFSET IS COMPUTED -- AND WHY IT MATTERS WHICH WAY
============================================================
There are two calibration modes, chosen automatically based on whether
`PlateSolveResult.position_angle_deg` is supplied:

* **Full attitude (position angle known).** `offset = true_attitude *
  raw_attitude.inverse()`, a genuine full 3-DOF quaternion. For a fixed
  ("constant bias") sensor-frame error this is *exact* for every
  subsequent pointing direction, not just near the calibration point --
  applying it to a raw reading anywhere else on the sky reproduces the
  true attitude to numerical precision (verified in the test suite).
  Use this mode if your plate solver reports field rotation (many do:
  ASTAP, astrometry.net, PlateSolve2/3, etc.). IMPORTANT: this mode is
  only as good as `position_angle_deg` being measured in the same
  convention the estimator assumes -- see `position_angle_convention`
  below; getting this wrong doesn't just bias the output, it corrupts
  automatic mount calibration itself (see mount_calibration.py), because
  the mismatch changes with hour angle and declination rather than
  staying constant.

* **Direction only (no position angle).** Here we only know a single
  pointing *direction*, which is 2 constraints, not enough to pin down a
  general 3-DOF rotation bias -- naively picking "the" rotation that
  maps the raw direction onto the true direction (e.g. the minimal-arc
  rotation between the two vectors) only corrects pointing near that
  one calibration direction and can be *badly wrong* elsewhere on the
  sky, because the leftover, unconstrained "twist" about the sighted
  direction gets applied as if it were a global correction. Instead this
  module assumes the physically realistic error model for a
  magnetometer+accelerometer-fused AHRS like the BNO085: the
  accelerometer/gyro tilt reference is trustworthy, so essentially all
  of the bias is a single **heading (yaw) error about the true local
  vertical**. A pure yaw-about-vertical rotation, unlike an arbitrary
  minimal-arc rotation, *does* generalize exactly to every other
  pointing direction (it's a fixed-axis rotation, so it's globally
  self-consistent by construction). If your BNO085 also has a
  significant, slowly-varying tilt bias, only the full-attitude mode
  (above) removes it -- but for typical use the yaw-only correction,
  refreshed on every successful solve, is what keeps a raw-vector-only
  plate solver's calibration useful across a whole slew.

WHY YOU DON'T NEED TO KNOW THE BNO085's INTERNAL AXIS CONVENTION
==================================================================
The BNO085's fused "Rotation Vector" output is referenced to *some*
Earth-fixed frame (heading via magnetometer, tilt via
accelerometer/gyroscope) -- but the exact axis labelling depends on the
firmware/library, on how the board is physically mounted to the OTA,
and on magnetic declination at your site. All of that reduces to a
single, roughly-constant, rotation between "IMU's idea of its reference
frame" and "true Alt/Az". Because we calibrate that rotation from
plate-solves rather than assuming it, all of it is automatically
absorbed into `offset` -- you do not need to align the IMU to true
north, correct for declination, or worry about the driver's remap.
What *does* matter is that the board stays rigidly fixed to the optical
tube (a mounting shift invalidates the calibration).

MOUNTING ORIENTATION -- AND AUTOMATIC MOUNT CALIBRATION
==========================================================
The BNO085 does not need to be aligned to true North, level, or any other
external reference -- that's the point of calibrating from plate solves
(see above). It also does not need its own body axes to line up with the
telescope's optical axis: `body_boresight` tells the estimator, in the
chip's own body-frame coordinates, roughly which direction the optical
axis points (default `[0, 0, 1]`, i.e. the chip's own +Z axis pointing
down the tube) -- e.g. set it to `[1, 0, 0]` if the chip is rotated 90
degrees so its +X axis is the one pointing down the tube.

`body_boresight` alone only pins down 2 of the chip's 3 rotational
degrees of freedom -- the boresight *direction*, not the roll ("twist")
about it. If that twist were simply left as an arbitrary guess, RA/Dec
(not just position angle) comes out wrong by an amount that grows with
however far the telescope has moved since the last plate-solve -- right
after a solve everything looks correct, then drifts/reverses while
slewing, then snaps back on the next solve.

Instead, by default (`require_mount_calibration=True`), the estimator
*learns* the chip's true mounting rotation -- both the direction and the
twist -- from your own plate-solves, via `mount_calibration.py`. Every
solve that includes a position angle contributes one data point; once
enough of them are spread across enough different attitudes (in
practice, a handful of solves on objects in genuinely different parts of
the sky), the true mounting rotation is fitted and used from then on,
and RA/Dec (and position angle) become accurate everywhere, not just
near wherever you last solved. See mount_calibration.py's module
docstring for the math, and `TelescopePointingEstimator.mount_calibrated`
/ `.mount_calibration_status` below for how to track progress.

**Until that fit completes, IMU-corrected estimates are suppressed** --
`estimate()` returns `None` (exactly as it does before the very first
solve) rather than handing back a direction that's only trustworthy near
the last solve point. Plate-solve results themselves are returned
normally throughout -- they're ground truth regardless of mount
calibration. In ordinary use this is not a separate "calibration mode"
you have to run: it happens automatically as you solve on a few
different objects, which is a normal part of starting an observing
session anyway.

Because this fits fresh from data every run, **it must, and does, redo
this calibration every time the process starts** -- nothing is persisted
to disk between power-ups. That's deliberate: if this plate-solver/IMU
combination is ever moved to a different telescope, there's no stale
mounting rotation from a previous scope to accidentally reuse.

`body_boresight` still matters in two ways even with automatic
calibration on: it's the starting guess used for the (yaw-only) direct-
solve estimate mode when your plate solver doesn't report a position
angle at all (see the calibration-mode section above) -- for that mode,
which never accumulates enough information to fit the twist, getting
`body_boresight` roughly right (a protractor and the mechanical drawing
is plenty) is what keeps things accurate; and it's used as the initial
guess before the automatic fit completes. The mounting must be **rigid**
regardless -- if the chip shifts relative to the tube after calibration,
the fitted (or assumed) mounting rotation is no longer valid until
recalibrated.

If you'd rather keep the old behaviour (assume `body_boresight` is
exact, including its twist, and never suppress IMU estimates), pass
`require_mount_calibration=False`.

USAGE SKETCH
============
    site = ObserverLocation(latitude_deg=51.5, longitude_deg=-0.1, elevation_m=30)
    estimator = TelescopePointingEstimator(site)

    # whenever the plate solver succeeds:
    estimator.calibrate(PlateSolveResult(ra_deg=..., dec_deg=..., timestamp=...),
                         IMUSample(x=..., y=..., z=..., w=..., timestamp=...))

    # every control-loop tick, whether or not a solve is available:
    estimate = estimator.estimate(IMUSample(x=..., y=..., z=..., w=..., timestamp=...))
    if estimate is not None:
        print(estimate.ra_deg, estimate.dec_deg, estimate.is_calibrated)
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Callable, Deque, Optional, Tuple
from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation

from astro_time import ObserverLocation, radec_to_altaz, altaz_to_radec
from attitude import (
    attitude_from_altaz_pa,
    altaz_pa_from_attitude,
    altaz_to_enu_vector,
    enu_vector_to_altaz,
    boresight_vector,
    body_frame_from_boresight,
    celestial_north_reference_enu,
    rotation_from_xyzw,
    angle_between,
    yaw_offset_deg,
    yaw_rotation,
    slerp,
)
from mount_calibration import MountCalibrator, MountCalibrationStatus


# --------------------------------------------------------------------------
# Data types
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class IMUSample:
    """One BNO085 Rotation Vector reading.

    x, y, z, w are the scalar-last quaternion components as returned by
    (e.g.) Adafruit's CircuitPython BNO08x driver's `.quaternion`
    property, which returns `(quat_i, quat_j, quat_k, quat_real)` --
    i.e. pass those four values straight through as (x, y, z, w).
    """
    x: float
    y: float
    z: float
    w: float
    timestamp: datetime
    accuracy_rad: Optional[float] = None  # BNO085 "quaternion accuracy" estimate, if available

    def rotation(self) -> Rotation:
        return rotation_from_xyzw(self.x, self.y, self.z, self.w)


@dataclass(frozen=True)
class PlateSolveResult:
    """One successful plate-solve."""
    ra_deg: float
    dec_deg: float
    timestamp: datetime
    position_angle_deg: Optional[float] = None  # None if your solver doesn't report field rotation


@dataclass(frozen=True)
class PointingEstimate:
    """Best current estimate of where the telescope is pointing."""
    ra_deg: float
    dec_deg: float
    position_angle_deg: Optional[float]
    timestamp: datetime
    source: str            # "plate_solve" or "imu_corrected"
    is_calibrated: bool    # whether an offset has ever been established
    offset_age_s: Optional[float]  # seconds since the offset used was computed
    is_stale: bool = False  # True if offset_age_s exceeds the estimator's max_offset_age


# --------------------------------------------------------------------------
# Core estimator
# --------------------------------------------------------------------------

class TelescopePointingEstimator:
    """Fuses plate-solver truth with BNO085 dead-reckoning.

    Parameters
    ----------
    site:
        Observer location, needed to convert between RA/Dec and Alt/Az.
    body_boresight:
        Unit vector, in the IMU's own body frame, along the telescope's
        optical axis. Defaults to +Z. If the BNO085 is mounted so a
        different chip axis points down the tube, set this accordingly
        (a rough guess is fine -- it only affects how quickly the very
        first calibration converges, not long-run accuracy, since the
        offset calibration re-derives the effective mapping each time).
    offset_smoothing:
        0.0 = never update after the first calibration: 1.0 = always
        fully replace with the newest calibration (default). Values in
        between exponentially blend the new offset with the old one,
        which can help ride out a single noisy plate-solve. Recommended
        to leave at 1.0 unless you see calibration jitter between
        consecutive solves.
    max_offset_age:
        If set, `estimate()` still returns an IMU-corrected estimate for
        offsets older than this, but sets `PointingEstimate.is_stale =
        True` (and always reports `offset_age_s`) so the caller can
        decide whether to trust it (e.g. widen search radius for the
        next plate-solve attempt, or fall back to a slower/blind slew
        rate).
    require_mount_calibration:
        Default True: suppress IMU-corrected estimates (`estimate()`
        returns None) until enough PA-bearing solves at sufficiently
        different attitudes have arrived to fit the chip's true 3-DOF
        mounting rotation (see "MOUNTING ORIENTATION -- AND AUTOMATIC
        MOUNT CALIBRATION" above, and mount_calibration.py). Plate-solve
        results are returned normally regardless of this setting. Set to
        False to restore the old behaviour: trust `body_boresight`
        (including its unconstrained twist) immediately, no suppression.
    mount_calibration_min_pair_deg, mount_calibration_min_axis_spread_deg,
    mount_calibration_history_len, mount_calibration_min_new_sample_deg:
        Tuning knobs passed straight through to `MountCalibrator` -- see
        its docstring. The defaults are reasonable starting points; widen
        `mount_calibration_min_axis_spread_deg` if the fit is accepting
        too early with a high residual, or the reverse if it never seems
        to complete despite solving on a good spread of objects. If
        calibration seems permanently stuck despite solving on plenty of
        genuinely different targets, and your solver runs fast relative
        to how long you dwell on each target, try raising
        `mount_calibration_min_new_sample_deg` -- see its docstring in
        mount_calibration.py for why a fast solver can silently starve
        the fit of the spread it needs.
    position_angle_convention:
        Which "north" your plate solver's position angle is measured
        from. Default `"equatorial"`: from the North Celestial Pole
        direction (towards increasing declination) -- the standard
        astronomical convention used by virtually every real plate
        solver (astrometry.net, ASTAP, PlateSolve2/3, ...), including
        (per direct confirmation) this package's own reference user.
        `"altaz"`: from local horizon/Alt-Az north instead -- only if
        your solver or a field-derotator genuinely reports roll that
        way. Getting this wrong doesn't just bias `position_angle_deg`
        in the output -- for the full-attitude calibration mode it feeds
        a subtly *wrong* `true_rot` into every calibration, and because
        the difference (the parallactic angle) changes continuously with
        hour angle and declination, it isn't a fixed error that more
        recalibration papers over, or that a fitted `body_mount_rotation`
        can compensate for either (mount fitting assumes a *constant*
        relationship between raw and true attitudes -- a wrong,
        position-dependent `true_rot` breaks that assumption and can
        produce a poor fit with a large residual, not just bias). See
        `attitude.celestial_north_reference_enu` for the geometry.
    """

    def __init__(
        self,
        site: ObserverLocation,
        body_boresight: np.ndarray = np.array([0.0, 0.0, 1.0]),
        offset_smoothing: float = 1.0,
        max_offset_age: Optional[timedelta] = None,
        history_len: int = 50,
        require_mount_calibration: bool = True,
        mount_calibration_min_pair_deg: float = 8.0,
        mount_calibration_min_axis_spread_deg: float = 25.0,
        mount_calibration_history_len: int = 30,
        mount_calibration_min_new_sample_deg: float = 3.0,
        position_angle_convention: str = "equatorial",
    ):
        self.site = site
        self.body_boresight = np.asarray(body_boresight, dtype=float)
        self.offset_smoothing = float(offset_smoothing)
        self.max_offset_age = max_offset_age
        self.require_mount_calibration = require_mount_calibration
        if position_angle_convention not in ("equatorial", "altaz"):
            raise ValueError('position_angle_convention must be "equatorial" or "altaz"')
        self.position_angle_convention = position_angle_convention

        self._offset: Optional[Rotation] = None          # world-frame correction quaternion
        self._offset_computed_at: Optional[datetime] = None
        self._has_position_angle = False                  # whether calibration included PA

        # Canonical (+Z=boresight) -> chip frame. Starts out as the
        # body_boresight-derived guess (direction only, twist arbitrary);
        # superseded by a fitted, fully-constrained rotation once the
        # MountCalibrator has enough spread -- see calibrate() below.
        self._mount_frame: Rotation = body_frame_from_boresight(self.body_boresight)
        self._mount_fitted = False
        self._mount_calibrator = MountCalibrator(
            min_pair_deg=mount_calibration_min_pair_deg,
            min_axis_spread_deg=mount_calibration_min_axis_spread_deg,
            history_len=mount_calibration_history_len,
            min_new_sample_deg=mount_calibration_min_new_sample_deg,
        )

        self._imu_history: Deque[IMUSample] = deque(maxlen=history_len)

    def _up_reference(self, ra_deg: float, dec_deg: float, timestamp: datetime) -> Optional[np.ndarray]:
        """The world-ENU 'north' vector to use as PA=0, per
        `self.position_angle_convention` -- None means attitude.py's
        legacy local-ENU-north default (the "altaz" convention)."""
        if self.position_angle_convention == "equatorial":
            return celestial_north_reference_enu(ra_deg, dec_deg, self.site, timestamp)
        return None

    # -- calibration ------------------------------------------------------

    def calibrate(self, solve: PlateSolveResult, imu_at_solve: IMUSample) -> Rotation:
        """Compute (and store) a fresh offset from a successful plate-solve
        plus the IMU sample taken at essentially the same instant.

        Returns the newly computed raw offset (before smoothing) in case
        the caller wants to log/inspect it.
        """
        true_alt, true_az = radec_to_altaz(solve.ra_deg, solve.dec_deg, self.site, solve.timestamp)
        raw_rot = imu_at_solve.rotation()

        if solve.position_angle_deg is not None:
            # Full 3-DOF calibration: compare complete attitudes, referenced
            # to the telescope's canonical boresight frame (not necessarily
            # the chip's raw +Z axis -- see self._mount_frame). This is
            # exact for a constant sensor-frame bias, everywhere on the sky
            # (see module docstring), *provided* self._mount_frame correctly
            # captures the fixed mounting geometry -- including the "roll
            # about the boresight axis" that body_boresight alone can't
            # constrain. That's what the block below is for: feed this solve
            # into the automatic mount calibrator, and adopt its fit as soon
            # as one is available (it only refuses while there isn't yet
            # enough angular spread to trust it -- see mount_calibration.py).
            up_ref = self._up_reference(solve.ra_deg, solve.dec_deg, solve.timestamp)
            true_rot = attitude_from_altaz_pa(true_alt, true_az, solve.position_angle_deg, up_reference=up_ref)

            self._mount_calibrator.add(raw_rot, true_rot, solve.timestamp)
            fit = self._mount_calibrator.try_fit()
            if fit is not None:
                self._mount_frame = fit.mount_frame
                self._mount_fitted = True

            raw_rot_boresight_frame = raw_rot * self._mount_frame
            new_offset = true_rot * raw_rot_boresight_frame.inv()
            self._has_position_angle = True
        else:
            # Direction-only calibration: pure yaw-about-vertical correction
            # (see module docstring for why this, and not a generic
            # vector-alignment rotation, is the right choice here).
            raw_bore_world = boresight_vector(raw_rot, self.body_boresight)
            raw_alt, raw_az = enu_vector_to_altaz(raw_bore_world)
            yaw = yaw_offset_deg(raw_alt, raw_az, true_alt, true_az)
            new_offset = yaw_rotation(yaw)
            self._has_position_angle = False

        if self._offset is None or self.offset_smoothing >= 1.0:
            self._offset = new_offset
        else:
            self._offset = slerp(self._offset, new_offset, self.offset_smoothing)

        self._offset_computed_at = solve.timestamp
        return new_offset

    @property
    def is_calibrated(self) -> bool:
        return self._offset is not None

    @property
    def offset_rotation(self) -> Optional[Rotation]:
        return self._offset

    @property
    def mount_calibrated(self) -> bool:
        """True once the chip's full 3-DOF mounting rotation has been
        fitted from a sufficiently spread-out set of PA-bearing solves
        (see mount_calibration.py). Irrelevant if
        `require_mount_calibration=False`, since IMU estimates aren't
        gated on it in that mode -- but this still reflects whether a fit
        has actually been achieved."""
        return self._mount_fitted

    @property
    def mount_calibration_status(self) -> MountCalibrationStatus:
        """Human-readable progress toward automatic mount calibration --
        see `MountCalibrationStatus.describe()`. Useful for telling the
        user what's still needed while `estimate()` is returning None."""
        return self._mount_calibrator.status()

    # -- estimation ---------------------------------------------------------

    def note_imu_sample(self, imu_sample: IMUSample) -> None:
        """Record an IMU reading for the angular-rate history, without
        computing a pointing estimate from it. `estimate()` calls this
        itself; call it directly if you read the IMU on a tick where you
        don't (yet) want a full estimate -- e.g. just to check
        `angular_rate_deg_s()` before deciding whether to attempt a solve
        -- so the rate calculation still sees every reading."""
        self._imu_history.append(imu_sample)

    def estimate(self, imu_sample: IMUSample) -> Optional[PointingEstimate]:
        """Return the best current pointing estimate for a raw IMU sample.

        Returns None only if no calibration has ever been performed (i.e.
        we have no way yet to relate IMU output to sky coordinates).

        Records `imu_sample` into the angular-rate history. If you already
        recorded this exact sample yourself (via `note_imu_sample`), use
        `estimate_without_recording()` instead to avoid counting it twice.
        """
        self.note_imu_sample(imu_sample)
        return self.estimate_without_recording(imu_sample)

    def estimate_without_recording(self, imu_sample: IMUSample) -> Optional[PointingEstimate]:
        """Same as `estimate()`, but does not append to the angular-rate
        history -- for callers (like `get_pointing_estimate()`) that
        already recorded this sample via `note_imu_sample()`."""
        if self._offset is None:
            return None
        if self.require_mount_calibration and self._has_position_angle and not self._mount_fitted:
            # We have an offset, but it (and self._mount_frame) were built
            # from an assumed, arbitrarily-twisted mounting guess -- good
            # enough to reproduce the last solve exactly, not yet trustworthy
            # anywhere else. See mount_calibration_status for progress.
            #
            # This only applies when calibration is using position angle at
            # all -- the yaw-only mode (no PA reported) never lets a twist
            # ambiguity into the offset in the first place (see module
            # docstring), so it has nothing to wait for here.
            return None

        raw_rot = imu_sample.rotation()
        corrected_rot = self._offset * raw_rot  # world-frame left-multiplication
        # Composing with self._mount_frame (canonical -> chip) converts this
        # back into the canonical (+Z=boresight) frame the solve itself was
        # expressed in -- so both the pointing direction *and* position
        # angle come out of the same, single composed attitude, correctly,
        # exactly matching the true attitude everywhere once self._mount_frame
        # is the fitted (not just assumed) mounting rotation.
        full_corrected = corrected_rot * self._mount_frame
        # alt/az from the boresight alone first (cheap, and unaffected by PA
        # convention) so ra/dec -- and therefore the matching up_reference --
        # are available before extracting PA in the right convention below.
        alt, az = enu_vector_to_altaz(full_corrected.apply([0.0, 0.0, 1.0]))
        ra, dec = altaz_to_radec(alt, az, self.site, imu_sample.timestamp)

        up_ref = self._up_reference(ra, dec, imu_sample.timestamp)
        _, _, pa = altaz_pa_from_attitude(full_corrected, up_reference=up_ref)
        pa_deg = pa if self._has_position_angle else None

        age_s = None
        is_stale = False
        if self._offset_computed_at is not None:
            age_s = (imu_sample.timestamp - self._offset_computed_at).total_seconds()
            if self.max_offset_age is not None:
                is_stale = age_s > self.max_offset_age.total_seconds()

        return PointingEstimate(
            ra_deg=ra,
            dec_deg=dec,
            position_angle_deg=pa_deg,
            timestamp=imu_sample.timestamp,
            source="imu_corrected",
            is_calibrated=True,
            offset_age_s=age_s,
            is_stale=is_stale,
        )

    # -- motion / slew-rate helpers ------------------------------------------

    def angular_rate_deg_s(self) -> Optional[float]:
        """Estimate current slewing rate (deg/s) from the last two IMU samples.

        Useful for deciding whether the plate-solver is likely to succeed
        (slow/stationary) or will be blurred (fast slew) -- e.g. only
        attempt a solve, or trust one, below some rate threshold.
        """
        if len(self._imu_history) < 2:
            return None
        a, b = self._imu_history[-2], self._imu_history[-1]
        dt = (b.timestamp - a.timestamp).total_seconds()
        if dt <= 0:
            return None
        ang = angle_between(a.rotation(), b.rotation())
        return ang / dt

    def settled_duration_s(self, max_rate_deg_s: float) -> float:
        """How long (seconds), counting backward from the most recent IMU
        reading, EVERY consecutive pair of recorded samples has shown a
        rate at or below `max_rate_deg_s` -- not just the last pair.

        `angular_rate_deg_s()` only ever looks at the newest two samples,
        which is enough to catch "clearly still slewing" but not enough to
        know the IMU has had time to physically and electronically settle
        after stopping. This exists because of a real, reported BNO085
        characteristic: the Rotation Vector sensor-fusion output can
        overshoot briefly after a fast rotation ends (worse after a faster
        slew), and that overshoot doesn't necessarily show up as a large
        instantaneous rate between consecutive readings -- it can look
        like "basically stopped" tick-to-tick while still being
        measurably wrong for several hundred milliseconds. A rate
        threshold alone, however tight, can't distinguish "genuinely
        settled" from "reads slow right now but still recovering" -- see
        `get_pointing_estimate()`'s `min_settle_s` for how this is used to
        require a sustained quiet period, not just an instantaneously low
        rate, before trusting a solve for calibration.

        Returns 0.0 if fewer than 2 samples are recorded yet, or if the
        most recent pair already exceeds the threshold.
        """
        samples = list(self._imu_history)
        if len(samples) < 2:
            return 0.0
        newest_t = samples[-1].timestamp
        earliest_ok_t = newest_t
        for i in range(len(samples) - 1, 0, -1):
            a, b = samples[i - 1], samples[i]
            dt = (b.timestamp - a.timestamp).total_seconds()
            if dt <= 0:
                break
            rate = angle_between(a.rotation(), b.rotation()) / dt
            if rate > max_rate_deg_s:
                break
            earliest_ok_t = a.timestamp
        return (newest_t - earliest_ok_t).total_seconds()


def combine_estimate(
    estimator: TelescopePointingEstimator,
    solve: Optional[PlateSolveResult],
    imu_sample: IMUSample,
) -> Optional[PointingEstimate]:
    """Convenience wrapper for a typical control loop tick:

    - If a plate solve is available this tick, use it as ground truth
      *and* (re)calibrate the offset from it.
    - Otherwise, fall back to the IMU + offset estimate.

    Pass `solve=None` on ticks where the plate-solver didn't run or
    failed to solve (e.g. during a fast slew).
    """
    if solve is not None:
        estimator.calibrate(solve, imu_sample)
        return PointingEstimate(
            ra_deg=solve.ra_deg,
            dec_deg=solve.dec_deg,
            position_angle_deg=solve.position_angle_deg,
            timestamp=solve.timestamp,
            source="plate_solve",
            is_calibrated=True,
            offset_age_s=0.0,
        )
    return estimator.estimate(imu_sample)


def get_pointing_estimate(
    estimator: TelescopePointingEstimator,
    capture_and_solve: Callable[[], Tuple[float, float, float, bool]],
    read_imu: Callable[[], IMUSample],
    max_rate_deg_s: Optional[float] = None,
    min_settle_s: Optional[float] = None,
) -> Optional[PointingEstimate]:
    """One call, one estimate: checks the IMU turning rate first, and only
    attempts a plate-solve if the scope isn't slewing too fast for one to
    plausibly succeed. This is the simplest way to wire the estimator into
    a loop -- everything happens synchronously, in this one call, with no
    threads or locks.

    * If the scope appears to be moving no faster than `max_rate_deg_s`
      (or its rate isn't known yet -- e.g. the very first call), this
      calls `capture_and_solve()` and waits for it to finish. On success,
      the offset is recalibrated from the result and the plate-solved
      RA/Dec/roll is returned. On failure, falls back to a *fresh* IMU
      reading (since the failed attempt may have taken a while).
    * If the scope is turning faster than `max_rate_deg_s`, `
      capture_and_solve()` is skipped entirely and an IMU-corrected
      estimate is returned immediately.
    * `max_rate_deg_s=None` (the default) disables the rate check --
      every call attempts a solve first, exactly like `pointing_loop_tick`.

    `capture_and_solve`: your capture+solve function, called with no
    arguments, returning `(ra_deg, dec_deg, roll_deg, solved)`.
    `read_imu`: a no-argument function returning one fresh `IMUSample`.

    min_settle_s: None (default) preserves the exact original behaviour
        below. Set this to require the scope to have read as "slow enough"
        for at least this many CONTINUOUS seconds (via
        `estimator.settled_duration_s()`), not merely on the single most
        recent reading, before a solve is attempted -- found necessary
        from a real user report: the BNO085's Rotation Vector fusion
        output can overshoot for several hundred milliseconds after a
        fast slew ends, in a way that doesn't reliably show up as a large
        instantaneous rate between consecutive readings, so calibrating
        from an IMU sample taken right as the rate check first clears can
        feed a genuinely wrong (still-settling) attitude into
        mount_calibration.py -- this showed up as a calibration residual
        that improved with the position-angle-convention geometry fixes
        but then plateaued well above the solve-noise floor. A reasonable
        starting point is your IMU's known overshoot-recovery time (e.g.
        ~0.5s was reported for the BNO085) -- tune from there.

        When set AND a solve succeeds, this also takes a FRESH IMU sample
        (rather than reusing the pre-capture one) to pair with the
        calibration -- see below for why that's a strict improvement, not
        just a timing nuance.

    WORTH KNOWING: this calls `capture_and_solve` directly and BLOCKS
    until it returns. The rate check filters out the common case (an
    obvious fast slew), but a solve can still occasionally fail -- and
    therefore still take however long your solver takes to give up --
    even while the IMU says the scope is nominally slow (poor seeing, not
    enough stars, a passing cloud, etc.). If that occasional stall is a
    problem for you, `pointing_loop_tick_nonblocking()` +
    `background_solver.BackgroundSolver` avoids it entirely by running
    the solve on its own thread -- but that's more moving parts, so start
    here and only reach for that if you actually need it.
    """
    imu_sample = read_imu()
    estimator.note_imu_sample(imu_sample)  # recorded exactly once for this reading

    rate = estimator.angular_rate_deg_s()
    moving_too_fast = (
        max_rate_deg_s is not None and rate is not None and rate > max_rate_deg_s
    )

    if not moving_too_fast and max_rate_deg_s is not None and min_settle_s is not None:
        # Instantaneously "slow enough" isn't the same as "settled" -- see
        # min_settle_s above. Treat an insufficiently-settled reading the
        # same as "too fast": skip the solve attempt for now, try again
        # next tick once more quiet time has accumulated.
        if estimator.settled_duration_s(max_rate_deg_s) < min_settle_s:
            moving_too_fast = True

    if moving_too_fast:
        # Already recorded above -- use the no-record variant so this
        # reading isn't double-counted in the rate history.
        return estimator.estimate_without_recording(imu_sample)

    ra_deg, dec_deg, roll_deg, solved = capture_and_solve()

    if solved:
        # Prefer a FRESH IMU sample, taken now (after capture_and_solve()
        # has finished), over the pre-capture one -- but ONLY when
        # min_settle_s is set, so this never silently changes behaviour
        # for a caller not opting into it. This is a strict improvement,
        # not just a different timing choice: a SUCCESSFUL solve already
        # implies the telescope was genuinely stationary throughout the
        # whole capture_and_solve() call (a moving scope blurs the frame
        # and fails), so the boresight direction hasn't changed either
        # way -- but reading the IMU now gives the Rotation Vector fusion
        # filter strictly more elapsed time to finish settling (whatever
        # capture_and_solve() itself took -- anywhere from ~20ms for an
        # encoder-rig read to several hundred ms for a real camera
        # exposure+solve), which is exactly the quantity min_settle_s
        # cares about.
        if min_settle_s is not None:
            imu_sample = read_imu()
            estimator.note_imu_sample(imu_sample)
        solve = PlateSolveResult(
            ra_deg=ra_deg,
            dec_deg=dec_deg,
            position_angle_deg=roll_deg,
            timestamp=imu_sample.timestamp,
        )
        return combine_estimate(estimator, solve, imu_sample)

    fresh_imu = read_imu()
    return estimator.estimate(fresh_imu)


def pointing_loop_tick(
    estimator: TelescopePointingEstimator,
    capture_and_solve: Callable[[], Tuple[float, float, float, bool]],
    read_imu: Callable[[], IMUSample],
) -> Optional[PointingEstimate]:
    """One call, one control-loop tick: returns a plate-solve result when
    the solver succeeds (and recalibrates the offset from it), or an
    IMU-corrected estimate when it doesn't.

    IMPORTANT: this calls `capture_and_solve` directly and BLOCKS until it
    returns -- if a bad image makes your solver take several seconds, this
    call takes several seconds too, right when you most need a fast
    stream of IMU estimates (during a slew). If that matters for your
    loop, use `pointing_loop_tick_nonblocking()` with
    `background_solver.BackgroundSolver` instead, which runs the
    capture+solve on its own thread so this never has to wait for it.

    `capture_and_solve`: your existing capture+solve function, called with
    no arguments, returning `(ra_deg, dec_deg, roll_deg, solved)` -- exactly
    the (ra, dec, roll, solve) tuple you described. It may take anywhere
    from ~200ms (good image) to several seconds (bad image) to return.

    `read_imu`: a no-argument function returning one fresh `IMUSample` (see
    bno085_interface.py's `read_imu_sample`).

    Why two IMU reads, not one: this takes an IMU sample *immediately
    before* calling `capture_and_solve`, on the assumption that's close to
    when the image was actually taken -- that's the sample used to
    (re)calibrate if the solve succeeds. Any error from "immediately
    before" not being the exact exposure instant is negligible precisely
    because a *successful* solve implies the telescope was stationary or
    tracking during that exposure, i.e. barely moving.

    If the solve fails, though, `capture_and_solve` may have just spent
    several seconds blocked (e.g. during a fast slew) -- so rather than
    handing back an estimate computed from that now-stale pre-capture IMU
    reading, this takes a *second*, fresh IMU sample right before
    returning, so the IMU-corrected estimate reflects where the telescope
    actually is *now*, not several seconds ago.
    """
    imu_at_capture = read_imu()
    ra_deg, dec_deg, roll_deg, solved = capture_and_solve()

    if solved:
        solve = PlateSolveResult(
            ra_deg=ra_deg,
            dec_deg=dec_deg,
            position_angle_deg=roll_deg,
            timestamp=imu_at_capture.timestamp,
        )
        return combine_estimate(estimator, solve, imu_at_capture)

    fresh_imu = read_imu()
    return estimator.estimate(fresh_imu)


def pointing_loop_tick_nonblocking(
    estimator: TelescopePointingEstimator,
    background_solver,  # a background_solver.BackgroundSolver -- not type-hinted
    read_imu: Callable[[], IMUSample],
) -> Optional[PointingEstimate]:
    """Like `pointing_loop_tick()`, but never blocks on the plate-solver.

    Use this together with `background_solver.BackgroundSolver`, which
    runs your capture+solve function continuously in its own thread. Each
    call here just checks whether a new solve has finished since the last
    call: if so, it's used to recalibrate and is returned as this tick's
    result; if not, a fresh IMU-corrected estimate is returned instead --
    either way, this function returns essentially immediately (microseconds
    to a millisecond or so), regardless of how long the solver is taking.

    `read_imu` here should be `background_solver.read_imu_locked` (or your
    own IMU read guarded by the *same* lock the BackgroundSolver uses) --
    see background_solver.py's module docstring for why the lock matters.
    """
    result = background_solver.get_latest_and_clear()
    if result is not None:
        ra_deg, dec_deg, roll_deg, solved, imu_at_capture = result
        if solved:
            solve = PlateSolveResult(
                ra_deg=ra_deg,
                dec_deg=dec_deg,
                position_angle_deg=roll_deg,
                timestamp=imu_at_capture.timestamp,
            )
            return combine_estimate(estimator, solve, imu_at_capture)
        # solve failed -- fall through to a fresh IMU estimate below

    fresh_imu = read_imu()
    return estimator.estimate(fresh_imu)
