"""
attitude.py
-----------
Quaternion / attitude helpers for the telescope pointing estimator.

Conventions used throughout this package:
  * Quaternions are `scipy.spatial.transform.Rotation` objects internally,
    and where raw components are needed they are in **scalar-last**
    (x, y, z, w) order -- this matches both scipy's convention and the
    tuple returned by Adafruit's BNO08x CircuitPython driver
    (`quat_i, quat_j, quat_k, quat_real`).
  * A rotation `q` maps a vector from a sensor/body frame to a world
    (topocentric horizontal, i.e. local Alt/Az) frame:
        v_world = q.apply(v_body)
  * The world frame is local ENU (East, North, Up) built from Alt/Az:
        x = East, y = North, z = Up (zenith)
  * "Position angle" (PA) of the telescope's image-up direction is
    measured from a "north" reference, increasing towards East, in the
    plane perpendicular to the boresight -- but WHICH north depends on
    `up_reference` (see `attitude_from_altaz_pa`/`altaz_pa_from_attitude`
    below): almost every real plate solver (astrometry.net, ASTAP,
    PlateSolve2/3, ...) means the North Celestial Pole direction (i.e.
    "up" = towards increasing declination) -- NOT local horizon/Alt-Az
    north, which only coincides with it momentarily, on the meridian. Get
    this wrong and PA (and therefore, for the full-attitude calibration
    mode, RA/Dec too) is corrupted by an error that changes continuously
    with hour angle and declination -- see telescope_pointing.py's
    `position_angle_convention` for how the estimator picks the right one
    automatically.

Nothing here needs to know the BNO085's own internal reference-frame
convention (NED, NWU, whatever the firmware/library uses) -- see the
module docstring in `telescope_pointing.py` for why that's fine.
"""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from astro_time import ObserverLocation, radec_to_altaz


def altaz_to_enu_vector(alt_deg: float, az_deg: float) -> np.ndarray:
    """Alt/Az (deg, Az from North through East) -> unit vector in local ENU."""
    alt = np.radians(alt_deg)
    az = np.radians(az_deg)
    x = np.cos(alt) * np.sin(az)  # East
    y = np.cos(alt) * np.cos(az)  # North
    z = np.sin(alt)               # Up
    return np.array([x, y, z])


def enu_vector_to_altaz(v: np.ndarray) -> Tuple[float, float]:
    """Unit vector in local ENU -> (alt_deg, az_deg)."""
    v = v / np.linalg.norm(v)
    alt = np.degrees(np.arcsin(np.clip(v[2], -1.0, 1.0)))
    az = np.degrees(np.arctan2(v[0], v[1])) % 360.0
    return float(alt), float(az)


def _perpendicular_reference(bore: np.ndarray, up_reference: Optional[np.ndarray]) -> np.ndarray:
    """A unit vector perpendicular to `bore`, built from `up_reference`
    (falling back to a fixed seed if `up_reference` is too close to
    parallel with `bore` to use directly -- e.g. looking straight at the
    celestial pole when `up_reference` is the celestial-north direction)."""
    if up_reference is None:
        up_reference = np.array([0.0, 1.0, 0.0])  # local ENU/Alt-Az north (legacy default)
    ref = up_reference
    if abs(np.dot(ref, bore)) > 0.9999:
        for seed in (np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])):
            if abs(np.dot(seed, bore)) < 0.9999:
                ref = seed
                break
    n = ref - np.dot(ref, bore) * bore
    return n / np.linalg.norm(n)


def attitude_from_altaz_pa(alt_deg: float, az_deg: float, pa_deg: float = 0.0, up_reference: Optional[np.ndarray] = None) -> Rotation:
    """Build a full body->world attitude quaternion from Alt/Az + position angle.

    The body frame's +Z axis is defined as the boresight; +Y is "image up"
    at the given position angle; +X completes a right-handed frame.

    up_reference: world-ENU-frame unit vector defining PA=0 ("north"),
        projected perpendicular to the boresight -- defaults to local ENU
        north [0,1,0] (i.e. local horizon/Alt-Az-referenced PA) if not
        given. Pass `celestial_north_reference_enu(...)` here for the
        standard astronomical convention almost all real plate solvers
        use -- see the module docstring and
        `TelescopePointingEstimator.position_angle_convention`.
    """
    bore = altaz_to_enu_vector(alt_deg, az_deg)
    n = _perpendicular_reference(bore, up_reference)
    e = np.cross(bore, n)
    e = e / np.linalg.norm(e)

    pa = np.radians(pa_deg)
    up = np.cos(pa) * n + np.sin(pa) * e
    right = np.cross(up, bore)
    right = right / np.linalg.norm(right)

    body_to_world = np.column_stack([right, up, bore])  # columns = body X,Y,Z in world coords
    return Rotation.from_matrix(body_to_world)


def altaz_pa_from_attitude(rot: Rotation, up_reference: Optional[np.ndarray] = None) -> Tuple[float, float, float]:
    """Inverse of attitude_from_altaz_pa. Returns (alt_deg, az_deg, pa_deg).

    up_reference: see `attitude_from_altaz_pa` -- must match whatever
    convention `pa_deg` is meant to be measured in, or the returned PA
    (only PA -- alt/az are unaffected) will be in the wrong convention.
    """
    mat = rot.as_matrix()
    right, up, bore = mat[:, 0], mat[:, 1], mat[:, 2]
    alt, az = enu_vector_to_altaz(bore)

    n = _perpendicular_reference(bore, up_reference)
    e = np.cross(bore, n)
    e = e / np.linalg.norm(e)

    pa = np.degrees(np.arctan2(np.dot(up, e), np.dot(up, n))) % 360.0
    return alt, az, float(pa)


def celestial_north_reference_enu(
    ra_deg: float,
    dec_deg: float,
    site: "ObserverLocation",
    timestamp,
    dec_step_deg: float = 1e-3,
) -> np.ndarray:
    """Local ENU-frame direction towards increasing declination ("celestial
    north") as seen from the given RA/Dec position, at this site/time --
    the correct `up_reference` for the STANDARD astronomical position-angle
    convention (measured from the North Celestial Pole through East), used
    by virtually every real plate solver.

    This is generally NOT the same as local horizon ("Alt-Az") north --
    the two coincide only momentarily, when the target is on the meridian;
    otherwise they differ by the parallactic angle, which changes
    continuously with hour angle and declination. Computed here via a
    small step in Dec through the existing, round-trip-validated
    `radec_to_altaz` conversion, rather than a hand-derived parallactic-
    angle formula -- one less place to get a sign convention wrong.
    """
    dec_hi = min(90.0 - 1e-6, dec_deg + dec_step_deg)
    alt0, az0 = radec_to_altaz(ra_deg, dec_deg, site, timestamp)
    alt1, az1 = radec_to_altaz(ra_deg, dec_hi, site, timestamp)
    v0 = altaz_to_enu_vector(alt0, az0)
    v1 = altaz_to_enu_vector(alt1, az1)
    d = v1 - v0
    norm = np.linalg.norm(d)
    if norm < 1e-12:
        # Degenerate (e.g. exactly at the pole) -- no well-defined
        # celestial-north direction there; local ENU north is as good as
        # anything.
        return np.array([0.0, 1.0, 0.0])
    return d / norm


def boresight_vector(rot: Rotation, body_boresight: np.ndarray = np.array([0.0, 0.0, 1.0])) -> np.ndarray:
    """World-frame direction the telescope is pointing, given a body->world attitude."""
    return rot.apply(body_boresight)


def minimal_rotation_between(a: np.ndarray, b: np.ndarray) -> Rotation:
    """Shortest-arc rotation that maps unit vector `a` onto unit vector `b`.

    Leaves rotation about the resulting axis (i.e. roll about `b`) at zero --
    this is the right tool when you only know two *directions* (e.g. Alt/Az
    without a position angle) and not a full 3-DOF attitude.
    """
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    dot = np.clip(np.dot(a, b), -1.0, 1.0)
    if dot > 1.0 - 1e-12:
        return Rotation.identity()
    if dot < -1.0 + 1e-12:
        # 180 degree case: pick any axis perpendicular to a
        perp = np.cross(a, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(perp) < 1e-6:
            perp = np.cross(a, np.array([0.0, 1.0, 0.0]))
        perp = perp / np.linalg.norm(perp)
        return Rotation.from_rotvec(perp * np.pi)
    axis = np.cross(a, b)
    axis = axis / np.linalg.norm(axis)
    angle = np.arccos(dot)
    return Rotation.from_rotvec(axis * angle)


def body_frame_from_boresight(body_boresight: np.ndarray) -> Rotation:
    """Rotation mapping a canonical "boresight frame" (+Z = boresight) into
    the sensor's own body/chip frame, given the boresight direction
    expressed in chip coordinates (`body_boresight`).

    i.e. `body_frame_from_boresight(b).apply([0,0,1]) == b` (normalized).
    Used to make attitude comparisons boresight-relative regardless of how
    the chip happens to be mounted; the roll about the boresight axis is
    left unconstrained (set to zero twist), consistent with the fact that
    `body_boresight` alone doesn't specify a mounting roll.
    """
    return minimal_rotation_between(np.array([0.0, 0.0, 1.0]), body_boresight)


def yaw_offset_deg(raw_alt_deg: float, raw_az_deg: float, true_alt_deg: float, true_az_deg: float) -> float:
    """Azimuth (heading) correction needed to rotate a raw Alt/Az onto a true
    Alt/Az, assuming the error is a pure yaw about the local vertical
    (zenith) axis -- i.e. assuming altitude/tilt is already trustworthy.

    This is the physically-appropriate model for a magnetometer+accelerometer
    fused AHRS such as the BNO085's Rotation Vector: gravity gives an
    accurate "down" reference (so altitude/tilt is reliable), while the
    magnetometer-derived heading is the part that carries most of the bias
    (uncompensated magnetic declination, local interference, imperfect mag
    calibration). Unlike a generic two-vector "minimal rotation" fit, a pure
    yaw-about-vertical correction is *exact* for every subsequent pointing
    direction, not just a local approximation near the calibration point --
    see the worked derivation in telescope_pointing.py's module docstring.
    """
    d = (true_az_deg - raw_az_deg + 180.0) % 360.0 - 180.0
    return d


def yaw_rotation(yaw_deg: float) -> Rotation:
    """World-frame (ENU) rotation that *increases azimuth* by `yaw_deg`.

    Azimuth is measured from North (+Y) through East (+X), which is a
    clockwise sense when viewed from above (+Z looking down) -- i.e. the
    opposite handedness from a standard right-handed rotation about +Z.
    Hence the sign flip here; `yaw_offset_deg(raw_alt, raw_az, true_alt,
    true_az)` fed straight into this function yields the rotation that
    carries the raw boresight vector onto the true one.
    """
    return Rotation.from_euler("z", -yaw_deg, degrees=True)


def quat_xyzw(rot: Rotation) -> np.ndarray:
    """Return the (x, y, z, w) scalar-last quaternion components."""
    return rot.as_quat()


def rotation_from_xyzw(x: float, y: float, z: float, w: float) -> Rotation:
    """Build a Rotation from scalar-last (x, y, z, w) components (auto-normalized)."""
    q = np.array([x, y, z, w], dtype=float)
    n = np.linalg.norm(q)
    if n == 0:
        raise ValueError("Zero-norm quaternion")
    return Rotation.from_quat(q / n)


def angle_between(rot_a: Rotation, rot_b: Rotation) -> float:
    """Angular difference (deg) between two attitudes (shortest-arc)."""
    rel = rot_a.inv() * rot_b
    return float(np.degrees(rel.magnitude()))


def slerp(rot_a: Rotation, rot_b: Rotation, t: float) -> Rotation:
    """Spherical linear interpolation between two attitudes, t in [0, 1]."""
    from scipy.spatial.transform import Slerp
    key_rots = Rotation.concatenate([rot_a, rot_b])
    interp = Slerp([0.0, 1.0], key_rots)
    return interp([t])[0]
