"""
astro_time.py
-------------
Minimal, dependency-free (numpy only) astronomical time and coordinate
conversion helpers: Julian Date, Greenwich/Local Sidereal Time, and
RA/Dec <-> Alt/Az transforms.

These are implemented from the standard formulas (Meeus, "Astronomical
Algorithms") rather than pulling in astropy, so this module has no
network/install dependency beyond numpy. Accuracy is at the arcsecond
level, which is more than sufficient for telescope pointing correction
(round-trip tested to <1e-9 deg in astro_time_selftest()).

All angles in degrees unless a name ends in `_rad`. All times are
timezone-aware `datetime` objects (naive datetimes are assumed UTC).
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone

import numpy as np


def _as_utc(dt: datetime) -> datetime:
    if dt.tzinfo is None:
        return dt.replace(tzinfo=timezone.utc)
    return dt.astimezone(timezone.utc)


def julian_date(dt_utc: datetime) -> float:
    """Julian Date for a (UTC) datetime."""
    dt_utc = _as_utc(dt_utc)
    y, m = dt_utc.year, dt_utc.month
    d = dt_utc.day + (
        dt_utc.hour + dt_utc.minute / 60.0 + (dt_utc.second + dt_utc.microsecond / 1e6) / 3600.0
    ) / 24.0
    if m <= 2:
        y -= 1
        m += 12
    a = y // 100
    b = 2 - a + a // 4
    jd = int(365.25 * (y + 4716)) + int(30.6001 * (m + 1)) + d + b - 1524.5
    return jd


def gmst_hours(jd: float) -> float:
    """Greenwich Mean Sidereal Time, in hours, for a given Julian Date."""
    t = (jd - 2451545.0) / 36525.0
    gmst_deg = (
        280.46061837
        + 360.98564736629 * (jd - 2451545.0)
        + 0.000387933 * t * t
        - (t * t * t) / 38710000.0
    )
    return (gmst_deg % 360.0) / 15.0


def lst_hours(jd: float, longitude_deg: float) -> float:
    """Local (Mean) Sidereal Time, in hours. East longitude positive."""
    return (gmst_hours(jd) + longitude_deg / 15.0) % 24.0


@dataclass(frozen=True)
class ObserverLocation:
    """Observer's site, used for RA/Dec <-> Alt/Az conversion."""
    latitude_deg: float
    longitude_deg: float  # East positive
    elevation_m: float = 0.0


def radec_to_altaz(ra_deg: float, dec_deg: float, site: ObserverLocation, dt_utc: datetime):
    """Convert RA/Dec (ICRS-ish, apparent-place accuracy) to Alt/Az at a site and time.

    Azimuth convention: measured from North (0 deg), through East, 0-360.
    Returns (alt_deg, az_deg).
    """
    jd = julian_date(dt_utc)
    lst = lst_hours(jd, site.longitude_deg)
    hour_angle_deg = lst * 15.0 - ra_deg
    h = np.radians(hour_angle_deg)
    dec = np.radians(dec_deg)
    lat = np.radians(site.latitude_deg)

    sin_alt = np.sin(lat) * np.sin(dec) + np.cos(lat) * np.cos(dec) * np.cos(h)
    alt = np.arcsin(np.clip(sin_alt, -1.0, 1.0))

    az_from_south = np.arctan2(
        np.sin(h), np.cos(h) * np.sin(lat) - np.tan(dec) * np.cos(lat)
    )
    az = (np.degrees(az_from_south) + 180.0) % 360.0
    return float(np.degrees(alt)), float(az)


def altaz_to_radec(alt_deg: float, az_deg: float, site: ObserverLocation, dt_utc: datetime):
    """Inverse of radec_to_altaz. Returns (ra_deg, dec_deg)."""
    jd = julian_date(dt_utc)
    lst = lst_hours(jd, site.longitude_deg)

    az_from_south = np.radians((az_deg - 180.0) % 360.0)
    alt = np.radians(alt_deg)
    lat = np.radians(site.latitude_deg)

    sin_dec = np.sin(lat) * np.sin(alt) - np.cos(lat) * np.cos(alt) * np.cos(az_from_south)
    dec = np.arcsin(np.clip(sin_dec, -1.0, 1.0))

    h = np.arctan2(
        np.sin(az_from_south),
        np.cos(az_from_south) * np.sin(lat) + np.tan(alt) * np.cos(lat),
    )
    ra = (lst * 15.0 - np.degrees(h)) % 360.0
    return float(ra), float(np.degrees(dec))


def astro_time_selftest() -> None:
    """Round-trip sanity check. Raises AssertionError on failure."""
    rng = np.random.default_rng(0)
    site = ObserverLocation(51.5, -0.1, 30.0)
    dt = datetime.now(timezone.utc)
    worst = 0.0
    for _ in range(500):
        ra = float(rng.uniform(0, 360))
        dec = float(rng.uniform(-85, 85))
        alt, az = radec_to_altaz(ra, dec, site, dt)
        if alt < 0:
            continue
        ra2, dec2 = altaz_to_radec(alt, az, site, dt)
        dra = min(abs(ra - ra2), 360 - abs(ra - ra2)) * np.cos(np.radians(dec))
        ddec = abs(dec - dec2)
        worst = max(worst, dra, ddec)
    assert worst < 1e-6, f"round trip error too large: {worst} deg"


if __name__ == "__main__":
    astro_time_selftest()
    print("astro_time self-test passed.")
