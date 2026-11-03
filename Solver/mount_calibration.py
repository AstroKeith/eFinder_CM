"""
mount_calibration.py
----------------------
Determines the IMU chip's true, full 3-DOF mounting rotation relative to
the telescope's own body frame (the boresight direction *and* the twist
about it) from multiple plate-solves taken at different attitudes --
instead of assuming it, the way `body_boresight` alone must.

WHY THIS EXISTS
================
`body_boresight` tells the estimator which way the telescope points in
the chip's own coordinates, but that's only 2 of the chip's 3 rotational
degrees of freedom -- it says nothing about the roll ("twist") of the
chip about that direction. The rest of the code used to fill that gap
with an arbitrary (but consistent) guess. That guess reproduces the
correct RA/Dec exactly *at* the most recent calibration point -- but as
soon as the telescope moves away from that point, before the next solve
arrives, the wrong twist assumption leaks into the IMU-corrected RA/Dec,
growing with the angle moved. That's what produced the "pointing looks
right immediately after a solve, drifts/reverses while slewing, snaps
back on the next solve" behaviour.

The fix: stop guessing the twist, and instead solve for it, using
several plate-solves taken at genuinely different attitudes. This module
does that continuously and automatically -- every solve that includes a
position angle is fed in here, and as soon as enough of them are spread
across enough different rotation axes, a full 3-DOF mount rotation is
fitted and handed back. There's no separate "calibration mode": ordinary
use (solving on a few different objects, e.g. at startup) naturally
produces exactly the kind of data this needs.

THE MATH, BRIEFLY
===================
Model: at every solve, `true_attitude = W * raw_attitude * R`, where `W`
is a *world*-frame bias (the sensor's own fusion error -- heading bias
etc, refreshed every solve) and `R` is a *body*-frame constant (the
chip's fixed mounting rotation -- what we're solving for here). Taking
the *relative* rotation between any two solves cancels `W` out entirely:

    true_i^-1 * true_j  =  R^-1 * (raw_i^-1 * raw_j) * R

i.e. the relative rotation seen by the chip and the relative rotation
seen by the solver are related by a fixed conjugation by R. Conjugation
preserves rotation angle and carries the rotation *axis* through R, so
each pair of solves gives one correspondence between a "raw axis" and a
"true axis" -- exactly Wahba's problem / the Kabsch algorithm, solved
here with an SVD.

One such correspondence only pins down 2 of R's 3 degrees of freedom --
the same limitation `body_boresight` alone has. Two correspondences with
sufficiently different axes are needed to fully constrain it, which in
practice means at least ~3 solves where the telescope moved in genuinely
different directions between them -- not just back and forth along one
line (e.g. only ever panning in azimuth, which keeps giving you the same
rotation axis no matter how far you pan). Solving on a few different
objects spread around the sky satisfies this naturally.

A REAL SOLVER CAN OUTRUN YOUR HISTORY BUFFER -- WHY add() DEDUPLICATES
==========================================================================
`history_len` is a count of *solves*, not of *distinct attitudes*. A real
camera can solve several times a second, while the telescope typically
sits on one target for many seconds or minutes while actually observing
-- so a single dwell can produce far more solves than `history_len`
keeps room for, all of them near-identical. Without a guard, every one of
those becomes a real, separate entry, and once a dwell outlasts the
buffer it evicts every genuinely-different-direction sample gathered
before it -- calibration can then stay stuck indefinitely ("waiting for a
THIRD solve in a different direction") no matter how many genuinely
different targets get visited, as long as each dwell alone is long enough
to fill the whole buffer on its own. `add()` below silently skips a new
solve that's too close to the last *stored* one (`min_new_sample_deg`) to
prevent exactly this -- see its docstring for the reasoning.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from datetime import datetime
from typing import Deque, Optional

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class MountCalibrationSample:
    raw_rot: Rotation
    true_rot: Rotation
    timestamp: datetime


@dataclass(frozen=True)
class _PairData:
    """Vectorized per-pair data: (P,3) raw axes, (P,3) true axes, (P,) angles
    in degrees -- one row per valid solve-pair. Internal to MountCalibrator."""
    raw_axes: np.ndarray
    true_axes: np.ndarray
    angles_deg: np.ndarray

    @classmethod
    def empty(cls) -> "_PairData":
        return cls(raw_axes=np.zeros((0, 3)), true_axes=np.zeros((0, 3)), angles_deg=np.zeros(0))


@dataclass(frozen=True)
class MountCalibrationFit:
    mount_frame: Rotation  # canonical (+Z=boresight) -> chip frame; same role/shape as body_frame_from_boresight()
    n_pairs_used: int
    residual_deg: float    # RMS angular residual of the fit -- rough quality indicator (bigger = noisier/less rigid)


@dataclass(frozen=True)
class MountCalibrationStatus:
    calibrated: bool
    n_solves: int
    n_valid_pairs: int           # solve-pairs with enough motion between them to trust their axis
    best_axis_spread_deg: float  # largest angular separation found between any two valid-pair axes so far
    residual_deg: Optional[float]

    def describe(self) -> str:
        if self.calibrated:
            return (
                f"mount calibrated ({self.n_valid_pairs} pairs used, "
                f"residual {self.residual_deg:.3f} deg)"
            )
        if self.n_solves < 2:
            return f"waiting for more solves ({self.n_solves} so far)"
        if self.n_valid_pairs < 2:
            # IMPORTANT: with only one usable pair, best_axis_spread_deg is
            # necessarily 0 -- there's nothing yet to compare it against,
            # no matter how big that one motion was. This is the single
            # most common point of confusion: moving 90 degrees in one go
            # produces exactly one pair, and one pair alone can never
            # measure spread (spread means comparing *two* pairs' axes).
            # What's needed is a *third* solve, ideally one that changes
            # direction (e.g. mixing altitude and azimuth movement, or
            # picking a genuinely different target) rather than continuing
            # further along the same swing -- a long, smooth, single slew
            # tends to keep re-using close to the same rotation axis, no
            # matter how far it travels in total.
            return (
                f"waiting for a THIRD solve in a different direction "
                f"({self.n_solves} solves so far, but only {self.n_valid_pairs} usable "
                f"pair -- distance moved doesn't matter here, only direction: one pair "
                f"can never show spread on its own, however far it was)"
            )
        return (
            f"waiting for more angular spread ({self.n_solves} solves, "
            f"{self.n_valid_pairs} usable pairs so far, best axis spread "
            f"{self.best_axis_spread_deg:.1f} deg -- point at something that "
            f"changes DIRECTION, not just distance: a long smooth slew "
            f"tends to keep reusing nearly the same rotation axis however "
            f"far it travels, so pick a target that isn't roughly along the "
            f"same swing as your other solves)"
        )


class MountCalibrator:
    """Accumulates (raw, true) attitude pairs from successful, PA-bearing
    plate-solves, and fits the chip's full 3-DOF mounting rotation once
    there's enough spread among them to constrain it.

    min_pair_deg:
        Only relative rotations between two solves of at least this many
        degrees are trusted enough to contribute an axis estimate --
        smaller motions are dominated by solve noise. Default 8 degrees.
    min_axis_spread_deg:
        The fit is only accepted once at least two trusted pairs have
        rotation axes separated by at least this much -- otherwise the
        twist about the (still effectively single) axis stays
        unconstrained, same as with `body_boresight` alone. Default 25
        degrees.
    history_len:
        How many recent solves to keep for fitting (older ones are
        dropped as new ones arrive). Default 30 -- more than enough for a
        solid fit without unbounded growth over a long session, PROVIDED
        `add()` is only fed genuinely new attitudes -- see
        `min_new_sample_deg` below for why that isn't automatic.
    min_new_sample_deg:
        `add()` silently ignores a new solve if it's less than this many
        degrees from the *last stored* sample's raw attitude. Exists
        because a real plate solver can run much faster than the
        telescope moves -- e.g. several solves a second while the scope
        sits on one target for minutes at a time -- and every one of
        those near-identical solves is a real call to `add()`. Without
        this gate, `history_len` counts *solves*, not *distinct
        attitudes*: a single multi-second dwell can fill (and, on a long
        enough dwell, completely overflow) the whole history buffer with
        near-duplicates of one spot, silently evicting the genuinely
        different-direction samples from wherever the telescope pointed
        before -- so calibration can end up permanently stuck (spread
        never accumulates, no matter how many genuinely different targets
        get visited) even though every individual solve looked fine.
        Default 3 degrees -- comfortably above realistic solve noise/dwell
        jitter, comfortably below `min_pair_deg`, so this only ever
        discards true duplicates, never a pair-worthy motion.
    """

    def __init__(
        self,
        min_pair_deg: float = 8.0,
        min_axis_spread_deg: float = 25.0,
        history_len: int = 30,
        min_new_sample_deg: float = 3.0,
    ):
        self.min_pair_deg = min_pair_deg
        self.min_axis_spread_deg = min_axis_spread_deg
        self.min_new_sample_deg = min_new_sample_deg
        self._samples: Deque[MountCalibrationSample] = deque(maxlen=history_len)

    def add(self, raw_rot: Rotation, true_rot: Rotation, timestamp: datetime) -> None:
        """Record one (raw, true) attitude pair from a successful,
        PA-bearing plate-solve -- unless it's too close to the last
        *stored* sample to be worth a history slot (see
        `min_new_sample_deg` above). This is deliberately compared only
        against the single most recent stored sample, not the whole
        history: that's exactly what distinguishes "the scope has been
        sitting on this same target, solve after solve" (skip) from "the
        scope moved to a new target, even if it's revisiting a direction
        from a while ago" (store -- extra samples of a previously-seen
        direction are harmless, and worth keeping in case the mount has
        drifted since)."""
        if self._samples:
            last_raw = self._samples[-1].raw_rot
            moved_deg = float(np.degrees((last_raw.inv() * raw_rot).magnitude()))
            if moved_deg < self.min_new_sample_deg:
                return
        self._samples.append(MountCalibrationSample(raw_rot, true_rot, timestamp))

    def _valid_pairs(self) -> "_PairData":
        """raw/true axis+angle arrays for every stored pair of solves whose
        relative rotation is at least `min_pair_deg`.

        Fully vectorized (no per-pair Python loop): with `history_len`
        solves kept, there are up to `history_len choose 2` pairs -- doing
        each pair's rotation composition and rotvec extraction as a separate
        scipy `Rotation` call in a Python loop turned out to be slow enough
        on a Pi-class CPU to noticeably lengthen the control loop (a real
        bug found from a user report, not a hypothetical) -- so this builds
        every pair's relative-rotation matrix as one batched numpy operation
        and converts the whole batch to rotation vectors in a single
        (C-level, not Python-level) scipy call instead.
        """
        samples = list(self._samples)
        n = len(samples)
        if n < 2:
            return _PairData.empty()

        raw_mats = np.stack([s.raw_rot.as_matrix() for s in samples])   # (n,3,3)
        true_mats = np.stack([s.true_rot.as_matrix() for s in samples])  # (n,3,3)

        i_idx, j_idx = np.triu_indices(n, k=1)  # every i<j pair, vectorized
        # raw_i^-1 * raw_j, as matrices: inverse of a rotation matrix is its
        # transpose, so this is raw_i^T @ raw_j for every pair at once.
        raw_rel = np.einsum("pji,pjk->pik", raw_mats[i_idx], raw_mats[j_idx])
        true_rel = np.einsum("pji,pjk->pik", true_mats[i_idx], true_mats[j_idx])

        raw_rotvecs = Rotation.from_matrix(raw_rel).as_rotvec()   # (P,3) -- one batched call
        true_rotvecs = Rotation.from_matrix(true_rel).as_rotvec()

        raw_angles_deg = np.degrees(np.linalg.norm(raw_rotvecs, axis=1))
        true_norms = np.linalg.norm(true_rotvecs, axis=1)

        valid = (raw_angles_deg >= self.min_pair_deg) & (true_norms >= 1e-9)
        if not np.any(valid):
            return _PairData.empty()

        raw_norms = np.linalg.norm(raw_rotvecs[valid], axis=1)
        raw_axes = raw_rotvecs[valid] / raw_norms[:, None]
        true_axes = true_rotvecs[valid] / true_norms[valid, None]

        return _PairData(raw_axes=raw_axes, true_axes=true_axes, angles_deg=raw_angles_deg[valid])

    @staticmethod
    def _best_axis_spread_deg(pairs: "_PairData") -> float:
        n = len(pairs.raw_axes)
        if n < 2:
            return 0.0
        dots = np.clip(pairs.raw_axes @ pairs.raw_axes.T, -1.0, 1.0)
        sep = np.degrees(np.arccos(dots))
        # A rotation axis is only defined up to sign (+angle about +axis is
        # the same motion as -angle about -axis) -- compare using whichever
        # separation is smaller.
        sep = np.minimum(sep, 180.0 - sep)
        np.fill_diagonal(sep, 0.0)  # a pair compared with itself isn't spread
        return float(np.max(sep))

    def _fit(self, pairs: "_PairData") -> Optional[MountCalibrationFit]:
        if len(pairs.raw_axes) < 2:
            return None
        P, Q, w = pairs.raw_axes, pairs.true_axes, pairs.angles_deg  # weight by angle: bigger motions -> more trustworthy axis

        # Kabsch/Wahba fit: find the rotation M with M.apply(P_i) ~= Q_i.
        H = (P * w[:, None]).T @ Q
        U, _, Vt = np.linalg.svd(H)
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        if d == 0:
            d = 1.0
        D = np.diag([1.0, 1.0, d])
        chip_to_canonical = Rotation.from_matrix(Vt.T @ D @ U.T)  # v_canonical = M.apply(v_chip)
        mount_frame = chip_to_canonical.inv()  # canonical -> chip; matches body_frame_from_boresight()'s role

        predicted = chip_to_canonical.apply(P)
        cos_residual = np.clip(np.sum(predicted * Q, axis=1), -1.0, 1.0)
        residuals_deg = np.degrees(np.arccos(cos_residual))
        residual_deg = float(np.sqrt(np.mean(residuals_deg ** 2)))

        return MountCalibrationFit(mount_frame=mount_frame, n_pairs_used=len(P), residual_deg=residual_deg)

    def status(self) -> MountCalibrationStatus:
        """Human-readable progress snapshot -- doesn't require a successful
        fit, useful for telling the user what's still needed."""
        pairs = self._valid_pairs()
        spread = self._best_axis_spread_deg(pairs)
        calibrated = spread >= self.min_axis_spread_deg
        residual_deg = None
        if calibrated:
            fit = self._fit(pairs)
            residual_deg = fit.residual_deg if fit is not None else None
        return MountCalibrationStatus(
            calibrated=calibrated,
            n_solves=len(self._samples),
            n_valid_pairs=len(pairs.raw_axes),
            best_axis_spread_deg=spread,
            residual_deg=residual_deg,
        )

    def try_fit(self) -> Optional[MountCalibrationFit]:
        """Attempt a fit using everything collected so far. Returns None if
        there isn't yet enough angular spread to trust one -- call
        `status()` (or `.describe()`) for a human-readable reason why, if
        you want to report progress to the user."""
        pairs = self._valid_pairs()
        if self._best_axis_spread_deg(pairs) < self.min_axis_spread_deg:
            return None
        return self._fit(pairs)
