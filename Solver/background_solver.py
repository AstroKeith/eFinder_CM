"""
background_solver.py
----------------------
Runs your (possibly slow) capture+solve function in its own background
thread, continuously, so the main pointing loop never has to wait for it.

THE PROBLEM THIS SOLVES
=========================
`pointing_loop_tick()` in telescope_pointing.py is simple, but it calls
your `capture_and_solve` function directly and waits for it to return --
so if a bad image makes the solve take several seconds, the *entire*
main loop stalls for those seconds. That's exactly backwards: a slew is
when the solver is most likely to take a long time (motion blur), and
also exactly when you most need a fast, uninterrupted stream of
IMU-corrected estimates for the mount to act on.

THE FIX: A PRODUCER/CONSUMER SPLIT
=====================================
`BackgroundSolver` runs your capture+solve function in a loop, in its
own thread ("the producer"). Each time it finishes -- whether it took
200ms or 5 seconds -- it drops the result into a single-slot mailbox and
immediately starts capturing the next frame.

Your main loop ("the consumer") never calls the solver directly. Instead,
each tick it takes a fresh IMU reading (near-instant) and *checks* the
mailbox: if a new solve result has shown up since the last check, it uses
it (recalibrating the offset); if not, it just returns the fresh
IMU-corrected estimate. The main loop's cycle time is therefore governed
by the IMU read and a bit of math -- typically well under a millisecond
-- never by the camera/solver.

Use `pointing_loop_tick_nonblocking()` from telescope_pointing.py together
with this class -- see main_loop_example.py for the full wiring.

A NOTE ON THREAD SAFETY
=========================
Only ONE thread (whichever calls `pointing_loop_tick_nonblocking`, i.e.
your main loop) ever touches the `TelescopePointingEstimator` -- the
background thread here never calls `.calibrate()` or `.estimate()`
itself, it only ever writes finished results into the mailbox. So the
estimator itself needs no locking.

The BNO085 *does* need protecting, though: both the background thread
(reading the IMU at each capture) and the main thread (reading it every
tick) talk to it over the same I2C bus, and I2C reads are not safe to
interleave from two threads at once. Pass a single shared `threading.Lock`
in `imu_lock` and this class serializes its own I2C reads through it --
make sure your main loop's `read_imu` acquires the *same* lock object
before its own reads (see main_loop_example.py).

SKIPPING SOLVES WHILE SLEWING (optional, `max_rate_deg_s`)
=============================================================
There's no point calling your capture+solve function while the telescope
is clearly moving too fast for it to succeed -- it'll just burn an
exposure and (per your numbers) several seconds of processing time,
before failing anyway. If you pass `max_rate_deg_s`, this class checks
the angular rate between successive raw IMU readings before each capture
attempt and skips calling `capture_and_solve` entirely whenever that
rate is above the threshold, polling the IMU again shortly after instead.

This check works out cleanly even before the estimator has ever been
calibrated: the *rate* of change between two raw IMU quaternions is
exactly the true angular rate regardless of any uncalibrated sensor bias
-- a fixed offset rotation doesn't change how fast two readings are
rotating relative to each other, only where they point. So this is safe
to rely on from the very first reading, with no calibration required.

Pick `max_rate_deg_s` from your optics: roughly, motion blur becomes a
problem for your solver once the sky moves more than about one
resolution element (star FWHM in your images, in degrees) during one
exposure. E.g. at 2 arcsec/pixel with ~3-pixel stars and a 0.5s exposure,
that's about (3 * 2 / 3600) / 0.5 =~ 0.003 deg/s as a rough floor --
in practice you'll want to set this empirically (start generous, e.g.
1-2 deg/s, and tighten it if you still see solve failures below that
rate, or loosen it if solves are being skipped when they'd have
succeeded).
"""
from __future__ import annotations

import threading
from typing import Callable, Optional, Tuple

from telescope_pointing import IMUSample
from attitude import angle_between


SolveResult = Tuple[float, float, float, bool, IMUSample]  # (ra, dec, roll, solved, imu_at_capture)


class BackgroundSolver:
    def __init__(
        self,
        capture_and_solve: Callable[[], Tuple[float, float, float, bool]],
        read_imu: Callable[[], IMUSample],
        imu_lock: Optional[threading.Lock] = None,
        max_rate_deg_s: Optional[float] = None,
        poll_interval_s: float = 0.05,
    ):
        """
        capture_and_solve: your existing function, called with no
            arguments, returning (ra_deg, dec_deg, roll_deg, solved).
        read_imu: a no-argument function returning one fresh IMUSample.
        imu_lock: a threading.Lock shared with your main loop's own IMU
            reads, so the two threads never talk to the I2C bus at the
            same time. If you don't pass one, this class makes its own --
            but then YOU must not read the IMU from the main loop except
            through `read_imu_locked` (below), or you'll still have the
            race. Simplest is to create one lock in your setup code and
            pass it in here (see main_loop_example.py).
        max_rate_deg_s: if set, skip calling `capture_and_solve` whenever
            the angular rate between successive IMU readings exceeds this
            (see "Skipping solves while slewing" above). None (default)
            disables this check -- capture_and_solve is called every cycle
            regardless of motion, as before.
        poll_interval_s: how often to re-check the IMU rate while capture
            is being skipped for being "too fast". Only relevant when
            max_rate_deg_s is set.
        """
        self._capture_and_solve = capture_and_solve
        self._read_imu = read_imu
        self._imu_lock = imu_lock or threading.Lock()
        self.max_rate_deg_s = max_rate_deg_s
        self.poll_interval_s = poll_interval_s

        self._mailbox_lock = threading.Lock()
        self._latest_result: Optional[SolveResult] = None
        self._last_rate_imu: Optional[IMUSample] = None
        self._last_known_rate_deg_s: Optional[float] = None

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    @property
    def imu_lock(self) -> threading.Lock:
        """The lock guarding IMU/I2C access -- share this with your main loop."""
        return self._imu_lock

    def start(self) -> None:
        self._thread.start()

    def stop(self, timeout: Optional[float] = None) -> None:
        """Signal the background thread to stop after its current
        capture+solve call returns, and wait for it to exit."""
        self._stop_event.set()
        self._thread.join(timeout=timeout)

    def read_imu_locked(self) -> IMUSample:
        """Thread-safe IMU read -- call this (not your raw read_imu) from
        your main loop if you want this class to own the shared lock."""
        with self._imu_lock:
            return self._read_imu()

    @property
    def last_known_rate_deg_s(self) -> Optional[float]:
        """Most recently measured IMU angular rate, deg/s (None until at
        least two readings have been taken). Useful for logging/tuning
        `max_rate_deg_s`, whether or not gating is enabled."""
        return self._last_known_rate_deg_s

    def _run(self) -> None:
        while not self._stop_event.is_set():
            with self._imu_lock:
                imu_now = self._read_imu()

            if self.max_rate_deg_s is not None:
                rate = self._update_rate(imu_now)
                if rate is not None and rate > self.max_rate_deg_s:
                    # Moving too fast for a solve to plausibly succeed --
                    # don't bother capturing/solving this cycle.
                    self._stop_event.wait(self.poll_interval_s)
                    continue

            imu_at_capture = imu_now
            ra, dec, roll, solved = self._capture_and_solve()
            with self._mailbox_lock:
                self._latest_result = (ra, dec, roll, solved, imu_at_capture)

    def _update_rate(self, imu_now: IMUSample) -> Optional[float]:
        prev = self._last_rate_imu
        self._last_rate_imu = imu_now
        if prev is None:
            return None
        dt = (imu_now.timestamp - prev.timestamp).total_seconds()
        if dt <= 0:
            return None
        rate = angle_between(prev.rotation(), imu_now.rotation()) / dt
        self._last_known_rate_deg_s = rate
        return rate

    def get_latest_and_clear(self) -> Optional[SolveResult]:
        """Return the most recent finished solve exactly once -- None if
        nothing new has finished since the last call. Non-blocking."""
        with self._mailbox_lock:
            result = self._latest_result
            self._latest_result = None
        return result
