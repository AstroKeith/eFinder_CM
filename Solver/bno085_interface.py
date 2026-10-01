"""
bno085_interface.py
--------------------
Example hardware read loop for a BNO085 (or BNO086) IMU, producing
`IMUSample` objects (see telescope_pointing.py) from the sensor's fused
Rotation Vector quaternion output.

This uses Adafruit's CircuitPython BNO08x driver, which works both on
microcontrollers (CircuitPython) and on Linux single-board computers
such as a Raspberry Pi (via Blinka). Install with:

    pip install adafruit-circuitpython-bno08x

Wiring: I2C (SDA/SCL/3V3/GND) is simplest; the driver also supports UART
and SPI variants (`BNO08X_UART`, `BNO08X_SPI`) if you need the extra
bandwidth/reliability for high-rate polling -- swap the constructor
below accordingly.

RASPBERRY PI 4B / COMPUTE MODULE 4: SOFTWARE (BIT-BANGED) I2C
================================================================
The BCM2711 SoC (Pi 4B, CM4) has a well-known I2C clock-stretching
errata that makes the BNO08x hang on the chip's native hardware I2C
peripheral. The standard workaround is a kernel-level bit-banged I2C
bus, enabled via `/boot/config.txt` (or `/boot/firmware/config.txt` on
Bookworm+):

    dtoverlay=i2c-gpio,bus=8,i2c_gpio_sda=23,i2c_gpio_scl=24

(bus number and GPIO pins are yours to choose -- adjust to match your
wiring; bus 8 above matches a common setup on the Pi 4B/CM4, since
buses 0-1 are reserved for the native peripherals). After a reboot this
exposes a normal-looking `/dev/i2c-8` device. In Blinka, that numbered
bus is accessed with
`adafruit_extended_bus.ExtendedI2C` rather than `board.I2C()`/`busio.I2C()`
(which only address the SoC's native peripheral pins) -- see
`open_bno085_software_i2c()` below. Requires:

    pip install adafruit-extended-bus

ROTATION VECTOR vs GAME ROTATION VECTOR
==========================================
By default this module uses the BNO085's Rotation Vector report: a full
gyro+accel+magnetometer fusion that gives an absolute heading (referenced
to magnetic north), at the cost of a noticeably heavier internal filter --
this is the "severe low-pass filter" behaviour other BNO085 users have
reported, and that on-hardware testing here confirmed directly (a real,
multi-second overshoot after a sharp stop -- see check_overshoot_recovery.py).

Pass `use_game_rotation_vector=True` to either `open_bno085_*()` function
below to switch to the Game Rotation Vector report instead: gyro+accel
only, roll/pitch still referenced to gravity but heading left completely
uncorrected by the magnetometer -- lighter filtering, and by reputation
(and Adafruit's own driver docstring: "some drift is expected") more
responsive, at the cost of slow, unconstrained yaw drift over time with
nothing to correct it except a fresh solve.

Whether that trade-off is actually fine for THIS estimator specifically
(not a given for every IMU application) comes down to how
telescope_pointing.py's calibration model works: the world-frame offset
`W` between "raw" and "true" attitude is recomputed FRESH at every single
successful solve (see mount_calibration.py's module docstring) -- so
Game Rotation Vector's heading drift only ever needs to stay small over
the time SINCE YOUR LAST SOLVE, not over the whole session. The one place
it can still matter: mount_calibration.py's mounting-rotation fit (`R`)
compares PAIRS of solves that can be spread across the whole session (its
history buffer holds up to 30 distinct-enough samples) -- if yaw drift
between two solves used as a pair is non-negligible, that drift leaks
into the fit as if it were mounting-rotation inconsistency. Watch for
that specific signature if you try this: residual that's fine early in a
session and creeps up the LONGER the session runs (drift-driven) is a
different symptom from residual that's fine right after calibrating and
plateaus quickly regardless of session length (overshoot-driven, what
you'd been chasing already). `read_imu_sample()` below transparently
reads from whichever report was enabled -- no other code in this package
needs to change either way.

STALE READS AND ONBOARD DYNAMIC CALIBRATION -- THE REAL FIX
================================================================
The multi-second settle that survived both the geometry fixes and the
Game-Rotation-Vector experiment turned out not to be a fusion-algorithm
property at all -- which is exactly why switching fusion modes made no
difference: both problems below sit BELOW the fusion algorithm, in the
raw read path and the chip's own bias tracking, so they hit Rotation
Vector and Game Rotation Vector equally. Confirmed fixed on real hardware
(see bno_plot.py and its before/after plots -- a clean, fast settle with
no lingering tail, versus the old several-second decay).

1. STALE READS. `bno.quaternion` (or `.game_quaternion`) only reflects
   whatever the driver's internal cache currently holds, and that cache
   only advances when `_process_available_packets()` actually runs -- the
   property getter calls it, but only once per access. If your read loop
   polls less often than the sensor produces reports, a backlog of
   unprocessed packets can build up, and a single property access only
   works through one packet's worth of it -- so the value you read can
   trail the sensor's true current state by however many reports are
   still queued, independent of which fusion mode produced them. Fixed by
   draining every buffered packet (looping `_process_available_packets()`
   until nothing's left) immediately before reading -- see
   `_read_latest_quaternion()` below, now used by every `read_imu_sample()`
   call automatically.

2. ONBOARD DYNAMIC CALIBRATION. The BNO085 continuously refines its own
   accel/gyro/mag bias estimates in the background, and by several
   independent users' reports (and confirmed here) that process doesn't
   handle a fast rotation well, producing exactly the kind of settle
   behaviour chased across this whole investigation -- present in the
   underlying gyro/accel bias tracking regardless of which top-level
   fusion report you read. `open_bno085_i2c()`/`open_bno085_software_i2c()`
   now send the SH2 command to disable it by default (see
   `_disable_dynamic_calibration()`) -- there's no public API for this in
   the Adafruit driver, so it talks directly to the sensor hub's
   executable-command channel. Pass `disable_dynamic_calibration=False`
   to restore the sensor's default (enabled) behaviour.

   Frozen calibration is a fine trade for THIS application specifically:
   telescope_pointing.py already re-anchors its world-frame offset fresh
   at every solve (see mount_calibration.py's docstring), so it was never
   depending on the chip's own long-term bias tracking to stay accurate --
   it only ever needed the sensor to hold still *between* solves, which is
   precisely what dynamic calibration was interfering with.

   CAVEAT (see next section): on-hardware testing after this fix went in
   turned up a real cost -- after a significant fast slew, the frozen
   bias can be measurably worse than what dynamic calibration would have
   settled to, showing up as steady drift for the rest of the session
   (see `disable_dynamic_calibration_warmup_s` below, and
   check_overshoot_recovery.py's `drift_rate_deg_s` metric, which exists
   specifically to surface this). Consider leaving dynamic calibration
   enabled (`disable_dynamic_calibration=False`) if your sessions involve
   aggressive slewing and the settle-time problem this was chasing turns
   out to already be solved by the stale-reads fix alone.

3. PERIODIC DCD AUTO-SAVE. Independently of dynamic calibration itself,
   the sensor hub also periodically saves its current calibration state
   (Dynamic Calibration Data, DCD) to its OWN non-volatile flash, on its
   own schedule, without any explicit "save" command from this code. If
   that autosave fires while a bad bias estimate is frozen in (dynamic
   calibration disabled, per point 2 above, possibly mid-slew), the bad
   state gets written to flash -- at which point it survives power
   cycles and neither re-enabling dynamic calibration nor power-cycling
   the sensor fixes it, because the corrupted calibration is now the
   chip's own saved default. This isn't a hypothetical: a sensor here
   ended up in exactly this state (calibration "couldn't be reset,
   disabled, or redone") and had to be physically swapped out.
   `open_bno085_i2c()`/`open_bno085_software_i2c()` now disable this
   autosave by default too (see `_disable_dcd_autosave()`) -- with it
   off, a bad frozen bias only ever lives in RAM, and a plain power
   cycle is always enough to clear it. Pass `disable_dcd_autosave=False`
   to restore the sensor's default (autosave enabled) behaviour.

CLEAR DCD AND RESET -- RECOVERY IF A SENSOR EVER GETS STUCK
================================================================
If a BNO085 ever ends up in the corrupted-calibration state described in
point 3 above -- readings look permanently wrong or drift in a way that
survives a power cycle -- `clear_dcd_and_reset()` sends the SH2 command
that wipes whatever calibration state the chip has saved to its own
flash and reboots the sensor hub back to factory-default calibration,
then re-applies your normal settings so it's immediately usable again.
This is a best-effort recovery tool, NOT yet validated on real hardware
here (unlike the fixes above, all confirmed on-device) -- try it, but if
the same `bno` object seems confused afterward (hangs, or never produces
a report even past the timeout), the reliable fallback is a real power
cycle followed by a fresh `open_bno085_i2c()`/`open_bno085_software_i2c()`
call, which now also disables DCD auto-save by default, so this
shouldn't be able to recur going forward regardless.

This module is standalone and guards its hardware-specific imports so
the rest of the package (and its tests) can be imported/run without the
BNO08x driver or any hardware attached.
"""
from __future__ import annotations

import time
from datetime import datetime, timezone
from typing import Iterator, Optional, Tuple

from telescope_pointing import IMUSample


def _disable_dynamic_calibration(bno) -> bool:
    """Send the raw SH2 'set dynamic calibration' command to turn OFF the
    BNO085's continuous background recalibration (accel/gyro/mag/planar
    all disabled) -- see the module docstring's "STALE READS AND ONBOARD
    DYNAMIC CALIBRATION" section for why this matters here. There's no
    public method for this in adafruit_bno08x, so this talks directly to
    the sensor hub's SH2 executable-command channel (channel 2).

    Best-effort and deliberately non-fatal: on driver variants where
    `_send_packet` doesn't exist or behaves differently (this has only
    been exercised via I2C), this prints a warning and returns False
    rather than blocking startup -- dynamic calibration is just left in
    its default (enabled) state in that case.
    """
    # [command ID 0x07 ("set dynamic calibration"), then accel/gyro/mag/
    # planar enable flags (all 0 = off), then reserved bytes (0)].
    cal_payload = bytearray([0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    try:
        bno._send_packet(2, cal_payload)
        return True
    except Exception as exc:  # noqa: BLE001 -- deliberately broad, see docstring
        print(f"  [bno085_interface] could not disable dynamic calibration (non-fatal): {exc}")
        return False


def _disable_dcd_autosave(bno) -> bool:
    """Send the SH2 'DCD auto-save' command (0x09) with P0=1 to turn OFF
    the BNO085's periodic automatic saving of its calibration state
    (Dynamic Calibration Data) to its own non-volatile flash -- see the
    module docstring's "PERIODIC DCD AUTO-SAVE" section for why this
    matters. P0=0 (the sensor's own default) would leave it enabled.

    Best-effort and deliberately non-fatal, matching
    `_disable_dynamic_calibration()`'s style.
    """
    autosave_payload = bytearray([0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    try:
        bno._send_packet(2, autosave_payload)
        return True
    except Exception as exc:  # noqa: BLE001 -- deliberately broad, see docstring
        print(f"  [bno085_interface] could not disable DCD auto-save (non-fatal): {exc}")
        return False


def _send_clear_dcd_and_reset(bno) -> bool:
    """Send the SH2 'Clear DCD and Reset' command (0x0B): discards
    whatever calibration state the sensor has saved to its own
    non-volatile flash and reboots the sensor hub back to factory-default
    calibration. This only sends the command -- use `clear_dcd_and_reset()`
    below for the full recovery flow (waits for the reboot, then
    re-applies your normal settings).
    """
    reset_payload = bytearray([0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    try:
        bno._send_packet(2, reset_payload)
        return True
    except Exception as exc:  # noqa: BLE001 -- deliberately broad, see docstring
        print(f"  [bno085_interface] could not send Clear DCD and Reset (non-fatal): {exc}")
        return False


def _read_latest_quaternion(bno, use_game_rotation_vector: bool) -> Tuple[float, float, float, float]:
    """Drain every currently-buffered SH2 packet before reading the
    quaternion property, so the value returned is the sensor's FRESHEST
    report -- not a stale one still waiting in an unprocessed backlog.
    See the module docstring's "STALE READS..." section for why a single
    property access alone isn't enough to guarantee that.
    """
    while True:
        try:
            if not bno._process_available_packets():
                break
        except Exception:
            break
    return bno.game_quaternion if use_game_rotation_vector else bno.quaternion


def _wait_for_valid_quaternion(
    bno, use_game_rotation_vector: bool = False, timeout_s: float = 3.0, poll_interval_s: float = 0.02
) -> None:
    """Block until the BNO085 has produced its first real report on
    whichever quaternion feature was enabled, or raise TimeoutError.

    Right after `enable_feature()`, the sensor hasn't necessarily produced
    a report yet -- reading `.quaternion` (or `.game_quaternion`) before it
    has typically returns (0, 0, 0, 0), which is not a valid rotation (it
    has zero length, so there's no way to tell what orientation it's meant
    to represent). Calling this once, right after opening the connection,
    avoids ever handing a caller that placeholder zero reading.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        x, y, z, w = bno.game_quaternion if use_game_rotation_vector else bno.quaternion
        if (x, y, z, w) != (0.0, 0.0, 0.0, 0.0):
            return
        time.sleep(poll_interval_s)
    report_name = "Game Rotation Vector" if use_game_rotation_vector else "Rotation Vector"
    raise TimeoutError(
        f"BNO085 did not produce a valid {report_name} report within {timeout_s}s "
        "of enabling the feature -- check wiring/address, or that the report was "
        "actually enabled successfully."
    )


def _configure_and_enable(
    bno,
    use_game_rotation_vector: bool,
    disable_dynamic_calibration: bool,
    disable_dynamic_calibration_warmup_s: float,
    disable_dcd_autosave: bool,
    report_interval_us: Optional[int],
) -> None:
    """Shared post-connect setup: applies the DCD auto-save / dynamic
    calibration settings, enables the chosen quaternion feature report,
    and blocks until the sensor's first valid reading arrives. Used by
    `open_bno085_i2c()`, `open_bno085_software_i2c()`, and
    `clear_dcd_and_reset()` so all three stay in sync.
    """
    from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_GAME_ROTATION_VECTOR

    if disable_dcd_autosave:
        # Independent of dynamic calibration -- see the module docstring's
        # "PERIODIC DCD AUTO-SAVE" section. Order relative to the other
        # commands doesn't matter, so this goes first for simplicity.
        _disable_dcd_autosave(bno)
    if disable_dynamic_calibration and disable_dynamic_calibration_warmup_s <= 0.0:
        # Sent before enable_feature(), matching the order validated on
        # hardware -- see the module docstring.
        _disable_dynamic_calibration(bno)
    report = BNO_REPORT_GAME_ROTATION_VECTOR if use_game_rotation_vector else BNO_REPORT_ROTATION_VECTOR
    if report_interval_us is not None:
        bno.enable_feature(report, report_interval_us)
    else:
        bno.enable_feature(report)
    if disable_dynamic_calibration and disable_dynamic_calibration_warmup_s > 0.0:
        # Let dynamic calibration run and converge for a while BEFORE
        # freezing it -- see disable_dynamic_calibration_warmup_s below.
        time.sleep(disable_dynamic_calibration_warmup_s)
        _disable_dynamic_calibration(bno)
    _wait_for_valid_quaternion(bno, use_game_rotation_vector=use_game_rotation_vector)
    bno._telescope_pointing_use_game_rotation_vector = use_game_rotation_vector  # read by read_imu_sample()


def open_bno085_i2c(
    i2c_bus=None,
    address: int = 0x4A,
    use_game_rotation_vector: bool = False,
    disable_dynamic_calibration: bool = True,
    disable_dynamic_calibration_warmup_s: float = 0.0,
    disable_dcd_autosave: bool = True,
    report_interval_us: Optional[int] = 10000,
):
    """Initialize a BNO085 over the SoC's native I2C peripheral.

    Returns the driver's `BNO08X` instance, only once it has produced its
    first valid report (see `_wait_for_valid_quaternion`) -- so it's always
    safe to call `read_imu_sample()` immediately after this returns.
    Requires:
        pip install adafruit-circuitpython-bno08x adafruit-blinka

    On a Raspberry Pi (or similar), `i2c_bus` is typically `board.I2C()`.

    use_game_rotation_vector: see the module docstring's "ROTATION VECTOR
    vs GAME ROTATION VECTOR" section -- enables `BNO_REPORT_GAME_ROTATION_VECTOR`
    (gyro+accel fusion only, no magnetometer correction, generally lower
    lag/overshoot but slow unconstrained yaw drift) instead of the default
    `BNO_REPORT_ROTATION_VECTOR`. `read_imu_sample()` on the returned `bno`
    automatically reads from whichever one was enabled here -- no other
    code needs to know or care which mode is active. (Confirmed on
    hardware to make no difference to the settle behaviour below -- the
    real cause was elsewhere -- but the toggle stays available.)

    disable_dynamic_calibration: see the module docstring's "STALE READS
    AND ONBOARD DYNAMIC CALIBRATION" section -- sends the sensor hub an
    SH2 command to turn off its continuous background bias recalibration,
    confirmed on hardware to fix the multi-second post-slew settle. Set to
    False to restore the sensor's default (enabled) behaviour.

    disable_dynamic_calibration_warmup_s: 0.0 (the default) disables
    dynamic calibration immediately, before `enable_feature()`, matching
    what was validated on hardware. A nonzero value instead enables
    features FIRST, waits this many seconds with dynamic calibration still
    running, THEN disables it -- worth trying if check_overshoot_recovery.py
    reports a larger steady drift rate than you'd like: dynamic calibration
    disabled at the instant of power-up freezes whatever (possibly poor,
    unconverged) bias estimate the chip started with, rather than resetting
    it to zero, so letting it run and converge for a while first may give a
    materially better frozen estimate. Ignored if
    `disable_dynamic_calibration` is False.

    disable_dcd_autosave: see the module docstring's "PERIODIC DCD
    AUTO-SAVE" section -- sends the sensor hub an SH2 command to turn off
    its periodic automatic saving of calibration state to its own
    non-volatile flash, independent of `disable_dynamic_calibration`
    above. Defaults to True after a sensor here ended up with a bad
    calibration state permanently stuck in flash (survived power cycles,
    couldn't be reset or redone) -- almost certainly this autosave
    catching a bad frozen bias mid-save. With it off, a bad bias only
    ever lives in RAM, so a plain power cycle is always enough to clear
    it. Set to False to restore the sensor's default (autosave enabled)
    behaviour. See `clear_dcd_and_reset()` if a sensor is already stuck.

    report_interval_us: passed straight to `enable_feature()`; the
    driver's own default is 50000 (20Hz). 10000 (100Hz) matches what was
    validated on hardware alongside the two fixes above -- pass None to
    leave the driver's default in place instead.

    NOTE: on a Pi 4B/CM4 this is the path that hits the clock-stretching
    errata described in the module docstring -- if the BNO085 hangs or
    fails to respond, use `open_bno085_software_i2c()` instead.
    """
    import board  # type: ignore
    import busio  # type: ignore
    from adafruit_bno08x.i2c import BNO08X_I2C

    if i2c_bus is None:
        i2c_bus = busio.I2C(board.SCL, board.SDA, frequency=400000)

    bno = BNO08X_I2C(i2c_bus, address=address)
    _configure_and_enable(
        bno,
        use_game_rotation_vector=use_game_rotation_vector,
        disable_dynamic_calibration=disable_dynamic_calibration,
        disable_dynamic_calibration_warmup_s=disable_dynamic_calibration_warmup_s,
        disable_dcd_autosave=disable_dcd_autosave,
        report_interval_us=report_interval_us,
    )
    return bno


def open_bno085_software_i2c(
    bus_number: int,
    address: int = 0x4A,
    use_game_rotation_vector: bool = False,
    disable_dynamic_calibration: bool = True,
    disable_dynamic_calibration_warmup_s: float = 0.0,
    disable_dcd_autosave: bool = True,
    report_interval_us: Optional[int] = 10000,
):
    """Initialize a BNO085 over a kernel-level bit-banged I2C bus.

    Use this on a Pi 4B/CM4 instead of `open_bno085_i2c()` to work around
    the BCM2711 clock-stretching errata (see module docstring). Requires
    a `dtoverlay=i2c-gpio,bus=<bus_number>,...` line in config.txt (a
    reboot after adding it), plus:

        pip install adafruit-extended-bus

    `bus_number` must match the `bus=` value you gave the overlay (i.e.
    this opens `/dev/i2c-<bus_number>`). Like `open_bno085_i2c()`, this
    only returns once the sensor has produced its first valid reading.

    use_game_rotation_vector, disable_dynamic_calibration,
    disable_dynamic_calibration_warmup_s, disable_dcd_autosave,
    report_interval_us: see `open_bno085_i2c()`'s docstring and this
    module's "ROTATION VECTOR vs GAME ROTATION VECTOR" / "STALE READS AND
    ONBOARD DYNAMIC CALIBRATION" sections.
    """
    from adafruit_extended_bus import ExtendedI2C
    from adafruit_bno08x.i2c import BNO08X_I2C

    i2c_bus = ExtendedI2C(bus_number)
    bno = BNO08X_I2C(i2c_bus, address=address)
    _configure_and_enable(
        bno,
        use_game_rotation_vector=use_game_rotation_vector,
        disable_dynamic_calibration=disable_dynamic_calibration,
        disable_dynamic_calibration_warmup_s=disable_dynamic_calibration_warmup_s,
        disable_dcd_autosave=disable_dcd_autosave,
        report_interval_us=report_interval_us,
    )
    return bno


def clear_dcd_and_reset(
    bno,
    use_game_rotation_vector: bool = False,
    disable_dynamic_calibration: bool = True,
    disable_dynamic_calibration_warmup_s: float = 0.0,
    disable_dcd_autosave: bool = True,
    report_interval_us: Optional[int] = 10000,
    reset_settle_s: float = 1.0,
) -> None:
    """RECOVERY TOOL -- see the module docstring's "CLEAR DCD AND RESET"
    section. Wipes whatever calibration state the BNO085 has saved to its
    own non-volatile flash and reboots the sensor hub back to
    factory-default calibration, then re-applies the settings you pass in
    (same meanings as `open_bno085_i2c()`'s parameters) so the same `bno`
    object is immediately usable again -- no need to reconnect.

    Call this on an already-open `bno` (from either `open_bno085_i2c()`
    or `open_bno085_software_i2c()`) if a sensor's readings look
    permanently wrong, or keep drifting in a way that doesn't clear even
    after a real power cycle -- the likely cause is a bad calibration
    state that got auto-saved to the chip's own flash before
    `disable_dcd_autosave` (now on by default) was in place to prevent
    that. NOT yet validated on real hardware here (this package's other
    fixes were all confirmed on-device; this one hasn't been exercised
    yet) -- try it, but if the same `bno` object seems confused
    afterward (hangs, or `_wait_for_valid_quaternion` times out even past
    `reset_settle_s`), the reliable fallback is a real power cycle
    followed by a fresh `open_bno085_i2c()`/`open_bno085_software_i2c()`
    call instead -- which now also disables DCD auto-save by default, so
    a sensor shouldn't be able to get stuck this way again regardless.

    reset_settle_s: how long to wait after sending the reset command
    before re-configuring -- the sensor hub reboots itself, so this needs
    to be long enough for it to come back up and start responding again
    before `_configure_and_enable()` tries to talk to it. 1.0s is a
    starting guess, not a validated figure; increase it if reconfiguration
    fails right after a reset.
    """
    print("  [bno085_interface] sending Clear DCD and Reset -- sensor will reboot...")
    _send_clear_dcd_and_reset(bno)
    time.sleep(reset_settle_s)
    _configure_and_enable(
        bno,
        use_game_rotation_vector=use_game_rotation_vector,
        disable_dynamic_calibration=disable_dynamic_calibration,
        disable_dynamic_calibration_warmup_s=disable_dynamic_calibration_warmup_s,
        disable_dcd_autosave=disable_dcd_autosave,
        report_interval_us=report_interval_us,
    )
    print("  [bno085_interface] Clear DCD and Reset complete -- sensor re-configured and producing reports again.")


def read_imu_sample(bno, accuracy_rad: Optional[float] = None) -> IMUSample:
    """Take one IMUSample reading from an already-initialized BNO08X driver.

    Drains any buffered backlog first (see `_read_latest_quaternion()` and
    the module docstring's "STALE READS..." section) so the value read is
    the sensor's freshest report, then reads `.game_quaternion` or
    `.quaternion` depending on which one the `open_bno085_*()` call that
    produced `bno` enabled (see `use_game_rotation_vector` there) --
    callers don't need to know or care which mode is active. Both
    properties return (quat_i, quat_j, quat_k, quat_real), i.e.
    (x, y, z, w) scalar-last -- exactly the order IMUSample expects, and
    confirmed identical in the driver's own source (both are unpacked by
    the same underlying parser, just from different report IDs).
    """
    use_grv = getattr(bno, "_telescope_pointing_use_game_rotation_vector", False)
    x, y, z, w = _read_latest_quaternion(bno, use_grv)
    return IMUSample(x=x, y=y, z=z, w=w, timestamp=datetime.now(timezone.utc), accuracy_rad=accuracy_rad)


def imu_sample_stream(bno, min_period_s: float = 0.0) -> Iterator[IMUSample]:
    """Yield IMUSample readings forever (Ctrl-C to stop).

    `min_period_s` can be used to cap the poll rate if your control loop
    doesn't need every report the sensor produces.
    """
    last = 0.0
    while True:
        now = time.monotonic()
        if now - last < min_period_s:
            time.sleep(min_period_s - (now - last))
        last = time.monotonic()
        yield read_imu_sample(bno)


if __name__ == "__main__":
    # Minimal smoke test on real hardware: print 20 readings.
    # On a Pi 4B/CM4, swap this for:
    #     bno = open_bno085_software_i2c(bus_number=8)   # match your dtoverlay bus=
    bno = open_bno085_i2c()
    for i, sample in enumerate(imu_sample_stream(bno, min_period_s=0.1)):
        print(sample)
        if i >= 19:
            break
