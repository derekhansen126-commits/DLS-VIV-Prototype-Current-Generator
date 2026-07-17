#This is some older code that Raven worked on before, it is included as an example to show that the original code works.

#This data logger controls two moteus motors
#both motors fire up, M2 shifts after 5 seconds
#script allows for kd and/or velocity shift
import asyncio
import csv
import math
import time
import moteus
from moteus import get_singleton_transport, Register

# ~40 Hz logging
SAMPLE_PERIOD_S = 0.025


def rget(values: dict, reg_or_int, default=0.0):
    """Get a value whether keys are enums or ints (enum.value)."""
    if isinstance(reg_or_int, int) and reg_or_int in values:
        return values[reg_or_int]
    if reg_or_int in values:
        return values[reg_or_int]
    try:
        v = getattr(reg_or_int, "value", None)
        if v is not None and v in values:
            return values[v]
    except Exception:
        pass
    return default


def make_filename(prefix: str, velocity_rps: float, kd_value: float, ext: str = "csv") -> str:
    """Create unique filenames"""
    ts = time.strftime("%Y%m%d-%H%M%S")
    return f"{prefix}_vel{velocity_rps:.2f}_kd{kd_value:.4f}_{ts}.{ext}"


class CsvController(moteus.Controller):
    """
    Logs position, velocity, torque, device-reported electrical power,
    and computed mechanical power P_MECH = tau * (2*pi*rps).
    """

    def __init__(self, filename, *args, relative_time=True, **kwargs):
        super().__init__(*args, **kwargs)

        self.fd = open(filename, "w", newline="")
        self.writer = csv.writer(self.fd)
        self.relative_time = relative_time
        self._t0 = time.time()

        # CSV header
        header = ["time", "POSITION", "VELOCITY", "TORQUE", "P_ELEC", "P_MECH"]
        self.writer.writerow(header)

        self.POWER_REG = getattr(Register, "POWER", 0x007)

    async def execute(self, command):
        try:
            result = await asyncio.wait_for(super().execute(command), timeout=1.0)
        except asyncio.TimeoutError:
            print(f"Timeout waiting for response from ID={self.id}")
            return None

        if result is not None:
            t = (time.time() - self._t0) if self.relative_time else time.time()
            pos = rget(result.values, Register.POSITION)
            vel = rget(result.values, Register.VELOCITY)   # rps
            tau = rget(result.values, Register.TORQUE)     # N·m
            pelec = rget(result.values, self.POWER_REG)    # W
            pmech = tau * (vel * 2.0 * math.pi)            # W
            self.writer.writerow([t, pos, vel, tau, pelec, pmech])
        return result

    def __enter__(self):
        self.fd.__enter__()
        return self

    def __exit__(self, et, ev, tb):
        return self.fd.__exit__(et, ev, tb)


async def run_motor_velocity(
    controller: CsvController,
    duration_s: float,
    velocity_rps: float,
    max_torque: float = 1.0,
):
    """Active velocity mode with a single constant velocity (no kd shift)."""
    start = time.time()
    while time.time() - start < duration_s:
        await controller.set_position(
            position=math.nan,
            velocity=velocity_rps,
            maximum_torque=max_torque,
            query=True,
        )
        await asyncio.sleep(SAMPLE_PERIOD_S)
    await controller.set_stop()


async def run_motor_velocity_with_kd_and_vel_shift(
    controller: CsvController,
    total_duration_s: float,
    vel_before: float,
    vel_after: float,
    shift_time_s: float = 5.0,
    kd_before: float = 1.0,
    kd_after: float = 0.1,
    max_torque: float = 1.0,
):
    """
    M2 behavior:
      - [0, shift_time_s): velocity = vel_before, kd_scale = kd_before
      - [shift_time_s, end): velocity = vel_after, kd_scale = kd_after
    """
    start = time.time()
    announced = False

    while True:
        elapsed = time.time() - start
        if elapsed >= total_duration_s:
            break

        if elapsed < shift_time_s:
            target_vel = vel_before
            kd_scale = kd_before
        else:
            target_vel = vel_after
            kd_scale = kd_after
            if not announced:
                print(
                    f"[{time.strftime('%H:%M:%S')}] "
                    f"Motor {controller.id} velocity shifted from {vel_before:.2f} rps "
                    f"to {vel_after:.2f} rps and kd_scale from {kd_before:.2f} to "
                    f"{kd_after:.2f} at t={elapsed:.2f}s"
                )
                announced = True

        await controller.set_position(
            position=math.nan,
            velocity=target_vel,
            kd_scale=kd_scale,
            maximum_torque=max_torque,
            query=True,
        )
        await asyncio.sleep(SAMPLE_PERIOD_S)

    await controller.set_stop()


async def main():
    # Ask moteus to include POWER (and a few basics) in every default reply
    qr = moteus.QueryResolution()
    qr.power = moteus.F32
    qr.position = moteus.F32
    qr.velocity = moteus.F32
    qr.torque = moteus.F32

    transport = get_singleton_transport()

    # parameter controls
    duration = 10.0          # total runtime (s)

    # M1: constant velocity, no kd shift
    m1_vel = 3.0             # rps for motor 1

    # M2: starts matching M1, then both kd & vel shift
    m2_vel_before = m1_vel   # matches M1 initially
    m2_vel_after = 3.4       # change per trial
    shift_time = 5.0         # seconds until velocity & kd change on M2

    kd_before = 1.0          # kd_scale before shift (M2)
    kd_after = 0.1           # kd_scale after shift (M2)

    # KD tags for filenames
    kd_m1_for_filename = 1.00
    kd_m2_for_filename = kd_after
    # ----------------------------------------

    # For filenames, tag:
    # - M1 velocity and kd
    # - M2 *post-shift* velocity and kd_after
    out1 = make_filename(
        "motor_1",
        velocity_rps=m1_vel,
        kd_value=kd_m1_for_filename,
        ext="csv",
    )
    out2 = make_filename(
        "35g_motor_2",
        velocity_rps=m2_vel_after,
        kd_value=kd_m2_for_filename,
        ext="csv",
    )

    print(f"Logging to:\n  {out1}\n  {out2}")

    with CsvController(out1, id=1, transport=transport, query_resolution=qr) as m1, \
         CsvController(out2, id=2, transport=transport, query_resolution=qr) as m2:

        # Clear faults
        await asyncio.gather(m1.set_stop(), m2.set_stop())

        await asyncio.gather(
            run_motor_velocity(
                m1,
                duration_s=duration,
                velocity_rps=m1_vel,
                max_torque=1.0,
            ),
            run_motor_velocity_with_kd_and_vel_shift(
                m2,
                total_duration_s=duration,
                vel_before=m2_vel_before,
                vel_after=m2_vel_after,
                shift_time_s=shift_time,
                kd_before=kd_before,
                kd_after=kd_after,
                max_torque=1.0,
            ),
        )


if __name__ == "__main__":
    asyncio.run(main())
