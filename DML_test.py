#!/usr/bin/env python3
import asyncio
import csv
import math
import time
import moteus
from moteus import get_singleton_transport, Register

SAMPLE_PERIOD_S = 0.025  # ~40 Hz logging

class CsvController(moteus.Controller):
    """Logs POSITION, VELOCITY, TORQUE plus computed P_MECH."""
    def __init__(self, filename, *args, relative_time=True, **kwargs):
        super().__init__(*args, **kwargs)
        # Stick to fields that are actually returned by the default query
        self.fields = [Register.POSITION, Register.VELOCITY, Register.TORQUE]
        self.fd = open(filename, "w", newline="")
        self.writer = csv.writer(self.fd)
        self.relative_time = relative_time
        self._t0 = time.time()

        header = ["time"] + [Register(x).name for x in self.fields] +["P_MECH"]
        self.writer.writerow(header)

    async def execute(self, command):
        try:
            result = await asyncio.wait_for(super().execute(command), timeout=1.0)
        except asyncio.TimeoutError:
            print(f"Timeout waiting for response from ID={self.id}")
            return None

        if result is not None:
            t = (time.time() - self._t0) if self.relative_time else time.time()
            pos   = result.values.get(Register.POSITION, 0.0)
            vel   = result.values.get(Register.VELOCITY, 0.0)      # rps
            tau   = result.values.get(Register.TORQUE, 0.0)        # N·m
            #iq   = result.values.get(Register.Q_CURRENT, 0.0)
            #vbus = result.values.get(Register.VFOC_VOLTAGE, 0.0)
            #p_elec = vbus * iq                  # electrical power (approx)
            p_mech = tau * (vel * 2.0*math.pi)  # mechanical power

            self.writer.writerow([t, pos, vel, tau, p_mech])
        return result

    def __enter__(self):
        self.fd.__enter__()
        return self

    def __exit__(self, et, ev, tb):
        return self.fd.__exit__(et, ev, tb)

async def run_motor_velocity_profile(controller, total_duration_s: float, max_torque: float = 1.0):
    """
    Keeps the motor actively controlled for total_duration_s, with a timed velocity:
      0–5 s: freewheel (velocity = nan, P/I killed)
      5–15 s: 3 rps
      15–20 s: freewheel again
    """
    start = time.time()
    while True:
        elapsed = time.time() - start
        if elapsed >= total_duration_s:
            break

        if elapsed < 5.0 or elapsed >= 15.0:
            # freewheel (regen-friendly style)
            await controller.set_position(
                position=math.nan,
                velocity=math.nan,       # no velocity target
                kp_scale=0.0,            # disable P
                ilimit_scale=0.0,        # disable I
                query=controller.query,
            )
        else:
            # actively drive at 3 rps
            await controller.set_position(
                position=math.nan,
                velocity=3.0,
                maximum_torque=max_torque,
                query=controller.query,
            )

        await asyncio.sleep(SAMPLE_PERIOD_S)

    await controller.set_stop()

async def run_motor_velocity(controller: CsvController, duration_s: float, velocity_rps: float, max_torque: float = 1.0):
    """Active velocity mode for Motor 1."""
    start = time.time()
    while time.time() - start < duration_s:
        await controller.set_position(
            position=math.nan,
            velocity=velocity_rps,
            maximum_torque=max_torque,
            query=True,  # default query (works on older moteus)
        )
        await asyncio.sleep(SAMPLE_PERIOD_S)
    await controller.set_stop()

async def regen_friendly_freewheel(controller: CsvController, duration_s: float):
    """Motor 2: no P/I, no targets; allow regen by NOT clamping maximum_torque."""
    start = time.time()
    while time.time() - start < duration_s:
        await controller.set_position(
            position=math.nan,
            velocity=math.nan,   # avoid 0-rps hold
            kp_scale=0.0,        # kill P
            ilimit_scale=0.0,    # kill I
            # (optional) kd_scale=0.0,  # uncomment to test pure freewheel (no damping)
            query=True,          # default query
        )
        await asyncio.sleep(SAMPLE_PERIOD_S)
    await controller.set_stop()

async def main():
    transport = get_singleton_transport()
    with CsvController("motor_1_log.csv", id=1, transport=transport) as m1, \
         CsvController("motor_2_log.csv", id=2, transport=transport) as m2:

        await asyncio.gather(m1.set_stop(), m2.set_stop())

        await asyncio.gather(
            run_motor_velocity(m1, duration_s=20.0, velocity_rps=3.0, max_torque=1.0),
            run_motor_velocity_profile(m2, total_duration_s=20.0, max_torque=1.0),
        )

if __name__ == "__main__":
    asyncio.run(main())
