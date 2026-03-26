# VTOL Test Flight Scripts

Two-stage VTOL flight test for a quadplane. Demonstrates a full mission cycle:
takeoff in fixed-wing, transition to multicopter for a precision VTOL landing,
then re-takeoff and return to the runway for a fixed-wing landing.

## Scripts

- **vtol_stage1.py** — Arms in TAKEOFF mode, climbs to cruise altitude in FW,
  flies a short cruise leg, transitions to MC, and performs a VTOL land.
- **vtol_stage2.py** — Re-arms at the stage 1 landing site, VTOL takeoffs in
  GUIDED mode, transitions to FW, flies to the far end of the runway, and
  performs a fixed-wing runway landing back at home.
- **full_vtol.py** — Runs both stages sequentially.

Run stage 1 first. After it lands and disarms, run stage 2.

## Running (SITL)

Start the simulator in one terminal:

```
sim_vehicle.py -v ArduPlane -f quadplane --console --map --out=127.0.0.1:14550 --out=127.0.0.1:14551 -w --custom-location=32.609354,-97.484479,216,0
```

Then run the flight script in a second terminal:

```
python vtol_stage1.py --connect udpin:0.0.0.0:14551
```
