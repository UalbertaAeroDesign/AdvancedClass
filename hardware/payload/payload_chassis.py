"""
payload_chassis.py — Fusion 360 API script

Generates a parametric 3D-printable chassis for the SAE Aero Design 2026
autonomous payload rover. All dimensions are in millimetres and live at
the top of this file — edit them and re-run to regenerate.

Components the chassis is sized for:
  - Raspberry Pi 5 (85 × 56, M2.5 holes at 58 × 49 mm)
  - 2× Pololu #5137 micro metal gearmotor with encoder (~10.5 × 16 × 26 mm body
    + 9 mm output shaft) — mounted internally with axles through side walls
  - 2× Pololu #1087 32 mm wheels — exterior, on the motor shafts
  - 1× Pololu #951 ball caster — front, mounts to floor
  - Pico H, DRV8835, RF receiver, LM2596 — sit inside on standoffs
  - Camera Module 3 — looks out the front wall

What the script generates:
  ✓ Hollow rectangular shell with open top
  ✓ Wheel axle holes through both side walls
  ✓ Camera cutout in front wall
  ✓ Cable pass-throughs in rear wall
  ✓ Four RPi mounting standoffs with M2.5 through-holes
  ✓ Ball caster mounting holes in floor

What you'll add manually after running this:
  ✗ Motor bracket screw holes (depends on Pololu #989 footprint — measure yours)
  ✗ Lid (flat plate, sketch and extrude — 5 minutes)
  ✗ Internal mounts for Pico / DRV8835 / RF / LM2596 (depends on layout)

How to run:
  Fusion 360  →  Utilities  →  Add-Ins  →  Scripts and Add-Ins…
  Scripts tab  →  '+'  →  New Script  →  paste this code  →  Run

After running once, use Fusion's timeline to undo before re-running.
"""

import adsk.core
import adsk.fusion
import traceback

# =============================================================================
# DIMENSIONS — EDIT HERE
# =============================================================================

# ---- Chassis outer ----
CHASSIS_L = 130    # length (front-to-back)
CHASSIS_W = 90     # width (side-to-side)
CHASSIS_H = 50     # total height
WALL      = 2.5    # wall thickness

# ---- Raspberry Pi 5 mounting (centred in chassis) ----
RPI_HOLE_X_SPACING = 58       # board hole spacing along length
RPI_HOLE_Y_SPACING = 49       # board hole spacing across width
RPI_HOLE_DIA       = 2.7      # M2.5 screw clearance
STANDOFF_DIA       = 5.5      # standoff outer diameter
STANDOFF_HEIGHT    = 8        # raises board above floor for clearance

# ---- Wheel axle holes (motor mounted internally, axle protrudes outward) ----
WHEEL_AXLE_DIA         = 4    # 3 mm shaft + 1 mm clearance
WHEEL_AXLE_FROM_REAR   = 32   # X-distance from back wall
WHEEL_AXLE_FROM_FLOOR  = 11   # Z-height above outer chassis bottom
                              # (= wheel_radius − ground_clearance; here 16−5)

# ---- Camera cutout (front wall) ----
CAM_CUTOUT_W = 18
CAM_CUTOUT_H = 18
CAM_CUTOUT_Z = 28             # centre height above outer chassis bottom

# ---- Cable pass-throughs (×2 in rear wall) ----
CABLE_HOLE_DIA      = 8
CABLE_HOLE_Z        = 32      # centre height above outer chassis bottom
CABLE_HOLE_Y_OFFSET = 18      # ±distance from chassis centreline

# ---- Ball caster mount (4-hole pattern in floor near front) ----
CASTER_HOLE_DIA       = 2.5
CASTER_HOLE_X_SPACING = 14
CASTER_HOLE_Y_SPACING = 14
CASTER_FROM_FRONT     = 18    # distance from front wall to caster centre

MM = 0.1   # Fusion API uses centimetres internally — 1 mm = 0.1 cm


# =============================================================================
# HELPERS
# =============================================================================
def add_rect(sketch, x1, y1, x2, y2):
    sketch.sketchCurves.sketchLines.addTwoPointRectangle(
        adsk.core.Point3D.create(x1 * MM, y1 * MM, 0),
        adsk.core.Point3D.create(x2 * MM, y2 * MM, 0))

def add_circle(sketch, x, y, dia):
    sketch.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(x * MM, y * MM, 0), dia / 2 * MM)

def all_profiles(sketch):
    coll = adsk.core.ObjectCollection.create()
    for i in range(sketch.profiles.count):
        coll.add(sketch.profiles.item(i))
    return coll

def offset_plane(rc, base_plane, offset_mm):
    pi = rc.constructionPlanes.createInput()
    pi.setByOffset(base_plane, adsk.core.ValueInput.createByReal(offset_mm * MM))
    return rc.constructionPlanes.add(pi)


# =============================================================================
# MAIN
# =============================================================================
def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('Open a Fusion design first (File → New Design)')
            return

        rc = design.rootComponent
        ext = rc.features.extrudeFeatures
        FOP = adsk.fusion.FeatureOperations

        # ---- 1. Solid chassis box ----
        sk = rc.sketches.add(rc.xYConstructionPlane)
        add_rect(sk, -CHASSIS_L/2, -CHASSIS_W/2, CHASSIS_L/2, CHASSIS_W/2)
        ei = ext.createInput(sk.profiles.item(0), FOP.NewBodyFeatureOperation)
        ei.setDistanceExtent(False, adsk.core.ValueInput.createByReal(CHASSIS_H * MM))
        ext.add(ei)
        body = rc.bRepBodies.item(0)

        # ---- 2. Shell — hollow it out, top face open ----
        topFace = None
        for f in body.faces:
            if (f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType
                    and abs(f.centroid.z - CHASSIS_H * MM) < 0.001):
                topFace = f
                break
        if topFace:
            faces = adsk.core.ObjectCollection.create()
            faces.add(topFace)
            si = rc.features.shellFeatures.createInput(faces, False)
            si.insideThickness = adsk.core.ValueInput.createByReal(WALL * MM)
            rc.features.shellFeatures.add(si)

        # ---- 3. Wheel axle holes (cut through both side walls) ----
        sk = rc.sketches.add(rc.xZConstructionPlane)
        add_circle(sk,
                   -CHASSIS_L/2 + WHEEL_AXLE_FROM_REAR,
                   WHEEL_AXLE_FROM_FLOOR,
                   WHEEL_AXLE_DIA)
        ei = ext.createInput(sk.profiles.item(0), FOP.CutFeatureOperation)
        # Symmetric extent (each-side mode): cut (CHASSIS_W/2 + margin) in ±Y
        ei.setSymmetricExtent(
            adsk.core.ValueInput.createByReal((CHASSIS_W/2 + 2) * MM), False)
        ext.add(ei)

        # ---- 4. Camera cutout in front wall (+X side) ----
        camPlane = offset_plane(rc, rc.yZConstructionPlane, CHASSIS_L/2)
        sk = rc.sketches.add(camPlane)
        # On YZ-parallel plane: sketch X = world Y, sketch Y = world Z
        add_rect(sk,
                 -CAM_CUTOUT_W/2, CAM_CUTOUT_Z - CAM_CUTOUT_H/2,
                  CAM_CUTOUT_W/2, CAM_CUTOUT_Z + CAM_CUTOUT_H/2)
        ei = ext.createInput(sk.profiles.item(0), FOP.CutFeatureOperation)
        # Negative direction = into chassis (−X from the offset plane)
        ei.setDistanceExtent(False,
            adsk.core.ValueInput.createByReal(-(WALL + 1) * MM))
        ext.add(ei)

        # ---- 5. Cable pass-throughs in rear wall (−X side) ----
        rearPlane = offset_plane(rc, rc.yZConstructionPlane, -CHASSIS_L/2)
        sk = rc.sketches.add(rearPlane)
        for sign in (-1, 1):
            add_circle(sk, sign * CABLE_HOLE_Y_OFFSET, CABLE_HOLE_Z, CABLE_HOLE_DIA)
        ei = ext.createInput(all_profiles(sk), FOP.CutFeatureOperation)
        ei.setDistanceExtent(False,
            adsk.core.ValueInput.createByReal((WALL + 1) * MM))
        ext.add(ei)

        # ---- 6. RPi standoffs: 4 cylinders joined to floor ----
        rpi_pos = [
            ( RPI_HOLE_X_SPACING/2,  RPI_HOLE_Y_SPACING/2),
            (-RPI_HOLE_X_SPACING/2,  RPI_HOLE_Y_SPACING/2),
            ( RPI_HOLE_X_SPACING/2, -RPI_HOLE_Y_SPACING/2),
            (-RPI_HOLE_X_SPACING/2, -RPI_HOLE_Y_SPACING/2),
        ]
        sk = rc.sketches.add(rc.xYConstructionPlane)
        for (x, y) in rpi_pos:
            add_circle(sk, x, y, STANDOFF_DIA)
        ei = ext.createInput(all_profiles(sk), FOP.JoinFeatureOperation)
        ei.setDistanceExtent(False,
            adsk.core.ValueInput.createByReal((WALL + STANDOFF_HEIGHT) * MM))
        ext.add(ei)

        # ---- 7. M2.5 holes through standoffs and floor ----
        sk = rc.sketches.add(rc.xYConstructionPlane)
        for (x, y) in rpi_pos:
            add_circle(sk, x, y, RPI_HOLE_DIA)
        ei = ext.createInput(all_profiles(sk), FOP.CutFeatureOperation)
        ei.setDistanceExtent(False,
            adsk.core.ValueInput.createByReal((WALL + STANDOFF_HEIGHT + 1) * MM))
        ext.add(ei)

        # ---- 8. Ball caster mounting holes (4 holes in front of floor) ----
        caster_x = CHASSIS_L/2 - CASTER_FROM_FRONT
        caster_pos = [
            (caster_x + CASTER_HOLE_X_SPACING/2,  CASTER_HOLE_Y_SPACING/2),
            (caster_x - CASTER_HOLE_X_SPACING/2,  CASTER_HOLE_Y_SPACING/2),
            (caster_x + CASTER_HOLE_X_SPACING/2, -CASTER_HOLE_Y_SPACING/2),
            (caster_x - CASTER_HOLE_X_SPACING/2, -CASTER_HOLE_Y_SPACING/2),
        ]
        sk = rc.sketches.add(rc.xYConstructionPlane)
        for (x, y) in caster_pos:
            add_circle(sk, x, y, CASTER_HOLE_DIA)
        ei = ext.createInput(all_profiles(sk), FOP.CutFeatureOperation)
        ei.setDistanceExtent(False,
            adsk.core.ValueInput.createByReal((WALL + 1) * MM))
        ext.add(ei)

        ui.messageBox(
            'Payload chassis generated.\n\n'
            '• Edit dimensions at the top of the script and re-run\n'
            '  to regenerate (use Fusion timeline to undo first).\n'
            '• Add a lid as a separate body when ready.\n'
            '• Measure your motor brackets and add their mounting\n'
            '  holes manually before printing.')
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
