"""
payload_chassis.py — Fusion 360 API script (debug version)

Generates a parametric payload chassis. Each step is wrapped in its own
try/except so if anything fails, the message box tells you exactly which
step broke and shows the full Python traceback.
"""

import adsk.core
import adsk.fusion
import traceback

# =============================================================================
# DIMENSIONS — EDIT HERE (millimetres)
# =============================================================================
CHASSIS_L = 130
CHASSIS_W = 90
CHASSIS_H = 50
WALL      = 2.5

RPI_HOLE_X_SPACING = 58
RPI_HOLE_Y_SPACING = 49
RPI_HOLE_DIA       = 2.7
STANDOFF_DIA       = 5.5
STANDOFF_HEIGHT    = 8

WHEEL_AXLE_DIA        = 4
WHEEL_AXLE_FROM_REAR  = 32
WHEEL_AXLE_FROM_FLOOR = 11

CAM_CUTOUT_W = 18
CAM_CUTOUT_H = 18
CAM_CUTOUT_Z = 28

CABLE_HOLE_DIA      = 8
CABLE_HOLE_Z        = 32
CABLE_HOLE_Y_OFFSET = 18

CASTER_HOLE_DIA       = 2.5
CASTER_HOLE_X_SPACING = 14
CASTER_HOLE_Y_SPACING = 14
CASTER_FROM_FRONT     = 18

MM = 0.1

# Step results so the failure point is obvious
results = []

def vi(mm):
    return adsk.core.ValueInput.createByReal(mm * MM)

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
    pi.setByOffset(base_plane, vi(offset_mm))
    return rc.constructionPlanes.add(pi)

def step(name, fn):
    try:
        fn()
        results.append(f"✓ {name}")
    except Exception:
        results.append(f"✗ {name}\n{traceback.format_exc()}")
        raise


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('Open a Fusion design first (File → New Design)')
            return

        rc  = design.rootComponent
        ext = rc.features.extrudeFeatures
        FOP = adsk.fusion.FeatureOperations

        body_holder = {}

        def step1_box():
            sk = rc.sketches.add(rc.xYConstructionPlane)
            add_rect(sk, -CHASSIS_L/2, -CHASSIS_W/2, CHASSIS_L/2, CHASSIS_W/2)
            ei = ext.createInput(sk.profiles.item(0), FOP.NewBodyFeatureOperation)
            ei.setDistanceExtent(False, vi(CHASSIS_H))
            f = ext.add(ei)
            body_holder['body'] = f.bodies.item(0)

        def step2_shell():
            body = body_holder['body']
            topFace = None
            for f in body.faces:
                if f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                    if abs(f.centroid.z - CHASSIS_H * MM) < 0.01:
                        topFace = f
                        break
            if topFace is None:
                raise RuntimeError("could not locate top face for shell")
            faces = adsk.core.ObjectCollection.create()
            faces.add(topFace)
            si = rc.features.shellFeatures.createInput(faces, False)
            si.insideThickness = vi(WALL)
            rc.features.shellFeatures.add(si)

        def step3_axle_holes():
            sk = rc.sketches.add(rc.xZConstructionPlane)
            add_circle(sk,
                       -CHASSIS_L/2 + WHEEL_AXLE_FROM_REAR,
                       WHEEL_AXLE_FROM_FLOOR,
                       WHEEL_AXLE_DIA)
            ei = ext.createInput(sk.profiles.item(0), FOP.CutFeatureOperation)
            ei.participantBodies = [body_holder['body']]
            ei.setDistanceExtent(True, vi(CHASSIS_W/2 + 2))
            ext.add(ei)

        def step4_camera():
            camPlane = offset_plane(rc, rc.yZConstructionPlane, CHASSIS_L/2)
            sk = rc.sketches.add(camPlane)
            add_rect(sk,
                     -CAM_CUTOUT_W/2, CAM_CUTOUT_Z - CAM_CUTOUT_H/2,
                      CAM_CUTOUT_W/2, CAM_CUTOUT_Z + CAM_CUTOUT_H/2)
            ei = ext.createInput(sk.profiles.item(0), FOP.CutFeatureOperation)
            ei.participantBodies = [body_holder['body']]
            ei.setDistanceExtent(False, vi(-(WALL + 1)))
            ext.add(ei)

        def step5_cables():
            rearPlane = offset_plane(rc, rc.yZConstructionPlane, -CHASSIS_L/2)
            sk = rc.sketches.add(rearPlane)
            for sign in (-1, 1):
                add_circle(sk, sign * CABLE_HOLE_Y_OFFSET, CABLE_HOLE_Z, CABLE_HOLE_DIA)
            ei = ext.createInput(all_profiles(sk), FOP.CutFeatureOperation)
            ei.participantBodies = [body_holder['body']]
            ei.setDistanceExtent(False, vi(WALL + 1))
            ext.add(ei)

        def step6_standoffs():
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
            ei.participantBodies = [body_holder['body']]
            ei.setDistanceExtent(False, vi(WALL + STANDOFF_HEIGHT))
            ext.add(ei)

            sk2 = rc.sketches.add(rc.xYConstructionPlane)
            for (x, y) in rpi_pos:
                add_circle(sk2, x, y, RPI_HOLE_DIA)
            ei2 = ext.createInput(all_profiles(sk2), FOP.CutFeatureOperation)
            ei2.participantBodies = [body_holder['body']]
            ei2.setDistanceExtent(False, vi(WALL + STANDOFF_HEIGHT + 1))
            ext.add(ei2)

        def step7_caster():
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
            ei.participantBodies = [body_holder['body']]
            ei.setDistanceExtent(False, vi(WALL + 1))
            ext.add(ei)

        # --- run all steps ---
        try:
            step("1: chassis box",         step1_box)
            step("2: shell (hollow)",      step2_shell)
            step("3: wheel axle holes",    step3_axle_holes)
            step("4: camera cutout",       step4_camera)
            step("5: cable pass-throughs", step5_cables)
            step("6: RPi standoffs",       step6_standoffs)
            step("7: caster mount holes",  step7_caster)
            ui.messageBox('All steps OK!\n\n' + '\n'.join(results))
        except Exception:
            ui.messageBox('Stopped at first failure:\n\n' + '\n'.join(results))

    except:
        if ui:
            ui.messageBox('Failed (top-level):\n{}'.format(traceback.format_exc()))
