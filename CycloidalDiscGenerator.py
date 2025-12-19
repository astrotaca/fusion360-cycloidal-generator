# Cycloidal Dual-Disc Sketch Generator (Fusion 360)
# - Robust “no-overlap” default (Fusion-friendly)
# - Optional “Exact geometry” mode (can touch / be tangent)
# Supports dual-disc reducers with correct phase and aligned output holes
#
# Notes:
# - In VS Code, Pylance will complain that adsk.* cannot be resolved. That’s normal outside Fusion.
# - Run inside Fusion 360 Scripts/Add-Ins.

import math
import time
import traceback
import adsk.core
import adsk.fusion

_HANDLERS = []

CMD_ID = 'CycloDualDiscGen'
CMD_NAME = 'Cycloidal Dual-Disc Sketch Generator'
CMD_DESC = 'Creates cycloidal reducer sketches using trochoidal parallel-curve geometry.'

# Units helpers
def _mm_to_cm(v): return v / 10.0
def _cm_to_mm(v): return v * 10.0

def _p3(x_mm, y_mm):
    return adsk.core.Point3D.create(_mm_to_cm(x_mm), _mm_to_cm(y_mm), 0)

# Geometry helpers
def _rot_xy(x, y, ang_rad):
    ca, sa = math.cos(ang_rad), math.sin(ang_rad)
    return (x * ca - y * sa, x * sa + y * ca)

def _add_circle(sk, cx_mm, cy_mm, r_mm):
    return sk.sketchCurves.sketchCircles.addByCenterRadius(_p3(cx_mm, cy_mm), _mm_to_cm(r_mm))

def _add_spline(sk, pts_mm, close=True, fix=True):
    """
    Draws a single fitted spline through all points.
    Much lighter for Fusion than polylines.
    """
    sk.isComputeDeferred = True
    try:
        coll = adsk.core.ObjectCollection.create()
        for (x, y) in pts_mm:
            coll.add(_p3(x, y))

        spline = sk.sketchCurves.sketchFittedSplines.add(coll)

        if close:
            spline.isClosed = True
        if fix:
            spline.isFixed = True

        return spline
    finally:
        sk.isComputeDeferred = False

# Validation
def _validate(N_ring, ring_pcd_mm, pin_diam_mm, ecc_mm, clearance_mm):
    if N_ring < 3:
        return False, "Ring pin count must be >= 3."
    if ring_pcd_mm <= 0 or pin_diam_mm <= 0:
        return False, "Ring PCD and pin diameter must be > 0."
    if ecc_mm <= 0:
        return False, "Eccentricity must be > 0."
    if clearance_mm < 0:
        return False, "Roller clearance must be >= 0."

    R = ring_pcd_mm / 2.0
    if ecc_mm * N_ring >= R:
        return False, f"Need E*N < R. Here E*N={ecc_mm*N_ring:.3f} mm, R={R:.3f} mm."

    pitch = (2.0 * math.pi * R) / N_ring
    if pin_diam_mm >= 0.98 * pitch:
        return False, "Ring pins too large for this PCD (pitch crowding)."

    # effective roller radius shouldn't exceed ring radius
    pin_r = pin_diam_mm / 2.0
    if (pin_r + clearance_mm) >= R:
        return False, "Clearance too large vs ring radius (would be degenerate)."

    return True, ""

def _trochoid_parallel_points(N_ring, ring_pcd_mm, ecc_mm, d_eff_mm, pts_per_tooth):
    R = ring_pcd_mm / 2.0
    lobes = N_ring - 1

    # SIMPLE, UNIFORM SAMPLING (fast & predictable)
    steps = max(240, lobes * pts_per_tooth)

    pts = []

    for i in range(steps + 1):
        p = 2.0 * math.pi * i / steps

        xa = R * math.cos(p) - ecc_mm * math.cos(N_ring * p)
        ya = R * math.sin(p) - ecc_mm * math.sin(N_ring * p)

        dxa = -R * math.sin(p) + (ecc_mm * N_ring) * math.sin(N_ring * p)
        dya =  R * math.cos(p) - (ecc_mm * N_ring) * math.cos(N_ring * p)

        s = math.hypot(dxa, dya)
        if s < 1e-9:
            continue

        x = xa - d_eff_mm * (dya / s)
        y = ya + d_eff_mm * (dxa / s)

        pts.append((x, y))

    return pts


def _hole_pattern_local(n_holes, hole_pcd_mm):
    r = hole_pcd_mm / 2.0
    pts = []
    for i in range(n_holes):
        a = 2.0 * math.pi * i / n_holes
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts

def _build_disc(sk, profile_world_pts, disc_center_mm, hole_centers_local, hole_rad_mm, center_bore_rad_mm):
    # Draw cycloidal profile as ONE spline (Fusion-friendly)
    _add_spline(sk, profile_world_pts, close=True, fix=True)

    # Center bore
    _add_circle(sk, disc_center_mm[0], disc_center_mm[1], center_bore_rad_mm)

    # Output holes
    for (hx, hy) in hole_centers_local:
        _add_circle(
            sk,
            disc_center_mm[0] + hx,
            disc_center_mm[1] + hy,
            hole_rad_mm
        )


def _computed_phase_deg(N_ring):
    lobes = N_ring - 1
    return 180.0 / lobes if lobes > 0 else 0.0

def _generate_profile_no_overlap(
    N, ring_pcd, ring_pin_d, ecc, clearance_mm, pts_per_tooth
):

    pin_r = ring_pin_d / 2.0
    d_eff = pin_r + max(0.0, clearance_mm)

    base_pts = _trochoid_parallel_points(
        N, ring_pcd, ecc, d_eff, pts_per_tooth
    )

    # Old behavior: no validation, no retries
    return base_pts, 0.0, 0.0, False

# Main generation
def _generate(design, opts):
    root = design.rootComponent
    sketches = root.sketches

    N = int(opts["N_ring"])
    ring_pcd = float(opts["ring_pcd"])
    ring_pin_d = float(opts["ring_pin_d"])
    ecc = float(opts["ecc"])

    clearance = float(opts["profile_offset"])

    out_n = int(opts["out_n"])
    out_pin_d = float(opts["out_pin_d"])
    out_pcd = float(opts["out_pcd"])
    hole_extra_diam = float(opts["hole_extra_diam"])

    bore_d = float(opts["bore_d"])
    pts_per_tooth = int(opts["pts_per_tooth"])

    draw_ring = bool(opts["draw_ring"])
    draw_outpins = bool(opts["draw_outpins"])

    dual = bool(opts["dual"])
    opposed = bool(opts["opposed"])

    phase_deg = opts["phase_deg"]  # None = auto
    if phase_deg is None:
        phase_deg = _computed_phase_deg(N)
    phase = math.radians(float(phase_deg))

    lobes = N - 1
    stamp = time.strftime("%H%M%S")
    prefix = f"CY_{stamp}"

    # Ring pins (housing)
    if draw_ring:
        sk_ring = sketches.add(root.xYConstructionPlane)
        sk_ring.name = f"{prefix}_RingPins"
        R = ring_pcd / 2.0
        rr = ring_pin_d / 2.0
        for i in range(N):
            a = 2.0 * math.pi * i / N
            _add_circle(sk_ring, R * math.cos(a), R * math.sin(a), rr)

    # Output pins
    if draw_outpins:
        sk_out = sketches.add(root.xYConstructionPlane)
        sk_out.name = f"{prefix}_OutputPins"
        pr = out_pcd / 2.0
        rr = out_pin_d / 2.0
        for i in range(out_n):
            a = 2.0 * math.pi * i / out_n
            _add_circle(sk_out, pr * math.cos(a), pr * math.sin(a), rr)

    ok, msg = _validate(N, ring_pcd, ring_pin_d, ecc, clearance)
    if not ok:
        raise RuntimeError(msg)

    base_pts, _, _, _ = _generate_profile_no_overlap(
        N, ring_pcd, ring_pin_d, ecc, clearance, pts_per_tooth
    )

    # Hole & bore sizing
    hole_rad = (out_pin_d / 2.0) + ecc + (hole_extra_diam / 2.0)
    bore_rad = max(0.01, bore_d / 2.0)

    # Hole pattern in DISC-LOCAL coordinates (patterned around disc center)
    holes_local = _hole_pattern_local(out_n, out_pcd)
    c1 = (ecc, 0.0)
    holes_world = [(hx + c1[0], hy + c1[1]) for (hx, hy) in holes_local]

    # DISC 1
    c1 = (ecc, 0.0)
    profile1_world = [(x + c1[0], y + c1[1]) for (x, y) in base_pts]

    holes1_local = holes_local

    sk_d1 = sketches.add(root.xYConstructionPlane)
    sk_d1.name = f"{prefix}_Disc1"

    _build_disc(sk_d1, profile1_world, c1, holes1_local, hole_rad, bore_rad)

    if not dual:
        return

    # DISC 2
    c2 = (-ecc, 0.0) if opposed else (ecc, 0.0)

    # Rotate disc profile by phase (disc-local), then shift to c2
    profile2_local = [_rot_xy(x, y, phase) for (x, y) in base_pts]
    profile2_world = [(x + c2[0], y + c2[1]) for (x, y) in profile2_local]

    holes2_local = [(hx - c2[0], hy - c2[1]) for (hx, hy) in holes_world]

    sk_d2 = sketches.add(root.xYConstructionPlane)
    sk_d2.name = f"{prefix}_Disc2_phase{phase_deg:.2f}"
    _build_disc(sk_d2, profile2_world, c2, holes2_local, hole_rad, bore_rad)

# UI
def _update_ui(ins):
    try:
        auto_phase = bool(ins.itemById("auto_phase").value)
        manual_N = bool(ins.itemById("manual_N").value)

        N_input = ins.itemById("N_ring")
        ratio_input = ins.itemById("reduction_ratio")

        if not manual_N:
            ratio = int(ratio_input.value)
            N_input.value = ratio + 1

        manual = ins.itemById("phase_deg_manual")
        readout = ins.itemById("phase_readout")

        manual.isVisible = (not auto_phase)
        readout.isVisible = auto_phase

        if auto_phase:
            readout.text = f"Auto: {_computed_phase_deg(int(N_input.value)):.2f}°"
        else:
            readout.text = f"Manual: {math.degrees(manual.value):.2f}°"
    except:
        pass


def _update_status(ins):
    try:
        N = int(ins.itemById("N_ring").value)
        ring_pcd = _cm_to_mm(ins.itemById("ring_pcd").value)
        ring_pin_d = _cm_to_mm(ins.itemById("ring_pin_d").value)
        ecc = _cm_to_mm(ins.itemById("ecc").value)
        clearance = _cm_to_mm(ins.itemById("profile_offset").value)

        ok, msg = _validate(N, ring_pcd, ring_pin_d, ecc, max(0.0, clearance))

        lobes = N - 1
        ratio = lobes

        auto_phase = bool(ins.itemById("auto_phase").value)
        phase_deg = _computed_phase_deg(N) if auto_phase else math.degrees(ins.itemById("phase_deg_manual").value)

        # Estimate disc diameter (rough): ring radius + ecc + effective roller radius
        pin_r = ring_pin_d / 2.0
        d_eff_est = pin_r + max(0.0, clearance)
        approx_dia = 2.0 * ((ring_pcd / 2.0) + ecc + max(0.0, d_eff_est))

        st = ins.itemById("status")

        if ok:
            st.text = (
                "OK\n"
                f"Ratio: {ratio}:1 | Lobes: {lobes} | Phase: {phase_deg:.2f}°\n"
                f"E: {ecc:.3f} mm | Pin Ø: {ring_pin_d:.3f} mm\n"
                f"Roller clearance: {max(0.0, clearance):.3f} mm | Est. disc Ø: ~{approx_dia:.1f} mm"
            )
        else:
            st.text = "Invalid\n" + msg
    except:
        pass

# Event handler
class _OnCreate(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            app = adsk.core.Application.get()
            cmd = args.command
            ins = cmd.commandInputs

            on_exec = _OnExecute()
            cmd.execute.add(on_exec)
            _HANDLERS.append(on_exec)

            on_val = _OnValidate()
            cmd.validateInputs.add(on_val)
            _HANDLERS.append(on_val)

            on_chg = _OnInputChanged()
            cmd.inputChanged.add(on_chg)
            _HANDLERS.append(on_chg)

            ins.addTextBoxCommandInput("status", "Status", "", 5, True)
            ins.addTextBoxCommandInput("status_spacer", "", "\n", 1, True)

            ins.addIntegerSpinnerCommandInput(
                "reduction_ratio",
                "Reduction ratio (X:1)",
                2, 100, 1, 8
            )

            ins.addBoolValueInput(
                "manual_N",
                "Manually set ring pins (N)",
                True, "", False
            )

            ins.addIntegerSpinnerCommandInput("N_ring", "Ring pins (N)", 3, 60, 1, 9)
            ins.addValueInput("ring_pcd", "Ring pin PCD", "mm", adsk.core.ValueInput.createByString("76 mm"))
            ins.addValueInput("ring_pin_d", "Ring pin diameter", "mm", adsk.core.ValueInput.createByString("6 mm"))
            ins.addValueInput("ecc", "Eccentricity (E)", "mm", adsk.core.ValueInput.createByString("1.8 mm"))

            ins.addValueInput(
                "profile_offset",
                "Roller clearance (recommended 0.05–0.15)",
                "mm",
                adsk.core.ValueInput.createByString("0.10 mm")
            )

            ins.addIntegerSpinnerCommandInput("pts_per_tooth", "Points per lobe", 10, 80, 5, 30)

            ins.addIntegerSpinnerCommandInput("out_n", "Output pin count", 1, 60, 1, 9)
            ins.addValueInput("out_pin_d", "Output pin diameter", "mm", adsk.core.ValueInput.createByString("4 mm"))
            ins.addValueInput("out_pcd", "Output pin PCD", "mm", adsk.core.ValueInput.createByString("44 mm"))
            ins.addValueInput("hole_extra_diam", "Hole extra clearance (diameter)", "mm", adsk.core.ValueInput.createByString("0.30 mm"))

            ins.addValueInput("bore_d", "Disc center bore diameter", "mm", adsk.core.ValueInput.createByString("22 mm"))

            ins.addBoolValueInput("draw_ring", "Draw ring pins sketch", True, "", True)
            ins.addBoolValueInput("draw_outpins", "Draw output pins sketch", True, "", True)

            ins.addBoolValueInput("dual", "Create Disc2", True, "", True)
            ins.addBoolValueInput("opposed", "Disc2 opposite eccentric", True, "", True)

            ins.addBoolValueInput("auto_phase", "Auto phase (180° / lobes)", True, "", True)
            ins.addValueInput("phase_deg_manual", "Manual phase (deg)", "deg", adsk.core.ValueInput.createByString("22.5 deg"))
            ins.addTextBoxCommandInput("phase_readout", "Phase", "", 1, True)

            _update_ui(ins)
            _update_status(ins)

        except:
            adsk.core.Application.get().userInterface.messageBox(traceback.format_exc())

class _OnInputChanged(adsk.core.InputChangedEventHandler):
    def notify(self, args):
        try:
            ins = args.firingEvent.sender.commandInputs
            _update_ui(ins)
            _update_status(ins)
        except:
            pass

class _OnValidate(adsk.core.ValidateInputsEventHandler):
    def notify(self, args):
        try:
            ins = args.firingEvent.sender.commandInputs
            N = int(ins.itemById("N_ring").value)
            ring_pcd = _cm_to_mm(ins.itemById("ring_pcd").value)
            ring_pin_d = _cm_to_mm(ins.itemById("ring_pin_d").value)
            ecc = _cm_to_mm(ins.itemById("ecc").value)
            clearance = max(0.0, _cm_to_mm(ins.itemById("profile_offset").value))

            ok, msg = _validate(N, ring_pcd, ring_pin_d, ecc, clearance)
            args.areInputsValid = ok

            _update_ui(ins)
            _update_status(ins)
        except:
            args.areInputsValid = False

class _OnExecute(adsk.core.CommandEventHandler):
    def notify(self, args):
        app = adsk.core.Application.get()
        ui = app.userInterface
        try:
            design = adsk.fusion.Design.cast(app.activeProduct)
            ins = args.firingEvent.sender.commandInputs

            
            manual_N = bool(ins.itemById("manual_N").value)

            if manual_N:
                N = int(ins.itemById("N_ring").value)
            else:
                ratio = int(ins.itemById("reduction_ratio").value)
                N = ratio + 1

            ring_pcd = _cm_to_mm(ins.itemById("ring_pcd").value)
            ring_pin_d = _cm_to_mm(ins.itemById("ring_pin_d").value)
            ecc = _cm_to_mm(ins.itemById("ecc").value)
            clearance = max(0.0, _cm_to_mm(ins.itemById("profile_offset").value))

            ok, msg = _validate(N, ring_pcd, ring_pin_d, ecc, clearance)
            if not ok:
                ui.messageBox("Fix inputs:\n" + msg)
                return

            out_n = int(ins.itemById("out_n").value)
            out_pin_d = _cm_to_mm(ins.itemById("out_pin_d").value)
            out_pcd = _cm_to_mm(ins.itemById("out_pcd").value)
            hole_extra_diam = _cm_to_mm(ins.itemById("hole_extra_diam").value)

            bore_d = _cm_to_mm(ins.itemById("bore_d").value)
            pts_per_tooth = int(ins.itemById("pts_per_tooth").value)

            draw_ring = bool(ins.itemById("draw_ring").value)
            draw_outpins = bool(ins.itemById("draw_outpins").value)

            dual = bool(ins.itemById("dual").value)
            opposed = bool(ins.itemById("opposed").value)

            auto_phase = bool(ins.itemById("auto_phase").value)
            if auto_phase:
                phase_deg = None
            else:
                phase_deg = math.degrees(ins.itemById("phase_deg_manual").value)

            opts = dict(
                N_ring=N,
                ring_pcd=ring_pcd,
                ring_pin_d=ring_pin_d,
                ecc=ecc,
                profile_offset=clearance,

                pts_per_tooth=pts_per_tooth,
                out_n=out_n,
                out_pin_d=out_pin_d,
                out_pcd=out_pcd,
                hole_extra_diam=hole_extra_diam,
                bore_d=bore_d,

                draw_ring=draw_ring,
                draw_outpins=draw_outpins,
                dual=dual,
                opposed=opposed,
                phase_deg=phase_deg
            )


            _generate(design, opts)
            ui.messageBox("Done.")
        except:
            ui.messageBox("Failed:\n" + traceback.format_exc())

# Entry points
def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        old = ui.commandDefinitions.itemById(CMD_ID)
        if old:
            old.deleteMe()

        cmd_def = ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, CMD_DESC)
        on_create = _OnCreate()
        cmd_def.commandCreated.add(on_create)
        _HANDLERS.append(on_create)

        cmd_def.execute()
        adsk.autoTerminate(False)

    except:
        if ui:
            ui.messageBox("Run failed:\n" + traceback.format_exc())
        adsk.autoTerminate(True)

def stop(context):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except:
        pass
    adsk.autoTerminate(True)
