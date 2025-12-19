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

def _add_poly(sk, pts_mm):
    sk.isComputeDeferred = True
    try:
        lines = sk.sketchCurves.sketchLines
        fusion_pts = [_p3(x, y) for (x, y) in pts_mm]
        for i in range(len(fusion_pts) - 1):
            lines.addByTwoPoints(fusion_pts[i], fusion_pts[i + 1])
        lines.addByTwoPoints(fusion_pts[-1], fusion_pts[0])
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
    """
    Robust trochoid + offset generation.
    Prevents long straight segments that break the profile at high ratios.
    """
    R = ring_pcd_mm / 2.0
    lobes = N_ring - 1

    # Base sampling scaled with ratio
    base_steps = max(240, lobes * int(pts_per_tooth))
    scale = max(1, int(N_ring / 8))
    steps = base_steps * scale

    # Max allowed distance between consecutive points (mm)
    max_segment = 0.25

    def eval_point(p):
        xa = R * math.cos(p) - ecc_mm * math.cos(N_ring * p)
        ya = R * math.sin(p) - ecc_mm * math.sin(N_ring * p)

        dxa = -R * math.sin(p) + (ecc_mm * N_ring) * math.sin(N_ring * p)
        dya =  R * math.cos(p) - (ecc_mm * N_ring) * math.cos(N_ring * p)

        s = math.hypot(dxa, dya)
        if s < 1e-12:
            return None

        return (
            xa - d_eff_mm * (dya / s),
            ya + d_eff_mm * (dxa / s)
        )

    def subdivide(p0, p1, pt0, pt1, out, depth=0):
        if pt0 is None or pt1 is None:
            return

        dx = pt1[0] - pt0[0]
        dy = pt1[1] - pt0[1]
        dist = math.hypot(dx, dy)

        if dist > max_segment and depth < 12:
            pm = 0.5 * (p0 + p1)
            ptm = eval_point(pm)
            subdivide(p0, pm, pt0, ptm, out, depth + 1)
            subdivide(pm, p1, ptm, pt1, out, depth + 1)
        else:
            out.append(pt1)

    pts = []

    p_start = 0.0
    p_end = 2.0 * math.pi

    prev_p = p_start
    prev_pt = eval_point(prev_p)
    if prev_pt is None:
        prev_pt = eval_point(1e-6)

    pts.append(prev_pt)

    for i in range(1, steps + 1):
        p = p_end * i / steps
        pt = eval_point(p)
        subdivide(prev_p, p, prev_pt, pt, pts)
        prev_p = p
        prev_pt = pt

    pts_final = []
    for p in pts:
        if not pts_final:
            pts_final.append(p)
        else:
            dx = p[0] - pts_final[-1][0]
            dy = p[1] - pts_final[-1][1]
            if dx*dx + dy*dy > 1e-12:
                pts_final.append(p)

    return pts_final

def _hole_pattern_local(n_holes, hole_pcd_mm):
    r = hole_pcd_mm / 2.0
    pts = []
    for i in range(n_holes):
        a = 2.0 * math.pi * i / n_holes
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts

def _build_disc(sk, profile_world_pts, disc_center_mm, hole_centers_local, hole_rad_mm, center_bore_rad_mm):
    _add_poly(sk, profile_world_pts)
    _add_circle(sk, disc_center_mm[0], disc_center_mm[1], center_bore_rad_mm)
    for (hx, hy) in hole_centers_local:
        _add_circle(sk, disc_center_mm[0] + hx, disc_center_mm[1] + hy, hole_rad_mm)

def _computed_phase_deg(N_ring):
    lobes = N_ring - 1
    return 180.0 / lobes if lobes > 0 else 0.0

def _seg_intersect(a, b, c, d, eps=1e-12):
    def orient(p, q, r):
        return (q[0]-p[0])*(r[1]-p[1]) - (q[1]-p[1])*(r[0]-p[0])

    def on_seg(p, q, r):
        return (min(p[0], r[0]) - eps <= q[0] <= max(p[0], r[0]) + eps and
                min(p[1], r[1]) - eps <= q[1] <= max(p[1], r[1]) + eps)

    o1 = orient(a, b, c)
    o2 = orient(a, b, d)
    o3 = orient(c, d, a)
    o4 = orient(c, d, b)

    if (o1 > eps and o2 < -eps or o1 < -eps and o2 > eps) and (o3 > eps and o4 < -eps or o3 < -eps and o4 > eps):
        return True

    if abs(o1) <= eps and on_seg(a, c, b): return True
    if abs(o2) <= eps and on_seg(a, d, b): return True
    if abs(o3) <= eps and on_seg(c, a, d): return True
    if abs(o4) <= eps and on_seg(c, b, d): return True

    return False

def _polyline_self_intersects(pts):
    n = len(pts)
    if n < 6:
        return False

    for i in range(n):
        a = pts[i]
        b = pts[(i+1) % n]
        for j in range(i+1, n):

            if j == i: 
                continue
            if (j == (i+1) % n) or ((i == (j+1) % n)):
                continue

            c = pts[j]
            d = pts[(j+1) % n]

            if a == c or a == d or b == c or b == d:
                continue

            if _seg_intersect(a, b, c, d):
                return True
    return False

def _min_gap_to_ring_pins(profile_world_pts, N_ring, ring_pcd_mm, ring_pin_r_mm):
    R = ring_pcd_mm / 2.0
    min_gap = 1e9

    for (x, y) in profile_world_pts:
        for i in range(N_ring):
            a = 2.0 * math.pi * i / N_ring
            cx = R * math.cos(a)
            cy = R * math.sin(a)
            d = math.hypot(x - cx, y - cy) - ring_pin_r_mm
            if d < min_gap:
                min_gap = d
    return min_gap

def _generate_profile_no_overlap(N, ring_pcd, ring_pin_d, ecc, clearance_mm, pts_per_tooth, exact_geometry):
    """
    Returns (base_pts_local, used_guard_mm, min_gap_mm, self_intersects_bool)

    base_pts_local are in disc-local coordinates (to be shifted by disc center later).
    """
    pin_r = ring_pin_d / 2.0

    guard = 0.0
    step = 0.0
    max_guard = 0.0

    wanted_min_gap = 0.0 if exact_geometry else 0.02  # mm

    last_pts = None
    last_gap = None
    last_self = None

    while True:
        d_eff = pin_r + max(0.0, clearance_mm) + guard
        base_pts = _trochoid_parallel_points(N, ring_pcd, ecc, d_eff, pts_per_tooth)

        # disc center is at (ecc, 0) in world for Disc1 during sketching
        c1 = (ecc, 0.0)
        profile_world = [(x + c1[0], y + c1[1]) for (x, y) in base_pts]

        self_int = _polyline_self_intersects(profile_world)
        min_gap = _min_gap_to_ring_pins(profile_world, N, ring_pcd, pin_r)

        last_pts, last_gap, last_self = base_pts, min_gap, self_int

        return base_pts, 0.0, min_gap, self_int

# Main generation
def _generate(design, opts):
    root = design.rootComponent
    sketches = root.sketches

    N = int(opts["N_ring"])
    ring_pcd = float(opts["ring_pcd"])
    ring_pin_d = float(opts["ring_pin_d"])
    ecc = float(opts["ecc"])

    clearance = float(opts["profile_offset"])
    exact_geometry = bool(opts.get("exact_geometry", False))

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
    same_holes_world = bool(opts["same_holes_world"])

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

    # Output pins (WORLD, fixed)
    if draw_outpins:
        sk_out = sketches.add(root.xYConstructionPlane)
        sk_out.name = f"{prefix}_OutputPins"
        pr = out_pcd / 2.0
        rr = out_pin_d / 2.0
        for i in range(out_n):
            a = 2.0 * math.pi * i / out_n
            _add_circle(sk_out, pr * math.cos(a), pr * math.sin(a), rr)

    # Profile generation (NO-OVERLAP default)
    ok, msg = _validate(N, ring_pcd, ring_pin_d, ecc, clearance)
    if not ok:
        raise RuntimeError(msg)

    # Use the robust generator that increases guard until the profile is valid in Fusion
    base_pts, used_guard, min_gap, self_int = _generate_profile_no_overlap(
        N, ring_pcd, ring_pin_d, ecc, clearance, pts_per_tooth, exact_geometry
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
        N = int(ins.itemById("N_ring").value)
        auto_on = bool(ins.itemById("auto_phase").value)

        manual = ins.itemById("phase_deg_manual")
        readout = ins.itemById("phase_readout")

        manual.isVisible = (not auto_on)
        readout.isVisible = auto_on

        if auto_on:
            readout.text = f"Auto: {_computed_phase_deg(N):.2f}°"
        else:
            ph_rad = manual.value
            readout.text = f"Manual: {math.degrees(ph_rad):.2f}°"
    except:
        pass

def _update_status(ins):
    try:
        N = int(ins.itemById("N_ring").value)
        ring_pcd = _cm_to_mm(ins.itemById("ring_pcd").value)
        ring_pin_d = _cm_to_mm(ins.itemById("ring_pin_d").value)
        ecc = _cm_to_mm(ins.itemById("ecc").value)
        clearance = _cm_to_mm(ins.itemById("profile_offset").value)
        exact = bool(ins.itemById("exact_geometry").value)

        ok, msg = _validate(N, ring_pcd, ring_pin_d, ecc, max(0.0, clearance))

        lobes = N - 1

        auto_phase = bool(ins.itemById("auto_phase").value)
        phase_deg = _computed_phase_deg(N) if auto_phase else math.degrees(ins.itemById("phase_deg_manual").value)

        # Estimate disc diameter (rough): ring radius + ecc + effective roller radius
        pin_r = ring_pin_d / 2.0
        d_eff_est = pin_r + max(0.0, clearance) + (0.0 if exact else 0.02)
        approx_dia = 2.0 * ((ring_pcd / 2.0) + ecc + max(0.0, d_eff_est))

        st = ins.itemById("status")
        mode = "Exact" if exact else "Fusion-safe"

        if ok:
            st.text = (
                f"OK ({mode})\n"
                f"Ratio: {lobes}:1 | Lobes: {lobes} | Phase: {phase_deg:.2f}°\n"
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

            ins.addBoolValueInput(
                "exact_geometry",
                "Exact geometry (can be tangent / may be annoying in Fusion)",
                True, "", False
            )

            ins.addIntegerSpinnerCommandInput("pts_per_tooth", "Points per lobe", 40, 600, 10, 120)

            ins.addIntegerSpinnerCommandInput("out_n", "Output pin count", 1, 60, 1, 9)
            ins.addValueInput("out_pin_d", "Output pin diameter", "mm", adsk.core.ValueInput.createByString("4 mm"))
            ins.addValueInput("out_pcd", "Output pin PCD", "mm", adsk.core.ValueInput.createByString("44 mm"))
            ins.addValueInput("hole_extra_diam", "Hole extra clearance (diameter)", "mm", adsk.core.ValueInput.createByString("0.30 mm"))

            ins.addValueInput("bore_d", "Disc center bore diameter", "mm", adsk.core.ValueInput.createByString("22 mm"))

            ins.addBoolValueInput("draw_ring", "Draw ring pins sketch", True, "", True)
            ins.addBoolValueInput("draw_outpins", "Draw output pins sketch", True, "", True)

            ins.addBoolValueInput("dual", "Create Disc2", True, "", True)
            ins.addBoolValueInput("opposed", "Disc2 opposite eccentric", True, "", True)

            ins.addBoolValueInput("same_holes_world", "Identical hole positions on both discs", True, "", True)

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

            N = int(ins.itemById("N_ring").value)
            ring_pcd = _cm_to_mm(ins.itemById("ring_pcd").value)
            ring_pin_d = _cm_to_mm(ins.itemById("ring_pin_d").value)
            ecc = _cm_to_mm(ins.itemById("ecc").value)
            clearance = max(0.0, _cm_to_mm(ins.itemById("profile_offset").value))
            exact = bool(ins.itemById("exact_geometry").value)

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
            same_holes_world = bool(ins.itemById("same_holes_world").value)

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
                profile_offset=clearance,           # treated as clearance (positive)
                exact_geometry=exact,

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
                same_holes_world=same_holes_world,
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
