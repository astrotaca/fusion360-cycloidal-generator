import math
import time
import traceback
import adsk.core
import adsk.fusion

_HANDLERS = []

CMD_ID = "CycloDualDiscGen"
CMD_NAME = "Cycloidal Dual-Disc Sketch Generator"
CMD_DESC = "Cycloidal reducer sketches (dual disc, aligned output holes)."

CLEARANCE_MM = 0.10
HOLE_EXTRA_DIAM_MM = 0.30

def _mm_to_cm(v): return v / 10.0
def _cm_to_mm(v): return v * 10.0

def _p3(x_mm, y_mm):
    return adsk.core.Point3D.create(_mm_to_cm(x_mm), _mm_to_cm(y_mm), 0.0)

def _rot(x, y, a):
    ca, sa = math.cos(a), math.sin(a)
    return (x * ca - y * sa, x * sa + y * ca)

def _add_circle(sk, cx_mm, cy_mm, r_mm):
    return sk.sketchCurves.sketchCircles.addByCenterRadius(_p3(cx_mm, cy_mm), _mm_to_cm(r_mm))

def _add_spline(sk, pts_mm, closed=True):
    coll = adsk.core.ObjectCollection.create()
    for x, y in pts_mm:
        coll.add(_p3(x, y))
    sp = sk.sketchCurves.sketchFittedSplines.add(coll)
    sp.isClosed = bool(closed)
    return sp

def _phase_deg_auto(N):
    lobes = N - 1
    return 180.0 / lobes if lobes > 0 else 0.0

def _validate(N, ring_pcd, pin_d, ecc, out_n, out_pin_d, out_pcd, bore_d):
    if N < 3: return False, "N must be >= 3"
    if ring_pcd <= 0 or pin_d <= 0: return False, "Ring PCD / pin dia must be > 0"
    if ecc <= 0: return False, "E must be > 0"
    if out_n < 1: return False, "Output pin count must be >= 1"
    if out_pin_d <= 0 or out_pcd <= 0: return False, "Output pin dia / PCD must be > 0"
    if bore_d < 0: return False, "Bore dia must be >= 0"

    R = ring_pcd / 2.0
    if ecc * N >= R:
        return False, "Need E*N < ring radius"

    pitch = (2.0 * math.pi * R) / N
    if pin_d >= 0.98 * pitch:
        return False, "Pins too large for this PCD"

    if (pin_d / 2.0 + CLEARANCE_MM) >= R:
        return False, "Clearance too large vs ring radius"

    return True, ""

def _trochoid_parallel_pts(N, ring_pcd, ecc, d_eff, pts_per_lobe):
    R = ring_pcd / 2.0
    lobes = N - 1

    pts_per_lobe = max(8, min(120, int(pts_per_lobe)))
    steps = max(240, min(1800, lobes * pts_per_lobe))

    pts = []
    prev_nx = None
    prev_ny = None

    for i in range(steps):
        p = 2.0 * math.pi * i / steps

        xa = R * math.cos(p) - ecc * math.cos(N * p)
        ya = R * math.sin(p) - ecc * math.sin(N * p)

        dxa = -R * math.sin(p) + (ecc * N) * math.sin(N * p)
        dya =  R * math.cos(p) - (ecc * N) * math.cos(N * p)

        s = math.hypot(dxa, dya)
        if s < 1e-10:
            continue

        nx = -dya / s
        ny =  dxa / s

        if prev_nx is not None and (prev_nx * nx + prev_ny * ny) < 0.0:
            nx, ny = -nx, -ny

        prev_nx, prev_ny = nx, ny
        pts.append((xa + d_eff * nx, ya + d_eff * ny))

    return pts

def _pattern_pts(n, pcd):
    r = pcd / 2.0
    return [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n)) for i in range(n)]

def _build_disc(sketches, plane, name, center_mm, profile_local_pts, profile_rot_rad, holes_world_pts, hole_r, bore_r):
    sk = sketches.add(plane)
    sk.name = name
    sk.isComputeDeferred = True
    try:
        cx, cy = center_mm

        prof = [_rot(x, y, profile_rot_rad) for (x, y) in profile_local_pts]
        prof_world = [(x + cx, y + cy) for (x, y) in prof]
        _add_spline(sk, prof_world, closed=True)

        if bore_r > 0:
            _add_circle(sk, cx, cy, bore_r)

        for (hx, hy) in holes_world_pts:
            _add_circle(sk, hx, hy, hole_r)
    finally:
        sk.isComputeDeferred = False

def _generate(design, rr, ring_pcd, ring_pin_d, ecc, pts_per_lobe, out_n, out_pin_d, out_pcd, bore_d, dual, opposed):
    root = design.rootComponent
    sketches = root.sketches
    plane = root.xYConstructionPlane

    N = int(rr) + 1

    ok, msg = _validate(N, ring_pcd, ring_pin_d, ecc, out_n, out_pin_d, out_pcd, bore_d)
    if not ok:
        raise RuntimeError(msg)

    phase_deg = _phase_deg_auto(N)
    phase = math.radians(phase_deg)

    stamp = time.strftime("%H%M%S")
    prefix = f"CY_{stamp}"

    sk_ring = sketches.add(plane)
    sk_ring.name = f"{prefix}_RingPins"
    sk_ring.isComputeDeferred = True
    try:
        R = ring_pcd / 2.0
        rr_pin = ring_pin_d / 2.0
        for i in range(N):
            a = 2.0 * math.pi * i / N
            _add_circle(sk_ring, R * math.cos(a), R * math.sin(a), rr_pin)
    finally:
        sk_ring.isComputeDeferred = False

    sk_out = sketches.add(plane)
    sk_out.name = f"{prefix}_OutputPins"
    sk_out.isComputeDeferred = True
    try:
        pr = out_pcd / 2.0
        rr_out = out_pin_d / 2.0
        for i in range(out_n):
            a = 2.0 * math.pi * i / out_n
            _add_circle(sk_out, pr * math.cos(a), pr * math.sin(a), rr_out)
    finally:
        sk_out.isComputeDeferred = False

    d_eff = ring_pin_d / 2.0 + CLEARANCE_MM
    profile_local = _trochoid_parallel_pts(N, ring_pcd, ecc, d_eff, pts_per_lobe)

    hole_r = (out_pin_d / 2.0) + ecc + (HOLE_EXTRA_DIAM_MM / 2.0)
    bore_r = bore_d / 2.0 if bore_d > 0 else 0.0

    holes_local = _pattern_pts(out_n, out_pcd)

    c1 = (ecc, 0.0)
    holes1_world = [(c1[0] + hx, c1[1] + hy) for (hx, hy) in holes_local]
    _build_disc(sketches, plane, f"{prefix}_Disc1", c1, profile_local, 0.0, holes1_world, hole_r, bore_r)

    if not dual:
        return

    c2 = (-ecc, 0.0) if opposed else (ecc, 0.0)
    holes2_world = [(c2[0] + hx, c2[1] + hy) for (hx, hy) in holes_local]
    _build_disc(sketches, plane, f"{prefix}_Disc2_{phase_deg:.3f}deg", c2, profile_local, phase, holes2_world, hole_r, bore_r)

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

            on_chg = _OnChanged()
            cmd.inputChanged.add(on_chg)
            _HANDLERS.append(on_chg)

            ins.addTextBoxCommandInput("status", "Status", "", 2, True)

            ins.addIntegerSpinnerCommandInput("rr", "Reduction ratio (X:1)", 2, 200, 1, 8)
            ins.addValueInput("ring_pcd", "Ring pin PCD", "mm", adsk.core.ValueInput.createByString("76 mm"))
            ins.addValueInput("ring_pin_d", "Ring pin diameter", "mm", adsk.core.ValueInput.createByString("6 mm"))
            ins.addValueInput("ecc", "Eccentricity (E)", "mm", adsk.core.ValueInput.createByString("1.8 mm"))
            ins.addIntegerSpinnerCommandInput("pts_per_lobe", "Points per lobe", 8, 120, 4, 30)

            ins.addIntegerSpinnerCommandInput("out_n", "Output pin count", 1, 120, 1, 9)
            ins.addValueInput("out_pin_d", "Output pin diameter", "mm", adsk.core.ValueInput.createByString("4 mm"))
            ins.addValueInput("out_pcd", "Output pin PCD", "mm", adsk.core.ValueInput.createByString("44 mm"))
            ins.addValueInput("bore_d", "Center bore diameter", "mm", adsk.core.ValueInput.createByString("22 mm"))

            ins.addBoolValueInput("dual", "Create Disc2", True, "", True)
            ins.addBoolValueInput("opposed", "Disc2 opposite eccentric", True, "", True)

            self._update_status(ins)
        except:
            adsk.core.Application.get().userInterface.messageBox("Create failed:\n" + traceback.format_exc())

    def _update_status(self, ins):
        try:
            rr = int(ins.itemById("rr").value)
            N = rr + 1

            ring_pcd = _cm_to_mm(ins.itemById("ring_pcd").value)
            ring_pin_d = _cm_to_mm(ins.itemById("ring_pin_d").value)
            ecc = _cm_to_mm(ins.itemById("ecc").value)

            out_n = int(ins.itemById("out_n").value)
            out_pin_d = _cm_to_mm(ins.itemById("out_pin_d").value)
            out_pcd = _cm_to_mm(ins.itemById("out_pcd").value)
            bore_d = max(0.0, _cm_to_mm(ins.itemById("bore_d").value))

            ok, msg = _validate(N, ring_pcd, ring_pin_d, ecc, out_n, out_pin_d, out_pcd, bore_d)
            phase_deg = _phase_deg_auto(N)

            st = ins.itemById("status")
            if ok:
                st.text = f"OK | N={N} | Phase={phase_deg:.3f}deg"
            else:
                st.text = "Invalid\n" + msg
        except:
            pass

class _OnChanged(adsk.core.InputChangedEventHandler):
    def notify(self, args):
        try:
            ins = args.firingEvent.sender.commandInputs
            # cheap status refresh
            rr = int(ins.itemById("rr").value)
            N = rr + 1
            phase_deg = _phase_deg_auto(N)
            ins.itemById("status").text = f"N={N} | Phase={phase_deg:.3f}deg"
        except:
            pass

class _OnValidate(adsk.core.ValidateInputsEventHandler):
    def notify(self, args):
        try:
            ins = args.firingEvent.sender.commandInputs

            rr = int(ins.itemById("rr").value)
            N = rr + 1

            ring_pcd = _cm_to_mm(ins.itemById("ring_pcd").value)
            ring_pin_d = _cm_to_mm(ins.itemById("ring_pin_d").value)
            ecc = _cm_to_mm(ins.itemById("ecc").value)

            out_n = int(ins.itemById("out_n").value)
            out_pin_d = _cm_to_mm(ins.itemById("out_pin_d").value)
            out_pcd = _cm_to_mm(ins.itemById("out_pcd").value)
            bore_d = max(0.0, _cm_to_mm(ins.itemById("bore_d").value))

            ok, _ = _validate(N, ring_pcd, ring_pin_d, ecc, out_n, out_pin_d, out_pcd, bore_d)
            args.areInputsValid = bool(ok)
        except:
            args.areInputsValid = False

class _OnExecute(adsk.core.CommandEventHandler):
    def notify(self, args):
        app = adsk.core.Application.get()
        ui = app.userInterface
        try:
            design = adsk.fusion.Design.cast(app.activeProduct)
            if not design:
                ui.messageBox("No active Fusion design.")
                return

            ins = args.firingEvent.sender.commandInputs

            rr = int(ins.itemById("rr").value)
            ring_pcd = _cm_to_mm(ins.itemById("ring_pcd").value)
            ring_pin_d = _cm_to_mm(ins.itemById("ring_pin_d").value)
            ecc = _cm_to_mm(ins.itemById("ecc").value)
            pts_per_lobe = int(ins.itemById("pts_per_lobe").value)

            out_n = int(ins.itemById("out_n").value)
            out_pin_d = _cm_to_mm(ins.itemById("out_pin_d").value)
            out_pcd = _cm_to_mm(ins.itemById("out_pcd").value)
            bore_d = max(0.0, _cm_to_mm(ins.itemById("bore_d").value))

            dual = bool(ins.itemById("dual").value)
            opposed = bool(ins.itemById("opposed").value)

            _generate(design, rr, ring_pcd, ring_pin_d, ecc, pts_per_lobe, out_n, out_pin_d, out_pcd, bore_d, dual, opposed)
            ui.messageBox("Done.")
        except:
            ui.messageBox("Failed:\n" + traceback.format_exc())

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
