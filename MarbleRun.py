"""
Modular Marble Run generator for Autodesk Fusion  (v2)

Marble loop:
  Archimedes screw (tilted, open, with guide rail)
    -> chute drops the marble tangentially onto the funnel edge (it spins)
    -> funnel exit drops it onto the inclined Plinko board
    -> Plinko exits fall into the open U-track
    -> long serpentine track with hairpin turns
    -> back into the bottom of the screw

Bodies: Base, Plinko, Funnel, Track, Screw, ScrewTower (rail + chute + motor mount),
        ControlBox, ControlBoxLid, Marble
Separate print (not in the assembly): HandCrank.stl

Run: Utilities > Add-Ins > Scripts and Add-Ins > MarbleRun > Run.
Writes log.txt (alignment heights + interference / marble-path checks) and exports/
(STEP, F3D, one STL per body) next to this file.

All dimensions are mm. Fusion's API uses cm internally; P()/vi() convert.
"""
import adsk.core
import adsk.fusion
import math
import os
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
LOG = []


def log(msg):
    LOG.append(str(msg))


# ---------------------------------------------------------------------------
# Parameters (mm / degrees)
# ---------------------------------------------------------------------------
BASE = dict(size=180.0, thick=4.0, m3_hole=3.4, hole_inset=8.0)
MARBLE_D = 8.0

# Open U-channel used for the track and the chute
CHANNEL = dict(id=9.0, od=14.0, lip=3.0)        # walls rise `lip` mm above the axis
TRACK = dict(slope=4.0, bend_r=15.0, line_gap=30.0)
CHUTE = dict(slope=5.0, bend_r=12.0, entry_r=37.0)  # entry_r: radius the marble lands on

FUNNEL = dict(drop=30.0, angle=100.0, wall=3.0, exit_d=10.0, fillet=5.0, lip_h=4.0)

PLINKO = dict(length=70.0, width=44.0, wall=3.0, floor=3.0, wall_h=11.0, incline=40.0,
              peg_w=3.0, peg_l=9.0, peg_h=8.0, peg_pitch=20.0, slot=17.0, chamfer=1.0,
              exit_w=11.0, drop_u=18.0)

SCREW = dict(d=20.0, core=10.0, pitch=12.0, height=170.0, flight_t=2.0, tilt=8.0,
             hub_top=9.0)
RAIL = dict(inner_r=10.6, back_r=13.3, back_t=3.0, half_gap=4.5, side_t=2.5)

N20 = dict(w=12.0, d=10.0, clear=0.2, shaft=3.0, d_flat=2.5, screw_pitch=9.0,
           clamp_h=22.0, plate=3.0, shaft_len=10.0)

# Layout (base centred on origin, top face at z = 4)
S = (68.0, -50.0)            # screw axis at the base
F = (0.0, -18.0)             # funnel axis / Plinko drop point (plan)
INTAKE_AXIS_Z = 11.0         # track axis height where it meets the screw
CONTROL_BOX = (65.0, 45.0)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
MM = 0.1
app = adsk.core.Application.get()
ui = app.userInterface
tb = adsk.fusion.TemporaryBRepManager.get()
OC = adsk.core.ObjectCollection
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation


def P(x, y, z):
    return adsk.core.Point3D.create(x * MM, y * MM, z * MM)


def vi(mm):
    return adsk.core.ValueInput.createByReal(mm * MM)


def unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def add(a, b, k=1.0):
    return tuple(a[i] + b[i] * k for i in range(len(a)))


def V(v):
    return adsk.core.Vector3D.create(*v)


def tcyl(p0, p1, r):
    return tb.createCylinderOrCone(P(*p0), r * MM, P(*p1), r * MM)


def tbox_c(center, ldir, wdir, L, W, H):
    obb = adsk.core.OrientedBoundingBox3D.create(P(*center), V(unit(ldir)), V(unit(wdir)),
                                                 L * MM, W * MM, H * MM)
    return tb.createBox(obb)


def tbox(x0, y0, z0, x1, y1, z1):
    c = ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
    return tbox_c(c, (1, 0, 0), (0, 1, 0), abs(x1 - x0), abs(y1 - y0), abs(z1 - z0))


def t_union(a, *bs):
    for b in bs:
        tb.booleanOperation(a, b, adsk.fusion.BooleanTypes.UnionBooleanType)
    return a


def t_cut(a, *bs):
    for b in bs:
        tb.booleanOperation(a, b, adsk.fusion.BooleanTypes.DifferenceBooleanType)
    return a


def t_inter_volume(a, b):
    """Volume (mm3) of the overlap of two bodies (copies are used)."""
    try:
        ca, cb = tb.copy(a), tb.copy(b)
        tb.booleanOperation(ca, cb, adsk.fusion.BooleanTypes.IntersectionBooleanType)
        return ca.volume * 1000.0
    except Exception:
        return 0.0


def add_body(comp, tbody, name):
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tbody, bf)
    bf.finishEdit()
    body = bf.bodies.item(bf.bodies.count - 1)
    body.name = name
    return body


def combine(comp, target, tools, op, keep=False):
    coll = OC.create()
    for t in tools:
        coll.add(t)
    ci = comp.features.combineFeatures.createInput(target, coll)
    ci.operation = op
    ci.isKeepToolBodies = keep
    ci.isNewComponent = False
    return comp.features.combineFeatures.add(ci)


def fillet(comp, edges, r):
    coll = OC.create()
    for e in edges:
        coll.add(e)
    fi = comp.features.filletFeatures.createInput()
    try:
        fi.edgeSetInputs.addConstantRadiusEdgeSet(coll, vi(r), True)
    except Exception:
        fi.addConstantRadiusEdgeSet(coll, vi(r), True)
    return comp.features.filletFeatures.add(fi)


def chamfer(comp, edges, d):
    coll = OC.create()
    for e in edges:
        coll.add(e)
    try:
        ci = comp.features.chamferFeatures.createInput2()
        ci.chamferEdgeSets.addEqualDistanceChamferEdgeSet(coll, vi(d), True)
    except Exception:
        ci = comp.features.chamferFeatures.createInput(coll, True)
        ci.setToEqualDistance(vi(d))
    return comp.features.chamferFeatures.add(ci)


def move_body(comp, body, matrix):
    coll = OC.create()
    coll.add(body)
    try:
        mi = comp.features.moveFeatures.createInput2(coll)
        mi.defineAsFreeMove(matrix)
    except Exception:
        mi = comp.features.moveFeatures.createInput(coll, matrix)
    comp.features.moveFeatures.add(mi)


def translation(dx, dy, dz):
    m = adsk.core.Matrix3D.create()
    m.translation = adsk.core.Vector3D.create(dx * MM, dy * MM, dz * MM)
    return m


def circle_edges(body, r, z, tol=0.05):
    out = []
    for e in body.edges:
        g = e.geometry
        if g.objectType == adsk.core.Circle3D.classType():
            if abs(g.radius / MM - r) < tol and abs(g.center.z / MM - z) < tol:
                out.append(e)
    return out


# --- screw tilt: rotate about a line parallel to X through (S, z=base top) ---
def tilt_matrix():
    m = adsk.core.Matrix3D.create()
    m.setToRotation(-math.radians(SCREW['tilt']), V((1, 0, 0)), P(S[0], S[1], BASE['thick']))
    return m


def tilt_pt(p):
    """Python mirror of tilt_matrix(): top of the screw leans toward +Y."""
    a = -math.radians(SCREW['tilt'])
    y, z = p[1] - S[1], p[2] - BASE['thick']
    return (p[0], S[1] + y * math.cos(a) - z * math.sin(a), BASE['thick'] + y * math.sin(a) + z * math.cos(a))


# ---------------------------------------------------------------------------
# Path utilities (plan paths with fillet arcs)
# ---------------------------------------------------------------------------
def fillet_segments(wps, r):
    segs = []
    cur = wps[0]
    for i in range(1, len(wps) - 1):
        a, b, c = wps[i - 1], wps[i], wps[i + 1]
        d1 = unit((b[0] - a[0], b[1] - a[1]))
        d2 = unit((c[0] - b[0], c[1] - b[1]))
        th = math.acos(max(-1, min(1, d1[0] * d2[0] + d1[1] * d2[1])))
        t = r * math.tan(th / 2)
        t1 = (b[0] - d1[0] * t, b[1] - d1[1] * t)
        t2 = (b[0] + d2[0] * t, b[1] + d2[1] * t)
        turn = d1[0] * d2[1] - d1[1] * d2[0]
        n = (-d1[1], d1[0]) if turn > 0 else (d1[1], -d1[0])
        cen = (t1[0] + n[0] * r, t1[1] + n[1] * r)
        a0 = math.atan2(t1[1] - cen[1], t1[0] - cen[0])
        a1 = math.atan2(t2[1] - cen[1], t2[0] - cen[0])
        da = a1 - a0
        while da > math.pi:
            da -= 2 * math.pi
        while da < -math.pi:
            da += 2 * math.pi
        if math.hypot(t1[0] - cur[0], t1[1] - cur[1]) > 1e-6:
            segs.append(('L', cur, t1))
        segs.append(('A', cen, a0, da, r))
        cur = t2
    segs.append(('L', cur, wps[-1]))
    return segs


def hairpin_path(points):
    """points: list of ('P', (x,y)) and ('U', centre, start_angle, sweep) items already
    expanded into explicit segments by the caller."""
    return points


def seg_length(sg):
    if sg[0] == 'L':
        return math.hypot(sg[2][0] - sg[1][0], sg[2][1] - sg[1][1])
    return abs(sg[3]) * sg[4]


def sample_segments(segs, step=10.0, arc_step_deg=10.0):
    """Samples (x, y, s) and 'pieces' [(p0, s0, p1, s1)] for the top-cut boxes."""
    pts, pieces = [], []
    s = 0.0
    first = segs[0][1] if segs[0][0] == 'L' else None
    if first is None:
        cen, a0, r = segs[0][1], segs[0][2], segs[0][4]
        first = (cen[0] + r * math.cos(a0), cen[1] + r * math.sin(a0))
    pts.append((first[0], first[1], 0.0))
    for sg in segs:
        ln = seg_length(sg)
        if sg[0] == 'L':
            p0, p1 = sg[1], sg[2]
            n = max(1, int(math.ceil(ln / step)))
            for k in range(1, n + 1):
                f = k / n
                pts.append((p0[0] + (p1[0] - p0[0]) * f, p0[1] + (p1[1] - p0[1]) * f, s + ln * f))
            pieces.append((p0, s, p1, s + ln))
        else:
            cen, a0, da, rr = sg[1], sg[2], sg[3], sg[4]
            n = max(2, int(math.ceil(abs(math.degrees(da)) / arc_step_deg)))
            prev = (cen[0] + rr * math.cos(a0), cen[1] + rr * math.sin(a0))
            ps = s
            for k in range(1, n + 1):
                a = a0 + da * k / n
                q = (cen[0] + rr * math.cos(a), cen[1] + rr * math.sin(a))
                qs = s + ln * k / n
                pts.append((q[0], q[1], qs))
                pieces.append((prev, ps, q, qs))
                prev, ps = q, qs
        s += ln
    # drop samples that sit almost on top of each other (a 0.3 mm stub before a bend
    # makes the fitted spline kink and the sweep self-intersect)
    clean = [pts[0]]
    for p in pts[1:]:
        if p[2] - clean[-1][2] >= 2.0:
            clean.append(p)
    if pts[-1][2] - clean[-1][2] > 1e-6:
        if len(clean) > 1 and pts[-1][2] - clean[-1][2] < 2.0:
            clean[-1] = pts[-1]
        else:
            clean.append(pts[-1])
    return clean, pieces, s


def point_at(samples, s_target):
    for i in range(1, len(samples)):
        if samples[i][2] >= s_target:
            a, b = samples[i - 1], samples[i]
            f = (s_target - a[2]) / max(1e-9, b[2] - a[2])
            return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)
    return samples[-1][:2]


def arc_seg(cen, r, a_from_deg, a_to_deg):
    a0 = math.radians(a_from_deg)
    return ('A', cen, a0, math.radians(a_to_deg - a_from_deg), r)


# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------
def layout():
    L = {}
    # --- funnel geometry (local z = 0 at outer bottom of the exit) ---
    half = math.radians(FUNNEL['angle'] / 2)
    k = math.tan(half)
    rh = FUNNEL['exit_d'] / 2
    dz = FUNNEL['wall'] / math.sin(half)
    L.update(f_k=k, f_rh=rh, f_dz=dz, f_rin_top=rh + FUNNEL['drop'] * k)
    L['f_lip_in'] = L['f_rin_top']
    L['f_lip_out'] = L['f_lip_in'] + FUNNEL['wall']
    L['f_lip_out_z'] = (L['f_lip_out'] - rh) / k          # outer cone meets lip outer wall

    # --- screw / tower (local frame = vertical screw at S; tilted afterwards) ---
    z0 = BASE['thick'] + 2.5
    L['z_screw0'] = z0
    L['z_screw1'] = z0 + SCREW['height']
    L['z_hub_top'] = L['z_screw1'] + SCREW['hub_top']
    L['z_bracket0'] = L['z_hub_top'] + 2.0
    L['z_bracket1'] = L['z_bracket0'] + N20['plate']
    L['z_exit_floor_local'] = L['z_screw1'] - 2.5          # chute floor (local height)
    L['z_rail_bottom_local'] = INTAKE_AXIS_Z + CHANNEL['lip'] + 1.0

    # chute start: flush with the top of the rail's back wall. The marble rides up the
    # back wall; when its bottom clears the wall top it rolls outward (8 deg tilt) onto the chute.
    cs = tilt_pt((S[0], S[1] - RAIL['back_r'] - 1.7, L['z_exit_floor_local']))
    L['chute_start'] = cs
    L['chute_floor0'] = cs[2]

    # chute plan path: south, bend west, run along the funnel's south side to the tangent point
    y_run = F[1] - CHUTE['entry_r']
    r = CHUTE['bend_r']
    cwps = [(cs[0], cs[1]), (cs[0], y_run), (F[0] - 2.0, y_run)]
    L['chute_wps'] = cwps
    csegs = fillet_segments(cwps, r)
    csamp, cpieces, clen = sample_segments(csegs, step=8.0)
    L['chute_samples'], L['chute_pieces'], L['chute_len'] = csamp, cpieces, clen
    ctan = math.tan(math.radians(CHUTE['slope']))
    ri = CHANNEL['id'] / 2
    ro = CHANNEL['od'] / 2
    # where the chute crosses the funnel lip (outer radius)
    s_cross = None
    for (x, y, s) in csamp:
        if math.hypot(x - F[0], y - F[1]) <= L['f_lip_out'] + ro:
            s_cross = s
            break
    floor_cross = L['chute_floor0'] - s_cross * ctan
    L['chute_floor_end'] = L['chute_floor0'] - clen * ctan
    # funnel: lip top 0.8 mm under the chute's underside where it crosses
    lip_top = floor_cross - (ro - ri) - 0.8
    L['z_funnel_exit'] = lip_top - FUNNEL['lip_h'] - FUNNEL['drop']
    L['z_funnel0'] = L['z_funnel_exit'] - dz
    L['z_funnel_rim'] = L['z_funnel_exit'] + FUNNEL['drop']
    L['z_lip_top'] = lip_top
    L['marble_drop_to_cone'] = (L['chute_floor_end']) - (L['z_funnel_exit'] + (CHUTE['entry_r'] - rh) / k)

    # --- Plinko board frame (slopes down toward -X) ---
    b = math.radians(PLINKO['incline'])
    c, s_ = math.cos(b), math.sin(b)
    u = (-c, 0.0, -s_)
    v = (0.0, -1.0, 0.0)
    w = unit(cross(u, v))
    L['pl_u'], L['pl_v'], L['pl_w'] = u, v, w
    # drop point: the board's top wall (its highest edge) must pass 1 mm under the funnel
    top_rise = (PLINKO['drop_u'] + PLINKO['wall']) * s_ + PLINKO['wall_h'] * c
    L['z_drop'] = L['z_funnel0'] - 1.0 - top_rise
    # the falling marble must clear the top wall's inner top edge
    L['pl_topwall_clear'] = PLINKO['drop_u'] * c - PLINKO['wall_h'] * s_ - MARBLE_D / 2
    du = PLINKO['drop_u']
    L['pl_origin'] = (F[0] - du * u[0], F[1] - du * u[1], L['z_drop'] - du * u[2])
    o = L['pl_origin']
    ub = PLINKO['length'] + PLINKO['wall']
    edge = add(o, u, ub)                       # bottom outer edge, floor top
    L['pl_edge'] = edge
    L['pl_wall_top_edge'] = add(edge, w, PLINKO['wall_h'])
    # floor underside at the bottom edge
    under_edge = add(edge, w, -PLINKO['floor'])
    # --- track ---
    x_b = edge[0] - 2.0
    x_f = x_b - TRACK['line_gap']
    L['x_b'], L['x_f'] = x_b, x_f
    L['z_track_start'] = under_edge[2] - CHANNEL['lip'] - 1.0
    L['z_track_end'] = INTAKE_AXIS_Z
    return L


def track_segments(L, Y):
    """Serpentine: start north of the Plinko, run south along x_b, hairpins at y=+-Y,
    two full laps, then a final run south, east along the front, north into the screw."""
    x_b, x_f = L['x_b'], L['x_f']
    R = TRACK['bend_r']
    xc = (x_b + x_f) / 2
    # intake end point: just outside the rail at the bottom of the tilted screw
    ax = tilt_pt((S[0], S[1], INTAKE_AXIS_Z))
    end = (S[0], ax[1] - RAIL['inner_r'])
    y_east = end[1] - R - 3.0
    segs = []
    y_start = F[1] + PLINKO['width'] / 2 + PLINKO['wall'] + 3.0
    segs.append(('L', (x_b, y_start), (x_b, -Y)))
    for lap in range(2):
        segs.append(arc_seg((xc, -Y), R, 0, -180))          # south hairpin, x_b -> x_f
        segs.append(('L', (x_f, -Y), (x_f, Y)))
        segs.append(arc_seg((xc, Y), R, 180, 0))            # north hairpin, x_f -> x_b
        if lap == 0:
            segs.append(('L', (x_b, Y), (x_b, -Y)))
    # final run south, bend east, run east, bend north into the screw
    segs.append(('L', (x_b, Y), (x_b, y_east + R)))
    segs.append(arc_seg((x_b + R, y_east + R), R, 180, 270))
    segs.append(('L', (x_b + R, y_east), (end[0] - R, y_east)))
    segs.append(arc_seg((end[0] - R, y_east + R), R, -90, 0))
    segs.append(('L', (end[0], y_east + R), end))
    return segs


def solve_track(L):
    target = (L['z_track_start'] - L['z_track_end']) / math.tan(math.radians(TRACK['slope']))
    lo, hi = 20.0, 75.0
    for _ in range(60):
        mid = (lo + hi) / 2
        ln = sum(seg_length(sg) for sg in track_segments(L, mid))
        if ln < target:
            lo = mid
        else:
            hi = mid
    Y = (lo + hi) / 2
    segs = track_segments(L, Y)
    samp, pieces, ln = sample_segments(segs)
    L['track_Y'] = Y
    L['track_samples'], L['track_pieces'], L['track_plan'] = samp, pieces, ln
    L['track_3d_len'] = ln / math.cos(math.radians(TRACK['slope']))
    return L


# ---------------------------------------------------------------------------
# Open U-channel along a plan path with constant slope
# ---------------------------------------------------------------------------
def build_channel(root, name, samples, pieces, z0, slope_deg, cap_start=False):
    tn = math.tan(math.radians(slope_deg))
    ri, ro, lip = CHANNEL['id'] / 2, CHANNEL['od'] / 2, CHANNEL['lip']

    def zc(s):
        return z0 - s * tn

    sk = root.sketches.add(root.xYConstructionPlane)
    sk.name = name + ' path'
    pts = OC.create()
    for (x, y, s) in samples:
        pts.add(sk.modelToSketchSpace(P(x, y, zc(s))))
    spline = sk.sketchCurves.sketchFittedSplines.add(pts)
    path = root.features.createPath(spline, False)
    body = None
    try:
        pin = root.constructionPlanes.createInput()
        pin.setByDistanceOnPath(spline, adsk.core.ValueInput.createByReal(0.0))
        plane = root.constructionPlanes.add(pin)
        psk = root.sketches.add(plane)
        psk.name = name + ' profile'
        cpt = psk.modelToSketchSpace(P(samples[0][0], samples[0][1], z0))
        psk.sketchCurves.sketchCircles.addByCenterRadius(cpt, ro * MM)
        psk.sketchCurves.sketchCircles.addByCenterRadius(cpt, ri * MM)
        prof = None
        for i in range(psk.profiles.count):
            if psk.profiles.item(i).profileLoops.count == 2:
                prof = psk.profiles.item(i)
        si = root.features.sweepFeatures.createInput(prof, path, NEW)
        body = root.features.sweepFeatures.add(si).bodies.item(0)
    except Exception:
        log('%s sweep failed, using pipe:\n%s' % (name, traceback.format_exc()))
        pi_ = root.features.pipeFeatures.createInput(path, NEW)
        pi_.sectionSize = vi(CHANNEL['od'])
        pi_.isHollow = True
        pi_.sectionThickness = vi(ro - ri)
        body = root.features.pipeFeatures.add(pi_).bodies.item(0)
    body.name = name

    if cap_start:
        p0, p1 = samples[0], samples[1]
        d = unit((p1[0] - p0[0], p1[1] - p0[1], zc(p1[2]) - zc(p0[2])))
        a = (p0[0], p0[1], z0)
        cap = tcyl(add(a, d, -2.0), a, ro)
        cb = add_body(root, cap, 'tmp_cap')
        combine(root, body, [cb], JOIN)
        body = root.bRepBodies.itemByName(name)

    # open the top: boxes from axis+lip upward, one per path piece
    cutter = None
    H = 6.0
    for (p0, s0, p1, s1) in pieces:
        a0 = (p0[0], p0[1], zc(s0))
        a1 = (p1[0], p1[1], zc(s1))
        ld = unit((a1[0] - a0[0], a1[1] - a0[1], a1[2] - a0[2]))
        wd = unit(cross(ld, (0, 0, 1)))
        hd = cross(wd, ld)
        if hd[2] < 0:
            hd = (-hd[0], -hd[1], -hd[2])
        ln = math.dist(a0, a1) + 2.6
        mid = tuple((a0[i] + a1[i]) / 2 for i in range(3))
        ctr = add(mid, hd, lip + H / 2)
        bx = tbox_c(ctr, ld, wd, ln, 2 * ro + 6, H)
        cutter = bx if cutter is None else t_union(cutter, bx)
    if cap_start:
        a = (samples[0][0], samples[0][1], z0)
        cutter = t_union(cutter, tbox(a[0] - 10, a[1] - 10, z0 + lip, a[0] + 10, a[1] + 10, z0 + lip + H))
    cb = add_body(root, cutter, 'tmp_open_top')
    combine(root, body, [cb], CUT)
    return root.bRepBodies.itemByName(name)


# ---------------------------------------------------------------------------
# Modules
# ---------------------------------------------------------------------------
def build_plinko(root, L):
    Lp, Wp, t, fl, wh = PLINKO['length'], PLINKO['width'], PLINKO['wall'], PLINKO['floor'], PLINKO['wall_h']
    hw = Wp / 2
    # local frame: x = u (downhill), y = v (across), z = w (normal)
    outer = add_body(root, tbox(-t, -hw - t, -fl, Lp + t, hw + t, wh), 'Plinko')
    cav = add_body(root, tbox(0, -hw, 0, Lp, hw, wh + 5), 'tmp_cavity')
    try:
        chamfer(root, list(cav.edges), PLINKO['chamfer'])
    except Exception:
        log('Plinko chamfer failed:\n' + traceback.format_exc())
    combine(root, outer, [cav], CUT)

    cuts = None
    # entry slot in the top wall (left side, as in the reference)
    cuts = tbox(-t - 1, -hw, 0.01, 0.5, -hw + PLINKO['slot'], wh + 1)
    # two exits in the bottom wall
    ew = PLINKO['exit_w']
    for (v0, v1) in ((-hw + 5, -hw + 5 + ew), (6.0, 6.0 + ew)):
        t_union(cuts, tbox(Lp - 0.5, v0, 0.01, Lp + t + 1, v1, wh + 1))
    cb = add_body(root, cuts, 'tmp_cuts')
    combine(root, outer, [cb], CUT)

    # pegs (bars along the slope) every 20 mm, staggered; sawtooth deflectors on the walls
    adds = None
    pw, pl, ph = PLINKO['peg_w'], PLINKO['peg_l'], PLINKO['peg_h']
    rows = [15.0, 15.0 + PLINKO['peg_pitch'], 15.0 + 2 * PLINKO['peg_pitch']]
    for i, ur in enumerate(rows):
        vs = (-11.0, 11.0) if i % 2 == 0 else (0.0,)
        for vv in vs:
            bx = tbox(ur - pl / 2, vv - pw / 2, -0.5, ur + pl / 2, vv + pw / 2, ph)
            adds = bx if adds is None else t_union(adds, bx)
    # sawtooth deflectors: a 45-degree-turned square centred on each side wall,
    # trimmed flush with the outside of the wall -> triangle 12 mm long, 6 mm deep
    a = 6.0 * math.sqrt(2)
    for ut in (25.0, 45.0, 62.0):
        for side in (-1, 1):
            dia = tbox_c((ut, side * hw, (wh - 0.5) / 2), (1, 1, 0), (-1, 1, 0), a, a, wh + 0.5)
            if side > 0:
                t_cut(dia, tbox(ut - 10, hw + t, -2, ut + 10, hw + t + 10, wh + 2))
            else:
                t_cut(dia, tbox(ut - 10, -hw - t - 10, -2, ut + 10, -hw - t, wh + 2))
            adds = t_union(adds, dia)
    ab = add_body(root, adds, 'tmp_pegs')
    combine(root, outer, [ab], JOIN)
    body = root.bRepBodies.itemByName('Plinko')

    # place the board
    m = adsk.core.Matrix3D.create()
    m.setWithCoordinateSystem(P(*L['pl_origin']), V(L['pl_u']), V(L['pl_v']), V(L['pl_w']))
    move_body(root, body, m)
    body = root.bRepBodies.itemByName('Plinko')

    # hood + backstop over the collector run so a fast marble can't overshoot
    x_b = L['x_b']
    wt = L['pl_wall_top_edge']
    lip0 = L['z_track_start'] + CHANNEL['lip']
    hood = tbox(x_b - CHANNEL['id'] / 2 - 2.0, F[1] - PLINKO['width'] / 2 - t,
                lip0 + 0.6, x_b - CHANNEL['id'] / 2 + 0.3, F[1] + PLINKO['width'] / 2 + t, wt[2] + 0.6)
    hb = add_body(root, hood, 'tmp_hood')
    combine(root, body, [hb], JOIN)
    return root.bRepBodies.itemByName('Plinko')


def build_funnel(root, L):
    rh, k, dz = L['f_rh'], L['f_k'], L['f_dz']
    drop, lip = FUNNEL['drop'], FUNNEL['lip_h']
    ztop = dz + drop
    sk = root.sketches.add(root.xZConstructionPlane)
    sk.name = 'Funnel profile'
    rz = [(rh, dz), (L['f_lip_in'], ztop), (L['f_lip_in'], ztop + lip), (L['f_lip_out'], ztop + lip),
          (L['f_lip_out'], L['f_lip_out_z']), (rh, 0.0)]
    sp = [sk.modelToSketchSpace(P(r, 0, z)) for r, z in rz]
    for i in range(len(sp)):
        sk.sketchCurves.sketchLines.addByTwoPoints(sp[i], sp[(i + 1) % len(sp)])
    ri = root.features.revolveFeatures.createInput(sk.profiles.item(0), root.zConstructionAxis, NEW)
    ri.setAngleExtent(False, adsk.core.ValueInput.createByReal(2 * math.pi))
    body = root.features.revolveFeatures.add(ri).bodies.item(0)
    body.name = 'Funnel'
    for r, z, tag in [(L['f_lip_in'], ztop, 'cone-to-lip'), (rh, dz, 'exit')]:
        try:
            es = circle_edges(root.bRepBodies.itemByName('Funnel'), r, z)
            if es:
                fillet(root, es, FUNNEL['fillet'])
            else:
                log('Funnel fillet: no %s edge' % tag)
        except Exception:
            log('Funnel %s fillet failed:\n%s' % (tag, traceback.format_exc()))
    body = root.bRepBodies.itemByName('Funnel')
    move_body(root, body, translation(F[0], F[1], L['z_funnel0']))
    body = root.bRepBodies.itemByName('Funnel')
    # notch the raised lip (never the cone) where the chute passes over it
    rim_z = L['z_funnel_exit'] + drop
    notch = None
    for (p0, s0, p1, s1) in L['chute_pieces']:
        mid = ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2)
        if math.hypot(mid[0] - F[0], mid[1] - F[1]) < L['f_lip_out'] + CHANNEL['od']:
            ld = unit((p1[0] - p0[0], p1[1] - p0[1], 0))
            bx = tbox_c((mid[0], mid[1], rim_z + 0.6 + 5), ld, (-ld[1], ld[0], 0),
                        math.dist(p0, p1) + 3, CHANNEL['od'] + 3, 10)
            notch = bx if notch is None else t_union(notch, bx)
    # the chute ends over the bowl; extend the notch 6 mm past its end
    p0, s0, p1, s1 = L['chute_pieces'][-1]
    ld = unit((p1[0] - p0[0], p1[1] - p0[1], 0))
    end = (p1[0] + ld[0] * 3, p1[1] + ld[1] * 3)
    t_union(notch, tbox_c((end[0], end[1], rim_z + 0.6 + 5), ld, (-ld[1], ld[0], 0), 6, CHANNEL['od'] + 3, 10))
    nb = add_body(root, notch, 'tmp_notch')
    combine(root, body, [nb], CUT)
    return root.bRepBodies.itemByName('Funnel')


def build_screw(root, L):
    x, y = S
    z0, z1, zh = L['z_screw0'], L['z_screw1'], L['z_hub_top']
    rs, rc = SCREW['d'] / 2, SCREW['core'] / 2
    core = tcyl((x, y, z0), (x, y, zh), rc)
    t_cut(core, tcyl((x, y, z0 - 1), (x, y, z0 + 6), 2.2),                     # pivot hole
          tcyl((x, y, zh - 8), (x, y, zh + 1), N20['shaft'] / 2 + 0.05))         # D-bore
    flat = N20['d_flat'] - N20['shaft'] / 2
    t_union(core, tbox(x + flat, y - 1.7, zh - 8, x + 1.8, y + 1.7, zh))
    core_b = add_body(root, core, 'Screw')

    sector = math.radians(360.0 * SCREW['flight_t'] / SCREW['pitch'])
    pin = root.constructionPlanes.createInput()
    pin.setByOffset(root.xYConstructionPlane, vi(z0))
    plane = root.constructionPlanes.add(pin)
    sk = root.sketches.add(plane)
    sk.name = 'Screw flight profile'

    def sp(r, a):
        return sk.modelToSketchSpace(P(x + r * math.cos(a), y + r * math.sin(a), z0))
    r_in = rc - 0.3
    sk.sketchCurves.sketchLines.addByTwoPoints(sp(r_in, 0), sp(rs, 0))
    sk.sketchCurves.sketchLines.addByTwoPoints(sp(r_in, sector), sp(rs, sector))
    sk.sketchCurves.sketchArcs.addByThreePoints(sp(r_in, 0), sp(r_in, sector / 2), sp(r_in, sector))
    sk.sketchCurves.sketchArcs.addByThreePoints(sp(rs, 0), sp(rs, sector / 2), sp(rs, sector))
    prof = sk.profiles.item(0)
    psk = root.sketches.add(root.xZConstructionPlane)
    psk.name = 'Screw axis'
    ln = psk.sketchCurves.sketchLines.addByTwoPoints(psk.modelToSketchSpace(P(x, y, z0)),
                                                    psk.modelToSketchSpace(P(x, y, z1)))
    path = root.features.createPath(ln, False)
    si = root.features.sweepFeatures.createInput(prof, path, NEW)
    si.twistAngle = adsk.core.ValueInput.createByReal(math.radians(360.0 * SCREW['height'] / SCREW['pitch']))
    flight = root.features.sweepFeatures.add(si).bodies.item(0)
    flight.name = 'tmp_flight'
    combine(root, core_b, [flight], JOIN)
    body = root.bRepBodies.itemByName('Screw')
    move_body(root, body, tilt_matrix())
    return root.bRepBodies.itemByName('Screw')


def build_tower(root, L):
    """Rail + column + motor bracket built vertical at S, tilted, then the chute added."""
    x, y = S
    R = RAIL
    zb = L['z_rail_bottom_local']
    z_back_top = L['z_exit_floor_local']
    zt0, zt1 = L['z_bracket0'], L['z_bracket1']
    hg, st = R['half_gap'], R['side_t']
    # rail (on -Y side): back wall + two side walls
    back = tbox(x - hg - st, y - R['back_r'] - R['back_t'], zb, x + hg + st, y - R['back_r'], z_back_top)
    sides = t_union(tbox(x - hg - st, y - R['back_r'] - R['back_t'], zb, x - hg, y - R['inner_r'], zt1),
                    tbox(x + hg, y - R['back_r'] - R['back_t'], zb, x + hg + st, y - R['inner_r'], zt1))
    tower = t_union(back, sides)
    # legs down to the base either side of the intake track, plus a cross bar
    yb0, yb1 = y - R['back_r'] - R['back_t'], y - R['back_r']
    for sx in (-1, 1):
        t_union(tower, tbox(x + sx * 10.5 - 1.5, yb0, -5, x + sx * 10.5 + 1.5, yb1, zb + 3))
    t_union(tower, tbox(x - 12, yb0, zb, x + 12, yb1, zb + 3))
    # column on the +Y side and a foot plate
    t_union(tower, tbox(x - 4, y + 12, -5, x + 4, y + 18, zt1),
            tbox(x - 12, y + 9, -5, x + 12, y + 24, BASE['thick'] + 3))
    # top bracket spanning rail -> column, with N20 clamp
    plate = tbox(x - 9, y - R['back_r'] - R['back_t'], zt0, x + 9, y + 18, zt1)
    t_cut(plate, tcyl((x, y, zt0 - 1), (x, y, zt1 + 1), 2.6))
    for sx in (-1, 1):
        px = x + sx * N20['screw_pitch'] / 2
        t_cut(plate, tcyl((px, y, zt0 - 1), (px, y, zt1 + 1), 0.9))
    cw, cd = N20['w'] + 2 * N20['clear'], N20['d'] + 2 * N20['clear']
    zc0, zc1 = zt1, zt1 + N20['clamp_h']
    clamp = t_cut(tbox(x - cw / 2 - 2, y - cd / 2 - 2, zc0, x + cw / 2 + 2, y + cd / 2 + 2, zc1),
                  tbox(x - cw / 2, y - cd / 2, zc0 - 1, x + cw / 2, y + cd / 2, zc1 + 1),
                  tbox(x - 3, y - cd / 2 - 3, zc1 - 8, x + 3, y + cd / 2 + 3, zc1 + 1))
    t_union(tower, plate, clamp)
    tb.transform(tower, tilt_matrix())
    # flatten the bottom onto the base
    t_cut(tower, tbox(-200, -200, -50, 200, 200, BASE['thick']))
    body = add_body(root, tower, 'ScrewTower')

    # chute (open channel) from the rail top to the funnel edge, joined to the tower
    ch = build_channel(root, 'tmp_chute', L['chute_samples'], L['chute_pieces'],
                       L['chute_floor0'] + CHANNEL['id'] / 2, CHUTE['slope'])
    # keep the chute out of the screw's swept volume
    clear = tcyl(tilt_pt((x, y, L['z_screw0'] - 5)), tilt_pt((x, y, L['z_hub_top'] + 1)), SCREW['d'] / 2 + 0.4)
    cb = add_body(root, clear, 'tmp_clear')
    combine(root, ch, [cb], CUT)
    combine(root, root.bRepBodies.itemByName('ScrewTower'), [root.bRepBodies.itemByName('tmp_chute')], JOIN)
    return root.bRepBodies.itemByName('ScrewTower')


def build_track(root, L):
    body = build_channel(root, 'Track', L['track_samples'], L['track_pieces'],
                         L['z_track_start'], TRACK['slope'], cap_start=True)
    return body


def build_crank(root, L):
    # built at the origin: it is a separate print, not part of the assembly
    cx, cy, z = 0.0, 0.0, 0.0
    disc = tcyl((cx, cy, z), (cx, cy, z + 4), 9)
    arm = tbox(cx, cy - 4, z, cx + 26, cy + 4, z + 4)
    knob = tcyl((cx + 24, cy, z + 4), (cx + 24, cy, z + 16), 4)
    # D-peg lying flat would need support; it is modelled pointing up so it prints upright
    peg = tcyl((cx, cy, z + 4), (cx, cy, z + 4 + 13), N20['shaft'] / 2 - 0.05)
    t_cut(peg, tbox(cx + (N20['d_flat'] - N20['shaft'] / 2), cy - 2, z + 4, cx + 2, cy + 2, z + 18))
    crank = t_union(disc, arm, knob, peg)
    return add_body(root, crank, 'HandCrank')


def build_control_box(root, L):
    cx, cy = CONTROL_BOX
    z = BASE['thick']
    W, D, H, t = 42.0, 32.0, 20.0, 2.0
    box = t_cut(tbox(cx - W / 2, cy - D / 2, z, cx + W / 2, cy + D / 2, z + H),
                tbox(cx - W / 2 + t, cy - D / 2 + t, z + t, cx + W / 2 - t, cy + D / 2 - t, z + H + 1),
                tbox(cx + W / 2 - t - 1, cy - 5, z + t + 1, cx + W / 2 + 1, cy + 5, z + t + 5),   # USB-C
                tcyl((cx - W / 2 - 1, cy, z + 12), (cx - W / 2 + t + 1, cy, z + 12), 3.0))         # motor wires
    b1 = add_body(root, box, 'ControlBox')
    lid = t_cut(tbox(cx - W / 2, cy - D / 2, z + H + 0.4, cx + W / 2, cy + D / 2, z + H + 2.4),
                tcyl((cx - 8, cy, z + H), (cx - 8, cy, z + H + 3), 3.6),     # button
                tcyl((cx + 8, cy, z + H), (cx + 8, cy, z + H + 3), 3.6))     # pot shaft
    t_union(lid, tbox(cx - W / 2 + t + 0.2, cy - D / 2 + t + 0.2, z + H - 2, cx + W / 2 - t - 0.2,
                      cy - D / 2 + t + 1.4, z + H + 0.5),
            tbox(cx - W / 2 + t + 0.2, cy + D / 2 - t - 1.4, z + H - 2, cx + W / 2 - t - 0.2,
                 cy + D / 2 - t - 0.2, z + H + 0.5))
    b2 = add_body(root, lid, 'ControlBoxLid')
    return b1, b2


def build_marble(root, L):
    sk = root.sketches.add(root.xZConstructionPlane)
    sk.name = 'Marble profile'
    r = MARBLE_D / 2
    sk.sketchCurves.sketchArcs.addByThreePoints(sk.modelToSketchSpace(P(0, 0, -r)),
                                                sk.modelToSketchSpace(P(r, 0, 0)),
                                                sk.modelToSketchSpace(P(0, 0, r)))
    sk.sketchCurves.sketchLines.addByTwoPoints(sk.modelToSketchSpace(P(0, 0, r)),
                                               sk.modelToSketchSpace(P(0, 0, -r)))
    ri = root.features.revolveFeatures.createInput(sk.profiles.item(0), root.zConstructionAxis, NEW)
    ri.setAngleExtent(False, adsk.core.ValueInput.createByReal(2 * math.pi))
    body = root.features.revolveFeatures.add(ri).bodies.item(0)
    body.name = 'Marble'
    s = 20.0
    px, py = point_at(L['track_samples'], s)
    zc = L['z_track_start'] - s * math.tan(math.radians(TRACK['slope']))
    move_body(root, body, translation(px, py, zc - CHANNEL['id'] / 2 + r))
    return root.bRepBodies.itemByName('Marble')


def build_base(root, L, modules):
    h = BASE['size'] / 2
    t = BASE['thick']
    base = tbox(-h, -h, 0, h, h, t)
    hi = h - BASE['hole_inset']
    for sx in (-1, 1):
        for sy in (-1, 1):
            t_cut(base, tcyl((sx * hi, sy * hi, -1), (sx * hi, sy * hi, t + 1), BASE['m3_hole'] / 2))
    posts = []
    tn = math.tan(math.radians(TRACK['slope']))
    ro = CHANNEL['od'] / 2
    samp = L['track_samples']

    def track_z(s):
        return L['z_track_start'] - s * tn

    # serpentine columns between the two lines, with an arm to every pass
    xc = (L['x_b'] + L['x_f']) / 2
    Y = L['track_Y']
    arms = 0
    for cy in (-Y, -Y / 2, 0.0, Y / 2, Y):
        passes, cur = [], []
        for (x, y, s) in samp:
            d = math.hypot(x - xc, y - cy)
            if d < 18.0:
                cur.append((d, x, y, s))
            elif cur:
                passes.append(min(cur))
                cur = []
        if cur:
            passes.append(min(cur))
        top = t
        for (d, x, y, s) in passes:
            zt = track_z(s) - ro + 1.5
            ld = unit((x - xc, y - cy, 0)) if d > 1e-3 else (1, 0, 0)
            mid = ((xc + x) / 2, (cy + y) / 2, zt - 2.5)
            posts.append(tbox_c(mid, ld, (-ld[1], ld[0], 0), d + 3.0, 5.0, 5.0))
            top = max(top, zt)
            arms += 1
        posts.append(tcyl((xc, cy, t - 0.5), (xc, cy, top), 3.0))
    log('Track columns: 5, arms: %d' % arms)
    # direct posts under the low final legs (nothing else above them there)
    s_end = L['track_plan']
    for ds in (12.0, 45.0, 80.0, 115.0):
        s = s_end - ds
        x, y = point_at(samp, s)
        posts.append(tcyl((x, y, t - 0.5), (x, y, track_z(s) - ro + 1.5), 2.5))
    # funnel: three posts under the cone
    pr = 40.0
    z_cone = L['z_funnel0'] + (pr - L['f_rh']) / L['f_k']
    for ang in (0, 90, 225):
        a = math.radians(ang)
        px, py = F[0] + pr * math.cos(a), F[1] + pr * math.sin(a)
        posts.append(tcyl((px, py, t - 0.5), (px, py, z_cone + 0.8), 2.5))
    # plinko: two posts under the board
    o, w = L['pl_origin'], L['pl_w']
    for py in (F[1] - 15, F[1] + 15):
        px = F[0] - 12
        # floor underside plane: w . (p - o) = -floor
        z = o[2] + (-PLINKO['floor'] - w[0] * (px - o[0]) - w[1] * (py - o[1])) / w[2]
        posts.append(tcyl((px, py, t - 0.5), (px, py, z + 1.5), 3.0))
    # chute: one post under its straight run
    cs = L['chute_samples']
    ctn = math.tan(math.radians(CHUTE['slope']))
    s_mid = None
    for (x, y, s) in cs:
        if abs(x - 32.0) < 5 and abs(y - (F[1] - CHUTE['entry_r'])) < 1:
            s_mid = s
            break
    if s_mid is not None:
        x, y = point_at(cs, s_mid)
        zc = L['chute_floor0'] + CHANNEL['id'] / 2 - s_mid * ctn
        posts.append(tcyl((x, y, t - 0.5), (x, y, zc - ro + 1.5), 3.0))
    # screw pivot pin
    pb = tilt_pt((S[0], S[1], L['z_screw0']))
    posts.append(tcyl((pb[0], pb[1], t - 0.5), (pb[0], pb[1], pb[2] + 4.0), 1.4))
    t_union(base, *posts)
    for m in modules:
        if m is not None:
            t_cut(base, tb.copy(m))
    return add_body(root, base, 'Base')


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------
def run_checks(root, L):
    names = [b.name for b in root.bRepBodies]
    bodies = {b.name: b for b in root.bRepBodies}
    log('--- interference check (mm3 of overlap; expect none) ---')
    found = False
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            vol = t_inter_volume(bodies[a], bodies[b])
            if vol > 0.5:
                found = True
                log('  OVERLAP %-13s x %-13s %8.1f' % (a, b, vol))
    if not found:
        log('  none')

    # marble path: a 7.6 mm "ghost marble" swept along track + chute centrelines
    log('--- marble path clearance (ghost marble r=3.8 along track and chute) ---')
    ghost = None
    tn = math.tan(math.radians(TRACK['slope']))
    off = CHANNEL['id'] / 2 - MARBLE_D / 2   # marble centre sits this far below the axis
    pts = [(x, y, L['z_track_start'] - s * tn - off) for (x, y, s) in L['track_samples']]
    ctn = math.tan(math.radians(CHUTE['slope']))
    cz0 = L['chute_floor0'] + CHANNEL['id'] / 2
    cpts = [(x, y, cz0 - s * ctn - off) for (x, y, s) in L['chute_samples']]
    for seq in (pts[:-2], cpts[2:]):
        for i in range(1, len(seq)):
            seg = tcyl(seq[i - 1], seq[i], 3.8)
            sph = tb.createSphere(P(*seq[i]), 3.8 * MM)
            t_union(seg, sph)
            ghost = seg if ghost is None else t_union(ghost, seg)
    found = False
    for n in names:
        if n == 'Marble':
            continue
        vol = t_inter_volume(ghost, bodies[n])
        if vol > 0.5:
            found = True
            log('  BLOCKED by %-13s %8.1f mm3' % (n, vol))
    if not found:
        log('  clear')

    # stacked laps: vertical gap between track passes that overlap in plan
    samp = L['track_samples']
    worst = 1e9
    for i in range(len(samp)):
        for j in range(i + 1, len(samp)):
            if samp[j][2] - samp[i][2] < 40:
                continue
            if math.hypot(samp[i][0] - samp[j][0], samp[i][1] - samp[j][1]) < CHANNEL['od']:
                dzv = (samp[j][2] - samp[i][2]) * tn
                worst = min(worst, dzv)
    log('Smallest vertical spacing between stacked track passes: %.1f mm (channel is %.1f mm tall)'
        % (worst, CHANNEL['od'] / 2 + CHANNEL['lip']))


def export_all(design, bodies):
    out = os.path.join(HERE, 'exports')
    stl_dir = os.path.join(out, 'stl')
    os.makedirs(stl_dir, exist_ok=True)
    for f in os.listdir(stl_dir):
        if f == 'HandCrank.stl':
            continue
        try:
            os.remove(os.path.join(stl_dir, f))
        except Exception:
            pass
    em = design.exportManager
    root = design.rootComponent
    try:
        em.execute(em.createSTEPExportOptions(os.path.join(out, 'marble_run.step'), root))
    except Exception:
        log('STEP export failed:\n' + traceback.format_exc())
    for b in bodies:
        if b is None:
            continue
        try:
            o = em.createSTLExportOptions(b, os.path.join(stl_dir, b.name + '.stl'))
            o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
            em.execute(o)
        except Exception:
            log('STL export failed for %s:\n%s' % (b.name, traceback.format_exc()))
    try:
        em.execute(em.createFusionArchiveExportOptions(os.path.join(out, 'marble_run.f3d'), root))
    except Exception:
        log('F3D export failed:\n' + traceback.format_exc())
    log('Exports written to ' + out)


def run(context):
    bodies = {}
    try:
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        design.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
        root = design.rootComponent
        L = layout()
        solve_track(L)

        steps = [('Funnel', lambda: build_funnel(root, L)),
                 ('Plinko', lambda: build_plinko(root, L)),
                 ('Track', lambda: build_track(root, L)),
                 ('Screw', lambda: build_screw(root, L)),
                 ('ScrewTower', lambda: build_tower(root, L)),
                 ('HandCrank', lambda: build_crank(root, L)),
                 ('ControlBox', lambda: build_control_box(root, L)),
                 ('Marble', lambda: build_marble(root, L))]
        for name, fn in steps:
            try:
                bodies[name] = fn()
                log('%s: OK' % name)
            except Exception:
                bodies[name] = None
                log('%s: FAILED\n%s' % (name, traceback.format_exc()))
        # the hand crank is a separate print: export it on its own, then take it out
        crank = root.bRepBodies.itemByName('HandCrank')
        if crank is not None:
            try:
                stl_dir = os.path.join(HERE, 'exports', 'stl')
                os.makedirs(stl_dir, exist_ok=True)
                em = design.exportManager
                o = em.createSTLExportOptions(crank, os.path.join(stl_dir, 'HandCrank.stl'))
                o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
                em.execute(o)
                log('HandCrank exported as a separate file')
            except Exception:
                log('HandCrank export failed:\n' + traceback.format_exc())
            root.features.removeFeatures.add(crank)
        # remove any leftover helper bodies
        for b in list(root.bRepBodies):
            if b.name.startswith('tmp_'):
                log('leftover helper body removed: ' + b.name)
                root.features.removeFeatures.add(b)
        try:
            mods = [root.bRepBodies.itemByName(n) for n in
                    ('Funnel', 'Plinko', 'Track', 'ScrewTower', 'Screw', 'ControlBox')]
            bodies['Base'] = build_base(root, L, mods)
            log('Base: OK')
        except Exception:
            bodies['Base'] = None
            log('Base: FAILED\n' + traceback.format_exc())

        log('--- alignment (mm) ---')
        for k in ('chute_floor0', 'chute_floor_end', 'z_lip_top', 'z_funnel_rim', 'z_funnel_exit',
                  'z_funnel0', 'marble_drop_to_cone', 'z_drop', 'z_track_start', 'z_track_end'):
            log('%-20s %7.2f' % (k, L[k]))
        log('chute length %.1f mm at %.1f deg; track length %.1f mm at %.1f deg; hairpin Y = %.1f'
            % (L['chute_len'], CHUTE['slope'], L['track_3d_len'], TRACK['slope'], L['track_Y']))
        for b in root.bRepBodies:
            bb = b.boundingBox
            log('body %-13s min(%.1f,%.1f,%.1f) max(%.1f,%.1f,%.1f) vol %.0f mm3' % (
                b.name, bb.minPoint.x / MM, bb.minPoint.y / MM, bb.minPoint.z / MM,
                bb.maxPoint.x / MM, bb.maxPoint.y / MM, bb.maxPoint.z / MM, b.volume * 1000))
        try:
            run_checks(root, L)
        except Exception:
            log('checks failed:\n' + traceback.format_exc())

        app.activeViewport.fit()
        export_all(design, [b for b in root.bRepBodies])
        failed = [k for k, v in bodies.items() if v is None]
        msg = 'Marble run built.' if not failed else 'Built with errors in: ' + ', '.join(failed)
    except Exception:
        log('FATAL\n' + traceback.format_exc())
        msg = 'Marble run script failed - see log.txt'
    finally:
        with open(os.path.join(HERE, 'log.txt'), 'w') as f:
            f.write('\n'.join(LOG))
    ui.messageBox(msg + '\n\nLog and exports are in:\n' + HERE)
