# Stylized CFM LEAP-1A high-bypass turbofan, built entirely with bpy.
# No downloaded models, textures, or HDRIs. No text, labels, or render stamp.
#
#   blender -b -P engine.py -- MODE VIEW [samples]
#
#   MODE  = preview | mid | still | setup
#   VIEW  = cut | ext | side     (side = ortho section of the cutaway)
#   still cut -> renders/cutaway.png
#   still ext -> renders/exterior.png
#   setup     -> engine.blend (scenes Exterior and Cutaway, modifiers live)
#
# Cut angle and blade counts are the constants just below. Rebuild to apply.

import bpy
import bmesh
import math
import os
import sys
import time
from mathutils import Vector, Matrix

# ------------------------------------------------------------------- figures
# Hard dimensions from public LEAP-1A data (see README). Internal radii are
# stylized so the bypass stays obviously larger than the core.
FAN_DIAMETER = 78 * 0.0254          # 1.9812 m, Safran: fan diameter 78 in
FAN_R = FAN_DIAMETER * 0.5
FLANGE_TO_TRF = 3.328               # m, EASA.E.110 flange to TRF aft flange
X0 = 0.0                            # fan-case forward flange
X_TRF = X0 + FLANGE_TO_TRF

# ------------------------------------------------------------------- knobs
CUT_DEG = 115.0                     # wedge removed, facing the cutaway camera
FAN_BLADES = 18                     # LEAP-1A fan blade count
OGV_BLADES = 32
LPC_ROTOR_N = (22, 26, 30)          # 3 booster stages
LPC_STATOR_N = (28, 32, 36)
HPC_STAGES = 10
HPC_ROTOR_N = 34
HPC_STATOR_N = 38
HPT_STAGES = 2
HPT_ROTOR_N = 40
HPT_STATOR_N = 34
LPT_STAGES = 7
LPT_ROTOR_N = 36
LPT_STATOR_N = 40
REAR_STRUTS = 8
FUEL_NOZZLES = 16
SPINNER_R = 0.300
LP_R = 0.086                        # LP shaft outer radius (stylized, readable)
HP_R_IN = 0.114
HP_R_OUT = 0.168

OUT = os.path.dirname(os.path.abspath(__file__))
if OUT in ('', '/'):
    OUT = '/workspace'

argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
MODE = argv[0] if argv else 'preview'
VIEW = argv[1] if len(argv) > 1 else 'cut'
SAMPLE_ARG = int(argv[2]) if len(argv) > 2 and argv[2].isdigit() else None

for sub in ('renders', 'previews', 'logs'):
    os.makedirs(os.path.join(OUT, sub), exist_ok=True)

LOG_LINES = []

def log(msg):
    print(msg, flush=True)
    LOG_LINES.append(str(msg))

# ------------------------------------------------------------------- scene
bpy.ops.wm.read_factory_settings(use_empty=True)
scene_ext = bpy.context.scene
scene_ext.name = 'Exterior'
scene_cut = bpy.data.scenes.new('Cutaway')


def set_scene(scene):
    bpy.context.window.scene = scene


def new_collection(name):
    col = bpy.data.collections.new(name)
    return col


def link_cols(scene, cols):
    for c in cols:
        if c.name not in [ch.name for ch in scene.collection.children]:
            scene.collection.children.link(c)


def new_mat(name, color=(0.5, 0.5, 0.5, 1), rough=0.5, metal=0.0, emit=None, estr=0.0, **kw):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes['Principled BSDF']
    b.inputs['Base Color'].default_value = color
    b.inputs['Roughness'].default_value = rough
    b.inputs['Metallic'].default_value = metal
    if emit is not None:
        b.inputs['Emission Color'].default_value = emit
        b.inputs['Emission Strength'].default_value = estr
    for k, v in kw.items():
        key = k.replace('_', ' ').title().replace('Ior', 'IOR')
        b.inputs[key].default_value = v
    return m


def link_obj(obj, coll):
    coll.objects.link(obj)
    return obj


def mesh_obj(name, bm, mats, coll, smooth=True):
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    if smooth:
        for p in me.polygons:
            p.use_smooth = True
    obj = bpy.data.objects.new(name, me)
    for m in mats:
        if m is not None:
            obj.data.materials.append(m)
    return link_obj(obj, coll)


def edge_split(obj, angle_deg=52.0):
    mod = obj.modifiers.new('sharp', 'EDGE_SPLIT')
    mod.split_angle = math.radians(angle_deg)
    mod.use_edge_angle = True
    return mod


def lerp(a, b, t):
    return a + (b - a) * t


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def ang_of(y, z):
    """Degrees, 0 at +Z, positive toward -Y (right-hand about +X)."""
    return math.degrees(math.atan2(-y, z)) % 360.0


def clean_profile(pts):
    out = []
    for x, r in pts:
        p = (float(x), max(float(r), 0.0006))
        if out and abs(out[-1][0] - p[0]) < 1e-8 and abs(out[-1][1] - p[1]) < 1e-8:
            continue
        out.append(p)
    if len(out) >= 2 and abs(out[0][0] - out[-1][0]) < 1e-8 and abs(out[0][1] - out[-1][1]) < 1e-8:
        out = out[:-1]
    if len(out) < 3:
        raise RuntimeError('profile has < 3 points: %s' % (pts[:4],))
    return out


def walk_boundary_loops(bm):
    unused = {e for e in bm.edges if e.is_boundary}
    loops = []
    while unused:
        e0 = unused.pop()
        loop = [e0]
        start = e0.verts[0]
        v = e0.verts[1]
        guard = 0
        while v != start and guard < 100000:
            guard += 1
            nxt = None
            for e in v.link_edges:
                if e in unused and e.is_boundary:
                    nxt = e
                    break
            if nxt is None:
                break
            unused.discard(nxt)
            loop.append(nxt)
            v = nxt.other_vert(v)
        loops.append(loop)
    return loops


def revolve(name, profile, coll, mat, cap_mat=None, full=False, full_steps=72):
    """Spin a closed (x, r) profile about +X.

    Partial sweeps start at +Z and run (360 - CUT) degrees, then the object
    is rotated so the open wedge is [0, CUT_DEG] (upper-left, toward the camera).
    End caps get cap_mat. Full spins merge the seam and have no caps.
    """
    pts = clean_profile(profile)
    bm = bmesh.new()
    verts = [bm.verts.new((x, 0.0, r)) for x, r in pts]
    for i in range(len(verts)):
        bm.edges.new((verts[i], verts[(i + 1) % len(verts)]))
    geom = list(bm.verts) + list(bm.edges)
    if full:
        angle = math.tau
        steps = full_steps
    else:
        angle = math.radians(360.0 - CUT_DEG)
        steps = max(10, int(round(full_steps * (360.0 - CUT_DEG) / 360.0)))
    bmesh.ops.spin(
        bm,
        geom=geom,
        cent=Vector((0.0, 0.0, 0.0)),
        axis=Vector((1.0, 0.0, 0.0)),
        dvec=Vector((0.0, 0.0, 0.0)),
        angle=angle,
        steps=steps,
        use_merge=bool(full),
        use_normal_flip=False,
        use_duplicate=False,
    )
    if full:
        bmesh.ops.remove_doubles(bm, verts=list(bm.verts), dist=1e-5)
    else:
        bm.edges.ensure_lookup_table()
        loops = walk_boundary_loops(bm)
        if len(loops) < 2:
            log('WARNING %s boundary loops=%d' % (name, len(loops)))
        for loop in loops:
            existing = set(bm.faces)
            try:
                bmesh.ops.triangle_fill(bm, edges=loop, use_beauty=True, use_dissolve=False)
            except Exception as ex:
                log('WARNING fill %s: %s' % (name, ex))
                continue
            for f in bm.faces:
                if f not in existing:
                    f.material_index = 1
                    f.smooth = False
    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    for f in bm.faces:
        if f.material_index == 0:
            f.smooth = True
    nm = sum(1 for e in bm.edges if not e.is_manifold)
    if nm:
        log('WARNING %s nonmanifold edges %d' % (name, nm))
    mats = [mat] if full or cap_mat is None else [mat, cap_mat]
    obj = mesh_obj(name, bm, mats, coll, smooth=False)
    if not full:
        obj.rotation_euler[0] = math.radians(CUT_DEG)
    edge_split(obj, 50.0 if full else 46.0)
    return obj


def arc_pts(cx, cy, rad, a0, a1, n):
    pts = []
    for i in range(n + 1):
        a = a0 + (a1 - a0) * (i / max(1, n))
        pts.append((cx + rad * math.cos(a), cy + rad * math.sin(a)))
    return pts


def wall(x0, x1, ri0, ri1, ro0, ro1):
    """Closed wall profile, inner front->back, outer back->front."""
    return [(x0, ri0), (x1, ri1), (x1, ro1), (x0, ro0)]


def loft_wall(stations):
    """stations: (x, r_inner, r_outer) front to back."""
    inner = [(x, ri) for x, ri, ro in stations]
    outer = [(x, ro) for x, ri, ro in stations]
    return inner + list(reversed(outer))


# ------------------------------------------------------------------- blades
def airfoil_pts(n, chord, thick, camber):
    half = max(3, n // 2)
    upper = []
    for i in range(half):
        ang = math.pi * i / (half - 1)
        x = -0.5 * chord * math.cos(ang)
        xn = x / (0.5 * chord) if chord else 0.0
        yt = math.sqrt(max(0.0, 1.0 - xn * xn)) * (0.5 * thick)
        s = (x + 0.5 * chord) / chord if chord else 0.0
        cam = camber * math.sin(math.pi * clamp(s, 0.0, 1.0))
        upper.append((x, cam + yt))
    pts = list(upper)
    for i in range(half - 2, 0, -1):
        ang = math.pi * i / (half - 1)
        x = -0.5 * chord * math.cos(ang)
        xn = x / (0.5 * chord) if chord else 0.0
        yt = math.sqrt(max(0.0, 1.0 - xn * xn)) * (0.5 * thick)
        s = (x + 0.5 * chord) / chord if chord else 0.0
        cam = camber * math.sin(math.pi * clamp(s, 0.0, 1.0))
        pts.append((x, cam - yt))
    return pts


def _span(v, t):
    if isinstance(v, (tuple, list)):
        return lerp(float(v[0]), float(v[1]), t)
    return float(v)


def make_blade(x, r_hub, r_tip, spec):
    """One blade, root at +Z, origin on the engine axis. Local +X is aft."""
    bm = bmesh.new()
    n_span = int(spec.get('n_span', 4))
    n_around = int(spec.get('n_around', 10))
    boost = float(spec.get('chord_boost', 0.0))
    rings = []
    span = max(1e-4, r_tip - r_hub)
    for i in range(n_span):
        t = i / (n_span - 1) if n_span > 1 else 0.0
        r = r_hub + span * t
        chord = _span(spec['chord'], t) + boost * math.sin(math.pi * t)
        thick = max(0.0035, _span(spec['thick'], t))
        tw = math.radians(_span(spec['twist'], t))
        cam = _span(spec['camber'], t)
        sweep = _span(spec.get('sweep', 0.0), t) * (t ** 1.35)
        lean = _span(spec.get('lean', 0.0), t) * (t ** 1.2)
        ring = []
        for dx, dy in airfoil_pts(n_around, chord, thick, cam):
            xr = dx * math.cos(tw) - dy * math.sin(tw)
            yr = dx * math.sin(tw) + dy * math.cos(tw)
            ring.append(bm.verts.new((x + xr + sweep, yr + lean, r)))
        rings.append(ring)
    m = len(rings[0])
    for i in range(n_span - 1):
        for j in range(m):
            j2 = (j + 1) % m
            try:
                bm.faces.new((rings[i][j], rings[i][j2], rings[i + 1][j2], rings[i + 1][j]))
            except ValueError:
                pass
    try:
        bm.faces.new(list(reversed(rings[0])))
    except ValueError:
        pass
    try:
        bm.faces.new(rings[-1])
    except ValueError:
        pass
    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    for f in bm.faces:
        f.smooth = True
    return bm


def blade_row(name, coll, rig, mat, x, r_hub, r_tip, n, spec, full=False):
    """Loft one blade and array it about +X. Partial rows leave the cut wedge empty."""
    if r_tip <= r_hub + 0.004 or n < 1:
        log('SKIP %s degenerate span' % name)
        return None
    margin = 0.0
    if not full:
        chord = _span(spec['chord'], 0.0)
        margin = math.degrees(math.atan2(max(chord, 0.02) * 0.55, max(r_hub, 0.08))) + 1.5
        step = 360.0 / n
        i0 = int(math.ceil((CUT_DEG + margin) / step - 1e-9))
        max_i = int(math.floor((360.0 - margin) / step + 1e-9))
        max_i = min(max_i, n - 1)
        count = max(0, max_i - i0 + 1)
        if count < 1:
            log('SKIP %s no blades outside cut' % name)
            return None
    else:
        step = 360.0 / n
        i0 = 0
        count = n
    bm = make_blade(x, r_hub, r_tip, spec)
    if i0:
        bmesh.ops.rotate(
            bm,
            verts=list(bm.verts),
            cent=Vector((0.0, 0.0, 0.0)),
            matrix=Matrix.Rotation(math.radians(i0 * step), 4, 'X'),
        )
    obj = mesh_obj(name, bm, [mat], coll)
    pivot = bpy.data.objects.new(name + '_pivot', None)
    pivot.location = (0.0, 0.0, 0.0)
    pivot.rotation_euler = (math.radians(step), 0.0, 0.0)
    pivot.empty_display_size = 0.15
    link_obj(pivot, rig)
    if spec.get('subsurf'):
        ss = obj.modifiers.new('sub', 'SUBSURF')
        ss.levels = 1
        ss.render_levels = 1
    arr = obj.modifiers.new('array', 'ARRAY')
    arr.use_relative_offset = False
    arr.use_constant_offset = False
    arr.use_object_offset = True
    arr.offset_object = pivot
    arr.count = count
    if not spec.get('subsurf'):
        edge_split(obj, 62.0)
    log('ROW %-18s n=%3d kept=%3d  r=%.3f..%.3f' % (name, n, count, r_hub, r_tip))
    return obj


# ------------------------------------------------------------------- materials
M = {}
# White nacelle paint, restrained product finish.
M['nacelle'] = new_mat('nacelle', (0.73, 0.74, 0.75, 1), 0.34, 0.0, coat_weight=0.28, coat_roughness=0.22)
# Cold section titanium.
M['ti'] = new_mat('ti', (0.52, 0.54, 0.56, 1), 0.33, 1.0)
M['ti_dark'] = new_mat('ti_dark', (0.34, 0.36, 0.38, 1), 0.38, 1.0)
M['ti_lpc'] = new_mat('ti_lpc', (0.62, 0.64, 0.66, 1), 0.32, 1.0)
# LEAP fan blades are 3D-woven composite: dark graphite, still in the cold family.
M['fan'] = new_mat('fan_blade', (0.105, 0.11, 0.115, 1), 0.48, 0.22)
# Hot section bronze, with a lighter HPT and a darker exhaust.
M['bronze'] = new_mat('bronze', (0.52, 0.32, 0.15, 1), 0.36, 1.0)
M['bronze_dark'] = new_mat('bronze_dark', (0.36, 0.21, 0.10, 1), 0.40, 1.0)
M['hpt'] = new_mat('hpt', (0.64, 0.42, 0.18, 1), 0.32, 1.0)
M['exhaust'] = new_mat('exhaust', (0.30, 0.17, 0.09, 1), 0.38, 1.0)
# Bypass duct wall, lighter than the core so the annulus reads.
M['bypass'] = new_mat('bypass', (0.66, 0.68, 0.70, 1), 0.40, 0.72)
# Shafts: one color each, neither titanium nor bronze.
# Shaft colors have to survive a dark studio. Fully metallic blue just
# mirrors the white nacelle, so these stay partly diffuse and a little emissive.
M['lp'] = new_mat('lp_shaft', (0.03, 0.16, 0.78, 1), 0.40, 0.25,
                  emit=(0.02, 0.12, 0.70, 1), estr=0.70)
M['hp'] = new_mat('hp_shaft', (0.90, 0.26, 0.02, 1), 0.38, 0.35,
                  emit=(0.70, 0.14, 0.0, 1), estr=0.45)
M['steel'] = new_mat('steel', (0.20, 0.21, 0.23, 1), 0.34, 1.0)
# Cut faces. A little emission so the red still reads in the wedge shadow.
M['cut'] = new_mat('cut_face', (0.62, 0.025, 0.018, 1), 0.55, 0.0,
                   emit=(0.55, 0.02, 0.012, 1), estr=0.12)
M['flame'] = new_mat('flame', (0.35, 0.06, 0.01, 1), 0.55, 0.0,
                     emit=(1.0, 0.30, 0.04, 1), estr=4.5)
M['studio'] = new_mat('studio', (0.012, 0.013, 0.015, 1), 0.88, 0.0)
M['pylon'] = new_mat('pylon', (0.66, 0.67, 0.68, 1), 0.40, 0.04, coat_weight=0.15)


def spinner_material():
    """White metallic spinner with a procedural spiral stripe. No image textures."""
    m = bpy.data.materials.new('spinner')
    m.use_nodes = True
    nt = m.node_tree
    b = nt.nodes['Principled BSDF']
    b.inputs['Metallic'].default_value = 0.85
    b.inputs['Roughness'].default_value = 0.34
    try:
        tc = nt.nodes.new('ShaderNodeTexCoord')
        sep = nt.nodes.new('ShaderNodeSeparateXYZ')
        atan = nt.nodes.new('ShaderNodeMath')
        atan.operation = 'ARCTAN2'
        scale = nt.nodes.new('ShaderNodeMath')
        scale.operation = 'MULTIPLY'
        scale.inputs[1].default_value = 1.0 / (2.0 * math.pi)
        pitch = nt.nodes.new('ShaderNodeMath')
        pitch.operation = 'MULTIPLY_ADD'
        pitch.inputs[1].default_value = 2.4
        pitch.inputs[2].default_value = 0.0
        add = nt.nodes.new('ShaderNodeMath')
        add.operation = 'ADD'
        fract = nt.nodes.new('ShaderNodeMath')
        fract.operation = 'FRACT'
        lt = nt.nodes.new('ShaderNodeMath')
        lt.operation = 'LESS_THAN'
        lt.inputs[1].default_value = 0.16
        mix = nt.nodes.new('ShaderNodeMix')
        mix.data_type = 'RGBA'
        mix.inputs[6].default_value = (0.78, 0.79, 0.80, 1)
        mix.inputs[7].default_value = (0.10, 0.105, 0.11, 1)
        nt.links.new(tc.outputs['Object'], sep.inputs['Vector'])
        nt.links.new(sep.outputs['Y'], atan.inputs[0])
        nt.links.new(sep.outputs['Z'], atan.inputs[1])
        nt.links.new(atan.outputs['Value'], scale.inputs[0])
        nt.links.new(sep.outputs['X'], pitch.inputs[0])
        nt.links.new(scale.outputs['Value'], add.inputs[0])
        nt.links.new(pitch.outputs['Value'], add.inputs[1])
        nt.links.new(add.outputs['Value'], fract.inputs[0])
        nt.links.new(fract.outputs['Value'], lt.inputs[0])
        nt.links.new(lt.outputs['Value'], mix.inputs[0])
        nt.links.new(mix.outputs[2], b.inputs['Base Color'])
    except Exception as ex:
        log('spinner spiral skipped: %s' % ex)
        b.inputs['Base Color'].default_value = (0.75, 0.76, 0.77, 1)
    return m


M['spinner'] = spinner_material()

# ------------------------------------------------------------------- collections
# Interiors are built once, cut, and linked to both scenes. Shells that the
# exterior camera can see exist as a full ring on Exterior and a cut ring
# on Cutaway. Fan blades are a full ring in both.
COLS = {}


def C(key):
    if key not in COLS:
        COLS[key] = new_collection(key)
    return COLS[key]


RIG = C('00_Rig')
SHAFTS = C('01_Shafts')
FAN = C('02_Fan')
LPC = C('03_LPC')
HPC = C('04_HPC')
COMB = C('05_Combustor')
HPT = C('06_HPT')
LPT = C('07_LPT')
EXH_CUT = C('08_Exhaust_Cut')
EXH_FULL = C('08_Exhaust_Full')
BYP_CUT = C('09_Bypass_Cut')
BYP_FULL = C('09_Bypass_Full')
NAC_CUT = C('10_Nacelle_Cut')
NAC_FULL = C('10_Nacelle_Full')
STUDIO = C('11_Studio')
L_CUT = C('12_Lights_Cut')
L_EXT = C('12_Lights_Ext')

SHARED = [RIG, SHAFTS, FAN, LPC, HPC, COMB, HPT, LPT, STUDIO]
link_cols(scene_ext, SHARED + [EXH_FULL, BYP_FULL, NAC_FULL, L_EXT])
link_cols(scene_cut, SHARED + [EXH_CUT, BYP_CUT, NAC_CUT, L_CUT])

# ------------------------------------------------------------------- aero specs
FAN_SPEC = dict(chord=(0.26, 0.34), chord_boost=0.12, thick=(0.052, 0.012),
                twist=(18.0, 58.0), camber=(0.040, 0.010), sweep=(0.0, -0.16),
                lean=(0.0, 0.035), n_span=8, n_around=16, subsurf=True)
LPC_ROTOR = dict(chord=0.064, thick=0.010, twist=(26.0, 46.0), camber=0.012,
                 n_span=4, n_around=10)
LPC_STATOR = dict(chord=0.058, thick=0.008, twist=(-42.0, -18.0), camber=-0.010,
                  n_span=4, n_around=10)
HPC_ROTOR = dict(chord=0.034, thick=0.0062, twist=(30.0, 50.0), camber=0.006,
                 n_span=3, n_around=10)
HPC_STATOR = dict(chord=0.030, thick=0.0052, twist=(-46.0, -22.0), camber=-0.005,
                  n_span=3, n_around=10)
HPT_ROTOR = dict(chord=0.050, thick=0.015, twist=(34.0, 56.0), camber=0.013,
                 n_span=4, n_around=12)
HPT_STATOR = dict(chord=0.046, thick=0.012, twist=(-52.0, -28.0), camber=-0.012,
                  n_span=4, n_around=12)
LPT_ROTOR = dict(chord=0.036, thick=(0.010, 0.006), twist=(28.0, 48.0), camber=0.008,
                 n_span=5, n_around=10)
LPT_STATOR = dict(chord=0.032, thick=0.006, twist=(-46.0, -20.0), camber=-0.007,
                  n_span=4, n_around=10)
OGV_SPEC = dict(chord=0.11, thick=0.012, twist=(-16.0, -8.0), camber=-0.006,
                n_span=4, n_around=10)
STRUT_SPEC = dict(chord=0.075, thick=0.020, twist=(0.0, 0.0), camber=0.0,
                  n_span=3, n_around=8)

# ------------------------------------------------------------------- parts
def add_disk(name, coll, mat, x, r_bore, r_rim, full=False, web=0.016, rim=0.026, steps=56):
    r_bore = max(r_bore, 0.012)
    r_mid = max(r_bore + 0.01, r_rim * 0.62)
    profile = [
        (x - web, r_bore),
        (x - web * 0.55, r_mid),
        (x - rim, r_rim * 0.94),
        (x - rim, r_rim),
        (x + rim, r_rim),
        (x + rim, r_rim * 0.94),
        (x + web * 0.55, r_mid),
        (x + web, r_bore),
    ]
    return revolve(name, profile, coll, mat, M['cut'], full=full, full_steps=steps)


def add_shaft_solid(name, coll, mat, x0, x1, radius, full=False):
    profile = [
        (x0, 0.001),
        (x1, 0.001),
        (x1, radius),
        (x0, radius),
    ]
    return revolve(name, profile, coll, mat, M['cut'], full=full, full_steps=48)


def add_tube(name, coll, mat, x0, x1, r_in, r_out, full=False, steps=56):
    return revolve(name, wall(x0, x1, r_in, r_in, r_out, r_out), coll, mat, M['cut'],
                   full=full, full_steps=steps)


def ellipse_solid(cx, rx, ry, rz, n=20):
    pts = []
    for i in range(n):
        a = 2.0 * math.pi * i / n
        pts.append((cx + rx * math.cos(a), ry + rz * math.sin(a)))
    return pts


def stage_xs(x0, x1, n, s_frac=0.24, r_frac=0.70):
    pitch = (x1 - x0) / n
    stators, rotors = [], []
    for i in range(n):
        stators.append(x0 + (i + s_frac) * pitch)
        rotors.append(x0 + (i + r_frac) * pitch)
    return stators, rotors


def build_shafts():
    # LP shaft runs fan disk to the back of the LPT. HP shaft is the orange
    # tube around it, only through the HP spool.
    add_shaft_solid('lp_shaft', SHAFTS, M['lp'], 0.16, 3.40, LP_R, full=False)
    add_tube('hp_shaft', SHAFTS, M['hp'], 1.08, 2.72, HP_R_IN, HP_R_OUT, full=False, steps=48)
    # Bearing rings (small, dark steel) so the shafts read as supported.
    for name, x, r0, r1 in (
        ('brg_fan', 0.46, LP_R - 0.002, LP_R + 0.045),
        ('brg_hp_front', 1.10, HP_R_OUT - 0.002, HP_R_OUT + 0.040),
        ('brg_hp_rear', 2.66, HP_R_OUT - 0.002, HP_R_OUT + 0.040),
        ('brg_lp_rear', 3.22, LP_R - 0.002, LP_R + 0.045),
    ):
        add_tube(name, SHAFTS, M['steel'], x - 0.012, x + 0.012, r0, r1, full=False, steps=40)


def build_fan():
    # Full ring of fan blades in both scenes. Disk and case are per scene
    # so the exterior inlet does not show a red cut.
    blade_row('fan_blades', FAN, RIG, M['fan'], 0.30, SPINNER_R - 0.006, FAN_R,
              FAN_BLADES, FAN_SPEC, full=True)
    for coll, full, tag in ((NAC_FULL, True, 'full'), (NAC_CUT, False, 'cut')):
        add_disk('fan_disk_' + tag, coll, M['ti'], 0.30, LP_R - 0.003, SPINNER_R - 0.004,
                 full=full, web=0.022, rim=0.055, steps=72)
        revolve(
            'fan_case_' + tag,
            wall(0.08, 0.76, FAN_R + 0.006, FAN_R + 0.004, FAN_R + 0.070, FAN_R + 0.064),
            coll, M['ti'], M['cut'], full=full, full_steps=112,
        )
        revolve('spinner_' + tag, spinner_profile(), coll, M['spinner'], M['cut'],
                full=full, full_steps=72)


def spinner_profile():
    x_tip, x_base, r_base = -0.12, 0.20, SPINNER_R
    outer = []
    n = 16
    for i in range(n + 1):
        t = i / n
        x = lerp(x_tip, x_base, t)
        r = r_base * (math.sin(t * math.pi * 0.5) ** 0.82)
        outer.append((x, max(r, 0.001)))
    # Axis tip -> axis base -> outer base back to the tip.
    return [(x_tip, 0.001), (x_base, 0.001)] + list(reversed(outer[1:]))


def build_nacelle(coll, full, tag):
    lip_cx, lip_cy, lip_r = -0.36, 1.155, 0.080
    outer_a = math.radians(52.0)
    inner_a = math.radians(292.0)  # around the highlight, +X math angles
    lip = arc_pts(lip_cx, lip_cy, lip_r, outer_a, inner_a, 16)
    inner_start = lip[-1]
    shoulder = lip[0]
    inner = [
        inner_start,
        (-0.08, 1.045),
        (0.05, 1.012),
        (0.09, 1.100),   # step out over the fan case
        (0.80, 1.100),
        (0.86, 1.020),   # back onto the bypass outer wall
        (1.55, 1.008),
        (2.35, 0.972),
        (2.95, 0.918),
        (3.16, 0.860),
    ]
    outer = [
        (3.16, 0.945),
        (2.95, 1.020),
        (2.20, 1.125),
        (1.20, 1.225),
        (0.40, 1.250),
        (-0.02, 1.228),
        shoulder,
    ]
    # Shallow cowl splits so the white shell reads as panels, not a blimp.
    def groove(pts, xg, depth, width):
        out = []
        for i, (x, r) in enumerate(pts):
            out.append((x, r))
            if i < len(pts) - 1 and pts[i][0] < xg <= pts[i + 1][0] or pts[i][0] > xg >= pts[i + 1][0]:
                pass
        return out
    profile = inner + outer + lip[1:-1]
    revolve('nacelle_' + tag, profile, coll, M['nacelle'], M['cut'], full=full, full_steps=128)
    # Pylon stub is exterior only (P2). It sits on the crown, outside the cut.
    if full:
        build_pylon(coll)


def build_pylon(coll):
    bm = bmesh.new()
    rings = []
    specs = ((1.14, 1.00, 0.20), (1.65, 0.72, 0.14), (2.28, 0.46, 0.09))
    x_c = 1.05
    for z, chord, thick in specs:
        ring = []
        for dx, dy in airfoil_pts(12, chord, thick, 0.0):
            ring.append(bm.verts.new((x_c + dx * 0.15 + dx, dy, z)))
        rings.append(ring)
    m = len(rings[0])
    for i in range(len(rings) - 1):
        for j in range(m):
            j2 = (j + 1) % m
            try:
                bm.faces.new((rings[i][j], rings[i][j2], rings[i + 1][j2], rings[i + 1][j]))
            except ValueError:
                pass
    try:
        bm.faces.new(list(reversed(rings[0])))
        bm.faces.new(rings[-1])
    except ValueError:
        pass
    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    obj = mesh_obj('pylon', bm, [M['pylon']], coll)
    edge_split(obj, 55.0)


def build_cowl(coll, full, tag):
    # Core cowl: inner wall of the bypass duct. Nose is the splitter.
    nose_cx, nose_cy, nose_r = 0.50, 0.455, 0.055
    nose = arc_pts(nose_cx, nose_cy, nose_r, math.radians(200.0), math.radians(20.0), 10)
    # arc from aft-inner-ish to forward. We'll assemble explicitly.
    outer = [  # bypass side, front to back
        (0.52, 0.445),
        (0.70, 0.500),
        (1.15, 0.545),
        (1.80, 0.575),
        (2.30, 0.590),
        (2.75, 0.610),
        (3.08, 0.640),
    ]
    inner = [  # core side, front to back (reversed when walking back)
        (0.62, 0.470),
        (1.10, 0.478),
        (1.70, 0.470),
        (2.15, 0.545),   # clears the combustor bulge
        (2.55, 0.520),
        (3.02, 0.600),
    ]
    nose = arc_pts(0.55, 0.470, 0.048, math.radians(110.0), math.radians(250.0), 8)
    # nose from outer-front (110 deg) around the front (180) to inner (250)
    profile = [nose[0]] + outer + list(reversed(inner)) + list(reversed(nose[1:]))
    revolve('core_cowl_' + tag, profile, coll, M['bypass'], M['cut'], full=full, full_steps=96)


def build_ogv(coll, full, tag):
    blade_row('ogv_' + tag, coll, RIG, M['ti_dark'], 0.66, 0.505, FAN_R - 0.010,
              OGV_BLADES, OGV_SPEC, full=full)


def build_lpc():
    # 3-stage booster tucked behind the fan, inside the splitter.
    x0, x1 = 0.50, 1.00
    stators, rotors = stage_xs(x0, x1, 3, 0.22, 0.68)
    r_case0, r_case1 = 0.418, 0.412
    add_tube('lpc_case', LPC, M['ti_lpc'], x0 - 0.02, x1 + 0.02,
             r_case0, r_case0 + 0.030, full=False, steps=64)
    # taper by a second thin skin? one straight case is readable; radii above fit the blades.
    hubs = (0.292, 0.305, 0.318)
    for i, (xs, xr, hub) in enumerate(zip(stators, rotors, hubs)):
        tip = lerp(r_case0, r_case1, (xr - x0) / (x1 - x0))
        blade_row('lpc_s%d' % (i + 1), LPC, RIG, M['ti_dark'], xs,
                  hub + 0.012, tip + 0.004, LPC_STATOR_N[i], LPC_STATOR, full=False)
        blade_row('lpc_r%d' % (i + 1), LPC, RIG, M['ti_lpc'], xr,
                  hub - 0.008, tip - 0.007, LPC_ROTOR_N[i], LPC_ROTOR, full=False)
        add_disk('lpc_disk%d' % (i + 1), LPC, M['ti_lpc'], xr, LP_R - 0.003, hub - 0.004,
                 full=False, web=0.012, rim=0.020, steps=48)


def build_hpc():
    x0, x1 = 1.14, 1.98
    n = HPC_STAGES
    stators, rotors = stage_xs(x0, x1, n, 0.22, 0.72)
    # Annulus closes a little toward the back, like a real HPC.
    def hub_tip(i):
        t = i / (n - 1)
        return lerp(0.292, 0.334, t), lerp(0.402, 0.364, t)
    r0 = hub_tip(0)[1] + 0.008
    r1 = hub_tip(n - 1)[1] + 0.008
    revolve('hpc_case', loft_wall([
        (x0 - 0.02, r0, r0 + 0.026),
        (lerp(x0, x1, 0.5), lerp(r0, r1, 0.5), lerp(r0, r1, 0.5) + 0.026),
        (x1 + 0.02, r1, r1 + 0.026),
    ]), HPC, M['ti'], M['cut'], full=False, full_steps=72)
    for i in range(n):
        hub, tip = hub_tip(i)
        blade_row('hpc_s%02d' % (i + 1), HPC, RIG, M['ti_dark'], stators[i],
                  hub + 0.010, tip + 0.004, HPC_STATOR_N, HPC_STATOR, full=False)
        blade_row('hpc_r%02d' % (i + 1), HPC, RIG, M['ti'], rotors[i],
                  hub - 0.006, tip - 0.006, HPC_ROTOR_N, HPC_ROTOR, full=False)
        add_disk('hpc_disk%02d' % (i + 1), HPC, M['ti'], rotors[i],
                 HP_R_OUT - 0.003, hub - 0.003, full=False, web=0.008, rim=0.012, steps=40)


def build_combustor():
    # Annular chamber: outer case bulge, two liners, glowing ring, nozzles.
    x0, x1 = 2.04, 2.42
    revolve('combustor_case', loft_wall([
        (x0, 0.455, 0.488),
        (2.14, 0.478, 0.515),
        (2.24, 0.486, 0.528),
        (2.34, 0.470, 0.508),
        (x1, 0.448, 0.482),
    ]), COMB, M['bronze'], M['cut'], full=False, full_steps=64)
    revolve('combustor_inner_liner', loft_wall([
        (2.08, 0.248, 0.268),
        (2.22, 0.255, 0.276),
        (2.36, 0.262, 0.282),
    ]), COMB, M['bronze_dark'], M['cut'], full=False, full_steps=48)
    revolve('combustor_outer_liner', loft_wall([
        (2.08, 0.400, 0.422),
        (2.22, 0.418, 0.442),
        (2.36, 0.392, 0.414),
    ]), COMB, M['bronze'], M['cut'], full=False, full_steps=48)
    # Front dome, a radial ring the nozzles pass through.
    add_tube('combustor_dome', COMB, M['bronze_dark'], 2.055, 2.078, 0.262, 0.430,
             full=False, steps=48)
    # Annulus between the liners (inner outer ~0.28, outer inner ~0.40).
    revolve('flame', ellipse_solid(2.22, 0.090, 0.338, 0.042, 18),
            COMB, M['flame'], M['flame'], full=False, full_steps=48)
    build_nozzles()


def build_nozzles():
    bm = bmesh.new()
    step = 360.0 / FUEL_NOZZLES
    made = 0
    for i in range(FUEL_NOZZLES):
        a = i * step
        if a < CUT_DEG + 6.0 or a > 360.0 - 6.0:
            continue
        th = math.radians(a)
        # From outside the case inward into the dome.
        p_out = Vector((2.066, -math.sin(th) * 0.56, math.cos(th) * 0.56))
        p_in = Vector((2.066, -math.sin(th) * 0.40, math.cos(th) * 0.40))
        _cone_between(bm, p_out, p_in, 0.012, 0.012, 8)
        head = p_in.lerp(p_out, 0.15)
        _cone_between(bm, head, head + (p_in - p_out).normalized() * 0.02, 0.020, 0.010, 8)
        made += 1
    if not bm.verts:
        bm.free()
        return
    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    obj = mesh_obj('fuel_nozzles', bm, [M['bronze']], COMB)
    edge_split(obj, 50.0)
    log('ROW fuel_nozzles       kept=%d' % made)


def _cone_between(bm, p0, p1, r0, r1, seg):
    d = p1 - p0
    length = d.length
    if length < 1e-6:
        return
    res = bmesh.ops.create_cone(
        bm, cap_ends=True, cap_tris=True, segments=seg,
        radius1=r0, radius2=r1, depth=length,
    )
    quat = d.normalized().to_track_quat('Z', 'Y')
    bmesh.ops.rotate(bm, verts=res['verts'], cent=Vector((0, 0, 0)), matrix=quat.to_matrix().to_4x4())
    bmesh.ops.translate(bm, verts=res['verts'], vec=(p0 + p1) * 0.5)


def build_hpt():
    x0, x1 = 2.48, 2.66
    n = HPT_STAGES
    stators, rotors = stage_xs(x0, x1, n, 0.22, 0.70)
    hubs = (0.312, 0.328)
    tips = (0.402, 0.418)
    revolve('hpt_case', loft_wall([
        (x0 - 0.015, tips[0] + 0.006, tips[0] + 0.034),
        (x1 + 0.015, tips[1] + 0.006, tips[1] + 0.034),
    ]), HPT, M['hpt'], M['cut'], full=False, full_steps=56)
    for i in range(n):
        blade_row('hpt_s%d' % (i + 1), HPT, RIG, M['bronze_dark'], stators[i],
                  hubs[i] + 0.010, tips[i] + 0.004, HPT_STATOR_N, HPT_STATOR, full=False)
        blade_row('hpt_r%d' % (i + 1), HPT, RIG, M['hpt'], rotors[i],
                  hubs[i] - 0.006, tips[i] - 0.006, HPT_ROTOR_N, HPT_ROTOR, full=False)
        add_disk('hpt_disk%d' % (i + 1), HPT, M['hpt'], rotors[i],
                 HP_R_OUT - 0.003, hubs[i] - 0.003, full=False, web=0.012, rim=0.020, steps=44)


def build_lpt():
    x0, x1 = 2.74, 3.22
    n = LPT_STAGES
    stators, rotors = stage_xs(x0, x1, n, 0.20, 0.68)
    def hub_tip(i):
        t = i / (n - 1)
        return lerp(0.308, 0.355, t), lerp(0.440, 0.590, t)
    stations = []
    for i in range(n):
        t = i / (n - 1)
        x = lerp(x0, x1, t)
        hub, tip = hub_tip(i)
        stations.append((x, tip + 0.006, tip + 0.030))
    stations[0] = (x0 - 0.02, stations[0][1], stations[0][2])
    stations[-1] = (x1 + 0.02, stations[-1][1], stations[-1][2])
    revolve('lpt_case', loft_wall(stations), LPT, M['bronze'], M['cut'], full=False, full_steps=72)
    for i in range(n):
        hub, tip = hub_tip(i)
        blade_row('lpt_s%d' % (i + 1), LPT, RIG, M['bronze_dark'], stators[i],
                  hub + 0.012, tip + 0.004, LPT_STATOR_N, LPT_STATOR, full=False)
        blade_row('lpt_r%d' % (i + 1), LPT, RIG, M['bronze'], rotors[i],
                  hub - 0.006, tip - 0.007, LPT_ROTOR_N, LPT_ROTOR, full=False)
        add_disk('lpt_disk%d' % (i + 1), LPT, M['bronze'], rotors[i],
                 LP_R - 0.003, hub - 0.003, full=False, web=0.010, rim=0.016, steps=44)
    # Turbine rear frame struts sit inside the core nozzle, clear of the plug.
    blade_row('trf_struts', LPT, RIG, M['exhaust'], 3.42, 0.36, 0.50,
              REAR_STRUTS, STRUT_SPEC, full=False)


def plug_profile():
    x_base, x_tip, r_base = 3.22, 3.88, 0.330
    outer = []
    n = 14
    for i in range(n + 1):
        t = i / n
        x = lerp(x_base, x_tip, t)
        r = r_base * ((1.0 - t) ** 0.72)
        outer.append((x, max(r, 0.001)))
    return [(x_base, 0.001), (x_tip, 0.001)] + list(reversed(outer[:-1]))


def build_exhaust(coll, full, tag):
    revolve('exhaust_plug_' + tag, plug_profile(), coll, M['exhaust'], M['cut'],
            full=full, full_steps=64)
    revolve('core_nozzle_' + tag, loft_wall([
        (3.28, 0.560, 0.588),
        (3.42, 0.500, 0.528),
        (3.58, 0.455, 0.482),
    ]), coll, M['exhaust'], M['cut'], full=full, full_steps=64)


def build_studio():
    # Large enough that the beauty cameras stay inside the sweep.
    R = 36.0
    zc = -1.55 + R
    nx, na = 28, 72
    x0, x1 = -8.0, 14.0
    a0, a1 = math.radians(-165.0), math.radians(165.0)
    bm = bmesh.new()
    grid = []
    for i in range(nx + 1):
        x = lerp(x0, x1, i / nx)
        colv = []
        for j in range(na + 1):
            a = lerp(a0, a1, j / na)
            y = R * math.sin(a)
            z = zc - R * math.cos(a)
            colv.append(bm.verts.new((x, y, z)))
        grid.append(colv)
    for i in range(nx):
        for j in range(na):
            bm.faces.new((grid[i][j], grid[i + 1][j], grid[i + 1][j + 1], grid[i][j + 1]))
    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    # Floor under the engine should face up.
    probe = None
    for f in bm.faces:
        c = f.calc_center_median()
        if abs(c.x - 1.5) < 0.8 and abs(c.y) < 0.8 and c.z < -1.0:
            probe = f
            break
    if probe is not None and probe.normal.z < 0:
        bmesh.ops.reverse_faces(bm, faces=list(bm.faces))
    for f in bm.faces:
        f.smooth = True
    mesh_obj('cyclorama', bm, [M['studio']], STUDIO)


# ------------------------------------------------------------------- world / lights / cameras
def build_world():
    world = bpy.data.worlds.new('StudioWorld')
    world.use_nodes = True
    bg = world.node_tree.nodes['Background']
    bg.inputs['Color'].default_value = (0.008, 0.009, 0.011, 1)
    bg.inputs['Strength'].default_value = 0.22
    scene_ext.world = world
    scene_cut.world = world


def look_at(obj, target_obj):
    # Local -Z points at the target. The up axis is local +Y, which keeps
    # world +Z at the top of the frame (the pylon stays above the nacelle).
    # Up axis 'Z' rolls the +Y-side camera until the pylon hangs downward.
    direction = Vector(target_obj.location) - obj.location
    obj.rotation_euler = direction.to_track_quat('-Z', 'Y').to_euler()


def add_area(name, coll, loc, target, size, energy, color, size_y=None):
    ld = bpy.data.lights.new(name, 'AREA')
    ld.energy = energy
    ld.color = color
    ld.shape = 'RECTANGLE'
    ld.size = size
    ld.size_y = size_y if size_y else size
    obj = bpy.data.objects.new(name, ld)
    obj.location = loc
    link_obj(obj, coll)
    tgt = bpy.data.objects.new(name + '_aim', None)
    tgt.location = target
    tgt.empty_display_size = 0.2
    link_obj(tgt, coll)
    look_at(obj, tgt)
    return obj


def add_camera(name, coll, scene, loc, target, lens=50.0, ortho=None):
    cd = bpy.data.cameras.new(name)
    cd.lens = lens
    cd.sensor_width = 36.0
    cd.clip_start = 0.05
    cd.clip_end = 80.0
    cd.dof.use_dof = False
    if ortho is not None:
        cd.type = 'ORTHO'
        cd.ortho_scale = ortho
    obj = bpy.data.objects.new(name, cd)
    obj.location = loc
    link_obj(obj, coll)
    tgt = bpy.data.objects.new(name + '_aim', None)
    tgt.location = target
    link_obj(tgt, coll)
    look_at(obj, tgt)
    scene.camera = obj
    return obj


def build_lights_and_cameras():
    # Exterior: uncut side (+Y), back far enough that inlet, pylon and plug all fit.
    ext_target = (1.50, 0.0, 0.28)
    add_area('ext_key', L_EXT, (-1.2, 7.2, 4.6), (1.4, 0.15, 0.35),
             8.0, 1100.0, (1.0, 0.97, 0.93), size_y=5.0)
    add_area('ext_fill', L_EXT, (5.6, 5.2, 1.6), (1.7, 0.0, 0.2),
             6.5, 280.0, (0.82, 0.88, 1.0), size_y=4.0)
    add_area('ext_rim', L_EXT, (6.4, -3.4, 2.8), (1.9, 0.0, 0.45),
             4.5, 620.0, (1.0, 0.95, 0.88), size_y=2.8)
    # Fill the inlet so the fan reads in the exterior still.
    add_area('ext_inlet', L_EXT, (-2.6, 2.4, 0.85), (0.25, 0.0, 0.12),
             3.2, 1100.0, (1.0, 0.97, 0.94), size_y=3.2)
    # More frontal than a pure side view, so the fan face and the nozzle both read.
    add_camera('Camera_Exterior', L_EXT, scene_ext,
               (-3.2, 9.4, 2.35), (1.65, 0.0, 0.22), lens=38.0)

    # Cutaway: from inside the open wedge (0 at +Z, positive toward -Y).
    mid = math.radians(CUT_DEG * 0.46)
    cut_target = (2.05, 0.0, 0.06)
    add_area('cut_key', L_CUT, (-1.6, -math.sin(mid) * 7.6, math.cos(mid) * 5.4),
             cut_target, 8.0, 980.0, (1.0, 0.97, 0.93), size_y=4.8)
    add_area('cut_fill', L_CUT, (4.2, 4.6, 2.4), (2.2, 0.0, 0.2),
             6.0, 260.0, (0.80, 0.86, 1.0), size_y=3.4)
    # Rim sits on the uncut side so it does not glare down the nozzle.
    add_area('cut_rim', L_CUT, (3.4, 5.5, 3.6), (2.0, 0.0, 0.4),
             4.2, 520.0, (1.0, 0.94, 0.86), size_y=2.6)
    add_area('cut_exhaust', L_CUT, (5.2, -math.sin(mid) * 2.4, math.cos(mid) * 1.6),
             (3.6, 0.0, 0.15), 2.2, 180.0, (1.0, 0.92, 0.82), size_y=1.4)
    # Soft panels in the empty wedge, outside the core, aimed inward.
    for i, x in enumerate((0.70, 1.70, 2.80)):
        ang = math.radians(CUT_DEG * 0.50)
        loc = (x, -math.sin(ang) * 0.78, math.cos(ang) * 0.78)
        add_area('cut_interior_%d' % i, L_CUT, loc, (x, 0.0, 0.05),
                 0.85, 120.0, (1.0, 0.96, 0.90), size_y=1.3)
    add_camera('Camera_Cutaway', L_CUT, scene_cut,
               (-3.4, -math.sin(mid) * 9.6, math.cos(mid) * 5.15),
               cut_target, lens=30.0)
    # Ortho section along the wedge bisector, for proportion checks.
    side_ang = math.radians(CUT_DEG * 0.50)
    add_camera('Camera_Side', L_CUT, scene_cut,
               (1.70, -math.sin(side_ang) * 12.0, math.cos(side_ang) * 12.0),
               (1.70, 0.0, 0.05), lens=50.0, ortho=5.6)
    # add_camera overwrites scene.camera; restore the beauty cutaway camera.
    scene_cut.camera = bpy.data.objects['Camera_Cutaway']


def setup_render(scene):
    scene.render.engine = 'CYCLES'
    scene.cycles.device = 'CPU'
    scene.cycles.use_denoising = True
    scene.cycles.denoiser = 'OPENIMAGEDENOISE'
    scene.cycles.use_adaptive_sampling = True
    scene.cycles.adaptive_threshold = 0.02
    scene.cycles.adaptive_min_samples = 12
    scene.cycles.samples = 128
    scene.cycles.max_bounces = 6
    scene.cycles.diffuse_bounces = 2
    scene.cycles.glossy_bounces = 3
    scene.cycles.transmission_bounces = 2
    scene.cycles.volume_bounces = 0
    scene.cycles.transparent_max_bounces = 4
    scene.cycles.sample_clamp_indirect = 6.0
    scene.cycles.caustics_reflective = False
    scene.cycles.caustics_refractive = False
    scene.cycles.blur_glossy = 0.35
    try:
        scene.cycles.use_light_tree = False
    except Exception:
        pass
    r = scene.render
    r.resolution_x = 1920
    r.resolution_y = 1080
    r.resolution_percentage = 100
    r.threads_mode = 'FIXED'
    r.threads = 4
    r.image_settings.file_format = 'PNG'
    r.image_settings.color_mode = 'RGB'
    r.image_settings.color_depth = '8'
    r.use_stamp = False
    for attr in ('use_stamp_date', 'use_stamp_time', 'use_stamp_render_time',
                 'use_stamp_frame', 'use_stamp_frame_range', 'use_stamp_camera',
                 'use_stamp_lens', 'use_stamp_scene', 'use_stamp_filename',
                 'use_stamp_marker', 'use_stamp_sequencer_strip', 'use_stamp_hostname'):
        if hasattr(r, attr):
            setattr(r, attr, False)
    r.film_transparent = False
    scene.view_settings.view_transform = 'AgX'
    try:
        scene.view_settings.look = 'AgX - Medium High Contrast'
    except Exception as ex:
        log('view look: %s' % ex)
    scene.view_settings.exposure = -0.35
    scene.frame_start = 1
    scene.frame_end = 1
    scene.unit_settings.system = 'METRIC'
    scene.unit_settings.scale_length = 1.0
    setup_compositor(scene)


def setup_compositor(scene):
    try:
        scene.use_nodes = True
        nt = scene.node_tree
        for n in list(nt.nodes):
            nt.nodes.remove(n)
        rl = nt.nodes.new('CompositorNodeRLayers')
        # A new Render Layers node inherits the *context* scene, not the
        # node tree's scene. Left alone, Cutaway composites Exterior.
        rl.scene = scene
        rl.layer = scene.view_layers[0].name
        gl = nt.nodes.new('CompositorNodeGlare')
        gl.glare_type = 'FOG_GLOW'
        gl.quality = 'HIGH'
        gl.threshold = 3.5
        gl.size = 5
        gl.mix = -0.40
        el = nt.nodes.new('CompositorNodeEllipseMask')
        el.width = 0.92
        el.height = 0.88
        bl = nt.nodes.new('CompositorNodeBlur')
        bl.size_x = 180
        bl.size_y = 140
        mp = nt.nodes.new('CompositorNodeMapRange')
        mp.inputs['From Min'].default_value = 0.0
        mp.inputs['From Max'].default_value = 1.0
        mp.inputs['To Min'].default_value = 0.84
        mp.inputs['To Max'].default_value = 1.0
        mix = nt.nodes.new('CompositorNodeMixRGB')
        mix.blend_type = 'MULTIPLY'
        mix.inputs[0].default_value = 1.0
        comp = nt.nodes.new('CompositorNodeComposite')
        nt.links.new(rl.outputs['Image'], gl.inputs['Image'])
        nt.links.new(el.outputs['Mask'], bl.inputs['Image'])
        nt.links.new(bl.outputs['Image'], mp.inputs['Value'])
        nt.links.new(gl.outputs['Image'], mix.inputs[1])
        nt.links.new(mp.outputs['Value'], mix.inputs[2])
        nt.links.new(mix.outputs['Image'], comp.inputs['Image'])
    except Exception as ex:
        log('compositor skipped: %s' % ex)


def apply_mode(scene):
    cy = scene.cycles
    r = scene.render
    if MODE == 'preview':
        r.resolution_x, r.resolution_y = 960, 540
        cy.samples = SAMPLE_ARG or 28
        cy.adaptive_threshold = 0.06
        cy.adaptive_min_samples = 8
    elif MODE == 'mid':
        r.resolution_x, r.resolution_y = 1280, 720
        cy.samples = SAMPLE_ARG or 64
        cy.adaptive_threshold = 0.03
        cy.adaptive_min_samples = 12
    elif MODE == 'still':
        r.resolution_x, r.resolution_y = 1920, 1080
        cy.samples = SAMPLE_ARG or 128
        cy.adaptive_threshold = 0.018
        cy.adaptive_min_samples = 16
    # setup keeps the 1920x1080 / 128-sample defaults from setup_render


def count_verts(scene):
    set_scene(scene)
    deps = bpy.context.evaluated_depsgraph_get()
    total = 0
    top = []
    for obj in scene.objects:
        if obj.type != 'MESH':
            continue
        ev = obj.evaluated_get(deps)
        me = ev.to_mesh()
        n = len(me.vertices)
        ev.to_mesh_clear()
        total += n
        top.append((n, obj.name))
    top.sort(reverse=True)
    log('VERTS %s total=%d' % (scene.name, total))
    for n, name in top[:12]:
        log('   %7d  %s' % (n, name))
    if total > 2_000_000:
        log('WARNING vertex budget exceeded')
    return total


def assert_no_text():
    bad = [o.name for o in bpy.data.objects if o.type in {'FONT', 'GPENCIL'}]
    if bad:
        raise SystemExit('text objects present: %s' % bad)
    for sc in (scene_ext, scene_cut):
        if sc.render.use_stamp:
            raise SystemExit('render stamp is on')


def build_all():
    t = time.time()
    build_world()
    build_studio()
    build_shafts()
    build_fan()
    build_nacelle(NAC_FULL, True, 'full')
    build_nacelle(NAC_CUT, False, 'cut')
    build_cowl(BYP_FULL, True, 'full')
    build_cowl(BYP_CUT, False, 'cut')
    build_ogv(BYP_FULL, True, 'full')
    build_ogv(BYP_CUT, False, 'cut')
    build_lpc()
    build_hpc()
    build_combustor()
    build_hpt()
    build_lpt()
    build_exhaust(EXH_FULL, True, 'full')
    build_exhaust(EXH_CUT, False, 'cut')
    build_lights_and_cameras()
    setup_render(scene_ext)
    setup_render(scene_cut)
    # Beauty cameras (side builder temporarily takes over Cutaway's camera).
    scene_ext.camera = bpy.data.objects['Camera_Exterior']
    scene_cut.camera = bpy.data.objects['Camera_Cutaway']
    assert_no_text()
    log('BUILD_SECONDS %.2f' % (time.time() - t))
    log('FAN_DIAMETER_M %.4f  FLANGE_TO_TRF_M %.3f  CUT_DEG %.1f' % (FAN_DIAMETER, FLANGE_TO_TRF, CUT_DEG))
    log('STAGES fan=1 lpc=3 hpc=%d hpt=%d lpt=%d fan_blades=%d' % (
        HPC_STAGES, HPT_STAGES, LPT_STAGES, FAN_BLADES))


def render_scene(scene, path, camera_name=None):
    if camera_name:
        scene.camera = bpy.data.objects[camera_name]
    apply_mode(scene)
    scene.render.filepath = path
    if scene.use_nodes and scene.node_tree:
        for node in scene.node_tree.nodes:
            if node.type == 'R_LAYERS':
                node.scene = scene
                node.layer = scene.view_layers[0].name
    log('RENDER %s cam=%s -> %s  %dx%d samples=%d' % (
        scene.name, scene.camera.name if scene.camera else None, path,
        scene.render.resolution_x, scene.render.resolution_y, scene.cycles.samples))
    t = time.time()
    bpy.ops.render.render(write_still=True, scene=scene.name)
    dt = time.time() - t
    log('RENDER_SECONDS %.1f  %s' % (dt, os.path.basename(path)))
    return dt


def write_log():
    path = os.path.join(OUT, 'logs', 'build.txt')
    with open(path, 'a') as f:
        f.write('\n'.join(LOG_LINES) + '\n')


def main():
    build_all()
    if MODE == 'setup':
        count_verts(scene_cut)
        count_verts(scene_ext)
        set_scene(scene_cut)
        path = os.path.join(OUT, 'engine.blend')
        bpy.ops.wm.save_as_mainfile(filepath=path, compress=True)
        log('SAVED %s' % path)
        write_log()
        return
    if VIEW in ('cut', 'side'):
        count_verts(scene_cut)
    else:
        count_verts(scene_ext)
    if VIEW == 'side':
        path = os.path.join(OUT, 'previews', '%s_side.png' % MODE)
        render_scene(scene_cut, path, 'Camera_Side')
    elif VIEW == 'ext':
        name = 'exterior.png' if MODE == 'still' else '%s_ext.png' % MODE
        folder = 'renders' if MODE == 'still' else 'previews'
        render_scene(scene_ext, os.path.join(OUT, folder, name), 'Camera_Exterior')
    elif VIEW == 'cut':
        name = 'cutaway.png' if MODE == 'still' else '%s_cut.png' % MODE
        folder = 'renders' if MODE == 'still' else 'previews'
        render_scene(scene_cut, os.path.join(OUT, folder, name), 'Camera_Cutaway')
    else:
        raise SystemExit('unknown view %s (use cut, ext, or side)' % VIEW)
    write_log()


if __name__ == '__main__':
    main()
