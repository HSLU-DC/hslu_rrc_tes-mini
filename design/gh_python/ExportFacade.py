# Grasshopper Component: Export Facade (JSON + Simulation STLs)
# ghenv.Component.Message = 'Export Facade v2 (tes-v4 SSOT)'
#
# Schreibt fab_data.json im tes-v4 SSOT-Schema mit sechs Top-Level-Sections
# (manifest, configuration, design, process, fabrication, state) und die drei
# STL-Files pro Element fuer den RobotStudio BeamSimulator. STLs landen in
# <json_dir>/geometry/.
#
# ==============================================================================
# INPUTS
# ==============================================================================
#   update (bool):                    Trigger export
#   file_path (str):                  Path to output JSON file (STL folder derived)
#   fab_data (DataTree):              Tree with {layer;element} structure (siehe
#                                     BRANCH_INDEX_MAP unten)
#   glue_planes_world_tree (DataTree, optional): Welt-KS Glue-Planes pro Element,
#                                     vor Reorient ins Anlagen-Workobject. Speist
#                                     process.glue_planes_world.
#   glue_planes_tree (DataTree):      Wobj-KS Glue-Planes (in ob_HSLU_Glue coords),
#                                     speist fabrication.glue_positions.
#   configuration_json (str, optional): JSON-String mit Configuration-Section.
#                                     Wenn None/leer: hartkodierte Defaults aus
#                                     DEFAULT_CONFIGURATION (PoC).
#
# ==============================================================================
# FAB_DATA BRANCH-INDEX MAPPING (Facade)
# ==============================================================================
#
#   0 = brep              << Finished beam geometry (= cutB, student input)
#   1 = centerline        << Line along beam centerline (Welt-KS, design)
#   2 = cut_plane_a       << Plane for first miter cut (Welt-KS, process)
#   3 = cut_plane_b       << Plane for second miter cut (Welt-KS, process)
#   4 = place_position    << Frame (in ob_HSLU_Place coordinates, fabrication)
#   5 = cut_position_a    << Frame (in ob_HSLU_Cut coordinates, fabrication)
#   6 = cut_position_b    << Frame (in ob_HSLU_Cut coordinates, fabrication)
#   7 = stock_category    << String, one of "400" / "550" / "750" / "1000"
#                             (= Rohlatten-Kategorie aus WoodStorage)
#
# ==============================================================================
# OUTPUT
# ==============================================================================
#   output (str):  Short status message
#   details (str): Detailed export log
#
# ==============================================================================
# STL FILES PRODUCED (per element, under <json_dir>/geometry/)
# ==============================================================================
#   L{layer}_E{element}_raw.stl    Raw stock beam (box)
#   L{layer}_E{element}_cutA.stl   After first miter cut
#   L{layer}_E{element}_cutB.stl   Finished beam (student's Brep)
#
# STLs are written in grip-center-local coordinates so the BeamSimulator
# SmartComponent can attach them at the gripper TCP without any transform.

import json
import os
import struct
import uuid
from datetime import datetime

import System
import compas
from compas.geometry import Frame, Line, Point, Vector

import Rhino
import Rhino.Geometry as rg

from Grasshopper import DataTree
from Grasshopper.Kernel.Data import GH_Path


# ==============================================================================
# Schema
# ==============================================================================
SCHEMA_VERSION = "tes-v4"
PIPELINE_VERSION = "tes-mini 0.2.0"
TARGET_CELL = "hslu_rrc_tes-mini"

# Default-Configuration als PoC-Stand-In, falls kein configuration_json eingespeist
# wird. Die echte Anbindung ans messe-platform-Frontend liefert diese Section
# spaeter ueber den optionalen String-Input.
DEFAULT_CONFIGURATION = {
    "project_name": "facade_default",
    "frame_size_mm": [600, 2500],
    "beam_section_mm": 25,
    "structural": None,
    "grid": None,
    "materials": {
        "wood_species": "spruce",
        "glue_type": "hot_melt",
    },
    "stock": {
        "lengths_mm": [400, 550, 750, 1000],
    },
}

ACTIONS = ("pick", "cut", "glue", "place")

# ==============================================================================
# Branch-Index mapping (fab_data DataTree)
# ==============================================================================
BREP_INDEX = 0
CENTERLINE_INDEX = 1
CUT_PLANE_A_WORLD_INDEX = 2
CUT_PLANE_B_WORLD_INDEX = 3
PLACE_POSITION_INDEX = 4
CUT_POSITION_A_INDEX = 5
CUT_POSITION_B_INDEX = 6
STOCK_CATEGORY_INDEX = 7

FABRICATION_FRAME_MAP = {
    PLACE_POSITION_INDEX: "place_position",
    CUT_POSITION_A_INDEX: "cut_position_a",
    CUT_POSITION_B_INDEX: "cut_position_b",
}

# ==============================================================================
# Geometry configuration
# ==============================================================================
BEAM_SECTION = 25.0                        # mm (must match globals.py)
# Stock lengths per stock_category category (must match VALID_CATEGORIES in
# process/_skills/WoodStorage/wood_storage.py).
STOCK_LENGTHS = {
    "400":  400.0,
    "550":  550.0,
    "750":  750.0,
    "1000": 1000.0,
}
VALID_STOCK_CATEGORIES = tuple(STOCK_LENGTHS.keys())
MESH_MIN_EDGE = 0.5
MESH_MAX_EDGE = 50.0
SPLIT_TOLERANCE = 0.01
SPLIT_PLANE_SIZE = 5000.0                  # half-size of cutting plane

GEOMETRY_SUBFOLDER = "geometry"            # relative to <json_dir>


# ==============================================================================
# Tree helpers
# ==============================================================================

def get_branch(tree, path):
    if tree is None:
        return []
    try:
        branch = tree.Branch(path)
        return list(branch) if branch else []
    except:
        return []


def analyze_tree(tree):
    if tree is None:
        return 0, {}
    paths = tree.Paths
    if not paths:
        return 0, {}

    elements_per_layer = {}
    for p in paths:
        if p.Length >= 2:
            layer = p[0]
            element = p[1]
            if layer not in elements_per_layer:
                elements_per_layer[layer] = 0
            elements_per_layer[layer] = max(elements_per_layer[layer], element + 1)

    n_layers = max(elements_per_layer.keys()) + 1 if elements_per_layer else 0
    return n_layers, elements_per_layer


# ==============================================================================
# Frame conversion (JSON)
# ==============================================================================

def to_compas_frame(plane):
    if plane is None:
        return None
    if not isinstance(plane, rg.Plane):
        return None
    return Frame(
        Point(plane.Origin.X, plane.Origin.Y, plane.Origin.Z),
        Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z),
        Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z),
    )


def to_compas_line(line):
    """rg.Line/LineCurve -> compas.geometry.Line (oder None)."""
    if line is None:
        return None
    if isinstance(line, rg.LineCurve):
        line = line.Line
    if not isinstance(line, rg.Line):
        return None
    return Line(
        Point(line.FromX, line.FromY, line.FromZ),
        Point(line.ToX, line.ToY, line.ToZ),
    )


def parse_configuration(configuration_json_str):
    """Parse optional JSON string, fall back to DEFAULT_CONFIGURATION."""
    if not configuration_json_str:
        return dict(DEFAULT_CONFIGURATION)
    try:
        return json.loads(configuration_json_str)
    except (ValueError, TypeError):
        return dict(DEFAULT_CONFIGURATION)


# ==============================================================================
# Geometry dereferencing (STL)
# ==============================================================================
# In Rhino 8 CPython, geometry from DataTrees often arrives as System.Guid
# references. GH-created geometry lives in ghdoc, Rhino-referenced geometry
# lives in the Rhino document. `deref` tries both.

def deref(item):
    if hasattr(item, "Value"):
        return deref(item.Value)

    if isinstance(item, System.Guid):
        import scriptcontext as sc
        old_doc = sc.doc

        try:
            sc.doc = ghdoc
            rhino_obj = sc.doc.Objects.FindId(item)
            if rhino_obj is not None:
                sc.doc = old_doc
                return rhino_obj.Geometry
        except:
            pass

        try:
            sc.doc = Rhino.RhinoDoc.ActiveDoc
            rhino_obj = sc.doc.Objects.FindId(item)
            if rhino_obj is not None:
                sc.doc = old_doc
                return rhino_obj.Geometry
        except:
            pass

        sc.doc = old_doc
        return None

    return item


def to_brep(item):
    geom = deref(item)
    if isinstance(geom, rg.Brep):
        return geom
    if isinstance(geom, rg.Extrusion):
        return geom.ToBrep()
    return None


def to_line(item):
    geom = deref(item)
    if isinstance(geom, rg.Line):
        return geom
    if isinstance(geom, rg.LineCurve):
        return geom.Line
    if isinstance(geom, rg.Curve):
        return rg.Line(geom.PointAtStart, geom.PointAtEnd)
    return None


def to_plane_geom(item):
    geom = deref(item)
    if isinstance(geom, rg.Plane):
        return geom
    if isinstance(geom, rg.PlaneSurface):
        ok, frame = geom.TryGetPlane()
        if ok:
            return frame
    if isinstance(geom, rg.Surface):
        ok, frame = geom.TryGetPlane()
        if ok:
            return frame
    return None


# ==============================================================================
# STL geometry helpers
# ==============================================================================

def get_stock_length(size):
    return STOCK_LENGTHS.get(size, STOCK_LENGTHS["400"])


def make_grip_frame(centerline):
    """Grip frame for the simulation: origin at centerline midpoint,
    X along beam, Z = -world Z (gripper points down), Y = Z x X."""
    grip_center = centerline.PointAt(0.5)

    grip_x = centerline.Direction
    grip_x.Unitize()
    grip_z = -rg.Vector3d.ZAxis
    grip_y = rg.Vector3d.CrossProduct(grip_z, grip_x)
    grip_y.Unitize()

    grip_plane = rg.Plane(grip_center, grip_x, grip_y)
    to_local = rg.Transform.PlaneToPlane(grip_plane, rg.Plane.WorldXY)
    return grip_center, grip_plane, to_local


def make_raw_brep(grip_plane, size):
    stock_len = get_stock_length(size)
    half_len = stock_len / 2.0
    half_sec = BEAM_SECTION / 2.0
    box = rg.Box(
        grip_plane,
        rg.Interval(-half_len, half_len),
        rg.Interval(-half_sec, half_sec),
        rg.Interval(-half_sec, half_sec),
    )
    return box.ToBrep()


def split_brep_with_plane(brep, cut_plane, grip_center):
    """Split brep with cut plane, keep the fragment closest to grip_center."""
    srf = rg.PlaneSurface(
        cut_plane,
        rg.Interval(-SPLIT_PLANE_SIZE, SPLIT_PLANE_SIZE),
        rg.Interval(-SPLIT_PLANE_SIZE, SPLIT_PLANE_SIZE),
    )
    cutter = srf.ToBrep()

    fragments = brep.Split(cutter, SPLIT_TOLERANCE)
    if fragments and len(fragments) > 0:
        best = None
        best_dist = float("inf")
        for frag in fragments:
            bb = frag.GetBoundingBox(False)
            dist = grip_center.DistanceTo(bb.Center)
            if dist < best_dist:
                best_dist = dist
                best = frag
        if best:
            capped = best.CapPlanarHoles(SPLIT_TOLERANCE)
            if capped is not None:
                best = capped
            return best, "Split OK ({} fragments)".format(len(fragments))

    trimmed = brep.Trim(cut_plane, SPLIT_TOLERANCE)
    if trimmed and len(trimmed) > 0:
        best = None
        best_dist = float("inf")
        for frag in trimmed:
            dist = grip_center.DistanceTo(frag.GetBoundingBox(False).Center)
            if dist < best_dist:
                best_dist = dist
                best = frag
        if best:
            capped = best.CapPlanarHoles(SPLIT_TOLERANCE)
            if capped is not None:
                best = capped
            return best, "Trim fallback OK ({} fragments)".format(len(trimmed))

    return brep, "WARNING: Split+Trim failed, using original"


def brep_to_mesh(brep):
    mp = rg.MeshingParameters.Default
    mp.MinimumEdgeLength = MESH_MIN_EDGE
    mp.MaximumEdgeLength = MESH_MAX_EDGE

    meshes = rg.Mesh.CreateFromBrep(brep, mp)
    if not meshes:
        return None

    combined = rg.Mesh()
    for m in meshes:
        combined.Append(m)
    combined.Faces.ConvertQuadsToTriangles()
    combined.Normals.ComputeNormals()
    combined.Compact()
    return combined


def write_binary_stl(mesh, file_path):
    with open(file_path, "wb") as f:
        header = b"HSLU Facade SimBeam" + b"\x00" * (80 - 19)
        f.write(header)

        n_faces = mesh.Faces.Count
        f.write(struct.pack("<I", n_faces))

        for i in range(n_faces):
            face = mesh.Faces[i]
            n = mesh.FaceNormals[i]
            f.write(struct.pack("<fff", n.X, n.Y, n.Z))

            v = mesh.Vertices[face.A]
            f.write(struct.pack("<fff", v.X, v.Y, v.Z))
            v = mesh.Vertices[face.B]
            f.write(struct.pack("<fff", v.X, v.Y, v.Z))
            v = mesh.Vertices[face.C]
            f.write(struct.pack("<fff", v.X, v.Y, v.Z))

            f.write(struct.pack("<H", 0))


def export_brep_as_stl(brep, to_local, file_path):
    if brep is None:
        return False, 0, "Null brep"

    mesh = brep_to_mesh(brep)
    if mesh is None:
        return False, 0, "Meshing failed"

    mesh.Transform(to_local)
    write_binary_stl(mesh, file_path)
    return True, mesh.Faces.Count, "OK"


# ==============================================================================
# Validation
# ==============================================================================

def validate_fabrication_element(element, layer_idx, elem_idx):
    warnings = []
    prefix = "L{} E{}".format(layer_idx, elem_idx)

    for field in ["place_position", "cut_position_a", "cut_position_b"]:
        if field not in element:
            warnings.append("{}: '{}' fehlt!".format(prefix, field))

    stock_category = element.get("stock_category", "")
    if stock_category not in VALID_STOCK_CATEGORIES:
        warnings.append("{}: stock_category '{}' ungueltig (erlaubt: {})".format(
            prefix, stock_category, ", ".join(VALID_STOCK_CATEGORIES)))

    place = element.get("place_position")
    if place is not None:
        x, y = place.point.x, place.point.y
        if x < -10 or x > 2510:
            warnings.append("{}: place X={:.0f} ausserhalb Rahmen (0-2500)".format(prefix, x))
        if y < -610 or y > 10:
            warnings.append("{}: place Y={:.0f} ausserhalb Rahmen (-600 - 0)".format(prefix, y))

    return warnings


# ==============================================================================
# Compact-Geometry JSON Writer
# ==============================================================================
# Pretty-print mit indent=4, aber compas-geometry Frames/Lines/Points/Vectors
# als 1-Liner. Reduziert das File von ~42k auf ~6k Zeilen ohne Information zu
# verlieren - compas.json_load liest beide Formate identisch.

class _OneLine(object):
    """Marker: wickelt einen Dict so dass der Encoder ihn als 1-Liner schreibt."""
    def __init__(self, data):
        self.data = data


def _wrap_geometry(obj):
    """Wickelt rekursiv alle compas.geometry/*-dicts in _OneLine."""
    if isinstance(obj, dict):
        if str(obj.get("dtype", "")).startswith("compas.geometry/"):
            return _OneLine(obj)
        return {k: _wrap_geometry(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_wrap_geometry(v) for v in obj]
    return obj


class _PlaceholderEncoder(json.JSONEncoder):
    """Schreibt _OneLine als Platzhalter-String, der spaeter durch eine
    kompakte JSON-Repraesentation ersetzt wird."""
    def __init__(self, *args, **kwargs):
        super(_PlaceholderEncoder, self).__init__(*args, **kwargs)
        self._compact = {}

    def default(self, obj):
        if isinstance(obj, _OneLine):
            key = "@@COMPACT_{}@@".format(uuid.uuid4().hex)
            self._compact[key] = json.dumps(obj.data, separators=(", ", ": "), sort_keys=True)
            return key
        return super(_PlaceholderEncoder, self).default(obj)


def dump_pretty_with_compact_geometry(data, path, indent=4):
    """Schreibt data als pretty JSON, aber compas-geometry Dicts in 1 Zeile.

    Voraussetzung: data laeuft erst durch compas.json_dumps, damit Frame-
    Objekte zu dtype-dicts werden. Danach wrap + custom encode.
    """
    json_str = compas.json_dumps(data, pretty=False)
    plain = json.loads(json_str)
    wrapped = _wrap_geometry(plain)
    enc = _PlaceholderEncoder(indent=indent, sort_keys=True)
    rendered = enc.encode(wrapped)
    for placeholder, compact in enc._compact.items():
        rendered = rendered.replace('"{}"'.format(placeholder), compact)
    with open(path, "w") as f:
        f.write(rendered)


# ==============================================================================
# Section builders (tes-v4 SSOT)
# ==============================================================================

def element_ref(layer_idx, elem_idx):
    return "L{}_E{}".format(layer_idx, elem_idx)


def build_manifest():
    return {
        "manifest_id": str(uuid.uuid4()),
        "created_at": datetime.now().isoformat(),
        "pipeline_version": PIPELINE_VERSION,
        "schema_version": SCHEMA_VERSION,
    }


def build_initial_state(all_refs):
    return {
        "run_id": None,
        "started_at": None,
        "last_updated_at": None,
        "elements": {
            ref: {action: {"status": "pending"} for action in ACTIONS}
            for ref in all_refs
        },
        "errors": [],
    }


# ==============================================================================
# STL export for a single element
# ==============================================================================

def export_element_stls(branch, stock_category, layer_idx, elem_idx, geometry_folder):
    """Export raw/cutA/cutB STLs for one element. Returns (n_files, errors)."""
    prefix = "L{}_E{}".format(layer_idx, elem_idx)
    errors = []

    finished_brep = to_brep(branch[BREP_INDEX]) if BREP_INDEX < len(branch) else None
    centerline = to_line(branch[CENTERLINE_INDEX]) if CENTERLINE_INDEX < len(branch) else None
    cut_plane_a = to_plane_geom(branch[CUT_PLANE_A_WORLD_INDEX]) if CUT_PLANE_A_WORLD_INDEX < len(branch) else None

    if finished_brep is None or centerline is None or cut_plane_a is None:
        errors.append("{} STL: missing brep/centerline/cut_plane_a".format(prefix))
        return 0, errors

    if stock_category not in VALID_STOCK_CATEGORIES:
        errors.append("{} STL: invalid stock_category '{}'".format(prefix, stock_category))
        return 0, errors

    grip_center, grip_plane, to_local = make_grip_frame(centerline)

    n_files = 0

    # Raw beam
    raw_brep = make_raw_brep(grip_plane, stock_category)
    raw_path = os.path.join(geometry_folder, "{}_raw.stl".format(prefix))
    ok, n_tri, msg = export_brep_as_stl(raw_brep, to_local, raw_path)
    if ok:
        n_files += 1
    else:
        errors.append("{} raw: {}".format(prefix, msg))

    # CutA (raw split by cut_plane_a)
    if raw_brep is not None:
        cut_a_brep, split_msg = split_brep_with_plane(raw_brep, cut_plane_a, grip_center)
    else:
        cut_a_brep, split_msg = None, "No raw brep"
    cut_a_path = os.path.join(geometry_folder, "{}_cutA.stl".format(prefix))
    ok, n_tri, msg = export_brep_as_stl(cut_a_brep, to_local, cut_a_path)
    if ok:
        n_files += 1
    else:
        errors.append("{} cutA: {} ({})".format(prefix, msg, split_msg))

    # CutB = finished Brep (both cuts already in student input)
    cut_b_path = os.path.join(geometry_folder, "{}_cutB.stl".format(prefix))
    ok, n_tri, msg = export_brep_as_stl(finished_brep, to_local, cut_b_path)
    if ok:
        n_files += 1
    else:
        errors.append("{} cutB: {}".format(prefix, msg))

    return n_files, errors


# ==============================================================================
# Main Export Logic
# ==============================================================================

log = []

if update:
    # Paths
    last_slash = max(file_path.rfind("/"), file_path.rfind("\\"))
    data_dir = file_path[:last_slash] if last_slash != -1 else ""
    geometry_folder = os.path.join(data_dir, GEOMETRY_SUBFOLDER)

    if data_dir and not os.path.exists(data_dir):
        os.makedirs(data_dir)
    if not os.path.exists(geometry_folder):
        os.makedirs(geometry_folder)

    # Analyze tree
    n_layers, elements_per_layer = analyze_tree(fab_data)

    if n_layers > 2:
        log.append("[WARNUNG] {} Layer erkannt - maximal 2 erlaubt!".format(n_layers))

    log.append("Erkannt: {} Layer".format(n_layers))
    for layer, count in sorted(elements_per_layer.items()):
        log.append("  Layer {}: {} Elemente".format(layer, count))

    # Build sections (JSON) and export STLs in one pass
    design_layers = []
    fabrication_layers = []
    process_cut_planes_world = []
    process_glue_planes_world = []
    all_refs = []

    total_elements = 0
    total_stl_files = 0
    all_warnings = []
    stl_errors = []

    for layer_idx in range(n_layers):
        n_elements = elements_per_layer.get(layer_idx, 0)
        log.append("")
        log.append("Verarbeite Layer {} ({} Elemente)...".format(layer_idx, n_elements))

        design_elements = []
        fabrication_elements = []

        for elem_idx in range(n_elements):
            branch = get_branch(fab_data, GH_Path(layer_idx, elem_idx))
            ref = element_ref(layer_idx, elem_idx)
            all_refs.append(ref)

            # stock_category (string) — gehoert in design + fabrication
            stock_category = ""
            if STOCK_CATEGORY_INDEX < len(branch) and branch[STOCK_CATEGORY_INDEX] is not None:
                stock_category = str(branch[STOCK_CATEGORY_INDEX])

            # --- design: centerline (Welt-KS) + length_mm
            design_element = {"id": elem_idx, "stock_category": stock_category}
            if CENTERLINE_INDEX < len(branch):
                line = to_compas_line(to_line(branch[CENTERLINE_INDEX]))
                if line is not None:
                    design_element["centerline"] = line
                    design_element["finished_length_mm"] = line.length
            design_elements.append(design_element)

            # --- process: cut_planes_world (Welt-KS, vor Reorient)
            cut_a_world = to_compas_frame(to_plane_geom(branch[CUT_PLANE_A_WORLD_INDEX])) \
                if CUT_PLANE_A_WORLD_INDEX < len(branch) else None
            cut_b_world = to_compas_frame(to_plane_geom(branch[CUT_PLANE_B_WORLD_INDEX])) \
                if CUT_PLANE_B_WORLD_INDEX < len(branch) else None
            if cut_a_world is not None or cut_b_world is not None:
                process_cut_planes_world.append({
                    "element_ref": ref,
                    "plane_a": cut_a_world,
                    "plane_b": cut_b_world,
                })

            # --- process: glue_planes_world (Welt-KS, vor Reorient)
            # Auch leere Listen schreiben, damit Konsistenz zu fabrication.glue_positions
            # gegeben ist (jedes Element hat einen Eintrag, ggf. mit planes=[]).
            glue_world_branch = get_branch(glue_planes_world_tree, GH_Path(layer_idx, elem_idx))
            glue_world_planes = [
                f for f in (to_compas_frame(p) for p in glue_world_branch) if f is not None
            ]
            process_glue_planes_world.append({
                "element_ref": ref,
                "planes": glue_world_planes,
            })

            # --- fabrication: heutiges fab_data 1:1 (Workobject-Koordinaten)
            fab_element = {"id": elem_idx, "stock_category": stock_category}
            for idx, pos_name in FABRICATION_FRAME_MAP.items():
                if idx < len(branch):
                    frame = to_compas_frame(branch[idx])
                    if frame is not None:
                        fab_element[pos_name] = frame
            glue_wobj_branch = get_branch(glue_planes_tree, GH_Path(layer_idx, elem_idx))
            fab_element["glue_positions"] = [
                f for f in (to_compas_frame(p) for p in glue_wobj_branch) if f is not None
            ]

            warnings = validate_fabrication_element(fab_element, layer_idx, elem_idx)
            all_warnings.extend(warnings)

            fabrication_elements.append(fab_element)

            # STL export
            n_stl, errs = export_element_stls(
                branch, stock_category, layer_idx, elem_idx, geometry_folder
            )
            total_stl_files += n_stl
            stl_errors.extend(errs)

        design_layers.append({"id": layer_idx, "elements": design_elements})
        fabrication_layers.append({"id": layer_idx, "elements": fabrication_elements})
        total_elements += len(fabrication_elements)

        if fabrication_elements:
            log.append("  Fabrication-Keys: {}".format(list(fabrication_elements[0].keys())))

    # Build SSOT export structure
    configuration = parse_configuration(configuration_json)
    export_data = {
        "manifest": build_manifest(),
        "configuration": configuration,
        "design": {"layers": design_layers},
        "process": {
            "cut_planes_world": process_cut_planes_world,
            "glue_planes_world": process_glue_planes_world,
        },
        "fabrication": {
            "target_cell": TARGET_CELL,
            "cell_config_ref": None,
            "layers": fabrication_layers,
        },
        "state": build_initial_state(all_refs),
    }

    # Warnings
    if all_warnings:
        log.append("")
        log.append("{} JSON-WARNUNGEN:".format(len(all_warnings)))
        for w in all_warnings:
            log.append("  - " + w)

    if stl_errors:
        log.append("")
        log.append("{} STL-FEHLER:".format(len(stl_errors)))
        for e in stl_errors:
            log.append("  - " + e)

    # Write JSON (pretty mit kompakten Frame/Line-1-Linern)
    dump_pretty_with_compact_geometry(export_data, file_path)

    # Summary
    log.append("")
    log.append("JSON (tes-v4 SSOT): {}".format(file_path))
    log.append("  manifest, configuration, design, process, fabrication, state")
    log.append("  process.cut_planes_world: {}".format(len(process_cut_planes_world)))
    log.append("  process.glue_planes_world: {}".format(len(process_glue_planes_world)))
    log.append("STLs: {} ({} files)".format(geometry_folder, total_stl_files))
    for layer in fabrication_layers:
        log.append("  Layer {}: {} Elemente".format(layer["id"], len(layer["elements"])))

    suffix = ""
    if all_warnings or stl_errors:
        suffix = " ({} warnings, {} stl errors)".format(len(all_warnings), len(stl_errors))

    output = "Done (tes-v4) - {} elements, {} STLs{}".format(
        total_elements, total_stl_files, suffix
    )

    details = "\n".join(log)

else:
    output = "Press Button to Export"
    details = ""
