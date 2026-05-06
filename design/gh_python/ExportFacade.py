# Grasshopper Component: Export Facade (JSON + Simulation STLs)
# ghenv.Component.Message = 'Export Facade v1'
#
# Paste this script into the fab_data export component in the GH template.
# It writes both the fab_data JSON and the three STL files per element
# used by the RobotStudio BeamSimulator. STLs land in <json_dir>/geometry/.
#
# ==============================================================================
# INPUTS
# ==============================================================================
#   update (bool):              Trigger export
#   file_path (str):            Path to output JSON file (STL folder derived)
#   fab_data (DataTree):        Tree with {layer;element} structure
#   glue_planes_tree (DataTree): Tree with {layer;element} structure, 0..N
#                                glue position frames (in ob_HSLU_Glue coords)
#                                per branch. Order = robot drive order.
#
# ==============================================================================
# FAB_DATA INDEX MAPPING (Facade)
# ==============================================================================
#
#   0 = brep              << Finished beam geometry (= cutB, student input)
#   1 = centerline        << Line along beam centerline (raw + grip frame)
#   2 = cut_plane_a       << Plane for first miter cut
#   3 = cut_plane_b       << Plane for second miter cut (currently unused in STL)
#   4 = place_position    << Frame (in ob_HSLU_Place coordinates)
#   5 = cut_position_a    << Frame (in ob_HSLU_Cut coordinates)
#   6 = cut_position_b    << Frame (in ob_HSLU_Cut coordinates)
#   7 = beam_size         << String, one of "400" / "550" / "750" / "1000"
#
# Glue positions are NOT part of fab_data — they live in their own tree
# (glue_planes_tree) so the per-element list can be variable length.
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

import os
import struct
from datetime import datetime

import System
import compas
from compas.geometry import Frame, Point, Vector

import Rhino
import Rhino.Geometry as rg

from Grasshopper import DataTree
from Grasshopper.Kernel.Data import GH_Path


# ==============================================================================
# Index mapping: JSON side
# ==============================================================================
INDEX_MAP = {
    4: "place_position",
    5: "cut_position_a",
    6: "cut_position_b",
}
BEAM_SIZE_INDEX = 7

# ==============================================================================
# Index mapping: STL side
# ==============================================================================
BREP_INDEX = 0
CENTERLINE_INDEX = 1
CUT_PLANE_A_INDEX = 2
CUT_PLANE_B_INDEX = 3

# ==============================================================================
# Geometry configuration
# ==============================================================================
BEAM_SECTION = 25.0                        # mm (must match globals.py)
# Stock lengths per beam_size category (must match VALID_CATEGORIES in
# process/_skills/WoodStorage/wood_storage.py).
STOCK_LENGTHS = {
    "400":  400.0,
    "550":  550.0,
    "750":  750.0,
    "1000": 1000.0,
}
VALID_BEAM_SIZES = tuple(STOCK_LENGTHS.keys())
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

def validate_element_basic(element, layer_idx, elem_idx):
    warnings = []
    prefix = "L{} E{}".format(layer_idx, elem_idx)

    for field in ["place_position", "cut_position_a", "cut_position_b"]:
        if field not in element:
            warnings.append("{}: '{}' fehlt!".format(prefix, field))

    beam_size = element.get("beam_size", "")
    if beam_size not in VALID_BEAM_SIZES:
        warnings.append("{}: beam_size '{}' ungueltig (erlaubt: {})".format(
            prefix, beam_size, ", ".join(VALID_BEAM_SIZES)))

    place = element.get("place_position")
    if place is not None:
        x, y = place.point.x, place.point.y
        if x < -10 or x > 2510:
            warnings.append("{}: place X={:.0f} ausserhalb Rahmen (0-2500)".format(prefix, x))
        if y < -610 or y > 10:
            warnings.append("{}: place Y={:.0f} ausserhalb Rahmen (-600 - 0)".format(prefix, y))

    return warnings


# ==============================================================================
# STL export for a single element
# ==============================================================================

def export_element_stls(branch, beam_size, layer_idx, elem_idx, geometry_folder):
    """Export raw/cutA/cutB STLs for one element. Returns (n_files, errors)."""
    prefix = "L{}_E{}".format(layer_idx, elem_idx)
    errors = []

    finished_brep = to_brep(branch[BREP_INDEX]) if BREP_INDEX < len(branch) else None
    centerline = to_line(branch[CENTERLINE_INDEX]) if CENTERLINE_INDEX < len(branch) else None
    cut_plane_a = to_plane_geom(branch[CUT_PLANE_A_INDEX]) if CUT_PLANE_A_INDEX < len(branch) else None

    if finished_brep is None or centerline is None or cut_plane_a is None:
        errors.append("{} STL: missing brep/centerline/cut_plane_a".format(prefix))
        return 0, errors

    if beam_size not in VALID_BEAM_SIZES:
        errors.append("{} STL: invalid beam_size '{}'".format(prefix, beam_size))
        return 0, errors

    grip_center, grip_plane, to_local = make_grip_frame(centerline)

    n_files = 0

    # Raw beam
    raw_brep = make_raw_brep(grip_plane, beam_size)
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

    # Build layers (JSON) and export STLs in one pass
    layers = []
    total_elements = 0
    total_stl_files = 0
    all_warnings = []
    stl_errors = []

    for layer_idx in range(n_layers):
        n_elements = elements_per_layer.get(layer_idx, 0)
        log.append("")
        log.append("Verarbeite Layer {} ({} Elemente)...".format(layer_idx, n_elements))

        elements = []

        for elem_idx in range(n_elements):
            branch = get_branch(fab_data, GH_Path(layer_idx, elem_idx))

            element = {"id": elem_idx}

            # beam_size (string)
            if BEAM_SIZE_INDEX < len(branch) and branch[BEAM_SIZE_INDEX] is not None:
                element["beam_size"] = str(branch[BEAM_SIZE_INDEX])

            # Position frames
            for idx, pos_name in INDEX_MAP.items():
                if idx < len(branch):
                    frame = to_compas_frame(branch[idx])
                    if frame is not None:
                        element[pos_name] = frame

            # Glue positions: variable-length list from glue_planes_tree.
            # Empty branch / missing tree -> empty list (= no gluing for this element).
            glue_branch = get_branch(glue_planes_tree, GH_Path(layer_idx, elem_idx))
            element["glue_positions"] = [
                f for f in (to_compas_frame(p) for p in glue_branch) if f is not None
            ]

            warnings = validate_element_basic(element, layer_idx, elem_idx)
            all_warnings.extend(warnings)

            elements.append(element)

            # STL export (index 11 as single source of truth for beam_size)
            beam_size = element.get("beam_size", "")
            n_stl, errs = export_element_stls(
                branch, beam_size, layer_idx, elem_idx, geometry_folder
            )
            total_stl_files += n_stl
            stl_errors.extend(errs)

        if elements:
            log.append("  Keys: {}".format(list(elements[0].keys())))

        layers.append({"id": layer_idx, "elements": elements})
        total_elements += len(elements)

    # Build export structure
    export_data = {
        "layers": layers,
        "metadata": {
            "created": datetime.now().isoformat(),
            "layer_count": len(layers),
            "total_element_count": total_elements,
            "version": "3.0",
            "project": "facade",
            "frame_size": [600, 2500],
            "beam_section": 25,
        },
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

    # Write JSON
    compas.json_dump(export_data, file_path, pretty=True)

    # Summary
    log.append("")
    log.append("JSON: {}".format(file_path))
    log.append("STLs: {} ({} files)".format(geometry_folder, total_stl_files))
    for layer in layers:
        log.append("  Layer {}: {} Elemente".format(layer["id"], len(layer["elements"])))

    suffix = ""
    if all_warnings or stl_errors:
        suffix = " ({} warnings, {} stl errors)".format(len(all_warnings), len(stl_errors))

    output = "Done - {} elements, {} STLs{}".format(
        total_elements, total_stl_files, suffix
    )

    details = "\n".join(log)

else:
    output = "Press Button to Export"
    details = ""
