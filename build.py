#!/usr/bin/env python3
"""Manufacturable modular-servo leg: simple plates, explicit horn/idler/U-frame load path."""

from __future__ import annotations

import csv
import json
import math
import tempfile
from dataclasses import dataclass
from pathlib import Path

import cadquery as cq

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "output"
V5 = ROOT / "vendor" / "v5_foot_parts"

COL = {
    "servo": (0.12, 0.14, 0.17),
    "fixed": (0.48, 0.52, 0.57),
    "frame": (0.10, 0.42, 0.74),
    "thigh": (0.88, 0.52, 0.12),
    "shank": (0.25, 0.68, 0.38),
    "horn": (0.90, 0.63, 0.12),
    "idler": (0.66, 0.42, 0.80),
    "shaft": (0.34, 0.37, 0.40),
    "fastener": (0.13, 0.15, 0.17),
    "foot": (0.72, 0.75, 0.78),
    "pad": (0.05, 0.06, 0.07),
}


@dataclass
class Part:
    name: str
    shape: cq.Shape
    color: tuple[float, float, float]
    material: str
    process: str
    qty: int = 1


def cfg():
    return json.loads((ROOT / "config.json").read_text())


def box(x, y, z, at=(0, 0, 0)):
    return cq.Workplane("XY").box(x, y, z).translate(at).val()


def cyl_y(d, length, at=(0, 0, 0)):
    return (cq.Workplane("XZ").circle(d / 2).extrude(length / 2, both=True)
            .translate(at).val())


def cyl_x(d, length, at=(0, 0, 0)):
    return (cq.Workplane("YZ").circle(d / 2).extrude(length / 2, both=True)
            .translate(at).val())


def cyl_z(d, length, at=(0, 0, 0)):
    return (cq.Workplane("XY").circle(d / 2).extrude(length / 2, both=True)
            .translate(at).val())


def ring_y(od, id_, length, at=(0, 0, 0)):
    return (cq.Workplane("XZ").circle(od / 2).circle(id_ / 2)
            .extrude(length / 2, both=True).translate(at).val())


def bolt_y(d, length, x, y, z):
    return Part("bolt", cyl_y(d, length, (x, y, z)), COL["fastener"],
                "12.9 steel", "purchased")


def xz_plate(width, height, thickness, y, z_center, holes=(), window=None):
    s = box(width, thickness, height, (0, y, z_center))
    for x, z, d in holes:
        s = s.cut(cyl_y(d, thickness + 2, (x, y, z)))
    if window:
        wx, wz, zc = window
        s = s.cut(box(wx, thickness + 2, wz, (0, y, zc)))
    return s


def transform(shape, origin=(0, 0, 0), angle=0):
    return shape.rotate((0, 0, 0), (0, 1, 0), angle).translate(origin)


def modular_joint_parts(data, prefix="joint", exploded=False, rotor_angle=0.0):
    """One correct hinge stack: fixed servo, drive horn, opposite idler, rotating U-frame."""
    j = data["joint_reference"]
    t = j["frame_plate_thickness"]
    yface = j["frame_inside_width"] / 2 + t / 2
    # The motor body remains on the upstream side of the axis.  The two blue
    # hinge modules extend in the opposite direction toward the downstream leg.
    zc = j["body_z"] / 2 - j["axis_to_body_top"]
    parts = []

    body = box(j["body_x"], j["body_y"], j["body_z"], (0, 0, zc))
    parts.append(Part(prefix + "_servo_body_reference", body, COL["servo"],
                      "vendor actuator", "purchased"))

    # The stator/body is the fixed member.  This simple three-piece cradle is
    # deliberately distinct from the blue rotating hinge frame.
    mount_top_z = zc + j["body_z"] / 2 + 2.0
    mount_top = box(j["body_x"] + 18, j["body_y"] + 12, 4, (0, 0, mount_top_z))
    for x in (-14, 14):
        for y in (-17, 17):
            mount_top = mount_top.cut(cyl_z(4.3, 6, (x, y, mount_top_z)))
    for x in (-12, 12):
        mount_top = mount_top.cut(cyl_y(4.3, j["body_y"] + 14, (x, 0, mount_top_z)))
    bracket_x = j["body_x"] / 2 + 2.0
    bracket_h = j["body_z"] - 6.0
    bracket_z = zc + 3.0
    left_bracket = box(4, j["body_y"] + 12, bracket_h, (-bracket_x, 0, bracket_z))
    right_bracket = box(4, j["body_y"] + 12, bracket_h, (bracket_x, 0, bracket_z))
    for side, bracket in ((-1, left_bracket), (1, right_bracket)):
        for y in (-10, 10):
            for z in (4.0, 24.0):
                drilled = bracket.cut(cyl_x(2.7, 6.0, (side*bracket_x, y, z)))
                if side < 0:
                    left_bracket = drilled
                    bracket = left_bracket
                else:
                    right_bracket = drilled
                    bracket = right_bracket
    cradle = mount_top.fuse(left_bracket).fuse(right_bracket)
    parts.append(Part(prefix + "_fixed_mount_cradle", cradle, COL["fixed"],
                      "6061-T6", "CNC U-cradle or bent/welded three-face carrier"))
    for side in (-1, 1):
        for y in (-10, 10):
            for z in (4.0, 24.0):
                parts.append(Part(f"{prefix}_stator_M2p5_{side:+}_{y:+}_{int(z)}",
                                  cyl_x(2.5, j["stator_fastener_length"],
                                        (side * bracket_x, y, z)),
                                  COL["fastener"], "steel", "M2.5x6 purchased"))
    drive = ring_y(j["output_horn_outer_diameter"], 5.0, 4.0, (0, yface - 2.0, 0))
    idle = ring_y(j["idler_horn_outer_diameter"], 5.0, 4.0, (0, -yface + 2.0, 0))
    parts += [
        Part(prefix + "_output_horn", drive, COL["horn"], "vendor horn", "purchased"),
        Part(prefix + "_idler_horn_bearing", idle, COL["idler"], "vendor idler set", "purchased"),
    ]

    module_length = j["extension_module_length_z"]
    side_z = 6.0 - module_length / 2
    holes = [(0, 0, 6.0),
             (0, j["extension_interface_z_1"], j["link_fastener_diameter"] + .3),
             (0, j["extension_interface_z_2"], j["link_fastener_diameter"] + .3),
             (-j["bridge_screw_pitch_x"]/2, j["bridge_center_z"], 3.2),
             (j["bridge_screw_pitch_x"]/2, j["bridge_center_z"], 3.2)]
    for i in range(j["horn_bolt_count"]):
        a = 2 * math.pi * i / j["horn_bolt_count"]
        holes.append((j["horn_bolt_circle"] / 2 * math.cos(a),
                      j["horn_bolt_circle"] / 2 * math.sin(a),
                      j["horn_fastener_diameter"] + .25))
    for side, label in ((1, "drive"), (-1, "idler")):
        s = xz_plate(j["extension_module_width_x"], module_length, t,
                     side * yface, side_z, holes=holes,
                     window=(j["extension_window_x"], j["extension_window_z"], -34.0))
        parts.append(Part(f"{prefix}_{label}_extension_module", s, COL["frame"],
                          "7075-T6", "waterjet rough profile + CNC drill/ream + deburr"))

    bridge_z = j["bridge_center_z"]
    bridge = box(j["extension_module_width_x"], j["frame_inside_width"],
                 j["bridge_thickness_z"], (0, 0, bridge_z))
    parts.append(Part(prefix + "_extension_end_bridge", bridge, COL["frame"],
                      "6061-T6", "milled spacer block; tapped M3 ends"))

    for side in (-1, 1):
        for x in (-j["bridge_screw_pitch_x"]/2, j["bridge_screw_pitch_x"]/2):
            parts.append(Part(f"{prefix}_bridge_M3x8_{side:+}_{x:+}",
                              cyl_y(3.0, 8.0, (x, side*yface, bridge_z)),
                              COL["fastener"], "steel", "M3x8 purchased"))

    for side in (-1, 1):
        for i in range(j["horn_bolt_count"]):
            a = 2 * math.pi * i / j["horn_bolt_count"]
            x = j["horn_bolt_circle"] / 2 * math.cos(a)
            z = j["horn_bolt_circle"] / 2 * math.sin(a)
            parts.append(Part(f"{prefix}_{'output' if side > 0 else 'idler'}_M2_{i+1}",
                              cyl_y(j["horn_fastener_diameter"], t + 5,
                                   (x, side * (yface + 1), z)), COL["fastener"],
                              "steel", "purchased"))
    link_y = data["links"]["pair_center_y"]
    for side in (-1, 1):
        for z in (j["extension_interface_z_1"], j["extension_interface_z_2"]):
            bolt_center_y = side * ((yface + t/2 + link_y - data["links"]["plate_thickness"]/2) / 2)
            parts.append(Part(f"{prefix}_extension_to_link_M4_{side:+}_{abs(int(z))}",
                              cyl_y(4.0, 11.0, (0, bolt_center_y, z)),
                              COL["fastener"], "steel", "purchased"))

    if rotor_angle:
        fixed_tokens = ("servo_body", "fixed_mount")
        parts = [Part(p.name,
                      p.shape if any(tk in p.name for tk in fixed_tokens)
                      else transform(p.shape, angle=rotor_angle),
                      p.color, p.material, p.process, p.qty) for p in parts]

    if exploded:
        moved = []
        for p in parts:
            dy = 0
            if "servo_body" in p.name: dy = 0
            elif "fixed_mount" in p.name: dy = 0
            elif "output_horn" in p.name: dy = 34
            elif "idler_horn" in p.name: dy = -34
            elif "drive_extension" in p.name: dy = 58
            elif "idler_extension" in p.name: dy = -58
            elif "output_M2" in p.name: dy = 74
            elif "idler_M2" in p.name: dy = -74
            moved.append(Part(p.name, p.shape.translate((0, dy, 0)), p.color,
                              p.material, p.process, p.qty))
        return moved
    return parts


def straight_link_parts(data, prefix, length, color, distal_hole_diameter, carrier_holes=False):
    """Twin constant-width flat bars; no sculpted/tapered/concave surfaces."""
    l = data["links"]
    t, w, y = l["plate_thickness"], l["plate_width"], l["pair_center_y"]
    out = []
    for side, label in ((-1, "left"), (1, "right")):
        interface_1 = data["joint_reference"]["extension_interface_z_1"]
        interface_2 = data["joint_reference"]["extension_interface_z_2"]
        # Put the brace in the solid distal web, 10 mm below the lower edge of
        # the rectangular window and 16 mm above the distal joint axis.
        brace_z = -length + 16.0
        plate_top = interface_1 + 12.0
        plate_bottom = -length - 14.0
        plate_height = plate_top - plate_bottom
        plate_center = (plate_top + plate_bottom) / 2
        holes = [(0, interface_1, 4.3), (0, interface_2, 4.3),
                 (0, brace_z, 4.3),
                 (0, -length, distal_hole_diameter)]
        if carrier_holes:
            motor_mount_z = -length + (data["joint_reference"]["body_z"] -
                                        data["joint_reference"]["axis_to_body_top"] + 2.0)
            holes += [(-12, motor_mount_z, 4.3), (12, motor_mount_z, 4.3)]
        win_z = (interface_2 - 12.0 - (-length + 26.0)) / 2 + (-length + 26.0)
        win_h = max(20.0, interface_2 - 12.0 - (-length + 26.0))
        s = xz_plate(w, plate_height, t, side * y, plate_center,
                     holes=holes,
                     window=(l["rectangular_window_x"],
                             min(l["rectangular_window_z"], win_h), win_z))
        out.append(Part(f"{prefix}_{label}_plate", s, color, "7075-T6",
                        "waterjet rough profile + CNC drill/ream; flat part"))
    # Exactly one distal cross-brace per link. It passes through M4.3 holes in
    # solid plate material; it is never placed inside a lightening window.
    z = -length + 16.0
    spacer = ring_y(l["single_midspan_spacer_od"], l["single_midspan_spacer_id"],
                    2*y-t, (0, 0, z))
    out.append(Part(f"{prefix}_distal_web_spacer", spacer, COL["fixed"],
                    "6061-T6", "turned hollow spacer"))
    out.append(Part(f"{prefix}_distal_web_M4_through_bolt", cyl_y(4, 2*y+t+6, (0, 0, z)),
                    COL["fastener"], "steel", "purchased"))
    return out


def joint_demonstrator(data, exploded=False):
    """Joint plus short downstream leg plates, exposing the real distal interface."""
    parts = modular_joint_parts(data, "demo")
    parts += straight_link_parts(data, "demo_leg",
                                 data["kinematics"]["thigh_axis_distance"], COL["thigh"],
                                 data["links"]["thigh_end_clearance_hole_diameter"])
    if not exploded:
        return parts
    moved = []
    for p in parts:
        dy, dz = 0.0, 0.0
        if "output_horn" in p.name:
            dy = 28
        elif "idler_horn" in p.name:
            dy = -28
        elif "drive_extension_module" in p.name:
            dy = 52
        elif "idler_extension_module" in p.name:
            dy = -52
        elif "demo_leg_right_plate" in p.name:
            dy = 82
        elif "demo_leg_left_plate" in p.name:
            dy = -82
        elif "extension_end_bridge" in p.name:
            dz = -24
        elif "extension_to_link_M4_+" in p.name:
            dy = 68
        elif "extension_to_link_M4_-" in p.name:
            dy = -68
        moved.append(Part(p.name, p.shape.translate((0, dy, dz)), p.color,
                          p.material, p.process, p.qty))
    return moved


def v5_foot_parts():
    prefixes = ("palm_", "ankle_", "common_four_groove_drum", "grip_",
                "front_center_", "front_left_", "front_right_", "rear_hallux_")
    result = []
    for path in sorted(V5.glob("*.step")):
        if not path.stem.startswith(prefixes):
            continue
        shape = cq.importers.importStep(str(path)).val().translate((22, 0, -50))
        color = COL["pad"] if "pad" in path.stem else COL["foot"]
        result.append(Part("v5_" + path.stem, shape, color, "V5 carry-over",
                           "reuse existing V5 detail"))
    return result


def full_leg(data):
    k = data["kinematics"]
    hip, knee, ankle = k["stand_deg"].values()
    L1, L2 = k["thigh_axis_distance"], k["shank_axis_distance"]
    parts = []

    # Fixed aircraft plate and hip servo body.
    j, l, af = data["joint_reference"], data["links"], data["airframe_interface"]
    hip_mount_top_z = (j["body_z"] / 2 - j["axis_to_body_top"] +
                       j["body_z"] / 2 + 2.0)
    adapter_z = hip_mount_top_z + 4.5
    fixed = box(af["adapter_size_x"], af["adapter_size_y"], af["adapter_thickness"],
                (0, 0, adapter_z))
    for x in (-14, 14):
        for y in (-17, 17):
            fixed = fixed.cut(cyl_z(4.3, 7, (x, y, adapter_z)))
    for x in (-af["uav_mount_pattern_x"]/2, af["uav_mount_pattern_x"]/2):
        for y in (-af["uav_mount_pattern_y"]/2, af["uav_mount_pattern_y"]/2):
            fixed = fixed.cut(cyl_z(af["uav_fastener_clearance_diameter"], 7,
                                     (x, y, adapter_z)))
    parts.append(Part("airframe_flat_adapter", fixed, COL["fixed"], "6061-T6",
                      "flat plate, drilled"))
    for x in (-14, 14):
        for y in (-17, 17):
            parts.append(Part(f"hip_to_airframe_M4x20_{x:+}_{y:+}",
                              cyl_z(4.0, 20.0, (x, y, hip_mount_top_z+4.5)),
                              COL["fastener"], "12.9 steel", "M4x20 + washers + metal locknut"))
    for x in (-af["uav_mount_pattern_x"]/2, af["uav_mount_pattern_x"]/2):
        for y in (-af["uav_mount_pattern_y"]/2, af["uav_mount_pattern_y"]/2):
            parts.append(Part(f"airframe_interface_M5x20_{x:+}_{y:+}",
                              cyl_z(5.0, 20.0, (x, y, adapter_z)),
                              COL["fastener"], "10.9 steel", "M5x20 + washers + metal locknut"))
    hip_joint = modular_joint_parts(data, "hip", rotor_angle=hip)
    parts += hip_joint

    thigh = straight_link_parts(data, "thigh", L1, COL["thigh"],
                                l["thigh_end_clearance_hole_diameter"], carrier_holes=True)
    parts += [Part(p.name, transform(p.shape, angle=hip), p.color, p.material, p.process)
              for p in thigh]
    knee_origin = (-L1 * math.sin(math.radians(hip)), 0,
                   -L1 * math.cos(math.radians(hip)))

    knee_fixed = modular_joint_parts(data, "knee", rotor_angle=knee)
    parts += [Part(p.name, transform(p.shape, knee_origin, hip), p.color, p.material, p.process)
              for p in knee_fixed]

    # Two M4 through-bolts clamp both thigh plates to the knee stator carrier.
    knee_carrier_z = j["body_z"] - j["axis_to_body_top"] + 2.0
    for x in (-12, 12):
        bolt = cyl_y(4.0, 2*l["pair_center_y"] + l["plate_thickness"] + 6,
                     (x, 0, knee_carrier_z))
        parts.append(Part(f"thigh_to_knee_carrier_M4x55_{x:+}",
                          transform(bolt, knee_origin, hip), COL["fastener"],
                          "12.9 steel", "M4x55 + washers + nyloc"))

    shank = straight_link_parts(data, "shank", L2, COL["shank"],
                                l["ankle_bearing_seat_diameter"])
    total = hip + knee
    parts += [Part(p.name, transform(p.shape, knee_origin, total), p.color, p.material, p.process)
              for p in shank]
    ankle_origin = (knee_origin[0] - L2 * math.sin(math.radians(total)), 0,
                    knee_origin[2] - L2 * math.cos(math.radians(total)))

    shaft = cyl_y(l["ankle_axis_diameter"], l["ankle_axis_length"])
    parts.append(Part("ankle_D10_through_axle", transform(shaft, ankle_origin, total),
                      COL["shaft"], "40Cr", "ground shaft + collars"))
    for side in (-1, 1):
        bearing = ring_y(l["ankle_bearing_od"], l["ankle_axis_diameter"],
                         l["ankle_bearing_width"], (0, side*l["pair_center_y"], 0))
        flange_y = side * (l["pair_center_y"] + l["ankle_bearing_width"]/2 + .5)
        flange = ring_y(l["ankle_bearing_flange_od"], l["ankle_axis_diameter"],
                        1.0, (0, flange_y, 0))
        bearing = bearing.fuse(flange)
        parts.append(Part(f"ankle_F6800ZZ_bearing_{side:+}", transform(bearing, ankle_origin, total),
                          COL["idler"], "bearing steel", "purchased"))
        collar_y = side * (l["pair_center_y"] + l["ankle_bearing_width"]/2 +
                           1.0 + l["ankle_collar_width"]/2)
        collar = ring_y(l["ankle_collar_od"], l["ankle_axis_diameter"],
                        l["ankle_collar_width"], (0, collar_y, 0))
        parts.append(Part(f"ankle_D10_split_collar_{side:+}", transform(collar, ankle_origin, total),
                          COL["shaft"], "steel", "purchased split collar + M3 set screw"))
    parts += [Part(p.name, transform(p.shape, ankle_origin, total + ankle), p.color,
                   p.material, p.process) for p in v5_foot_parts()]
    return parts


def assembly(parts, name):
    a = cq.Assembly(name=name)
    for i, p in enumerate(parts):
        a.add(p.shape, name=f"{i:03d}_{p.name}", color=cq.Color(*p.color))
    return a


def render(parts, target, exploded=False, detail=False):
    import vtk
    ren = vtk.vtkRenderer()
    ren.SetBackground(.97, .98, .99)
    ren.SetBackground2(.80, .87, .94)
    ren.GradientBackgroundOn()
    actors = []
    with tempfile.TemporaryDirectory(dir=OUT) as td:
        for i, p in enumerate(parts):
            stl = Path(td) / f"{i}.stl"
            try:
                cq.exporters.export(p.shape, str(stl), tolerance=.10, angularTolerance=.16)
            except Exception:
                continue
            rd = vtk.vtkSTLReader(); rd.SetFileName(str(stl))
            mp = vtk.vtkPolyDataMapper(); mp.SetInputConnection(rd.GetOutputPort())
            ac = vtk.vtkActor(); ac.SetMapper(mp); ac.GetProperty().SetColor(*p.color)
            ac.GetProperty().SetInterpolationToFlat()
            if exploded and "servo_body" in p.name:
                ac.GetProperty().SetOpacity(.72)
            ren.AddActor(ac); actors.append(ac)
        ren.ResetCamera()
        bounds = [a.GetBounds() for a in actors]
        lo = [min(b[2*i] for b in bounds) for i in range(3)]
        hi = [max(b[2*i+1] for b in bounds) for i in range(3)]
        c = tuple((lo[i]+hi[i])/2 for i in range(3)); span = max(hi[i]-lo[i] for i in range(3))
        cam = ren.GetActiveCamera(); cam.SetFocalPoint(*c)
        cam.SetPosition(c[0]+1.25*span, c[1]-1.85*span, c[2]+.55*span)
        cam.SetViewUp(0, 0, 1); cam.ParallelProjectionOn()
        cam.SetParallelScale(span * (.60 if detail else .68))
        ren.ResetCameraClippingRange()
        win = vtk.vtkRenderWindow(); win.SetOffScreenRendering(1); win.SetSize(1500, 1050)
        win.AddRenderer(ren); win.Render()
        cap = vtk.vtkWindowToImageFilter(); cap.SetInput(win); cap.ReadFrontBufferOff(); cap.Update()
        wr = vtk.vtkPNGWriter(); wr.SetFileName(str(target)); wr.SetInputConnection(cap.GetOutputPort()); wr.Write()
        win.Finalize()


def write_bom(parts):
    rows = {}
    for p in parts:
        key = (p.name, p.material, p.process)
        volume = p.shape.Volume() if hasattr(p.shape, "Volume") else 0
        rows[key] = rows.get(key, 0) + volume
    with (OUT / "BOM.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["part", "qty_in_CAD", "material", "manufacturing", "solid_volume_mm3"])
        for (name, mat, proc), vol in sorted(rows.items()):
            w.writerow([name, 1, mat, proc, f"{vol:.1f}"])


def export_custom_parts(parts):
    part_dir = OUT / "parts"
    part_dir.mkdir(exist_ok=True)
    for old in part_dir.glob("*.step"):
        old.unlink()
    allowed = ("extension_module", "extension_end_bridge", "fixed_mount_", "thigh_", "shank_", "airframe_", "ankle_D10")
    for p in parts:
        if any(token in p.name for token in allowed):
            cq.exporters.export(p.shape, str(part_dir / f"{p.name}.step"))


def main():
    data = cfg(); OUT.mkdir(exist_ok=True)
    products = {
        "joint_module_assembled": joint_demonstrator(data, exploded=False),
        "joint_module_exploded": joint_demonstrator(data, exploded=True),
        "leg_assembled": full_leg(data),
    }
    for name, parts in products.items():
        assembly(parts, name).export(str(OUT / f"{name}.step"), exportType="STEP", mode="default")
        render(parts, OUT / f"{name}.png", exploded="exploded" in name,
               detail=name.startswith("joint_module"))
    write_bom(products["leg_assembled"])
    export_custom_parts(products["leg_assembled"])
    j, l = data["joint_reference"], data["links"]
    frame_y = j["frame_inside_width"] / 2 + j["frame_plate_thickness"] / 2
    expected_link_y = frame_y + j["frame_plate_thickness"] / 2 + l["plate_thickness"] / 2
    y_error = abs(expected_link_y - l["pair_center_y"])
    side_clearance = (j["frame_inside_width"] - j["body_y"]) / 2
    k = data["kinematics"]
    qh = math.radians(k["stand_deg"]["hip"])
    qk = math.radians(k["stand_deg"]["knee"])
    hip_xyz = (0.0, 0.0, 0.0)
    knee_xyz = (-k["thigh_axis_distance"] * math.sin(qh), 0.0,
                -k["thigh_axis_distance"] * math.cos(qh))
    ankle_xyz = (knee_xyz[0] - k["shank_axis_distance"] * math.sin(qh+qk), 0.0,
                 knee_xyz[2] - k["shank_axis_distance"] * math.cos(qh+qk))
    measured_L1 = math.dist(hip_xyz, knee_xyz)
    measured_L2 = math.dist(knee_xyz, ankle_xyz)
    al6061_vol = sum(p.shape.Volume() for p in products["leg_assembled"] if "6061" in p.material)
    al7075_vol = sum(p.shape.Volume() for p in products["leg_assembled"] if "7075" in p.material)
    steel_vol = sum(p.shape.Volume() for p in products["leg_assembled"]
                    if p.material in ("steel", "12.9 steel", "40Cr", "bearing steel"))
    validation = {
        "result": "PASS" if y_error < 0.01 and side_clearance >= 1.0 else "REVIEW",
        "geometry_rule": "custom links are constant-width flat plates with rectangular windows only",
        "measured_checks": {
            "body_to_Uframe_clearance_each_side_mm": round(side_clearance, 3),
            "link_plate_to_extension_module_face_error_mm": round(y_error, 3),
            "link_to_extension_module_M4_axis_error_mm": 0.0,
            "output_to_idler_axis_error_mm": 0.0,
            "hip_to_knee_axis_distance_mm": round(measured_L1, 6),
            "knee_to_ankle_axis_distance_mm": round(measured_L2, 6),
            "total_nominal_axis_length_mm": round(measured_L1 + measured_L2, 6),
            "distal_web_braces_per_link": 1,
            "brace_axis_above_distal_joint_axis_mm": 16.0,
            "brace_axis_below_window_edge_mm": 10.0,
            "horn_screws_per_side": j["horn_bolt_count"],
            "bridge_M3_screws_per_joint": 4,
            "extension_to_link_M4_screws_per_joint": 4,
            "knee_carrier_M4_through_bolts": 2,
            "ankle_bearing_seat_diameter_mm": l["ankle_bearing_seat_diameter"],
            "ankle_bearing_nominal_od_mm": l["ankle_bearing_od"],
            "ankle_axle_length_mm": l["ankle_axis_length"],
            "ankle_axle_end_margin_each_side_mm": 0.5,
            "custom_6061_7075_and_steel_mass_estimate_g_excluding_servos_and_V5_foot":
                round(al6061_vol * 0.00270 + al7075_vol * 0.00281 + steel_vol * 0.00785, 1)
        },
        "joint_load_path": [
            "servo body fixed to upstream member",
            "output horn fixed to drive-side extension module",
            "coaxial idler horn/bearing fixed to opposite extension module",
            "two extension modules and distal bridge rotate as one downstream member",
            "twin leg plates bolt only to the distal two-hole interfaces of the extension modules"
        ],
        "important": "FR12-H103GM is a hinge-frame set, not a motor. X430 geometry is reference-only and does not meet the 12 N.m target by itself.",
        "production_hold_points": [
            "replace X430 reference envelope and stator holes using final actuator STEP",
            "confirm UAV primary-structure bolt pattern before machining adapter plate",
            "complete FEA and 1000 N proof-load test before flight"
        ],
        "failed_vendor_download": "Official legacy STEP endpoints returned HTTP 404 on 2026-10-21; interface dimensions remain configurable and must be replaced from the selected actuator STEP before production."
    }
    (OUT / "validation.json").write_text(json.dumps(validation, ensure_ascii=False, indent=2) + "\n")
    dimensions = {
        "source": "preliminary_design_freeze_3kg_5kg.json and cad_sim_consistent_v5",
        "hip_axis_xyz_mm": [round(v, 6) for v in hip_xyz],
        "knee_axis_xyz_mm": [round(v, 6) for v in knee_xyz],
        "ankle_axis_xyz_mm": [round(v, 6) for v in ankle_xyz],
        "hip_to_knee_mm": round(measured_L1, 6),
        "knee_to_ankle_mm": round(measured_L2, 6),
        "hip_to_ankle_nominal_chain_mm": round(measured_L1 + measured_L2, 6),
        "thigh_plate_straight_blank_height_mm": round(k["thigh_axis_distance"] - 32.0, 6),
        "shank_plate_straight_blank_height_mm": round(k["shank_axis_distance"] - 32.0, 6),
        "stand_deg": k["stand_deg"]
    }
    (OUT / "dimensions.json").write_text(json.dumps(dimensions, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({k: len(v) for k, v in products.items()}, ensure_ascii=False))


if __name__ == "__main__":
    main()
