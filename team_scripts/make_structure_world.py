#!/usr/bin/env python3
"""Create a Gazebo world containing the building structure and room furniture."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys
import xml.dom.minidom
import xml.etree.ElementTree as ET


FLOOR_RE = re.compile(r"(?:^|_)floor_(\d+)(?:_|$)")
ROOM_LINK_RE = re.compile(r"^floor_\d+_room_\d+_")
OBSTACLE_MODEL_PREFIXES = (
    "danger_red_sphere",
    "distractor_red_box",
    "distractor_green_sphere",
)
DEFAULT_UPPER_TRANSPARENCY = 0.80
DEFAULT_CAMERA_POSE = (24.0, 30.0, 30.0, 0.0, 0.65, -2.0)


def _floor_index(name: str) -> int | None:
    match = FLOOR_RE.search(name)
    return int(match.group(1)) if match else None


def _is_furniture_link(name: str) -> bool:
    # Room-shell links all contain "_wall". Other links under a room are the
    # generated furniture set, which is useful for checking visual occlusion.
    return bool(ROOM_LINK_RE.match(name)) and "_wall" not in name


def _set_visual_transparency(element: ET.Element, transparency: float) -> int:
    changed = 0
    for visual in element.findall(".//visual"):
        node = visual.find("transparency")
        if node is None:
            node = ET.SubElement(visual, "transparency")
        node.text = f"{transparency:.2f}"
        changed += 1
    return changed


def _pretty_xml(root: ET.Element) -> str:
    raw = ET.tostring(root, encoding="utf-8")
    document = xml.dom.minidom.parseString(raw)
    return document.toprettyxml(indent="  ", encoding="utf-8").decode("utf-8")


def make_structure_world(
    input_path: Path,
    output_path: Path,
    upper_transparency: float,
    camera_pose: tuple[float, float, float, float, float, float],
) -> dict[str, object]:
    tree = ET.parse(input_path)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"world element is missing in {input_path}")

    preserved_obstacle_models = 0
    preserved_furniture_links = 0
    upper_floor_visuals = 0

    for model in list(world):
        if model.tag != "model":
            continue
        model_name = model.get("name", "")
        if model_name.startswith(OBSTACLE_MODEL_PREFIXES):
            # Keep every obstacle opaque so red/green markers stay easy to
            # identify on upper floors too.
            preserved_obstacle_models += 1
            continue
        model_floor = _floor_index(model_name)
        if model_floor is not None and model_floor > 0:
            upper_floor_visuals += _set_visual_transparency(model, upper_transparency)

        if model_name != "generated_building":
            continue
        for link in model.findall("link"):
            link_name = link.get("name", "")
            if _is_furniture_link(link_name):
                preserved_furniture_links += 1
            link_floor = _floor_index(link_name)
            if link_name == "roof" or (link_floor is not None and link_floor > 0):
                upper_floor_visuals += _set_visual_transparency(link, upper_transparency)

    gui = world.find("gui")
    if gui is None:
        gui = ET.SubElement(world, "gui", {"fullscreen": "false"})
    else:
        for camera in list(gui.findall("camera")):
            gui.remove(camera)
    camera = ET.SubElement(gui, "camera", {"name": "user_camera"})
    ET.SubElement(camera, "pose").text = " ".join(f"{value:.4f}" for value in camera_pose)
    ET.SubElement(camera, "view_controller").text = "orbit"

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(_pretty_xml(root), encoding="utf-8")
    return {
        "input_world": str(input_path),
        "output_world": str(output_path),
        "preserved_obstacle_models": preserved_obstacle_models,
        "preserved_furniture_links": preserved_furniture_links,
        "upper_floor_visuals": upper_floor_visuals,
        "upper_floor_transparency": upper_transparency,
        "camera_pose": list(camera_pose),
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--upper-transparency",
        type=float,
        default=DEFAULT_UPPER_TRANSPARENCY,
        help="Gazebo visual transparency for floors above floor 0 (0=opaque, 1=hidden).",
    )
    parser.add_argument(
        "--camera-pose",
        type=float,
        nargs=6,
        default=DEFAULT_CAMERA_POSE,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if not 0.0 <= args.upper_transparency <= 1.0:
        print("--upper-transparency must be in [0, 1]", file=sys.stderr)
        return 2
    summary = make_structure_world(
        args.input,
        args.output,
        args.upper_transparency,
        tuple(args.camera_pose),
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
