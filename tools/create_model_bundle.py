#!/usr/bin/env python3
"""Create the OpenArmX robot-model plugin without installing it on target."""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml

from humanoid_manager.deployment import DeploymentError, pack_directory


def _arguments() -> argparse.Namespace:
    script_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--profile-dir",
        type=Path,
        default=script_root / "deployment/openarmx_v10_bimanual",
    )
    parser.add_argument(
        "--description-share",
        type=Path,
        help="Defaults to the installed openarmx_description package share.",
    )
    return parser.parse_args()


def _copy_file(source: Path, destination: Path) -> None:
    if not source.is_file():
        raise DeploymentError(f"OpenArmX resource is missing: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, destination)


def _write_kinematic_urdf(source: Path, destination: Path) -> None:
    """Keep the model needed by motion control without packaging visual assets."""
    if not source.is_file():
        raise DeploymentError(f"OpenArmX URDF is missing: {source}")
    try:
        tree = ET.parse(source)
    except (OSError, ET.ParseError) as error:
        raise DeploymentError(f"failed to parse OpenArmX URDF: {error}") from error
    for link in tree.getroot().findall("link"):
        for tag in ("visual", "collision"):
            for element in list(link.findall(tag)):
                link.remove(element)
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def main() -> int:
    arguments = _arguments()
    profile = arguments.profile_dir.resolve()
    description_share = (
        arguments.description_share
        if arguments.description_share is not None
        else Path(get_package_share_directory("openarmx_description"))
    ).resolve()

    with tempfile.TemporaryDirectory(prefix="openarmx_robot_bundle_") as temporary:
        root = Path(temporary) / "bundle"
        resources = root / "resources"
        resource_sources = {
            "motion_params": (profile / "model/motion.yaml", "motion.yaml"),
            "sdk_config": (profile / "model/sdk.yaml", "sdk.yaml"),
            "channel_config": (profile / "model/channels.yaml", "channels.yaml"),
            "tool_config": (profile / "model/tools.yaml", "tools.yaml"),
        }
        receiver_config = profile / "model/hc_teleop.yaml"
        if receiver_config.is_file():
            resource_sources["hc_teleop_config"] = (receiver_config, "hc_teleop.yaml")
        manifest_resources: dict[str, str] = {}
        for key, (source, name) in resource_sources.items():
            _copy_file(source, resources / name)
            manifest_resources[key] = f"resources/{name}"
        _write_kinematic_urdf(
            description_share / "urdf/robot/openarmx_robot.urdf",
            resources / "openarmx_robot.urdf",
        )
        manifest_resources["urdf"] = "resources/openarmx_robot.urdf"

        manifest = {
            "schema_version": 1,
            "artifact_type": "plugin",
            "plugin_type": "robot_model",
            "plugin_id": "openarmx_v10_bimanual_model",
            "name": "OpenArmX v10 bimanual kinematic model",
            "resources": manifest_resources,
        }
        (root / "manifest.yaml").write_text(
            yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8"
        )
        pack_directory(root, arguments.output)
    print(arguments.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
