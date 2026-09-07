#!/usr/bin/env python3
"""Package an installed OpenArmX driver prefix for no-build target deployment."""

from __future__ import annotations

import argparse
from pathlib import Path
import platform
import shutil
import tempfile

import yaml

from humanoid_manager.deployment import DeploymentError, pack_directory


PACKAGE = "openarmx_driver"
PLUGIN_CLASS = "openarmx_driver/OpenArmXRos2ControlDriver"
LIBRARY_NAME = "libopenarmx_ros2_control_driver.so"
DRIVER_CONFIG = "openarmx_v10_bimanual.yaml"


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "installed_prefix",
        type=Path,
        help="Isolated colcon prefix returned by `ros2 pkg prefix openarmx_driver`.",
    )
    parser.add_argument("output", type=Path)
    return parser.parse_args()


def _copy(source: Path, destination: Path) -> None:
    if not source.is_file():
        raise DeploymentError(f"installed OpenArmX driver file is missing: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, destination)


def main() -> int:
    arguments = _arguments()

    installed = arguments.installed_prefix.resolve()
    package_xml = installed / f"share/{PACKAGE}/package.xml"

    with tempfile.TemporaryDirectory(prefix="openarmx_driver_bundle_") as temporary:
        root = Path(temporary) / "bundle"
        prefix = root / "prefix"
        _copy(package_xml, prefix / f"share/{PACKAGE}/package.xml")
        _copy(
            installed / f"share/{PACKAGE}/plugins/openarmx_driver_plugins.xml",
            prefix / f"share/{PACKAGE}/plugins/openarmx_driver_plugins.xml",
        )
        _copy(
            installed / f"share/{PACKAGE}/config/{DRIVER_CONFIG}",
            prefix / f"share/{PACKAGE}/config/{DRIVER_CONFIG}",
        )
        _copy(
            installed / f"share/ament_index/resource_index/packages/{PACKAGE}",
            prefix / f"share/ament_index/resource_index/packages/{PACKAGE}",
        )
        _copy(installed / f"lib/{LIBRARY_NAME}", prefix / f"lib/{LIBRARY_NAME}")

        architecture = platform.machine().lower()
        architecture = {"amd64": "x86_64", "arm64": "aarch64"}.get(
            architecture, architecture
        )
        manifest = {
            "schema_version": 1,
            "artifact_type": "plugin",
            "plugin_type": "hardware_driver",
            "plugin_id": PACKAGE,
            "name": "OpenArmX ROS 2 control driver",
            "compatibility": {
                "ros_distro": "humble",
                "architecture": architecture,
                "driver_interface_abi": 1,
            },
            "package_name": PACKAGE,
            "ament_prefix": "prefix",
            "plugin_xml": f"prefix/share/{PACKAGE}/plugins/openarmx_driver_plugins.xml",
            "library": f"prefix/lib/{LIBRARY_NAME}",
            "plugin_class": PLUGIN_CLASS,
            "resources": {
                "driver_params": f"prefix/share/{PACKAGE}/config/{DRIVER_CONFIG}",
            },
        }
        root.mkdir(parents=True, exist_ok=True)
        (root / "manifest.yaml").write_text(
            yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8"
        )
        pack_directory(root, arguments.output)
    print(arguments.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
