#!/usr/bin/env python3
"""Generate named GO2 occupancy-map assets using atomic replacements."""

import argparse
import os
from pathlib import Path
import subprocess
import tempfile

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "src" / "go2_map_tools" / "config"


def format_ros_value(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def build_converter_command(
    source: Path,
    pgm: Path,
    yaml_path: Path,
    config_path: Path,
):
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    command = [
        "rosrun", "go2_map_tools", "pcd_to_pgm_node",
        f"_input_pcd:={source}", f"_output_pgm:={pgm}",
        f"_output_yaml:={yaml_path}",
    ]
    path_keys = {"input_pcd", "output_pgm", "output_yaml"}
    for key in sorted(config):
        if key in path_keys:
            continue
        value = config[key]
        if isinstance(value, (dict, list)):
            raise ValueError(f"unsupported nested converter parameter: {key}")
        command.append(f"_{key}:={format_ros_value(value)}")
    return command


def convert(
    source: Path,
    pgm: Path,
    yaml_path: Path,
    config_path: Path,
) -> None:
    subprocess.run(
        build_converter_command(source, pgm, yaml_path, config_path),
        check=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("map_name")
    parser.add_argument(
        "--map-root", type=Path,
        default=Path(os.environ.get("GO2_MAP_ROOT", "~/go2_mid360_nav/maps")).expanduser(),
    )
    args = parser.parse_args()
    map_dir = (args.map_root / args.map_name).resolve()
    source = map_dir / "public_map.pcd"
    if not source.is_file():
        raise SystemExit(f"missing saved point-cloud map: {source}")

    map_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".finalize-", dir=str(map_dir)) as raw:
        temp_dir = Path(raw)
        outputs = (
            ("map_raw.pgm", "map_raw.yaml", "corridor_raw.yaml"),
            ("map.pgm", "map.yaml", "corridor_nav.yaml"),
        )
        for pgm_name, yaml_name, config_name in outputs:
            convert(
                source,
                temp_dir / pgm_name,
                temp_dir / yaml_name,
                CONFIG_DIR / config_name,
            )
        for name in ("map_raw.pgm", "map_raw.yaml", "map.pgm", "map.yaml"):
            os.replace(temp_dir / name, map_dir / name)
    print(f"finalized map assets in {map_dir}")


if __name__ == "__main__":
    main()
