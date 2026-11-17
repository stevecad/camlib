"""Update selected workload expectations from a successful benchmark result."""

import argparse
import json
from pathlib import Path
import re
from tempfile import NamedTemporaryFile

from compare import OPERATION_FIELDS, WORKLOAD_FIELDS

DIRECTORY = Path(__file__).resolve().parent / "expectations"


def project(workload):
    toolpath = workload["toolpath"]
    if toolpath["post_error"] is not None or toolpath["posted_blocks"] is None:
        raise ValueError(f"{workload['name']}: posting failed")
    expected = {key: toolpath[key] for key in WORKLOAD_FIELDS}
    material = toolpath["material"]
    expected["material"] = (
        {key: material[key] for key in ("leftover_mm3", "overcut_mm3")} if material is not None else None
    )
    operations = {}
    for name, op in workload["operations"].items():
        operations[name] = {key: op[key] for key in OPERATION_FIELDS}
        if "engagement" in op:
            operations[name]["engagement"] = op["engagement"]
    return dict(name=workload["name"], operations=operations, toolpath=expected)


def update(data, names, directory=DIRECTORY):
    from workloads import BY_NAME

    if data["failures"]:
        raise ValueError("cannot update expectations from a failed run")
    rows = {w["name"]: w for w in data["workloads"]}
    if len(rows) != len(data["workloads"]):
        raise ValueError("duplicate workload measurement")
    files = {}
    for name in names:
        if name not in rows or name not in BY_NAME or not re.fullmatch(r"[a-zA-Z0-9_-]+", name):
            raise ValueError(f"unknown or missing workload: {name}")
        if set(rows[name]["operations"]) != set(BY_NAME[name].toolpaths):
            raise ValueError(f"{name}: incomplete operation measurements")
        files[directory / f"{name}.json"] = (
            json.dumps(project(rows[name]), indent=2, sort_keys=True, allow_nan=False) + "\n"
        )
    directory.mkdir(exist_ok=True)
    for path, content in files.items():
        with NamedTemporaryFile(mode="w", encoding="utf-8", dir=directory, delete=False) as temp:
            temp.write(content)
        Path(temp.name).replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result", type=Path)
    parser.add_argument("workloads", nargs="+")
    args = parser.parse_args()
    try:
        update(json.loads(args.result.read_text(encoding="utf-8")), args.workloads)
    except (KeyError, ValueError) as error:
        parser.exit(1, f"invalid benchmark measurement: {error}\n")


if __name__ == "__main__":
    main()
