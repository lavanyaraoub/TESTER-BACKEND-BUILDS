#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"

export LD_LIBRARY_PATH="${project_root}:/opt/etherlab/lib:/mnt/app/jamun${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export CGO_CFLAGS="${CGO_CFLAGS:-} -I${project_root} -I/opt/etherlab/include"

# Functional tests without the unsupported race detector.
go test -mod=vendor \
    motordriver/rotary_coordinates.go \
    motordriver/rotary_coordinates_test.go

go test -mod=vendor ./settings \
    -run 'TestHomingGeometry|TestUISettingsCannotOverwrite' \
    -count=1

# Run focused motion tests without the unrelated broken baseline tests.
python3 - <<'PY'
import json
import subprocess

info = json.loads(subprocess.check_output([
    "go", "list", "-mod=vendor", "-tags", "unit",
    "-json", "./motordriver"
]))

files = [
    "motordriver/" + name
    for name in info["GoFiles"] + info.get("CgoFiles", [])
]

files += [
    "motordriver/mock_driver_test.go",
    "motordriver/drive_angle_manipulation_test.go",
    "motordriver/rotary_axis_test.go",
    "motordriver/rotary_coordinates_test.go",
]

subprocess.run([
    "go", "test", "-mod=vendor",
    "-tags", "unit",
    "-run",
    "TestContinuous|TestEndpointResidual|TestFixedEndpoint|TestCalculateFixedAbsoluteGoal",
    "-count=1",
] + files, check=True)
PY

printf '\nAll focused functional tests passed.\n'