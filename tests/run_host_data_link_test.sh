#!/usr/bin/env bash
set -euo pipefail
BSP_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)
cmake -S "$BSP_ROOT/tools/communication_tests" -B "$BSP_ROOT/build/communication_tests" -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build "$BSP_ROOT/build/communication_tests" --parallel 4
ctest --test-dir "$BSP_ROOT/build/communication_tests" --output-on-failure
