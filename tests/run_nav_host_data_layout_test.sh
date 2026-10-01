#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
MODULE_DIR=$(dirname -- "$SCRIPT_DIR")
BUILD_DIR=$(mktemp -d /tmp/nav-host-data-layout-test.XXXXXX)
trap 'rm -rf "$BUILD_DIR"' EXIT

g++ -std=c++20 -Wall -Wextra -Werror \
  -I"$MODULE_DIR" \
  -I"$MODULE_DIR/../../Middlewares/Third_Party/LibXR/src/core" \
  -I"$MODULE_DIR/../../Middlewares/Third_Party/LibXR/src/core/assert" \
  "$SCRIPT_DIR/nav_host_data_layout_test.cpp" \
  -o "$BUILD_DIR/nav_host_data_layout_test"

"$BUILD_DIR/nav_host_data_layout_test"
