#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
MODULE_DIR=$(dirname -- "$SCRIPT_DIR")
BUILD_DIR=$(mktemp -d /tmp/nav-host-data-layout-test.XXXXXX)
trap 'rm -rf "$BUILD_DIR"' EXIT

g++ -DHOST_DATA_LINK_PROTOCOL_ONLY -std=c++20 -Wall -Wextra -Werror \
  -I"$MODULE_DIR" \
  -I"$MODULE_DIR/../../.tooling/LibXR-v2.0.1/src/core" \
  -I"$MODULE_DIR/../../.tooling/LibXR-v2.0.1/src/core/assert" \
  "$SCRIPT_DIR/host_data_link_layout_test.cpp" \
  -o "$BUILD_DIR/host_data_link_layout_test"

"$BUILD_DIR/host_data_link_layout_test"
