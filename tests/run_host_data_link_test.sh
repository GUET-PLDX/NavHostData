#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
binary=$(mktemp /tmp/host_data_link_test.XXXXXX)
trap 'rm -f "$binary"' EXIT

"${CXX:-c++}" -DHOST_DATA_LINK_PROTOCOL_ONLY -std=c++20 -Wall -Wextra -Werror \
  -I"$script_dir/../../../.tooling/LibXR-v2.0.1/src/core" \
  -I"$script_dir/../../../.tooling/LibXR-v2.0.1/src/core/assert" \
  "$script_dir/host_data_link_test.cpp" -o "$binary"
"$binary"

"${PYTHON:-python3}" "$script_dir/host_data_link_config_regression.py"
printf 'PASS: HostDataLink behavior and configuration regression\n'
