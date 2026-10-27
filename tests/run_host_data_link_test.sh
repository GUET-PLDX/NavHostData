#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
binary=$(mktemp /tmp/host_data_link_test.XXXXXX)
trap 'rm -f "$binary"' EXIT

"${CXX:-c++}" -std=c++20 -Wall -Wextra -Werror \
  -I"$script_dir/../../../Middlewares/Third_Party/LibXR/src/core" \
  -I"$script_dir/../../../Middlewares/Third_Party/LibXR/src/core/assert" \
  "$script_dir/host_data_link_test.cpp" -o "$binary"
"$binary"

"${PYTHON:-python3}" "$script_dir/host_data_link_config_regression.py"
printf 'PASS: HostDataLink behavior and configuration regression\n'
