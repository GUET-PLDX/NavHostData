#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
binary=$(mktemp /tmp/nav_host_data_test.XXXXXX)
trap 'rm -f "$binary"' EXIT

"${CXX:-c++}" -std=c++20 -Wall -Wextra -Werror \
  -I"$script_dir/../../../Middlewares/Third_Party/LibXR/src/core" \
  -I"$script_dir/../../../Middlewares/Third_Party/LibXR/src/core/assert" \
  "$script_dir/nav_host_data_test.cpp" -o "$binary"
"$binary"

"${PYTHON:-python3}" "$script_dir/nav_host_data_config_regression.py"
printf 'PASS: NavHostData behavior and configuration regression\n'
