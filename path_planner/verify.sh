#!/usr/bin/env bash
set -euo pipefail

cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/path_planner maps/map_public.txt 0 0 9 7 0
./build/path_planner maps/map_no_path.txt 0 0 4 4 0
