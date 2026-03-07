#!/bin/bash

STEP_FILE=$1
BASE_NAME=$(basename "$STEP_FILE" .step)

STL_FILE="${BASE_NAME}.stl"
DAE_FILE="${BASE_NAME}.dae"

echo "STEP → STL ..."
freecadcmd step_to_stl.py "$STEP_FILE" "$STL_FILE"

echo "STL → DAE ..."
blender --background --python stl_to_dae.py -- "$STL_FILE" "$DAE_FILE"

echo "Done."
echo "Output: $DAE_FILE"
