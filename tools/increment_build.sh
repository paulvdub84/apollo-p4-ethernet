#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_FILE="$ROOT_DIR/build_number.txt"
VERSION_FILE="$ROOT_DIR/version.txt"
OUT_FILE="$ROOT_DIR/main/build_version.h"

# Init build number if missing
if [ ! -f "$BUILD_FILE" ]; then
  echo "0" > "$BUILD_FILE"
fi

BUILD_NUM=$(cat "$BUILD_FILE")
BUILD_NUM=$((BUILD_NUM + 1))
echo "$BUILD_NUM" > "$BUILD_FILE"

VERSION=$(cat "$VERSION_FILE")

cat > "$OUT_FILE" <<EOF
#pragma once
#define APP_VERSION_STRING "${VERSION}.${BUILD_NUM}"
#define APP_BUILD_NUMBER ${BUILD_NUM}
EOF

echo "Build version: ${VERSION}.${BUILD_NUM}"
