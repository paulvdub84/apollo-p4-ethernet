#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTDIR="${ROOT}/main/generated"
OUT="${OUTDIR}/build_info.h"
COUNTER_FILE="${ROOT}/tools/build_number.txt"

mkdir -p "${OUTDIR}"

# Increment build number
BUILD_NUM="$(cat "${COUNTER_FILE}" 2>/dev/null || echo 0)"
BUILD_NUM=$((BUILD_NUM + 1))
echo "${BUILD_NUM}" > "${COUNTER_FILE}"

# Version string from git describe (fallback if not a repo)
GIT_VER="unknown"
if command -v git >/dev/null 2>&1 && git -C "${ROOT}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  GIT_VER="$(git -C "${ROOT}" describe --tags --always --dirty 2>/dev/null || echo unknown)"
fi

TS="$(date -u +"%Y-%m-%dT%H:%M:%SZ")"

cat > "${OUT}" <<HDR
#pragma once

// Auto-generated. Do not edit.
#define APP_VERSION_STRING "${GIT_VER}"
#define APP_BUILD_NUMBER   ${BUILD_NUM}
#define APP_BUILD_UTC      "${TS}"
HDR

echo "Generated: ${OUT}"
echo "APP_VERSION_STRING=${GIT_VER}"
echo "APP_BUILD_NUMBER=${BUILD_NUM}"
echo "APP_BUILD_UTC=${TS}"
