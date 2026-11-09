#!/usr/bin/env bash
# Create a self-contained build toolchain in ./.toolchain: OpenCASCADE (without VTK), CMake,
# Ninja and a C++ compiler from conda-forge, installed with micromamba. Needs no root access
# and writes nothing outside this directory.
#
#   scripts/setup-toolchain.sh                      # OCCT 8.0.0
#   OCCT_VERSION=7.9.3 scripts/setup-toolchain.sh   # another OCCT version
#   TOOLCHAIN_PREFIX=/opt/occt79 OCCT_VERSION=7.9.3 scripts/setup-toolchain.sh
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
prefix="${TOOLCHAIN_PREFIX:-$root/.toolchain}"
case "$prefix" in /*) ;; *) prefix="$PWD/$prefix" ;; esac  # micromamba reads bare names as env names
occt_version="${OCCT_VERSION:-8.0.0}"
mamba="$root/.toolchain-bin/micromamba"

if [ ! -x "$mamba" ]; then
    case "$(uname -s)-$(uname -m)" in
        Linux-x86_64) platform=linux-64 ;;
        Linux-aarch64) platform=linux-aarch64 ;;
        Darwin-x86_64) platform=osx-64 ;;
        Darwin-arm64) platform=osx-arm64 ;;
        *) echo "Unsupported platform: $(uname -s)-$(uname -m)" >&2; exit 1 ;;
    esac
    mkdir -p "$root/.toolchain-bin"
    curl -fsSL "https://micro.mamba.pm/api/micromamba/$platform/latest" |
        tar -xj -C "$root/.toolchain-bin" --strip-components=1 bin/micromamba
fi

# OCCT before 7.8 links the system OpenGL and X11 libraries; bring them along.
extra=()
case "$occt_version" in
    7.[0-7].*) extra=(libgl-devel xorg-libx11) ;;
esac

export MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-$root/.toolchain-bin/root}"
"$mamba" create -y -p "$prefix" -c conda-forge "occt==$occt_version=novtk*" cmake ninja cxx-compiler \
    ${extra[@]+"${extra[@]}"}

cat <<EOF

Toolchain ready in $prefix. Build with:
  $mamba run -p $prefix cmake -S $root -B $root/build -G Ninja -DCMAKE_PREFIX_PATH=$prefix
  $mamba run -p $prefix cmake --build $root/build
EOF
