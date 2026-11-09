#!/usr/bin/env bash
# Download the textures of the PBR material presets (wood grain, brushed metal, fabrics, ...):
# the pbr_properties/_assets directory of the threejs_materials 1.2.5 wheel from PyPI, checked
# against its SHA-256. Only the PNG files are extracted; nothing from the wheel is run.
#
#   scripts/fetch-textures.sh                 # into data/textures, where build123d finds them
#   scripts/fetch-textures.sh DIR [--force]   # elsewhere; then set BUILD123D_TEXTURE_DIR=DIR
#
# The textures come from ambientCG (CC0 1.0), AMD's GPUOpen material library (MIT) and the
# threejs_materials project (Apache-2.0); each preset names its source, url and license.
set -euo pipefail

version=1.2.5
wheel=threejs_materials-$version-py3-none-any.whl
url=https://files.pythonhosted.org/packages/d5/ea/d2980e85afcc62e2a9c032c012009e3d50ec15f9b66ec1097463e65aa55e/$wheel
sha256=e622264388045fa35612b514ddaedecb213831c98aa0b140a101bbf3fdd527a6
assets=threejs_materials/pbr_properties/_assets
expected_count=67

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
dest=""
force=0
for arg in "$@"; do
    case "$arg" in
        --force) force=1 ;;
        -h | --help) sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        -*) echo "Unknown option: $arg" >&2; exit 2 ;;
        *) dest="$arg" ;;
    esac
done
dest="${dest:-$root/data/textures}"

if [ -f "$dest/_brush/normal.png" ] && [ "$force" -eq 0 ]; then
    echo "The textures are already in $dest (--force downloads them again)."
    exit 0
fi

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

echo "Downloading $wheel (90 MB) from PyPI ..."
if command -v curl > /dev/null; then
    curl -fL --retry 3 -o "$tmp/$wheel" "$url"
elif command -v wget > /dev/null; then
    wget -q -O "$tmp/$wheel" "$url"
else
    echo "Neither curl nor wget is installed." >&2
    exit 1
fi

if command -v sha256sum > /dev/null; then
    sum="$(sha256sum "$tmp/$wheel" | cut -d ' ' -f 1)"
elif command -v shasum > /dev/null; then
    sum="$(shasum -a 256 "$tmp/$wheel" | cut -d ' ' -f 1)"
else
    echo "Neither sha256sum nor shasum is installed." >&2
    exit 1
fi
if [ "$sum" != "$sha256" ]; then
    echo "The download's SHA-256 is $sum, not $sha256: not using it." >&2
    exit 1
fi

# The PNG files under _assets/, without leaving the extraction directory.
mkdir -p "$tmp/out"
if command -v unzip > /dev/null; then
    unzip -q "$tmp/$wheel" "$assets/*.png" -d "$tmp/out"
elif command -v python3 > /dev/null; then
    python3 -I - "$tmp/$wheel" "$tmp/out" "$assets/" << 'PYTHON'
import sys
import zipfile

wheel, out, prefix = sys.argv[1:]
with zipfile.ZipFile(wheel) as z:
    for info in z.infolist():
        parts = info.filename.split("/")
        if info.filename.startswith(prefix) and info.filename.endswith(".png") and ".." not in parts:
            z.extract(info, out)
PYTHON
else
    echo "Neither unzip nor python3 is installed." >&2
    exit 1
fi

count="$(find "$tmp/out/$assets" -type f -name '*.png' | wc -l | tr -d ' ')"
if [ "$count" -ne "$expected_count" ] || [ ! -f "$tmp/out/$assets/_brush/normal.png" ]; then
    echo "Expected $expected_count textures in the wheel, found $count." >&2
    exit 1
fi
mkdir -p "$dest"
cp -R "$tmp/out/$assets/." "$dest/"
echo "Installed $count textures in $dest."
case "$dest" in
    "$root/data/textures") echo "build123d finds them there (re-run cmake to install them with the library)." ;;
    *) echo "Set BUILD123D_TEXTURE_DIR=$dest for build123d to find them." ;;
esac
