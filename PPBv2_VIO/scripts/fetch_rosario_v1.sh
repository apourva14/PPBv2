#!/usr/bin/env bash
# Download one Rosario v1 sequence (bag parts, calibration, ground truth).
# Runs on the host or in the container; resumes interrupted downloads.
#
#   scripts/fetch_rosario_v1.sh <1-6> [DEST]     (default DEST: data/rosario_v1/sequenceNN/download)
#
# Then convert inside the container (verifies the published MD5):
#   tools/prepare_rosario_v1.py N /data/rosario_v1/sequenceNN/download /data/rosario_v1/sequenceNN --delete-parts
set -euo pipefail

usage() { sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[ $# -ge 1 ] || usage
N=$1
case "$N" in 1|2|3|4|5|6) ;; *) usage ;; esac
NN=$(printf '%02d' "$N")
# Parts per sequence, as published (must match tools/ppb_vio_tools/rosario_v1.py).
PARTS=(_ 3 2 1 1 2 2)
BASE=http://fs01.cifasis-conicet.gov.ar:90/~robot_desmalezador/robot
HERE=$(cd "$(dirname "$0")/.." && pwd)
DEST=${2:-$HERE/data/rosario_v1/sequence$NN/download}
mkdir -p "$DEST"

FILES=("calibration$NN.yaml" "sequence${NN}_gt.txt")
if [ "${PARTS[$N]}" = 1 ]; then
  FILES+=("sequence$NN.bag")
else
  for ((k = 0; k < PARTS[N]; k++)); do FILES+=("$(printf 'sequence%s.bag.%02d' "$NN" "$k")"); done
fi

for f in "${FILES[@]}"; do
  # A complete file makes `curl -C -` fail (HTTP 416), so compare sizes first.
  remote=$(curl -fsSI "$BASE/$f" | tr -d '\r' | sed -n 's/^[Cc]ontent-[Ll]ength: //p' | tail -1)
  if [ -f "$DEST/$f" ] && [ "$(wc -c < "$DEST/$f" | tr -d ' ')" = "$remote" ]; then
    echo "== $f (complete)"; continue
  fi
  echo "== $f"
  curl -fS -C - --retry 5 --retry-delay 10 -o "$DEST/$f" "$BASE/$f"
done
echo "downloaded to $DEST"
