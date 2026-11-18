#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <StudentID>"
    exit 1
fi

id="$1"
root="$(cd "$(dirname "$0")/.." && pwd)"
dist="$root/dist"
target="$dist/$id"
report="$root/report/$id.pdf"

if [[ ! -f "$report" ]]; then
    echo "Missing report: $report"
    exit 1
fi
if ! command -v rar >/dev/null 2>&1; then
    echo "The 'rar' tool is required (a renamed zip is penalised)."
    exit 1
fi

make -C "$root" test >/dev/null

rm -rf "$target" "$dist/$id.rar"
mkdir -p "$target"
cp "$root"/src/{PathNode.h,PathNode.cpp,Algo.h,Algo.cpp,main.cpp} "$target/"
cp "$report" "$target/"

(cd "$dist" && rar a -idq "$id.rar" "$id")
echo "Created $dist/$id.rar"
