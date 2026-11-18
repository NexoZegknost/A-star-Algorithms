#!/usr/bin/env bash
set -euo pipefail

dir="${1:-src}"
allowed='^(iostream|fstream|string|cmath|vector|algorithm)$'
status=0

while IFS= read -r line; do
    file="${line%%:*}"
    header="$(sed -E 's/.*#include[[:space:]]*<([^>]+)>.*/\1/' <<<"$line")"
    if ! [[ "$header" =~ $allowed ]]; then
        echo "Disallowed header <$header> in $file"
        status=1
    fi
done < <(grep -H -E '#include[[:space:]]*<' "$dir"/*.h "$dir"/*.cpp 2>/dev/null || true)

if [[ $status -eq 0 ]]; then
    echo "Header check passed: only CodeRunner standard headers are used."
fi
exit $status
