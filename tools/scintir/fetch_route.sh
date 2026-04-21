#!/usr/bin/env bash
#
# Scintir: rsync a single route from the user-managed server to the local
# workstation so tools/scintir/analyze_route.py can chew on it.
#
# Usage:
#   SCINTIR_REMOTE=user@host:/srv/scintir/ ./fetch_route.sh <route_id> [dest_dir]
#
# Example:
#   SCINTIR_REMOTE=user@host:/srv/scintir/ \
#     ./tools/scintir/fetch_route.sh 2026-04-20--14-00-00--abc123 ~/scintir_routes
#
# Exit codes:
#   0 success
#   1 usage error
#   2 missing SCINTIR_REMOTE
#   >2 rsync failed (rsync's own code)
#
set -euo pipefail

ROUTE="${1:-}"
DEST="${2:-./scintir_routes}"
REMOTE="${SCINTIR_REMOTE:-}"

if [[ -z "$ROUTE" ]]; then
  echo "usage: fetch_route.sh <route_id> [dest_dir]" >&2
  exit 1
fi
if [[ -z "$REMOTE" ]]; then
  echo "error: set SCINTIR_REMOTE (e.g. user@host:/srv/scintir/)" >&2
  exit 2
fi

mkdir -p "$DEST"
src="${REMOTE%/}/${ROUTE}"
echo "rsync ${src} -> ${DEST}"
rsync -az --partial --info=progress2 "$src" "$DEST/"
echo
echo "done. to analyze:"
echo "  python3 tools/scintir/analyze_route.py ${DEST}/${ROUTE}"
