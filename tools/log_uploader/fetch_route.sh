#!/usr/bin/env bash
#
# rsync a single route from the user-managed server to the local
# workstation so tools/log_uploader/analyze_route.py can chew on it.
#
# Usage:
#   LOG_UPLOAD_REMOTE=user@host:/srv/logs/ ./fetch_route.sh <route_id> [dest_dir]
#
# Example:
#   LOG_UPLOAD_REMOTE=user@host:/srv/logs/ \
#     ./tools/log_uploader/fetch_route.sh 2026-04-20--14-00-00--abc123 ~/log_routes
#
# Exit codes:
#   0 success
#   1 usage error
#   2 missing LOG_UPLOAD_REMOTE
#   >2 rsync failed (rsync's own code)
#
set -euo pipefail

ROUTE="${1:-}"
DEST="${2:-./log_routes}"
REMOTE="${LOG_UPLOAD_REMOTE:-}"

if [[ -z "$ROUTE" ]]; then
  echo "usage: fetch_route.sh <route_id> [dest_dir]" >&2
  exit 1
fi
if [[ -z "$REMOTE" ]]; then
  echo "error: set LOG_UPLOAD_REMOTE (e.g. user@host:/srv/logs/)" >&2
  exit 2
fi

mkdir -p "$DEST"
src="${REMOTE%/}/${ROUTE}"
echo "rsync ${src} -> ${DEST}"
rsync -az --partial --info=progress2 "$src" "$DEST/"
echo
echo "done. to analyze:"
echo "  python3 tools/log_uploader/analyze_route.py ${DEST}/${ROUTE}"
