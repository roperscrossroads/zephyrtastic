#!/usr/bin/env bash
# Reproducibly pin the upstream Meshtastic firmware tree that harvest.py reads,
# without turning it into a submodule.
#
# Why this exists
# ----------------
# harvest.py takes `--upstream <tree>` and trusts whatever is checked out
# there. The `firmware/` sibling checkout (see workspace CLAUDE.md) is a
# regular, moving clone that people also read manually for parity work -- its
# HEAD drifts over time and this script must never move it out from under
# someone mid-session. What we actually want is: fetch one exact pinned
# commit, harvest from a disposable copy of it, leave `firmware/` untouched.
#
# `firmware/` already has full history locally and `config.pb.h` is a
# committed file there (not generated at build time -- confirmed 2026-08-06),
# so there is no need for a second network clone. `git worktree` gives a
# disposable checkout of one exact commit that shares the same object store:
# no extra bandwidth, no risk to the primary checkout's branch/HEAD.
#
# Usage
# -----
#   ./sync-upstream.sh --ref <tag-or-sha>   # pin to a new upstream commit,
#                                            # regenerate header + lock
#   ./sync-upstream.sh --check              # re-fetch the CURRENTLY locked
#                                            # commit into a clean worktree and
#                                            # verify the checked-in header
#                                            # still reproduces from it (catches
#                                            # hand-edits / lock<->header skew)
#
# Either mode ends with `firmware/` exactly as it was before the script ran.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"          # main/
UPSTREAM_REPO="$(cd "$REPO/../firmware" && pwd)"
LOCK="$HERE/upstream.lock"

REF=""
CHECK=0

usage() {
	cat <<EOF
Usage: $(basename "$0") [--ref <tag-or-sha>] [--check] [--upstream-repo <path>]

  --ref <tag-or-sha>    Pin to this upstream commit and regenerate the header
                         + lock. Fetches it into firmware/ first if not
                         already present locally.
  --check                No --ref: re-verify the commit already recorded in
                         upstream.lock, from a clean worktree. Writes nothing.
  --upstream-repo <path> Local firmware clone to worktree from (default:
                         $UPSTREAM_REPO).
EOF
}

while [[ $# -gt 0 ]]; do
	case "$1" in
	--ref)
		REF="$2"
		shift 2
		;;
	--check)
		CHECK=1
		shift
		;;
	--upstream-repo)
		UPSTREAM_REPO="$(cd "$2" && pwd)"
		shift 2
		;;
	-h | --help)
		usage
		exit 0
		;;
	*)
		echo "error: unknown argument: $1" >&2
		usage
		exit 1
		;;
	esac
done

if [[ -z "$REF" ]]; then
	if [[ $CHECK -ne 1 ]]; then
		echo "error: --ref is required unless --check is given (see --help)" >&2
		exit 1
	fi
	if [[ ! -f "$LOCK" ]]; then
		echo "error: no upstream.lock yet -- run with --ref first to create one" >&2
		exit 1
	fi
	# upstream.lock's "upstream" field is "<name> @ <short-sha>" (see
	# harvest.py:upstream_revision). Pull just the sha back out.
	REF="$(python3 -c '
import json, sys
print(json.load(open(sys.argv[1]))["upstream"].split("@")[-1].strip())
' "$LOCK")"
	echo "no --ref given; re-checking currently locked commit: $REF"
fi

if [[ ! -d "$UPSTREAM_REPO/.git" ]]; then
	echo "error: $UPSTREAM_REPO is not a git checkout" >&2
	exit 1
fi

# Resolve locally first so a bump to a brand-new upstream commit doesn't
# silently worktree a stale ref -- only fetch if we don't already have it.
if ! git -C "$UPSTREAM_REPO" rev-parse -q --verify "${REF}^{commit}" >/dev/null; then
	echo "fetching $REF into $UPSTREAM_REPO ..."
	git -C "$UPSTREAM_REPO" fetch origin "$REF"
	REF="$(git -C "$UPSTREAM_REPO" rev-parse FETCH_HEAD)"
fi

SCRATCH_PARENT="$(mktemp -d)"
# harvest.py labels the header from the checkout dir's basename
# (upstream_revision() -> `upstream.name`) -- name it "firmware" so a
# worktree-sourced harvest reads the same as one run straight against the
# firmware/ sibling checkout.
SCRATCH="$SCRATCH_PARENT/firmware"
cleanup() {
	git -C "$UPSTREAM_REPO" worktree remove --force "$SCRATCH" >/dev/null 2>&1 || true
	rm -rf "$SCRATCH_PARENT"
}
trap cleanup EXIT

echo "checking out $REF into a disposable worktree (firmware/ itself is left untouched)..."
git -C "$UPSTREAM_REPO" worktree add --detach --quiet "$SCRATCH" "$REF"

ARGS=(--upstream "$SCRATCH")
[[ $CHECK -eq 1 ]] && ARGS+=(--check)

python3 "$HERE/harvest.py" "${ARGS[@]}"
