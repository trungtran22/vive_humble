#!/usr/bin/env bash
# register_steamvr_app.sh
#
# One-time setup: tells host SteamVR about the vive_ros application manifest
# so the container's vive_node / vive_pose can pass app-key validation
# (otherwise SteamVR returns "Unable to init path manager:
# VRInitError_Init_Internal").
#
# Run this on the HOST (not inside the container) after installing SteamVR:
#     ./scripts/register_steamvr_app.sh
#
# It is idempotent. After running it, restart SteamVR for the change to take
# effect.

set -euo pipefail

STEAM_CFG="${HOME}/.local/share/Steam/config"
APPCONFIG="${STEAM_CFG}/appconfig.json"
TARGET_MANIFEST="${STEAM_CFG}/vive_ros.vrmanifest"

if [[ ! -d "${STEAM_CFG}" ]]; then
    echo "ERROR: ${STEAM_CFG} does not exist. Install Steam + SteamVR first." >&2
    exit 1
fi

# Manifest distributed with this package (works in two scenarios: developer
# checkout next to this script, or the colcon-installed share dir).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_MANIFEST=""
for candidate in \
    "${SCRIPT_DIR}/../share/manifest/vive_ros.vrmanifest" \
    "${SCRIPT_DIR}/../../share/vive_ros/manifest/vive_ros.vrmanifest" \
    "/ros2_ws/install/vive_ros/share/vive_ros/manifest/vive_ros.vrmanifest"; do
    if [[ -f "${candidate}" ]]; then
        SRC_MANIFEST="${candidate}"
        break
    fi
done

if [[ -z "${SRC_MANIFEST}" ]]; then
    echo "ERROR: could not locate vive_ros.vrmanifest." >&2
    exit 1
fi

cp -v "${SRC_MANIFEST}" "${TARGET_MANIFEST}"

python3 - "${APPCONFIG}" "${TARGET_MANIFEST}" <<'PY'
import json, os, sys
appconfig_path, manifest_path = sys.argv[1], sys.argv[2]
data = {}
if os.path.isfile(appconfig_path):
    with open(appconfig_path) as f:
        try:
            data = json.load(f)
        except json.JSONDecodeError:
            print(f"WARNING: {appconfig_path} was not valid JSON; rewriting.")
paths = data.get("manifest_paths", []) or []
if manifest_path not in paths:
    paths.append(manifest_path)
    print(f"Added {manifest_path} to manifest_paths.")
else:
    print(f"{manifest_path} already in manifest_paths.")
data["manifest_paths"] = paths
with open(appconfig_path, "w") as f:
    json.dump(data, f, indent=3)
PY

cat <<EOF

OK. Manifest registered.

Next steps:
  1. Quit SteamVR (close the small SteamVR status window).
  2. Re-launch SteamVR.
  3. Run the container as usual:
       docker compose -f docker/docker-compose.yml up vive_ros
EOF
