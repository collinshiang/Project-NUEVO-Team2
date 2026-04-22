#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"

usage() {
    cat <<'EOF'
Usage:
  ros2_ws/docker/enter_ros2.sh [--build] [rpi|vm|jetson|/path/to/docker-compose.yml]

Behavior:
  - If COMPOSE is set, it takes precedence.
  - Otherwise:
      rpi    -> ros2_ws/docker/docker-compose.rpi.yml    (service: ros2_runtime)
      vm     -> ros2_ws/docker/docker-compose.vm.yml     (service: ros2_runtime)
      jetson -> ros2_ws/docker/docker-compose.jetson.yml (service: global_gps)
  - Default is rpi.
  - If the selected service is already running, the script opens another shell
    in that same container.
  - If the service is not running, the script starts it with
    `docker compose up -d --wait`.
  - Pass --build to rebuild the image before entering:
    `docker compose up -d --build --wait`.

Examples:
  ./ros2_ws/docker/enter_ros2.sh
  ./ros2_ws/docker/enter_ros2.sh rpi
  ./ros2_ws/docker/enter_ros2.sh --build rpi
  ./ros2_ws/docker/enter_ros2.sh jetson
  COMPOSE=ros2_ws/docker/docker-compose.rpi.yml ./ros2_ws/docker/enter_ros2.sh
EOF
}

build=0
target="rpi"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --build|--rebuild)
            build=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            target="$1"
            shift
            ;;
    esac
done

if [[ "${target}" == "-h" || "${target}" == "--help" ]]; then
    usage
    exit 0
fi

service="ros2_runtime"

if [[ -n "${COMPOSE:-}" ]]; then
    compose_file="${COMPOSE}"
else
    case "${target}" in
        rpi)
            compose_file="${repo_root}/ros2_ws/docker/docker-compose.rpi.yml"
            ;;
        vm)
            compose_file="${repo_root}/ros2_ws/docker/docker-compose.vm.yml"
            ;;
        jetson)
            compose_file="${repo_root}/ros2_ws/docker/docker-compose.jetson.yml"
            service="global_gps"
            ;;
        *.yml|*.yaml)
            compose_file="${target}"
            ;;
        *)
            echo "Unknown target: ${target}" >&2
            usage >&2
            exit 1
            ;;
    esac
fi

container_id="$(docker compose -f "${compose_file}" ps -q "${service}" 2>/dev/null || true)"
running="false"
if [[ -n "${container_id}" ]]; then
    running="$(docker inspect -f '{{.State.Running}}' "${container_id}" 2>/dev/null || echo false)"
fi

if [[ "${running}" == "true" && "${build}" -eq 0 ]]; then
    echo "[enter_ros2] Entering existing ${service} container ${container_id:0:12}..."
elif [[ "${build}" -eq 1 ]]; then
    echo "[enter_ros2] Rebuilding and starting ${service} with docker compose up -d --build --wait..."
    docker compose -f "${compose_file}" up -d --build --wait "${service}"
else
    echo "[enter_ros2] Starting ${service} with docker compose up -d --wait..."
    docker compose -f "${compose_file}" up -d --wait "${service}"
fi

exec docker compose -f "${compose_file}" exec "${service}" bash -lc '
source /opt/ros/jazzy/setup.bash
cd /ros2_ws

if [[ -f /ros2_ws/install/setup.bash ]]; then
    source /ros2_ws/install/setup.bash
fi

exec bash -i
'
