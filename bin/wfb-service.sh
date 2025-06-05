#!/usr/bin/env bash
set -euo pipefail
shopt -s extglob

export LC_ALL=C

##################################
# DEFAULTS  (overridable by CLI)
##################################
REGDOMAIN="US"         # -r / --reg XX
BANDWIDTH="HT40+"       # -b / --bw HTxx[+|-]
WHITELIST=(service0 service1)

declare -A CHANNEL=(
  [service0]=104
  [service1]=149
)
##################################

usage() {
  cat <<EOF
Usage: $(basename "$0") [options] [iface[@chan] ...]
Options:
  -r, --reg   <CC>      Regulatory domain (default: $REGDOMAIN)
  -b, --bw    <HTxx>    Bandwidth string for 'iw set channel' (default: $BANDWIDTH)
  -h, --help            Show this help

Positional arguments (optional):
  iface[@chan]          Interfaces to initialise, optionally with centre channel.
                        If *any* are supplied they REPLACE the built-in whitelist.
                        Examples: bonnet2@104 bonnet1
EOF
}

##################################
# CLI PARSER
##################################
while [[ $# -gt 0 ]]; do
  case $1 in
    -r|--reg)  REGDOMAIN=${2:?}; shift 2 ;;
    -b|--bw)   BANDWIDTH=${2:?}; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    --)        shift; break ;;      # explicit end of options
    -*)        echo "Unknown option $1"; usage; exit 1 ;;
    *)         break ;;             # first non-option → iface list
  esac
done

# Remaining tokens → iface[@chan] descriptors
if [[ $# -gt 0 ]]; then
  WHITELIST=()
  while [[ $# -gt 0 ]]; do
    IFS='@' read -r ifc chan <<<"$1"
    WHITELIST+=("$ifc")
    [[ -n $chan ]] && CHANNEL["$ifc"]=$chan
    shift
  done
fi
##################################

##################################
# HOUSE-KEEPING
##################################
_cleanup() {
  local pids
  pids=$(jobs -pr) && [[ -n $pids ]] && kill -TERM $pids 2>/dev/null || true
}
trap _cleanup EXIT
##################################

iw reg set "$REGDOMAIN"

# 1. Detect existing PHY interfaces
mapfile -t detected_ifaces < <(iw dev | awk '/Interface/ {print $2}')

# 2. Intersect with whitelist
initialised_ifaces=()
for ifc in "${WHITELIST[@]}"; do
  if [[ " ${detected_ifaces[*]} " == *" ${ifc} "* ]]; then
    echo "Initialising ${ifc} …"

    # NetworkManager hand-off
    if command -v nmcli >/dev/null && ! nmcli device show "$ifc" | grep -q '(unmanaged)'; then
      nmcli device set "$ifc" managed no
      sleep 1
    fi

    ip link set "$ifc" down
    iw dev "$ifc" set monitor otherbss
    ip link set "$ifc" up
    iw dev "$ifc" set channel "${CHANNEL[$ifc]:-149}" "$BANDWIDTH"

    initialised_ifaces+=("$ifc")
  else
    echo "Skipping ${ifc}: not present."
  fi
done

[[ ${#initialised_ifaces[@]} -eq 0 ]] && {
  echo "No whitelist interfaces present – nothing started."
  exit 0
}

echo "Interfaces ready: ${initialised_ifaces[*]}"
echo "Reg-domain: $REGDOMAIN | Bandwidth: $BANDWIDTH"

# 3. Launch each WFB pipeline ONCE, fed all interfaces
wfb_rx -f -c 127.0.0.1 -u 10000 -p 0   -i 7669206  -R 2097152 "${initialised_ifaces[@]}" &
wfb_rx -f -c 127.0.0.1 -u 10001 -p 16  -i 7669206  -R 2097152 "${initialised_ifaces[@]}" &
wfb_tx -I 11002               -R 2097152 "${initialised_ifaces[@]}" &
wfb_rx -f -c 127.0.0.1 -u 10002 -p 32  -i 7669206  -R 2097152 "${initialised_ifaces[@]}" &
wfb_tx -I 11004               -R 2097152 "${initialised_ifaces[@]}" &
wfb_rx -f -c 127.0.0.1 -u 10003 -p 127 -i 10531917 -R 2097152 "${initialised_ifaces[@]}" &
wfb_tx -I 11006               -R 2097152 "${initialised_ifaces[@]}" &

echo "WFB-ng init done"
wait -n
