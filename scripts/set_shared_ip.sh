#!/usr/bin/env bash
# Set a fixed subnet on a NetworkManager shared Ethernet interface.
#
# Usage:
#   ./scripts/set_shared_ip.sh <interface> <ip/cidr>
#
# Example:
#   ./scripts/set_shared_ip.sh enxa0cec8a55c8d 10.42.0.1/24
set -euo pipefail

if [[ $# -ne 2 ]]; then
    echo "Usage:"
    echo "  $0 <interface> <ip/cidr>"
    echo ""
    echo "Example:"
    echo "  $0 enxa0cec8a55c8d 10.42.0.1/24"
    exit 1
fi

IFACE="$1"
IP_CIDR="$2"

if ! command -v nmcli >/dev/null 2>&1; then
    echo "Error: nmcli not found. This script requires NetworkManager."
    exit 1
fi

if ! nmcli -t -f DEVICE device status | grep -qx "${IFACE}"; then
    echo "Error: interface '${IFACE}' not found."
    exit 1
fi

CONN_NAME="$(nmcli -t -f DEVICE,CONNECTION device status | awk -F: -v dev="${IFACE}" '$1 == dev { print $2 }')"

if [[ -z "${CONN_NAME}" || "${CONN_NAME}" == "--" ]]; then
    CONN_NAME="shared-${IFACE}"
    nmcli connection add type ethernet ifname "${IFACE}" con-name "${CONN_NAME}"
fi

nmcli connection modify "${CONN_NAME}" ipv4.method shared ipv4.addresses "${IP_CIDR}"
nmcli connection modify "${CONN_NAME}" ipv6.method ignore

nmcli connection down "${CONN_NAME}" >/dev/null 2>&1 || true
nmcli connection up "${CONN_NAME}"

echo ""
echo "Applied shared IP configuration:"
echo "  Interface:  ${IFACE}"
echo "  Connection: ${CONN_NAME}"
echo "  Address:    ${IP_CIDR}"

echo ""
ip addr show "${IFACE}"
