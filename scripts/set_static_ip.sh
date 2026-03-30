#!/usr/bin/env bash
# Set a static IPv4 address on a selected NetworkManager-managed interface.
#
# Usage:
#   ./scripts/set_static_ip.sh <interface> <ip> [prefix]
#
# Examples:
#   ./scripts/set_static_ip.sh enxa0cec8a55c8d 10.42.0.1
#   ./scripts/set_static_ip.sh eth0 10.42.0.2
#   ./scripts/set_static_ip.sh eth0 192.168.1.50 16
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
    echo "Usage:"
    echo "  $0 <interface> <ip> [prefix]"
    echo ""
    echo "Examples:"
    echo "  $0 enxa0cec8a55c8d 10.42.0.1"
    echo "  $0 eth0 10.42.0.2"
    echo "  $0 eth0 192.168.1.50 16"
    exit 1
fi

IFACE="$1"
IP_ADDR="$2"
PREFIX="${3:-24}"
IP_CIDR="${IP_ADDR}/${PREFIX}"

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
    CONN_NAME="static-${IFACE}"
    nmcli connection add type ethernet ifname "${IFACE}" con-name "${CONN_NAME}"
fi

# Set method and address in one call so NetworkManager never sees an
# invalid "manual with no address" intermediate state.
nmcli connection modify "${CONN_NAME}" ipv4.method manual ipv4.addresses "${IP_CIDR}"
nmcli connection modify "${CONN_NAME}" ipv4.gateway ""
nmcli connection modify "${CONN_NAME}" ipv4.dns ""

nmcli connection modify "${CONN_NAME}" ipv6.method ignore

nmcli connection down "${CONN_NAME}" >/dev/null 2>&1 || true
nmcli connection up "${CONN_NAME}"

echo ""
echo "Applied static IP:"
echo "  Interface:  ${IFACE}"
echo "  Connection: ${CONN_NAME}"
echo "  Address:    ${IP_CIDR}"

echo ""
ip addr show "${IFACE}"
