#!/usr/bin/env bash
set -euo pipefail

OUTDIR="${PWD}/egpu_diagnostics_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUTDIR"

echo "Collecting system information into $OUTDIR"

# Basic system info
uname -a > "$OUTDIR/uname.txt" 2>&1 || true
cat /etc/os-release > "$OUTDIR/os-release.txt" 2>&1 || true
if command -v lsb_release >/dev/null 2>&1; then lsb_release -a > "$OUTDIR/lsb_release.txt" 2>&1 || true; fi
hostnamectl > "$OUTDIR/hostnamectl.txt" 2>&1 || true

# PCI / USB
if command -v lspci >/dev/null 2>&1; then
  lspci -vvnn > "$OUTDIR/lspci_all.txt" 2>&1 || true
  lspci -nnk | grep -i -A6 nvidia > "$OUTDIR/lspci_nvidia.txt" 2>&1 || true
  lspci | egrep -i "thunderbolt|tbt" > "$OUTDIR/lspci_thunderbolt.txt" 2>&1 || true
fi
if command -v lsusb >/dev/null 2>&1; then lsusb > "$OUTDIR/lsusb.txt" 2>&1 || true; fi

# Search sysfs for NVIDIA vendor (0x10de)
grep -R --line-number "0x10de" /sys/bus/pci/devices/*/vendor > "$OUTDIR/sys_pci_nvidia_vendor.txt" 2>&1 || true

# Kernel modules and drivers
lsmod > "$OUTDIR/lsmod.txt" 2>&1 || true
lsmod | grep -i nvidia > "$OUTDIR/lsmod_nvidia.txt" 2>&1 || true
if command -v modinfo >/dev/null 2>&1; then modinfo nvidia > "$OUTDIR/modinfo_nvidia.txt" 2>&1 || true; fi

# nvidia-smi (if available)
if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi -q > "$OUTDIR/nvidia_smi.txt" 2>&1 || true
else
  echo "nvidia-smi: not found" > "$OUTDIR/nvidia_smi.txt"
fi

# Logs
if command -v dmesg >/dev/null 2>&1; then
  dmesg | tail -n 500 > "$OUTDIR/dmesg_tail.txt" 2>&1 || true
  dmesg | egrep -i "nvidia|nvrm|thunderbolt|tbt|pci|fail|error|drm" > "$OUTDIR/dmesg_filtered.txt" 2>&1 || true
fi
if command -v journalctl >/dev/null 2>&1; then
  journalctl -k -b > "$OUTDIR/journalctl_k.txt" 2>&1 || true
  journalctl -b | egrep -i "nvidia|tbt|thunderbolt|pci|drm|nvrm|offload" > "$OUTDIR/journal_filtered.txt" 2>&1 || true
fi

# Boltctl (Thunderbolt) if present
if command -v boltctl >/dev/null 2>&1; then boltctl list > "$OUTDIR/boltctl_list.txt" 2>&1 || true; fi

# Sysfs device info for NVIDIA devices
for dev in /sys/bus/pci/devices/*; do
  vendor_file="$dev/vendor"
  if [ -f "$vendor_file" ]; then
    vendor_id=$(cat "$vendor_file" 2>/dev/null || true)
    if [ "$vendor_id" = "0x10de" ]; then
      echo "Found NVIDIA device: $dev" >> "$OUTDIR/sys_nvidia_devices.txt"
      printf "Device path: %s\n" "$dev" >> "$OUTDIR/sys_nvidia_devices.txt"
      cat "$dev/device" >> "$OUTDIR/sys_nvidia_devices.txt" 2>&1 || true
      echo "" >> "$OUTDIR/sys_nvidia_devices.txt"
      ls -la "$dev" >> "$OUTDIR/sys_nvidia_devices.txt" 2>&1 || true
    fi
  fi
done

# Try to list NVIDIA entries from lspci
if command -v lspci >/dev/null 2>&1; then
  lspci -nn | egrep "10de|NVIDIA" > "$OUTDIR/lspci_10de.txt" 2>&1 || true
fi

# Create tarball
tar -czf "${OUTDIR}.tar.gz" -C "$(dirname "$OUTDIR")" "$(basename "$OUTDIR")" 2>/dev/null || true

echo "Done. Outputs: ${OUTDIR} and ${OUTDIR}.tar.gz"
exit 0
