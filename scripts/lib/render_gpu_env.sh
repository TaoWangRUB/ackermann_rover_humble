#!/usr/bin/env bash
# Emit env-var exports that route OpenGL + CUDA to the right GPU.
#
# - x86 host with TITAN X eGPU attached  -> CUDA/OpenGL pinned to GPU 1 (eGPU)
# - x86 host without eGPU                -> CUDA/OpenGL on GPU 0 (A2000 / dGPU)
# - Jetson (aarch64)                     -> CUDA/OpenGL on GPU 0 (Tegra iGPU)
#
# This script is intended to be sourced from a shell or invoked inside a
# `bash -c 'source ... && exec <prog>'` wrapper. It exports vars; it does not
# launch anything itself. The launch files do the equivalent inline via
# SetEnvironmentVariable + Node(additional_env=...), so this file is mostly
# for command-line / interactive use.
#
# Usage:
#   source scripts/lib/render_gpu_env.sh
#   gz sim -r world.sdf

# Default = GPU 0 (works on x86 single-GPU AND Jetson — Tegra is index 0).
_cuda_idx=0

if [[ "$(uname -m)" == "x86_64" ]] \
   && command -v nvidia-smi >/dev/null 2>&1 \
   && nvidia-smi -L 2>/dev/null | grep -q 'TITAN X'; then
  _cuda_idx=1
  echo "[render_gpu_env] eGPU TITAN X detected → graphics on GPU 1" >&2
else
  echo "[render_gpu_env] single-GPU mode → graphics on GPU 0" >&2
fi

export CUDA_VISIBLE_DEVICES="${_cuda_idx}"
# PRIME render offload — required on x86 hybrid laptops (Intel iGPU + NVIDIA
# dGPU) so OpenGL apps actually use the NVIDIA card. Harmless on Jetson and on
# desktop dGPUs, so we leave it on unconditionally.
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
unset _cuda_idx
