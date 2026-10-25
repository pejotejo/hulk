#!/usr/bin/env bash
# Shared setup for the existing simulator launchers. Keep stdout free for MCP.

simulator_die() {
  echo "Simulator setup: $*" >&2
  exit 1
}

simulator_require() {
  command -v "$1" >/dev/null 2>&1 || simulator_die "Missing $1. $2"
}

simulator_assets() {
  local missing
  if missing="$(python3 - "$1" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
meshes = root / "tools/mujoco-simulator/mujoco-simulator/K1/meshes"
models = root / "etc/neural_networks"
branding = root / "tools/simulate/assets/branding"
assets = set(meshes.glob("*")) | set(models.glob("*.onnx")) | set(branding.glob("*_panel.png"))
assets |= {meshes / "Trunk.STL", *(models / name for name in (
    "K1_walk.onnx", "K1_stand_up.onnx", "K1_kick_inner.onnx", "K1_kick_inner_soft.onnx"))}
assets |= {branding / f"{name}_panel.png" for name in (
    "hulks", "kdab", "cc", "draeger", "gea", "haspa", "tk", "gegenstrom", "tuhh")}
missing = []
for path in sorted(assets):
    if not path.exists():
        missing.append(str(path.relative_to(root)))
    elif path.is_file():
        with path.open("rb") as asset:
            if asset.read(100).startswith(b"version https://git-lfs.github.com/spec/v1"):
                missing.append(str(path.relative_to(root)))
print("\n".join(missing))
sys.exit(bool(missing))
PY
)"; then
    return
  fi
  [[ -n "$missing" ]] || simulator_die "Could not inspect simulator assets; see the Python error above."
  if $simulator_dry_run; then
    echo "Would fetch missing simulator LFS assets: $missing" >&2
    return
  fi
  [[ "${2:-true}" == true ]] || simulator_die "Simulator assets are still missing after Git LFS download: $missing"
  simulator_require git-lfs "Install Git LFS, then retry (Debian/Ubuntu: sudo apt install git-lfs; Fedora: sudo dnf install git-lfs)."
  echo "Fetching simulator meshes, ONNX models and arena branding with Git LFS..." >&2
  git -C "$1" lfs pull --include='etc/neural_networks/*.onnx,tools/mujoco-simulator/mujoco-simulator/K1/meshes/**,tools/simulate/assets/branding/*_panel.png' >&2 \
    || simulator_die "Git LFS download failed. Check access to the repository's LFS remote and retry."
  # Check again so skipped or unavailable LFS objects cannot reach Cargo or inference.
  simulator_assets "$1" false
}

simulator_prepare() {
  local root="$1" bhuman="$2" builder cache source_args=() setup_args=() library_dir
  simulator_require cargo "Install the repository's Rust toolchain; see docs/setup/development_environment.md."
  simulator_require python3 "Install Python >=3.12 and retry."
  python3 -c 'import sys; sys.exit(sys.version_info < (3, 12))' \
    || simulator_die "Python >=3.12 is required; upgrade python3 and retry."
  simulator_require rustc "Install the repository's Rust toolchain; see docs/setup/development_environment.md."
  python3 - "$root/Cargo.toml" <<'PY'
from pathlib import Path
import subprocess
import sys
import tomllib

required = tomllib.loads(Path(sys.argv[1]).read_text())["workspace"]["package"]["rust-version"]
rustc = subprocess.run(["rustc", "--version"], capture_output=True, text=True)
hint = (f"Install it with rustup toolchain install {required}, then launch with "
        f"RUSTUP_TOOLCHAIN={required} ./twix --simulator --bhuman /hulks/1 "
        "(or set the same variable for ./simulator-mcp).")
if rustc.returncode:
    sys.exit(f"Simulator setup: rustc is unavailable. {hint}\n{rustc.stderr.strip()}")
current = rustc.stdout.split()[1]
if tuple(map(int, current.split("-")[0].split("."))) < tuple(map(int, required.split("."))):
    sys.exit(f"Simulator setup: Rust >={required} is required; active rustc is {current}. {hint}")
PY
  [[ "$(uname -s)" == Linux ]] || simulator_die "The simulator renderer currently requires Linux."
  builder="$root/tools/simulate/bhuman/build.py"
  cache="$(python3 "$builder" --print-cache)"
  if [[ -n "${BHUMAN_SOURCE:-}" ]]; then
    [[ -d "$BHUMAN_SOURCE/Config" ]] || simulator_die "BHUMAN_SOURCE must contain B-Human's Config directory: $BHUMAN_SOURCE"
    export BHUMAN_SOURCE="$(readlink -f "$BHUMAN_SOURCE")"
    source_args=(--source "$BHUMAN_SOURCE")
  fi
  if $simulator_dry_run; then
    setup_args=(--dry-run)
  fi
  export MUJOCO_DOWNLOAD_DIR="${MUJOCO_DOWNLOAD_DIR:-${XDG_CACHE_HOME:-$HOME/.cache}/mujoco-rs}"
  [[ "$MUJOCO_DOWNLOAD_DIR" == /* ]] || simulator_die "MUJOCO_DOWNLOAD_DIR must be an absolute directory: $MUJOCO_DOWNLOAD_DIR"
  if [[ -n "${MUJOCO_STATIC_LINK_DIR:-}" ]]; then
    [[ "$MUJOCO_STATIC_LINK_DIR" == /* && -f "$MUJOCO_STATIC_LINK_DIR/libmujoco.a" ]] \
      || simulator_die "MUJOCO_STATIC_LINK_DIR must be an absolute directory containing libmujoco.a."
  else
    library_dir="${MUJOCO_DYNAMIC_LINK_DIR:-$MUJOCO_DOWNLOAD_DIR/mujoco-3.9.0/lib}"
    if [[ -n "${MUJOCO_DYNAMIC_LINK_DIR:-}" ]]; then
      [[ "$library_dir" == /* && -f "$library_dir/libmujoco.so" ]] \
        || simulator_die "MUJOCO_DYNAMIC_LINK_DIR must be an absolute directory containing libmujoco.so (MuJoCo 3.9.0)."
    fi
    export LD_LIBRARY_PATH="$library_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  fi

  if [[ -n "${ORT_DYLIB_PATH:-}" ]]; then
    [[ -f "$ORT_DYLIB_PATH" ]] || simulator_die "ORT_DYLIB_PATH does not name a shared library: $ORT_DYLIB_PATH. Fix it or unset it for automatic CPU runtime setup."
    export ORT_DYLIB_PATH="$(readlink -f "$ORT_DYLIB_PATH")"
  fi
  simulator_assets "$root"
  if $bhuman; then
    if [[ -n "${BHUMAN_WORKER:-}" ]]; then
      [[ -x "$BHUMAN_WORKER" ]] || simulator_die "BHUMAN_WORKER is not executable: $BHUMAN_WORKER"
      [[ -n "${BHUMAN_SOURCE:-}" ]] || simulator_die "Set BHUMAN_SOURCE alongside BHUMAN_WORKER, or unset BHUMAN_WORKER to build automatically."
      export BHUMAN_WORKER="$(readlink -f "$BHUMAN_WORKER")"
    else
      python3 "$builder" "${source_args[@]}" "${setup_args[@]}" >&2
      export BHUMAN_WORKER="$cache/worker-build/bhuman-motion-worker"
      export BHUMAN_SOURCE="${BHUMAN_SOURCE:-$cache/source}"
      source_args=(--source "$BHUMAN_SOURCE")
    fi
  fi
  if [[ -z "${ORT_DYLIB_PATH:-}" ]]; then
    python3 "$builder" --runtime-only "${source_args[@]}" "${setup_args[@]}" >&2
    export ORT_DYLIB_PATH="$(python3 "$builder" --print-ort "${source_args[@]}")"
  fi
  # ort's environment must be destroyed before the library's C++ globals.
  export LD_PRELOAD="$ORT_DYLIB_PATH${LD_PRELOAD:+:$LD_PRELOAD}"
}

simulator_launch() {
  if $simulator_dry_run; then
    printf 'MUJOCO_DOWNLOAD_DIR=%q\nORT_DYLIB_PATH=%q\nLD_LIBRARY_PATH=%q\nLD_PRELOAD=%q\n' \
      "${MUJOCO_DOWNLOAD_DIR:-}" "${ORT_DYLIB_PATH:-}" "${LD_LIBRARY_PATH:-}" "${LD_PRELOAD:-}"
    if [[ -n "${BHUMAN_WORKER:-}" ]]; then
      printf 'BHUMAN_WORKER=%q\nBHUMAN_SOURCE=%q\n' "$BHUMAN_WORKER" "${BHUMAN_SOURCE:-}"
    fi
    printf 'Command:'
    printf ' %q' "$@"
    printf '\n'
    return
  fi
  exec "$@"
}
