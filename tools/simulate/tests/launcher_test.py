#!/usr/bin/env python3
"""Offline launcher checks; never invokes Cargo builds or downloads."""

import hashlib
import importlib.util
import io
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[3]
BUILDER = ROOT / "tools/simulate/bhuman/build.py"


class Launchers(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="hulk-launcher-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / "Cargo.toml").write_text('[workspace.package]\nrust-version = "1.98.0"\n')
        for relative in ("twix", "simulator-mcp", "mujoco-simulator",
                         "tools/simulate/runtime.sh", "tools/simulate/bhuman/build.py"):
            destination = self.root / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / relative, destination)
        (self.root / "tools/mujoco-simulator/mujoco-simulator").mkdir(parents=True)
        for relative in ("tools/mujoco-simulator/mujoco-simulator/K1/meshes/Trunk.STL",
                         *(f"etc/neural_networks/{name}.onnx" for name in (
                             "K1_walk", "K1_stand_up", "K1_kick_inner", "K1_kick_inner_soft")),
                         *(f"tools/simulate/assets/branding/{name}_panel.png" for name in (
                             "hulks", "kdab", "cc", "draeger", "gea", "haspa", "tk", "gegenstrom", "tuhh"))):
            destination = self.root / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(b"fixture asset")
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.command("cargo", "printf '%s\\n' '{\"jsonrpc\":\"2.0\"}'")
        self.command("rustc", "echo 'rustc 1.98.0 (fixture)'")
        self.env = {key: value for key, value in os.environ.items()
                    if not key.startswith(("BHUMAN_", "MUJOCO_", "ORT_", "LD_", "XDG_CACHE_HOME"))}
        self.env.update(PATH=f"{self.bin}:{os.environ['PATH']}", XDG_CACHE_HOME=str(self.root / "cache"))
        self.ort = self.root / "custom runtime.so"
        self.ort.touch()

    def command(self, name, body):
        path = self.bin / name
        path.write_text(f"#!/bin/bash\nset -e\n{body}\n")
        path.chmod(0o755)

    def launch(self, name, *args, **env):
        return subprocess.run(["/bin/bash", "-e", str(self.root / name), *args],
                              env=self.env | env, cwd="/tmp", capture_output=True, text=True)

    def test_help_does_not_set_up_or_build(self):
        self.command("python3", "exit 91")
        for name in ("twix", "simulator-mcp", "mujoco-simulator"):
            with self.subTest(name=name):
                result = self.launch(name, "--help")
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("Usage:", result.stdout)
        self.assertFalse((self.root / "cache").exists())

    def test_native_dry_run_preserves_arguments_and_does_not_write_cache(self):
        result = self.launch("twix", "--simulator", "--bhuman", "--dry-run", "/hulks/1", "name with spaces")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--features bhuman -- --zenoh-namespace hulk_simulator /hulks/1 name\\ with\\ spaces", result.stdout)
        self.assertIn("BHUMAN_WORKER=", result.stdout)
        self.assertFalse((self.root / "cache").exists())

    def test_ordinary_twix_has_no_simulator_setup(self):
        self.command("python3", "exit 91")
        result = self.launch("twix", "--dry-run", "--", "--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("--features", result.stdout)
        self.assertIn(" -- --help", result.stdout)

    def test_native_overrides_skip_build_and_preserve_loader_paths(self):
        source = self.root / "native source"
        (source / "Config").mkdir(parents=True)
        worker = self.bin / "worker"
        self.command("worker", "exit 0")
        library = self.root / "mujoco lib"
        library.mkdir()
        (library / "libmujoco.so").touch()
        self.command("cmake", "exit 92")
        result = self.launch("twix", "--bhuman", "--dry-run",
                             BHUMAN_SOURCE=str(source), BHUMAN_WORKER=str(worker),
                             ORT_DYLIB_PATH=str(self.ort), MUJOCO_DYNAMIC_LINK_DIR=str(library),
                             LD_LIBRARY_PATH="/existing/library/path")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("custom\\ runtime.so", result.stdout)
        self.assertIn("mujoco\\ lib:/existing/library/path", result.stdout)
        self.assertNotIn("Cache:", result.stderr)

    def test_bad_overrides_fail_with_actionable_errors(self):
        for overrides, message in (
            ({"BHUMAN_WORKER": str(self.bin / "cargo")}, "Set BHUMAN_SOURCE alongside"),
            ({"ORT_DYLIB_PATH": "/missing/library.so"}, "unset it for automatic CPU runtime setup"),
            ({"MUJOCO_DOWNLOAD_DIR": "relative"}, "must be an absolute directory"),
        ):
            with self.subTest(overrides=overrides):
                result = self.launch("twix", "--bhuman" if "BHUMAN_WORKER" in overrides else "--simulator",
                                     "--dry-run", **overrides)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(message, result.stderr)

    def test_old_rust_reports_per_launch_toolchain_override(self):
        self.command("rustc", "echo 'rustc 1.95.0 (fixture)'")
        result = self.launch("twix", "--simulator", "--bhuman", "--dry-run")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Rust >=1.98.0 is required; active rustc is 1.95.0", result.stderr)
        self.assertIn("RUSTUP_TOOLCHAIN=1.98.0 ./twix", result.stderr)
        self.assertFalse((self.root / "cache").exists())

    def test_lfs_pointer_dry_run_does_not_fetch(self):
        (self.root / "etc/neural_networks/K1_walk.onnx").write_text("version https://git-lfs.github.com/spec/v1\n")
        result = self.launch("simulator-mcp", "--dry-run", ORT_DYLIB_PATH=str(self.ort))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Would fetch missing simulator LFS assets:", result.stderr)
        self.assertFalse((self.root / "cache").exists())

    def test_failed_lfs_fetch_is_not_retried_forever(self):
        (self.root / "etc/neural_networks/K1_walk.onnx").unlink()
        log = self.root / "git-calls"
        self.command("git-lfs", "exit 0")
        self.command("git", 'printf "%s\\n" "$*" >> "$LFS_CALL_LOG"')
        result = self.launch("simulator-mcp", ORT_DYLIB_PATH=str(self.ort), LFS_CALL_LOG=str(log))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("still missing after Git LFS download", result.stderr)
        self.assertEqual(len(log.read_text().splitlines()), 1)
        self.assertEqual(result.stdout, "")

    def test_mcp_stdout_remains_protocol_only(self):
        result = self.launch("simulator-mcp", ORT_DYLIB_PATH=str(self.ort))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, '{"jsonrpc":"2.0"}\n')

    def test_lfs_assets_are_hydrated_before_launch(self):
        model = self.root / "etc/neural_networks/K1_walk.onnx"
        model.write_text("version https://git-lfs.github.com/spec/v1\n")
        self.command("git-lfs", "exit 0")
        self.command("git", 'printf "hydrated fixture" > "$LFS_MODEL"')
        result = self.launch("simulator-mcp", ORT_DYLIB_PATH=str(self.ort), LFS_MODEL=str(model))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(model.read_text(), "hydrated fixture")
        self.assertEqual(result.stdout, '{"jsonrpc":"2.0"}\n')
        self.assertIn("Fetching simulator meshes", result.stderr)

    def test_branding_only_lfs_assets_are_hydrated_before_cargo(self):
        log = self.root / "launch-order"
        self.command("git-lfs", "exit 0")
        self.command("git", 'printf "%s\\n" "$*" > "$LFS_CALL_LOG"\n'
                            'printf "hydrated panel" > "$LFS_PANEL"\n'
                            'echo hydration > "$LAUNCH_ORDER"')
        self.command("cargo", '[[ "$(cat "$LFS_PANEL")" == "hydrated panel" ]]\n'
                              'echo cargo >> "$LAUNCH_ORDER"')
        for name in ("hulks", "tuhh"):
            panel = self.root / f"tools/simulate/assets/branding/{name}_panel.png"
            for state in ("pointer", "missing"):
                with self.subTest(name=name, state=state):
                    if state == "pointer":
                        panel.write_text("version https://git-lfs.github.com/spec/v1\n")
                    else:
                        panel.unlink()
                    result = self.launch("simulator-mcp", ORT_DYLIB_PATH=str(self.ort),
                                         LFS_PANEL=str(panel), LAUNCH_ORDER=str(log),
                                         LFS_CALL_LOG=str(self.root / "git-calls"))
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertEqual(log.read_text().splitlines(), ["hydration", "cargo"])
                    self.assertIn("tools/simulate/assets/branding/*_panel.png",
                                  (self.root / "git-calls").read_text())

    def test_legacy_dry_run_is_independent_of_caller_directory(self):
        result = self.launch("mujoco-simulator", "--dry-run", "--example", "value with spaces")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Command: uv run main.py --example value\\ with\\ spaces", result.stdout)


class RuntimeBuilder(unittest.TestCase):
    def test_missing_build_tools_are_reported_without_traceback(self):
        with tempfile.TemporaryDirectory() as empty_path:
            result = subprocess.run([sys.executable, str(BUILDER), "--dry-run"],
                                    env=os.environ | {"PATH": empty_path}, capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Missing build tools: git, cmake, ninja, clang++", result.stderr)
        self.assertNotIn("Traceback", result.stderr)

    def test_runtime_extracts_verified_cache_and_repairs_missing_library(self):
        spec = importlib.util.spec_from_file_location("bhuman_builder", BUILDER)
        builder = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(builder)
        with tempfile.TemporaryDirectory() as directory:
            cache = Path(directory)
            archive = cache / "onnxruntime.tar.gz"
            with tarfile.open(archive, "w:gz") as package:
                library = tarfile.TarInfo("onnxruntime/lib/libonnxruntime.so.1")
                library.size = len(b"fixture")
                package.addfile(library, io.BytesIO(b"fixture"))
            checksum = hashlib.sha256(archive.read_bytes()).hexdigest()
            ort = cache / "onnxruntime/lib"
            with patch.object(builder, "ORT_SHA256", checksum), patch.object(builder, "urlopen", side_effect=AssertionError("network")):
                builder.install_runtime(cache, ort)
                self.assertEqual((ort / "libonnxruntime.so.1").read_bytes(), b"fixture")
                (ort / "libonnxruntime.so.1").unlink()
                builder.install_runtime(cache, ort)
                self.assertTrue((ort / "libonnxruntime.so.1").is_file())
                with patch.object(builder, "download", side_effect=AssertionError("redundant download")):
                    builder.install_runtime(cache, ort)
                (ort / "libonnxruntime.so.1").unlink()
                archive.write_bytes(b"tampered archive")
                with self.assertRaisesRegex(RuntimeError, "Checksum mismatch"):
                    builder.install_runtime(cache, ort)


if __name__ == "__main__":
    unittest.main()
