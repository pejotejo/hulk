#!/usr/bin/env python3
"""Build the pinned Linux/x86-64 B-Human motion worker outside the HULK checkout."""

import argparse
import fcntl
import hashlib
import os
from pathlib import Path
import platform
import shlex
import shutil
import subprocess
import sys
import tarfile
from urllib.request import urlopen

REVISION = "2e3374ba896da8288607f9a1881b876642fe18b8"
FLATBUFFERS_VERSION = "25.2.10"
FLATBUFFERS_SHA256 = "b9c2df49707c57a48fc0923d52b8c73beb72d675f9d44b2211e4569be40a7421"
ORT_VERSION = "1.22.0"
ORT_SHA256 = "8344d55f93d5bc5021ce342db50f62079daf39aaafb5d311a451846228be49b3"
ADAPTER = Path(__file__).resolve().parent


def run(*args):
    subprocess.run([str(arg) for arg in args], check=True, stdout=sys.stderr)


def download(url, destination, expected):
    if not destination.exists():
        partial = destination.with_suffix(".part")
        try:
            with urlopen(url, timeout=60) as response, partial.open("wb") as output:
                while chunk := response.read(1024 * 1024):
                    output.write(chunk)
            partial.replace(destination)
        finally:
            partial.unlink(missing_ok=True)
    with destination.open("rb") as source:
        actual = hashlib.file_digest(source, "sha256").hexdigest()
    if actual != expected:
        raise RuntimeError(f"Checksum mismatch for {destination}: {actual}. Remove this archive and retry.")


def prerequisites():
    missing = [command for command in ("git", "cmake", "ninja", "clang++") if not shutil.which(command)]
    if missing:
        raise RuntimeError(
            f"Missing build tools: {', '.join(missing)}. Install Git, CMake >=3.24, Ninja and Clang "
            "with your package manager (Debian/Ubuntu: sudo apt install git cmake ninja-build clang; "
            "Fedora: sudo dnf install git cmake ninja-build clang)."
        )
    version = subprocess.check_output(["cmake", "--version"], text=True).split()[2]
    if tuple(map(int, version.split(".")[:2])) < (3, 24):
        raise RuntimeError(f"CMake >=3.24 is required; found {version}. Upgrade CMake and retry.")


def runtime_directory(cache, source=None):
    native = (source or cache / "source") / "Util/onnxruntime/lib/Linux"
    return native if source or (native / "libonnxruntime.so.1").is_file() else cache / "onnxruntime/lib"


def install_runtime(cache, ort):
    version_file = ort / "installed_version"
    if (version_file.is_file() and version_file.read_text().strip() == ORT_VERSION
            and (ort / "libonnxruntime.so.1").is_file()):
        return
    archive = cache / "onnxruntime.tar.gz"
    download(f"https://github.com/microsoft/onnxruntime/releases/download/v{ORT_VERSION}/"
             f"onnxruntime-linux-x64-{ORT_VERSION}.tgz", archive, ORT_SHA256)
    ort.mkdir(parents=True, exist_ok=True)
    with tarfile.open(archive) as package:
        for member in package.getmembers():
            if "/lib/" in member.name and (member.isfile() or member.issym()):
                member.name = member.name.split("/lib/", 1)[1]
                package.extract(member, ort, filter="data")
    version_file.write_text(ORT_VERSION)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    default = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache") / "hulk-bhuman" / REVISION
    parser.add_argument("--cache", type=Path, default=default)
    parser.add_argument("--print-cache", action="store_true", help="Print the resolved cache directory without building")
    parser.add_argument("--print-ort", action="store_true", help="Print the ONNX Runtime library path without setup")
    parser.add_argument("--runtime-only", action="store_true", help="Set up only the pinned CPU ONNX Runtime (no worker or compiler needed)")
    parser.add_argument("--dry-run", action="store_true", help="Check prerequisites and print paths without downloading or building")
    parser.add_argument("--source", type=Path, help="Reuse an existing pinned release checkout")
    parser.add_argument("--jobs", type=int, default=os.environ.get("BHUMAN_JOBS", os.cpu_count() or 1),
                        help="Number of parallel build jobs (default: %(default)s CPU threads)")
    parser.add_argument("--generate-rust", action="store_true", help="Regenerate the checked-in Rust protocol")
    args = parser.parse_args()
    if args.print_cache:
        print(args.cache.resolve())
        return
    cache = args.cache.resolve()
    source = args.source.resolve() if args.source else None
    if args.print_ort:
        print(runtime_directory(cache, source) / "libonnxruntime.so.1")
        return
    if sys.version_info < (3, 12):
        parser.error("Python >=3.12 is required. Install a recent Python 3 and retry.")
    if platform.system() != "Linux" or platform.machine() not in ("x86_64", "AMD64"):
        parser.error("Automatic B-Human/ONNX Runtime setup supports Linux/x86-64. For HULK-only runs on other Linux architectures, set ORT_DYLIB_PATH yourself.")
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    if not args.runtime_only:
        prerequisites()
        if source and not (source / ".git").exists():
            parser.error(f"--source must be an existing B-Human Git checkout: {source}")
    if args.dry_run:
        print(f"Cache: {cache}")
        runtime_source = source if args.runtime_only else source or cache / "source"
        print(f"ONNX Runtime: {runtime_directory(cache, runtime_source) / 'libonnxruntime.so.1'}")
        if not args.runtime_only:
            print(f"Worker: {cache / 'worker-build/bhuman-motion-worker'}")
            print(f"B-Human source: {source or cache / 'source'}")
        return
    cache.mkdir(parents=True, exist_ok=True)
    # Launchers share this cache; do not extract or reconfigure it concurrently.
    with (cache / "setup.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if args.runtime_only:
            ort = runtime_directory(cache, source)
            install_runtime(cache, ort)
            print(f"ONNX Runtime: {ort / 'libonnxruntime.so.1'}")
            return
        build_worker(args, cache, source or cache / "source")


def build_worker(args, cache, source):
    if not source.exists():
        run("git", "clone", "--filter=blob:none", "--no-checkout",
            "https://github.com/bhuman/BHumanCodeRelease.git", source)
        run("git", "-C", source, "sparse-checkout", "set",
            "Src", "Make", "Config/Robots", "Config/Locations", "Config/Scenarios", "Config/NeuralNets", "Config/Behavior",
            "Util/CABSL", "Util/CompiledNN2ONNX", "Util/GameController", "Util/fftw-3.3",
            "Util/hdf5", "Util/libjpeg", "Util/onnxruntime", "Util/voronoi")
        run("git", "-C", source, "checkout", "--detach", REVISION)
    revision = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    if revision != REVISION:
        raise RuntimeError(f"Expected B-Human {REVISION}, found {revision} in {source}. Use the pinned checkout or unset BHUMAN_SOURCE.")
    if subprocess.run(["git", "-C", str(source), "diff", "--exit-code", "--quiet", "HEAD"]).returncode:
        raise RuntimeError(f"B-Human checkout has tracked changes: {source}. Use a clean pinned checkout; your changes were left intact.")
    if not (source / "Config/Behavior/Strategies").is_dir():
        run("git", "-C", source, "sparse-checkout", "add", "Config/Behavior")
    run("git", "-C", source, "submodule", "update", "--init", "--depth", "1",
        "--filter=blob:none", "Util/CompiledNN", "Util/SimRobot")

    flatbuffers = cache / f"flatbuffers-{FLATBUFFERS_VERSION}"
    archive = cache / "flatbuffers.tar.gz"
    if not flatbuffers.exists():
        download(f"https://codeload.github.com/google/flatbuffers/tar.gz/refs/tags/v{FLATBUFFERS_VERSION}",
                 archive, FLATBUFFERS_SHA256)
        with tarfile.open(archive) as package:
            package.extractall(cache, filter="data")
    flatbuffers_build = cache / "flatbuffers-build"
    run("cmake", "-S", flatbuffers, "-B", flatbuffers_build, "-G", "Ninja",
        "-DCMAKE_BUILD_TYPE=Release", "-DFLATBUFFERS_BUILD_TESTS=OFF",
        "-DFLATBUFFERS_BUILD_FLATLIB=OFF", "-DFLATBUFFERS_BUILD_SHAREDLIB=OFF")
    run("cmake", "--build", flatbuffers_build, "--target", "flatc", "-j", args.jobs)
    flatc = flatbuffers_build / "flatc"

    install_runtime(cache, source / "Util/onnxruntime/lib/Linux")
    build = cache / "worker-build"
    run("cmake", "-S", source / "Make/CMake", "-B", build, "-G", "Ninja",
        "-DCMAKE_BUILD_TYPE=Release", "-DMINIMAL_PROJECT=ON",
        "-DCMAKE_DISABLE_FIND_PACKAGE_Python3=TRUE",
        f"-DCMAKE_PROJECT_INCLUDE={ADAPTER / 'Inject.cmake'}",
        f"-DFLATBUFFERS_SOURCE={flatbuffers}", f"-DFLATC={flatc}")
    run("cmake", "--build", build, "--target", "bhuman-motion-worker", "-j", args.jobs)
    if args.generate_rust:
        run(flatc, "--rust", "-o", ADAPTER.parent / "src/bhuman", ADAPTER / "protocol.fbs")
        generated = ADAPTER.parent / "src/bhuman/protocol_generated.rs"
        generated.write_text("\n".join(line.rstrip() for line in generated.read_text().splitlines()).rstrip() + "\n")
    print(f"Worker: {build / 'bhuman-motion-worker'}")
    print(f"B-Human source: {source}")


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as error:
        sys.exit(f"B-Human setup failed ({error.returncode}): {shlex.join(map(str, error.cmd))}\n"
                 "See the command output above. Install missing system libraries or fix checkout access, then retry; cached downloads are reused.")
    except (OSError, RuntimeError, tarfile.TarError) as error:
        sys.exit(f"B-Human/ONNX Runtime setup failed: {error}\n"
                 "Check cache permissions and network access, then retry. Use --cache PATH for a writable cache.")
