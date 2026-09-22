#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Hermetic cmake build for the IsaacTeleop Python wheel.
#
# All third-party dependencies that IsaacTeleop's CMakeLists.txt would
# normally pull via FetchContent are pre-fetched by Bazel http_archive rules
# and provided through ISAACTELEOP_TP_ROOT. FETCHCONTENT_FULLY_DISCONNECTED=ON
# prevents any outbound network calls during the cmake configure step.
#
# Python 3.12 is provided from the @ros2noble build-dep sysroot (python3.12-dev
# apt layer). SetupPython.cmake's `uv python install` step is bypassed by
# setting ISAAC_TELEOP_PYTHON_CONFIGURED=TRUE via cmake cache. numpy >= 2.0 is
# pre-extracted from ISAACTELEOP_NUMPY_WHL into a temp site-packages directory
# and injected via PYTHONPATH so that SetupPython.cmake's numpy venv creation
# step also finds it offline.
#
# Required env vars (set by isaacteleop_source.bzl before calling this script):
#   ISAACTELEOP_SRC                   — IsaacTeleop source tree root
#   ISAACTELEOP_MANIFEST              — expected wheel file manifest
#   ISAACTELEOP_OUT_ROOT              — Bazel output package root
#   ISAACTELEOP_BUILD_DEP_LAYERS_MANIFEST — list of @ros2noble apt layer tarballs
#   ISAACTELEOP_PATCHELF              — patchelf executable
#   ISAACTELEOP_UV_WHEEL              — uv pip wheel (.whl) for uv binary extraction
#   ISAACTELEOP_TP_ROOT               — root of vendored FetchContent source trees
#   ISAACTELEOP_NUMPY_WHL             — path to numpy 2.x .whl file
#   ISAACTELEOP_CLOUDXR_SDK           — (optional) arch-specific CloudXR SDK tarball
#   ISAACTELEOP_TARGET_ARCH           — target CPU architecture: x86_64 or aarch64
#
# For aarch64 cross-compilation from an x86_64 host:
#   - ISAACTELEOP_TARGET_ARCH must be "aarch64"
#   - /usr/bin/aarch64-linux-gnu-gcc-13 and -g++-13 must be present in the RBE container

set -euo pipefail

: "${ISAACTELEOP_SRC:?ISAACTELEOP_SRC must point to the IsaacTeleop source tree}"
: "${ISAACTELEOP_MANIFEST:?ISAACTELEOP_MANIFEST must point to the expected wheel file manifest}"
: "${ISAACTELEOP_OUT_ROOT:?ISAACTELEOP_OUT_ROOT must point to the Bazel output package root}"
: "${ISAACTELEOP_BUILD_DEP_LAYERS_MANIFEST:?ISAACTELEOP_BUILD_DEP_LAYERS_MANIFEST must list Bazel-provided apt layers}"
: "${ISAACTELEOP_PATCHELF:?ISAACTELEOP_PATCHELF must point to the Bazel-provided patchelf executable}"
: "${ISAACTELEOP_UV_WHEEL:?ISAACTELEOP_UV_WHEEL must point to the Bazel-provided uv wheel}"
: "${ISAACTELEOP_TP_ROOT:?ISAACTELEOP_TP_ROOT must point to the vendored FetchContent source tree root}"
: "${ISAACTELEOP_NUMPY_WHL:?ISAACTELEOP_NUMPY_WHL must point to the numpy 2.x wheel file}"

if [[ ! -f "$ISAACTELEOP_SRC/CMakeLists.txt" ]]; then
  echo "Missing IsaacTeleop source at $ISAACTELEOP_SRC" >&2
  exit 1
fi

build_root="$(mktemp -d "$PWD/isaacteleop_build.XXXXXXXX")"
trap 'rm -rf "$build_root"' EXIT
src_overlay="$build_root/src_overlay"
build_dir="$build_root/build"
tool_bin="$build_root/tool_bin"
build_dep_sysroot="$build_root/build_deps"
numpy_site="$build_root/numpy_site"

mkdir -p "$src_overlay" "$tool_bin" "$build_dep_sysroot" "$numpy_site"
cp -rs "$(cd "$ISAACTELEOP_SRC" && pwd)/." "$src_overlay/"

# ---------------------------------------------------------------------------
# Compiler / cross-compilation selection (mirrors cuvslam_cmake_build.sh)
# ---------------------------------------------------------------------------
_target_arch="${ISAACTELEOP_TARGET_ARCH:-x86_64}"
_host_arch="$(uname -m)"
_cross_compile=false
if [[ $_target_arch == "aarch64" && $_host_arch == "x86_64" ]]; then
  _cross_compile=true
fi

if $_cross_compile; then
  _c_compiler=/usr/bin/aarch64-linux-gnu-gcc-13
  _cxx_compiler=/usr/bin/aarch64-linux-gnu-g++-13
  _host_multiarch="aarch64-linux-gnu"
  _python_soabi="cpython-312-aarch64-linux-gnu"
  export _PYTHON_HOST_PLATFORM="linux-aarch64"
else
  _c_compiler=/usr/bin/gcc
  _cxx_compiler=/usr/bin/g++
  _host_multiarch="$(/usr/bin/gcc -print-multiarch)"
  _python_soabi="cpython-312-$_host_multiarch"
fi

# ---------------------------------------------------------------------------
# Cross-compilation cmake preparations
# ---------------------------------------------------------------------------
# Two issues arise when cross-compiling for aarch64 from an x86_64 host:
#
# 1. flatc (the FlatBuffers schema compiler) must EXECUTE on the build host.
#    cmake's FetchContent builds flatc with the cross-compiler → aarch64 ELF
#    → "Exec format error" when cmake runs it for schema header generation.
#    Fix: pre-build flatc with HOST gcc, patch GenerateFlatBuffers.cmake to
#    use it via ISAACTELEOP_HOST_FLATC, set FLATBUFFERS_BUILD_FLATC=OFF.
#
# 2. cmake's FindPython3 always re-detects Python3_SOABI by running the Python
#    interpreter (host Python → cpython-312-x86_64-linux-gnu), and FORCE-sets
#    it, overriding our -DPython3_SOABI cache flag. Fix: in the uv wrapper,
#    rename the x86_64-named .so files to aarch64 before wheel packaging.
_host_flatc=""
if $_cross_compile; then
  _fb_host_build="$build_root/flatbuffers_host"
  mkdir -p "$_fb_host_build"
  /usr/bin/cmake -S "$ISAACTELEOP_TP_ROOT/flatbuffers" -B "$_fb_host_build" \
    -DFLATBUFFERS_BUILD_TESTS=OFF \
    -DFLATBUFFERS_BUILD_FLATLIB=OFF \
    -DFLATBUFFERS_INSTALL=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER=/usr/bin/gcc \
    -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
    >/dev/null 2>&1
  /usr/bin/cmake --build "$_fb_host_build" --target flatc \
    --parallel "$(/usr/bin/nproc)" >/dev/null 2>&1
  _host_flatc="$_fb_host_build/flatc"

  # Patch GenerateFlatBuffers.cmake to accept ISAACTELEOP_HOST_FLATC, which
  # lets us bypass the `$<TARGET_FILE:flatc>` generator expression (that target
  # is disabled via FLATBUFFERS_BUILD_FLATC=OFF to prevent cross-compilation of
  # the code-generator binary) and use the pre-built x86_64 flatc instead.
  rm -f "$src_overlay/cmake/GenerateFlatBuffers.cmake"
  cat >"$src_overlay/cmake/GenerateFlatBuffers.cmake" <<'CMAKE_PATCH'
# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Patched for hermetic cross-compilation: when ISAACTELEOP_HOST_FLATC is set,
# use the pre-built host (x86_64) flatc instead of the cmake-built target so
# that flatc can actually execute during the cross-compilation build step.

function(generate_flatbuffer_headers OUT_VAR INPUT_DIR OUTPUT_DIR)
  if(ISAACTELEOP_HOST_FLATC)
    # Cross-compilation: use a pre-built host flatc (not a cmake target).
    set(FLATC_EXECUTABLE "${ISAACTELEOP_HOST_FLATC}")
    set(_flatc_dep "${ISAACTELEOP_HOST_FLATC}")
  else()
    # Ensure flatc is available from FetchContent.
    if(NOT TARGET flatc)
      message(FATAL_ERROR "flatc target not found. Make sure FlatBuffers is fetched via FetchContent before calling this function.")
    endif()
    # Use generator expression to get flatc executable path at build time.
    set(FLATC_EXECUTABLE $<TARGET_FILE:flatc>)
    set(_flatc_dep flatc)
  endif()

  # Find all .fbs files in input directory.
  file(GLOB FBS_FILES RELATIVE ${INPUT_DIR} CONFIGURE_DEPENDS "${INPUT_DIR}/*.fbs")
  if(NOT FBS_FILES)
    message(FATAL_ERROR "No .fbs files found in ${INPUT_DIR}")
    set(${OUT_VAR} "" PARENT_SCOPE)
    return()
  endif()

  set(GENERATED_HEADER_LIST "")

  foreach(SCHEMA_FILE IN LISTS FBS_FILES)
    get_filename_component(SCHEMA_NAME ${SCHEMA_FILE} NAME_WE)
    set(OUT_HEADER "${OUTPUT_DIR}/${SCHEMA_NAME}_generated.h")
    set(OUT_BFBS_HEADER "${OUTPUT_DIR}/${SCHEMA_NAME}_bfbs_generated.h")

    add_custom_command(
      OUTPUT ${OUT_HEADER} ${OUT_BFBS_HEADER}
      COMMAND ${CMAKE_COMMAND} -E make_directory ${OUTPUT_DIR}
      COMMAND ${FLATC_EXECUTABLE}
              --cpp
              --cpp-ptr-type std::shared_ptr
              --gen-object-api
              --gen-mutable
              --schema
              --bfbs-gen-embed
              --reflect-names
              --reflect-types
              -I ${INPUT_DIR}
              -o ${OUTPUT_DIR}
              ${INPUT_DIR}/${SCHEMA_FILE}
      DEPENDS ${INPUT_DIR}/${SCHEMA_FILE} ${_flatc_dep}
      COMMENT "Generating FlatBuffers C++ for ${SCHEMA_FILE}"
      VERBATIM
    )

    list(APPEND GENERATED_HEADER_LIST ${OUT_HEADER} ${OUT_BFBS_HEADER})
  endforeach()

  set(${OUT_VAR} ${GENERATED_HEADER_LIST} PARENT_SCOPE)
endfunction()
CMAKE_PATCH
fi

# Stage CloudXR SDK tarball so cmake finds it at deps/cloudxr/<filename>.
# cp -rs creates symlinks, so removing the symlink and writing a new file leaves
# the original untouched. When ISAACTELEOP_CLOUDXR_SDK is set (normal case),
# we hard-link the tarball into the overlay. Otherwise we stub out the download
# script so cmake can proceed with ENABLE_CLOUDXR_BUNDLE_CHECK=OFF.
rm -f "$src_overlay/scripts/download_cloudxr_runtime_sdk.sh"
if [[ -n ${ISAACTELEOP_CLOUDXR_SDK:-} ]]; then
  cloudxr_dest_dir="$src_overlay/deps/cloudxr"
  mkdir -p "$cloudxr_dest_dir"
  chmod u+w "$cloudxr_dest_dir"

  # cmake derives the expected tarball name from CMAKE_SYSTEM_PROCESSOR, which is
  # always the BUILD host (x86_64 in CI) — even during cross-compilation for aarch64.
  # Stage the tarball under BOTH arch names so cmake finds it regardless of whether
  # the host is x86_64 or aarch64. cmake just extracts the .so files; it doesn't
  # validate their binary architecture, so the right target-arch libs still get bundled.
  _sdk_src="$(basename "$ISAACTELEOP_CLOUDXR_SDK")" # e.g. CloudXR-6.2.1-Linux-arm64-sdk.tar.gz
  _sdk_ver="${_sdk_src#CloudXR-}"
  _sdk_ver="${_sdk_ver%%-Linux-*}" # e.g. 6.2.1
  for _arch in amd64 arm64; do
    cp -L "$ISAACTELEOP_CLOUDXR_SDK" "$cloudxr_dest_dir/CloudXR-${_sdk_ver}-Linux-${_arch}-sdk.tar.gz"
  done
  unset _sdk_src _sdk_ver _arch

  printf '#!/usr/bin/env bash\n# Hermetic: SDK already staged in deps/cloudxr/.\nexit 0\n' \
    >"$src_overlay/scripts/download_cloudxr_runtime_sdk.sh"
else
  printf '#!/usr/bin/env bash\n# Hermetic build: no CloudXR SDK provided.\nexit 0\n' \
    >"$src_overlay/scripts/download_cloudxr_runtime_sdk.sh"
fi
chmod +x "$src_overlay/scripts/download_cloudxr_runtime_sdk.sh"

# Replace the pybind11 stub generator with a hermetic version that creates
# minimal .pyi placeholders without fetching pybind11-stubgen from PyPI.
# The cmake python_stubs target runs:
#   uv run --project <stubgen_dir> python generate_stubs.py <module> <pkg_dir>
# With no dependencies in stubgen_pyproject.toml, uv run creates an empty venv
# (no network needed). The replacement generate_stubs.py writes a one-line .pyi
# placeholder so cmake's build stamp (py.typed) is created and python_wheel can
# proceed. Stubs are IDE intellisense only and don't affect wheel runtime.
stubgen_src="$src_overlay/src/core/python"

# Patch pyproject.toml.in to remove `python-preference = "only-managed"`.
# The generated pyproject.toml includes a [tool.uv] section that tells uv to
# only use managed Python installations. In RBE there is no managed Python; uv
# would try to download one (network call) and time out silently. Changing the
# preference to "system" makes uv use the Python 3.12 binary from the
# build-dep sysroot that is already in PATH. UV_PYTHON_PREFERENCE=system (set
# below) also covers this, but patching the generated source ensures the
# setting holds even when uv reads its config from the project file.
rm -f "$stubgen_src/pyproject.toml.in"
# Two patches to pyproject.toml.in (which cmake instantiates into
# python_package/Release/pyproject.toml):
#
# 1. python-preference "only-managed" → "system": prevents uv from trying to
#    download a managed Python in RBE where there is no internet.
#
# 2. license = "Apache-2.0" → license = {text = "Apache-2.0"}: the SPDX-string
#    form (PEP 639) is only recognized by setuptools >= 70. Ubuntu noble ships
#    setuptools 68.1.2, which expects the legacy {text = ...} object form. The
#    string form triggers "must be valid exactly by one definition (2 matches)"
#    in setuptools' JSON schema validation and aborts the wheel build.
sed \
  -e 's/python-preference = "only-managed"/python-preference = "system"/' \
  -e 's/^license = "Apache-2.0"$/license = {text = "Apache-2.0"}/' \
  -e 's/optional-dependencies\.retargeters-lite/optional-dependencies.retargeters_lite/' \
  "$ISAACTELEOP_SRC/src/core/python/pyproject.toml.in" \
  >"$stubgen_src/pyproject.toml.in"

# Patch runtime.py: _setup_openxr_dir copies libopenxr_cloudxr.so and
# openxr_cloudxr.json with shutil.copy2, which preserves the source file's
# read-only permissions (r-xr-xr-x from the Bazel sandbox). On subsequent
# runs the destination files can't be overwritten → PermissionError. Fix:
# chmod the destination to 0o644 before copying if it already exists.
_runtime_py="$src_overlay/src/core/cloudxr/python/runtime.py"
rm -f "$_runtime_py"
sed \
  -e 's|shutil\.copy2(src, os\.path\.join(openxr_dir, name))|dst = os.path.join(openxr_dir, name)\n        if os.path.exists(dst):\n            os.chmod(dst, 0o644)\n        shutil.copy2(src, dst)|' \
  "$ISAACTELEOP_SRC/src/core/cloudxr/python/runtime.py" \
  >"$_runtime_py"
unset _runtime_py

rm -f "$stubgen_src/stubgen_pyproject.toml"
cat >"$stubgen_src/stubgen_pyproject.toml" <<'TOML'
[project]
name = "isaacteleop-stubgen"
version = "0.0.0"
description = "Stub generation placeholder — no deps needed in hermetic builds"
requires-python = ">=3.10,<3.14"
dependencies = []
TOML

rm -f "$stubgen_src/generate_stubs.py"
cat >"$stubgen_src/generate_stubs.py" <<'PY'
"""Hermetic stub generator: creates minimal .pyi placeholders without pybind11-stubgen."""
import sys
from pathlib import Path

def main() -> int:
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <module_name> <package_dir>")
        return 1
    module_name = sys.argv[1]
    package_dir = Path(sys.argv[2]).resolve()
    if not package_dir.exists():
        print(f"Package directory does not exist: {package_dir}", file=sys.stderr)
        return 1
    pyi_path = package_dir / Path(*module_name.split(".")).with_suffix(".pyi")
    pyi_path.parent.mkdir(parents=True, exist_ok=True)
    if not pyi_path.exists():
        pyi_path.write_text("# Stub placeholder (hermetic build)\n")
    print(f"  Created stub placeholder for {module_name}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
PY

# ---------------------------------------------------------------------------
# Extract @ros2noble apt layers into a fake sysroot
# ---------------------------------------------------------------------------
while IFS= read -r layer; do
  if [[ -n $layer ]]; then
    /usr/bin/tar -xzf "$layer" -C "$build_dep_sysroot"
  fi
done <"$ISAACTELEOP_BUILD_DEP_LAYERS_MANIFEST"

# ---------------------------------------------------------------------------
# Extract uv binary from the pip wheel (it's a zip)
# ---------------------------------------------------------------------------
/usr/bin/python3 - "$ISAACTELEOP_UV_WHEEL" "$tool_bin/_uv_real" <<'PY'
import sys
import zipfile

wheel_path, uv_path = sys.argv[1:]
with zipfile.ZipFile(wheel_path) as wheel:
    uv_entries = [
        name for name in wheel.namelist()
        if name.endswith(".data/scripts/uv")
    ]
    if len(uv_entries) != 1:
        print(
            f"uv wheel {wheel_path} has unexpected uv script entries: {uv_entries}",
            file=sys.stderr,
        )
        sys.exit(1)

    with wheel.open(uv_entries[0]) as src, open(uv_path, "wb") as dst:
        dst.write(src.read())
PY
chmod +x "$tool_bin/_uv_real"
# Move real uv binary; wrapper is written after python3_executable is resolved.
mv "$tool_bin/_uv_real" "$tool_bin/_uv"

# ---------------------------------------------------------------------------
# Sysroot-derived paths for cmake and compiler flags
# ---------------------------------------------------------------------------
build_dep_libdir="$build_dep_sysroot/usr/lib/$_host_multiarch"
build_dep_include_path="$build_dep_sysroot/usr/include;$build_dep_sysroot/usr/include/$_host_multiarch"
build_dep_library_path="$build_dep_libdir;$build_dep_sysroot/usr/lib"
build_dep_compile_flags="-isystem $build_dep_sysroot/usr/include -isystem $build_dep_sysroot/usr/include/$_host_multiarch"
build_dep_link_flags="-L$build_dep_libdir -L$build_dep_sysroot/usr/lib -Wl,-rpath-link,$build_dep_libdir -Wl,-rpath-link,$build_dep_sysroot/usr/lib"
export CFLAGS="${CFLAGS:-} $build_dep_compile_flags"
export CXXFLAGS="${CXXFLAGS:-} $build_dep_compile_flags"
export LDFLAGS="${LDFLAGS:-} $build_dep_link_flags"
export PKG_CONFIG_PATH="$build_dep_libdir/pkgconfig:$build_dep_sysroot/usr/lib/pkgconfig:$build_dep_sysroot/usr/share/pkgconfig"
export PKG_CONFIG_SYSROOT_DIR="$build_dep_sysroot"

uv_find_links="$build_root/uv_find_links"
mkdir -p "$uv_find_links"
# Symlink the pre-fetched numpy wheel so uv can satisfy the wheel build's
# numpy>=2.0 build-system requirement offline. setuptools and wheel are
# bundled unconditionally by uv even when listed in [build-system] requires,
# so they do not need to be in the find-links dir.
ln -sf "$ISAACTELEOP_NUMPY_WHL" "$uv_find_links/$(basename "$ISAACTELEOP_NUMPY_WHL")"

export PATH="$tool_bin:$build_dep_sysroot/usr/bin:/usr/local/bin:/usr/bin:/bin"
export UV_CACHE_DIR="$build_root/uv_cache"
export UV_PYTHON_INSTALL_DIR="$build_root/uv_python"
export UV_PYTHON_INSTALL_BIN=0
export UV_NO_INDEX=1
export UV_OFFLINE=1
export UV_FIND_LINKS="$uv_find_links"
export UV_PYTHON_PREFERENCE=system
export UV_PYTHON_DOWNLOADS=never
export GIT_ROOT="$src_overlay"
export HOME="$build_root/home"
mkdir -p "$HOME"

# ---------------------------------------------------------------------------
# Python 3.12 — resolve from build-dep sysroot; fall back to host system.
# The sysroot python3.12-dev layer provides headers; the binary is used to
# bypass SetupPython.cmake's `uv python install` via cmake cache variables.
# ---------------------------------------------------------------------------
if $_cross_compile; then
  # Cross-compiling: the sysroot Python is a target-arch (aarch64) ELF that
  # cannot execute on the x86_64 host. Use the host Python for running build
  # tools (wheel builder, uv, stub generator). Headers and libs still come
  # from the target-arch sysroot for compiling the extension modules.
  python3_executable="$(command -v python3.12 2>/dev/null || true)"
  if [[ -z $python3_executable ]]; then
    echo "python3.12 not found on host PATH (required for cross-compilation build tools)" >&2
    exit 1
  fi
else
  python3_executable="$build_dep_sysroot/usr/bin/python3.12"
  if [[ ! -x $python3_executable ]]; then
    python3_executable="$(command -v python3.12 2>/dev/null || true)"
    if [[ -z $python3_executable ]]; then
      echo "python3.12 not found in build-dep sysroot ($build_dep_sysroot/usr/bin) or host PATH" >&2
      exit 1
    fi
  fi
fi
python3_include="$build_dep_sysroot/usr/include/python3.12"
python3_lib="$build_dep_libdir/libpython3.12.so"

# ---------------------------------------------------------------------------
# numpy >= 2.0 — extract the pre-fetched wheel into a temp site-packages dir
# and inject it via PYTHONPATH so that SetupPython.cmake's execute_process
# call ("import numpy; check version") finds numpy 2.x without network access.
# ---------------------------------------------------------------------------
/usr/bin/python3 -m zipfile -e "$ISAACTELEOP_NUMPY_WHL" "$numpy_site"
# Include the sysroot's dist-packages so that python3-setuptools and python3-wheel
# (extracted from @ros2noble apt layers) are importable by uv build --no-build-isolation.
build_dep_dist_packages="$build_dep_sysroot/usr/lib/python3/dist-packages"
export PYTHONPATH="$numpy_site:$build_dep_dist_packages${PYTHONPATH:+:$PYTHONPATH}"

# Write wheel_builder.py: uses setuptools.build_meta directly.
# This bypasses uv's build-env resolution (which hangs on network even with
# UV_OFFLINE=1) and pip's isolated-env install (which also tries PyPI). The
# setuptools package from python3-setuptools (extracted into build_dep_sysroot)
# vendors its own wheel-building code, so the standalone 'wheel' package is
# not needed. numpy is in PYTHONPATH from the pre-extracted wheel.
cat >"$build_root/wheel_builder.py" <<'PY'
"""Build an isaacteleop wheel via setuptools.build_meta — fully offline."""
import os
import sys

wheel_dir = sys.argv[1]
src_dir = sys.argv[2]

os.chdir(src_dir)
os.makedirs(wheel_dir, exist_ok=True)

try:
    import setuptools.build_meta as backend
    whl = backend.build_wheel(wheel_dir)
    print(f"Built: {os.path.join(wheel_dir, whl)}")
except Exception as exc:
    print(f"wheel_builder: {exc}", file=sys.stderr)
    sys.exit(1)
PY

# Write the uv wrapper now that python3_executable and build_root are known.
# 'uv build': call wheel_builder.py directly — no uv network calls, no venv.
# 'uv run' and all other subcommands: delegate to the real uv binary.
# $python3_executable, $build_root, $_cross_compile expand at wrapper-write
# time (unquoted HEREDOC delimiter); \$... expand at wrapper-execution time.
if $_cross_compile; then
  cat >"$tool_bin/uv" <<UV_WRAPPER
#!/usr/bin/env bash
real_uv="\$(dirname "\$(readlink -f "\$0")")/_uv"
if [[ "\${1:-}" == "build" ]]; then
    shift
    out_dir=""
    src_dir=""
    while [[ \$# -gt 0 ]]; do
        case "\$1" in
            --wheel|--no-build-isolation) shift;;
            --out-dir|--outdir) out_dir="\$2"; shift 2;;
            --*) echo "uv wrapper: unknown flag '\$1' — ignoring" >&2; shift;;
            *) src_dir="\$1"; shift;;
        esac
    done
    # cmake's FindPython3 detects Python3_SOABI by running the host (x86_64)
    # Python interpreter regardless of CMAKE_SYSTEM_PROCESSOR, so extension
    # modules are named cpython-312-x86_64-linux-gnu.so even though their ELF
    # content is aarch64 (compiled with the cross-compiler). Rename to the
    # correct aarch64 suffix before packaging so the wheel matches the manifest.
    find "\$src_dir" -name "*-x86_64-linux-gnu.so" | while IFS= read -r _f; do
        mv "\$_f" "\${_f/x86_64-linux-gnu/aarch64-linux-gnu}"
    done
    # Fix RPATH on Python extension modules to \$ORIGIN/../../.. as required by
    # verify_deb.py. Intentionally excludes CloudXR native libs (libcloudxr.so,
    # libNvStreamBase.so, etc.) so they keep their original \$ORIGIN RPATH and
    # can find each other in the same native/ directory at runtime.
    find "\$src_dir" -name "*.cpython-*.so" | while IFS= read -r _f; do
        $ISAACTELEOP_PATCHELF --set-rpath '\$ORIGIN/../../..' "\$_f" 2>/dev/null || true
    done
    exec $python3_executable $build_root/wheel_builder.py "\$out_dir" "\$src_dir"
else
    exec "\$real_uv" "\$@"
fi
UV_WRAPPER
else
  cat >"$tool_bin/uv" <<UV_WRAPPER
#!/usr/bin/env bash
real_uv="\$(dirname "\$(readlink -f "\$0")")/_uv"
if [[ "\${1:-}" == "build" ]]; then
    shift
    out_dir=""
    src_dir=""
    while [[ \$# -gt 0 ]]; do
        case "\$1" in
            --wheel|--no-build-isolation) shift;;
            --out-dir|--outdir) out_dir="\$2"; shift 2;;
            --*) echo "uv wrapper: unknown flag '\$1' — ignoring" >&2; shift;;
            *) src_dir="\$1"; shift;;
        esac
    done
    # Fix RPATH on Python extension modules to \$ORIGIN/../../.. as required by
    # verify_deb.py. Intentionally excludes CloudXR native libs (libcloudxr.so,
    # libNvStreamBase.so, etc.) so they keep their original \$ORIGIN RPATH and
    # can find each other in the same native/ directory at runtime.
    find "\$src_dir" -name "*.cpython-*.so" | while IFS= read -r _f; do
        $ISAACTELEOP_PATCHELF --set-rpath '\$ORIGIN/../../..' "\$_f" 2>/dev/null || true
    done
    exec $python3_executable $build_root/wheel_builder.py "\$out_dir" "\$src_dir"
else
    exec "\$real_uv" "\$@"
fi
UV_WRAPPER
fi
chmod +x "$tool_bin/uv"

# ---------------------------------------------------------------------------
# CMake configure — fully offline
# ---------------------------------------------------------------------------
# Build cross-compilation cmake flags (empty array for native builds).
_cross_cmake_flags=()
if $_cross_compile; then
  _cross_cmake_flags+=(
    -DCMAKE_SYSTEM_NAME=Linux
    -DCMAKE_SYSTEM_PROCESSOR="$_target_arch"
    -DCMAKE_C_COMPILER="$_c_compiler"
    -DCMAKE_CXX_COMPILER="$_cxx_compiler"
    -DPython3_SOABI="$_python_soabi"
    # flatc is a BUILD-HOST tool; disable the cross-compiled FetchContent target
    # and point cmake to the pre-built x86_64 flatc via GenerateFlatBuffers.cmake.
    -DFLATBUFFERS_BUILD_FLATC=OFF
    -DISAACTELEOP_HOST_FLATC="$_host_flatc"
  )
fi

/usr/bin/cmake -S "$src_overlay" -B "$build_dir" \
  -DCMAKE_INSTALL_PREFIX="$build_dir/install" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_SUPPRESS_REGENERATION:BOOL=TRUE \
  -DCMAKE_PREFIX_PATH="$build_dep_sysroot/usr" \
  -DCMAKE_INCLUDE_PATH="$build_dep_include_path" \
  -DCMAKE_LIBRARY_PATH="$build_dep_library_path" \
  -DCMAKE_C_FLAGS="$CFLAGS" \
  -DCMAKE_CXX_FLAGS="$CXXFLAGS" \
  -DCMAKE_EXE_LINKER_FLAGS="$LDFLAGS" \
  -DCMAKE_SHARED_LINKER_FLAGS="$LDFLAGS" \
  -DCMAKE_MODULE_LINKER_FLAGS="$LDFLAGS" \
  -DPATCHELF_EXECUTABLE="$ISAACTELEOP_PATCHELF" \
  -DISAAC_TELEOP_PYTHON_VERSION=3.12 \
  -DBUILD_PYTHON_BINDINGS=ON \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_EXAMPLE_TELEOP_ROS2=OFF \
  -DBUILD_TESTS=OFF \
  -DBUILD_PLUGINS=OFF \
  -DBUILD_PLUGIN_OAK_CAMERA=OFF \
  -DBUILD_TESTING=OFF \
  -DBUILD_CONFORMANCE_TESTS=OFF \
  -DBUILD_API_LAYERS=OFF \
  -DBUILD_VIZ=OFF \
  -DENABLE_CLANG_FORMAT_CHECK=OFF \
  -DBUNDLE_ROBOTIC_GROUNDING=OFF \
  \
  -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
  "-DFETCHCONTENT_SOURCE_DIR_SANITIZERS-CMAKE=$ISAACTELEOP_TP_ROOT/sanitizers-cmake" \
  "-DFETCHCONTENT_SOURCE_DIR_OPENXR-SDK=$ISAACTELEOP_TP_ROOT/openxr-sdk" \
  "-DFETCHCONTENT_SOURCE_DIR_YAML-CPP=$ISAACTELEOP_TP_ROOT/yaml-cpp" \
  "-DFETCHCONTENT_SOURCE_DIR_PYBIND11=$ISAACTELEOP_TP_ROOT/pybind11" \
  "-DFETCHCONTENT_SOURCE_DIR_FLATBUFFERS=$ISAACTELEOP_TP_ROOT/flatbuffers" \
  "-DFETCHCONTENT_SOURCE_DIR_MCAP=$ISAACTELEOP_TP_ROOT/mcap" \
  \
  -DISAAC_TELEOP_PYTHON_CONFIGURED:BOOL=TRUE \
  -DPython3_EXECUTABLE:FILEPATH="$python3_executable" \
  -DPYTHON_EXECUTABLE:FILEPATH="$python3_executable" \
  -DPython3_INCLUDE_DIRS:PATH="$python3_include" \
  -DPython3_LIBRARIES:FILEPATH="$python3_lib" \
  -DPYTHON_INCLUDE_DIRS:PATH="$python3_include" \
  -DPYTHON_LIBRARIES:FILEPATH="$python3_lib" \
  -DPYBIND11_PYTHON_VERSION:STRING=3.12 \
  -DPYBIND11_PYTHON_INCLUDE_DIR:PATH="$python3_include" \
  -DPYBIND11_PYTHON_LIBRARIES:FILEPATH="$python3_lib" \
  \
  "${_cross_cmake_flags[@]}" \
  \
  "-DENABLE_CLOUDXR_BUNDLE_CHECK=$([[ -n ${ISAACTELEOP_CLOUDXR_SDK:-} ]] && echo ON || echo OFF)"

/usr/bin/cmake --build "$build_dir" --target python_wheel --parallel "$(/usr/bin/nproc)"

shopt -s nullglob
wheels=("$build_dir"/wheels/isaacteleop-*.whl)
if [[ ${#wheels[@]} -ne 1 ]]; then
  echo "Expected one isaacteleop wheel under $build_dir/wheels, found ${#wheels[@]}" >&2
  exit 1
fi

/usr/bin/python3 - "$ISAACTELEOP_MANIFEST" "$ISAACTELEOP_OUT_ROOT" "${wheels[0]}" <<'PY'
import os
import shutil
import sys
import zipfile

manifest_path, out_root, wheel_path = sys.argv[1:]

with open(manifest_path, encoding="utf-8") as manifest_file:
    expected = {
        line.strip()
        for line in manifest_file
        if line.strip() and not line.startswith("#")
    }

with zipfile.ZipFile(wheel_path) as wheel:
    actual = {name for name in wheel.namelist() if not name.endswith("/")}
    missing = sorted(expected - actual)
    extra = sorted(actual - expected)
    if missing or extra:
        if missing:
            print("Wheel is missing expected files:", file=sys.stderr)
            for path in missing:
                print(f"  {path}", file=sys.stderr)
        if extra:
            print("Wheel contains files not listed in the manifest:", file=sys.stderr)
            for path in extra:
                print(f"  {path}", file=sys.stderr)
        sys.exit(1)

    for relpath in sorted(expected):
        dest = os.path.join(out_root, relpath)
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        with wheel.open(relpath) as src, open(dest, "wb") as dst:
            shutil.copyfileobj(src, dst)
PY
