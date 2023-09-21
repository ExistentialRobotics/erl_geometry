import os
import subprocess
import sys
from argparse import ArgumentParser

from setuptools import Extension
from setuptools import find_packages
from setuptools import setup
from setuptools.command.build_ext import build_ext

python_pkg_name = "erl_geometry"
pybind_module_name = "pyerl_geometry"

if os.environ.get("ROS_VERSION", "0") == "1":
    from catkin_pkg.python_setup import generate_distutils_setup

    setup_args = generate_distutils_setup(
        packages=find_packages("python"),
        package_dir={python_pkg_name: f"python/{python_pkg_name}"},
    )
    setup(**setup_args)
    exit(0)

project_dir = os.path.dirname(os.path.realpath(__file__))
src_python_dir = os.path.join(project_dir, "python", python_pkg_name)
egg_info_dir = os.path.join(project_dir, f"{python_pkg_name}.egg-info")
parser = ArgumentParser()
parser.add_argument("--build-type", default="Release", choices=["Release", "Debug"], type=str, help="build type")
parser.add_argument("--clean-before-build", action="store_true", help="clean before build")
args, unknown = parser.parse_known_args()
sys.argv = [sys.argv[0]] + unknown

if os.path.exists(egg_info_dir):
    os.system(f"rm -rf {egg_info_dir}")
erl_dependencies = ["erl_common"]
temp_build_dir = os.path.join(project_dir, "build")
temp_install_dir = os.path.join(project_dir, "build", "temp_install")
n_proc = os.cpu_count()
os.makedirs(temp_install_dir, exist_ok=True)
for dependency in erl_dependencies:
    src_dir = os.path.join(project_dir, "..", dependency)
    assert os.path.exists(src_dir), f"Dependency {dependency} not found"
    build_dir = os.path.join(temp_build_dir, dependency)
    os.makedirs(build_dir, exist_ok=True)
    subprocess.check_call(
        ["cmake", src_dir, "-DCMAKE_BUILD_TYPE=" + args.build_type, "-DCMAKE_INSTALL_PREFIX=" + temp_install_dir],
        cwd=build_dir,
    )
    subprocess.check_call(
        ["cmake", "--build", ".", "--target", "install", "--", "-j", str(n_proc)],
        cwd=build_dir,
    )


class CMakeExtension(Extension):
    def __init__(self, name: str, source_dir: str = project_dir):
        super().__init__(name, sources=[])
        self.source_dir = os.path.abspath(source_dir)
        self.build_type = args.build_type


class CMakeBuild(build_ext):
    def run(self) -> None:
        try:
            subprocess.check_output(["cmake", "--version"])
        except OSError:
            raise RuntimeError(
                "CMake must be installed to build the following extensions: "
                + ", ".join(e.name for e in self.extensions)
            )

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext: CMakeExtension) -> None:
        original_full_path: str = self.get_ext_fullpath(ext.name)
        if os.path.exists(original_full_path):
            os.remove(original_full_path)

        # ext_dir equals to build/lib.linux-$(architecture)-cpython-${python_version}
        editable = os.path.dirname(original_full_path) == project_dir  # editable install
        if editable:
            ext_dir: str = src_python_dir
        else:
            ext_dir: str = os.path.abspath(os.path.dirname(original_full_path))
            ext_dir: str = os.path.join(ext_dir, self.distribution.get_name())
        old_ext_path = os.path.join(ext_dir, os.path.basename(original_full_path))
        if os.path.exists(old_ext_path):
            os.remove(old_ext_path)

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={ext_dir}",
            f"-DPython3_ROOT_DIR={os.path.dirname(os.path.dirname(sys.executable))}",
            f"-DCMAKE_BUILD_TYPE={ext.build_type}",
            f"-DCMAKE_PREFIX_PATH={temp_install_dir}",
            f"-DCMAKE_VERBOSE_MAKEFILE:BOOL=ON",
        ]
        os.makedirs(self.build_temp, exist_ok=True)
        subprocess.check_call(["cmake", ext.source_dir] + cmake_args, cwd=self.build_temp)
        subprocess.check_call(
            ["cmake", "--build", ".", "--target", pybind_module_name, "--", "-j", f"{n_proc}"],
            cwd=self.build_temp,
        )


if os.path.exists("requirements.txt"):
    with open("requirements.txt", "r") as f:
        requires = f.readlines()
else:
    requires = []
for i, require in enumerate(requires):
    if require.startswith("git"):
        left, pkg_name = require.split("=")
        pkg_name = pkg_name.strip()
        requires[i] = f"{pkg_name} @ {require.strip()}"

if os.path.exists("entry_points.txt"):
    with open("entry_points.txt", "r") as f:
        entry_points = f.readlines()
else:
    entry_points = []
for i, entry_point in enumerate(entry_points):
    entry_points[i] = entry_point.strip()

setup(
    name=python_pkg_name,
    version="0.1.0",
    author="Zhirui Dai",
    author_email="zhdai@ucsd.edu",
    ext_modules=[CMakeExtension(pybind_module_name)],
    cmdclass={"build_ext": CMakeBuild},
    install_requires=requires,
    packages=find_packages("python"),
    package_dir={python_pkg_name: f"python/{python_pkg_name}"},
    include_package_data=True,
    entry_points={"console_scripts": entry_points},
)
