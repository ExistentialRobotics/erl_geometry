import importlib
import os
import subprocess
import sys
import tomllib

from setuptools import Extension
from setuptools import find_packages
from setuptools import setup
from setuptools.command.build_ext import build_ext

# read project configuration from pyproject.toml
with open("pyproject.toml", "r") as f:
    config = tomllib.loads("".join(f.readlines()))
python_pkg_name = config["erl"]["python_pkg_name"]
pybind_module_name = config["erl"]["pybind_module_name"]
build_type = config["erl"]["build_type"]
erl_dependencies = config["erl"]["erl_dependencies"]

# detect if ROS1 is enabled
if os.environ.get("ROS_VERSION", "0") == "1":
    catkin = importlib.import_module("catkin_pkg.python_setup")
    setup_args = catkin.generate_distutils_setup(
        packages=find_packages("python"),
        package_dir={python_pkg_name: f"python/{python_pkg_name}"},
    )
    setup(**setup_args)
    exit(0)

# ROS not enabled, build the package from source
# check if cmake is installed
cmake_paths = [
    "/usr/bin/cmake",
    "/usr/local/bin/cmake",
]
cmake_path = None
for path in cmake_paths:
    if os.path.exists(path):
        cmake_path = path
assert cmake_path is not None, f"cmake is not found in {cmake_paths}"

# load configuration
build_type = os.environ.get("BUILD_TYPE", build_type)
available_build_types = ["Release", "Debug", "RelWithDebInfo"]
assert build_type in available_build_types, f"build type {build_type} is not in {available_build_types}"
print(f"Build type: {build_type}")
clean_before_build = os.environ.get("CLEAN_BEFORE_BUILD", "0") == "1"
n_proc = os.cpu_count()

# compute paths
project_dir = os.path.dirname(os.path.realpath(__file__))
src_python_dir = os.path.join(project_dir, "python", python_pkg_name)
egg_info_dir = os.path.join(project_dir, f"{python_pkg_name}.egg-info")
build_dir = os.path.join(project_dir, "build", build_type)
temp_install_dir = os.path.join(build_dir, "temp_install")
os.makedirs(temp_install_dir, exist_ok=True)

# clean up
if os.path.exists(egg_info_dir):
    os.system(f"rm -rf {egg_info_dir}")
if clean_before_build:
    os.system(f"rm -rf {build_dir}")

# install erl_dependencies
for dependency in erl_dependencies:
    src_dir = os.path.join(project_dir, "..", dependency)
    assert os.path.exists(src_dir), f"Dependency {dependency} not found"
    temp_build_dir = os.path.join(build_dir, dependency)
    if os.path.exists(temp_build_dir):
        continue  # assume that the dependency is already built
    os.makedirs(temp_build_dir, exist_ok=True)
    subprocess.check_call(
        [
            cmake_path,
            src_dir,
            "-DCMAKE_BUILD_TYPE=" + build_type,
            "-DCMAKE_INSTALL_PREFIX=" + temp_install_dir,
            f"-DCMAKE_VERBOSE_MAKEFILE:BOOL=ON",
        ],
        cwd=temp_build_dir,
    )
    subprocess.check_call(
        [cmake_path, "--build", ".", "--target", "install", "--", "-j", str(n_proc)],
        cwd=temp_build_dir,
    )


# build the package
class CMakeExtension(Extension):
    def __init__(self, name: str, source_dir: str = project_dir):
        super().__init__(name, sources=[])
        self.source_dir = os.path.abspath(source_dir)


class CMakeBuild(build_ext):
    def run(self) -> None:
        try:
            subprocess.check_output([cmake_path, "--version"])
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

        # ext_dir equals to {build_dir}/lib.linux-$(architecture)-cpython-${python_version}
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
            f"-DCMAKE_BUILD_TYPE={build_type}",
            f"-DCMAKE_PREFIX_PATH={temp_install_dir}",
            f"-DCMAKE_VERBOSE_MAKEFILE:BOOL=ON",
        ]
        build_temp = os.path.join(build_dir, ext.name)
        os.makedirs(build_temp, exist_ok=True)
        subprocess.check_call([cmake_path, ext.source_dir] + cmake_args, cwd=build_temp)
        subprocess.check_call(
            [cmake_path, "--build", ".", "--target", pybind_module_name, "--", "-j", f"{n_proc}"],
            cwd=build_temp,
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

setup(
    name=python_pkg_name,
    description=config["erl"]["description"],
    version=config["erl"]["version"],
    author=config["erl"]["author"],
    author_email=config["erl"]["author_email"],
    license=config["erl"]["license"],
    ext_modules=[CMakeExtension(pybind_module_name)],
    cmdclass={"build_ext": CMakeBuild},
    install_requires=requires,
    packages=find_packages("python"),
    package_dir={python_pkg_name: f"python/{python_pkg_name}"},
    include_package_data=True,
    entry_points=config["erl"]["entry_points"],
)
