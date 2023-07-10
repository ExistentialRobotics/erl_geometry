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
parser = ArgumentParser()
parser.add_argument("--build-type", default="Release", choices=["Release", "Debug"], type=str, help="build type")
parser.add_argument("--clean-before-build", action="store_true", help="clean before build")
args, unknown = parser.parse_known_args()
sys.argv = [sys.argv[0]] + unknown


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
        ext_dir: str = os.path.abspath(os.path.dirname(original_full_path))  # equal to project_dir
        ext_dir: str = os.path.join(ext_dir, "python", self.distribution.get_name())
        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={ext_dir}",
            f"-DPython3_ROOT_DIR={os.path.dirname(os.path.dirname(sys.executable))}",
            f"-DCMAKE_BUILD_TYPE={ext.build_type}",
        ]
        os.makedirs(self.build_temp, exist_ok=True)
        subprocess.check_call(["cmake", ext.source_dir] + cmake_args, cwd=self.build_temp)
        subprocess.check_call(
            ["cmake", "--build", ".", "--target", pybind_module_name, "--", "-j", f"{os.cpu_count()}"],
            cwd=self.build_temp,
        )


if os.path.exists("requirements.txt"):
    with open("requirements.txt", "r") as f:
        requires = f.readlines()
else:
    requires = []

for i in range(len(requires)):
    require = requires[i]
    if require.startswith("git"):
        left, pkg_name = require.split("=")
        pkg_name = pkg_name.strip()
        requires[i] = f"{pkg_name} @ {require.strip()}"

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
)
