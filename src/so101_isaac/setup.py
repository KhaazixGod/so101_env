import os
from glob import glob

from setuptools import find_packages, setup

package_name = "so101_isaac"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "isaac"), glob("isaac/*.py")),
        # conveyor_ref/ is a subdirectory -- glob("isaac/*.py") above doesn't
        # recurse into it, so it needs its own data_files entry (setuptools
        # data_files has no recursive-glob option; each destination dir needs
        # its own tuple). Without this, so101_table_scene.py's
        # --conveyor-engine warp builds fine but conveyor_physics.py's
        # `import cb_kernels` (etc.) fails at runtime in an installed
        # (non-symlink) package, since conveyor_ref/ never reaches install/.
        (
            os.path.join("share", package_name, "isaac", "conveyor_ref"),
            glob("isaac/conveyor_ref/*.py") + glob("isaac/conveyor_ref/*.md"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hung",
    maintainer_email="hung@todo.todo",
    description="Isaac Sim scene for the SO-101 arm on a table, bridged to ROS 2.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_commander = so101_isaac.joint_commander:main",
        ],
    },
)
