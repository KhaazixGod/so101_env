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
