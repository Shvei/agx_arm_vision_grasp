from glob import glob
import os

from setuptools import find_packages, setup


package_name = "agx_arm_vision_grasp"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="Eye-in-hand RealSense tissue packet detection and grasping.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tissue_detector = agx_arm_vision_grasp.tissue_detector_node:main",
            "tissue_pick = agx_arm_vision_grasp.tissue_pick_node:main",
        ],
    },
)
