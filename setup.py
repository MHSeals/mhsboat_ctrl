import os
from glob import glob
from setuptools import find_packages, setup

package_name = "mhsboat_ctrl"

setup(
    name=package_name,
    version="0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', '*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Wavefire5201, Alec Jensen",
    description="Control Package for the MHSeals Boat",
    license="GNU GPLv3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mhsboat_ctrl = mhsboat_ctrl.mhsboat_ctrl:main",
            "sensors = mhsboat_ctrl.sensors:main",
            "simulated_map = mhsboat_ctrl.simulated_map:main",
            "display_map = mhsboat_ctrl.display_map:main",
            "buoy_recognition = mhsboat_ctrl.buoy_recognition:main",
            "center_of_clusters = mhsboat_ctrl.center_of_clusters:main",
            "bag_recorder = mhsboat_ctrl.bag_recorder:main",
            "vision = mhsboat_ctrl.vision:main",
            "vision_mhsboat_ctrl = mhsboat_ctrl.vision_mhsboat_ctrl:main"
        ],
    },
)
