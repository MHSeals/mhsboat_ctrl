from setuptools import find_packages, setup

package_name = "mhsboat_ctrl"

setup(
    name=package_name,
    version="0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Wavefire5201",
    maintainer_email="enoch.zhu154@gmail.com",
    description="Multitool package for Roboboat",
    license="GNU GPLv3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "boat_arm = mhsboat_ctrl.boat_arm:main",
            "boat_mode = mhsboat_ctrl.boat_mode:main",
            "boat_controller = mhsboat_ctrl.boat_mavros_controller:main",
            "boat_taskone = mhsboat_ctrl.boat_taskone:main",
            "boat_camera = mhsboat_ctrl.boat_camera:main",
            "boat_taskone_waypoint = mhsboat_ctrl.boat_taskone_waypoint:main",
            "buoy_recognition = mhsboat_ctrl.buoy_recognition:main",
            "locate_buoys = mhsboat_ctrl.locate_buoys:main",
            "center_of_clusters = mhsboat_ctrl.center_of_clusters:main",
            "average_buoy_location = mhsboat_ctrl.average_buoy_location:main",
            "jon_tasktwo = mhsboat_ctrl.tasktwo:main",
            "jon_taskone = mhsboat_ctrl.taskone:main",
        ],
    },
)
