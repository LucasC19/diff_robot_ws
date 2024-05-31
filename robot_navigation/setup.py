from setuptools import find_packages, setup
import os
from glob import glob

package_name = "robot_navigation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (
            os.path.join("share", package_name, "models/turtlebot_waffle_gps"),
            glob("models/turtlebot_waffle_gps/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros_humble",
    maintainer_email="lcostanzo97@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "logged_waypoint_follower_ = nav2_gps_moray.logged_waypoint_follower_:main",
            "nav_through_poses = puma2_navigation.nav_through_poses:main",
        ],
    },
)
