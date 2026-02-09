from glob import glob
from setuptools import find_packages, setup

package_name = "spot_navigation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/map", glob("map/*.pcd") + glob("map/*.vgh")),
        ("share/" + package_name + "/scripts", glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="spot",
    maintainer_email="spot@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            'publish_goal_markers = spot_navigation.publish_goal_markers:main',
            'goal1 = scripts.goal1:main',
            'goal2 = scripts.goal2:main',
            'goal3 = scripts.goal3:main',
            'entrance = scripts.entrance:main',
            'mission_tracker = scripts.mission_tracker:main',
        ],
    },
)
