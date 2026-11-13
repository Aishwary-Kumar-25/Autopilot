import os
from glob import glob

from setuptools import setup

package_name = "autopilot_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "scenarios"), glob("scenarios/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="E-Club Autopilot Team",
    maintainer_email="eclub-autopilot@example.com",
    description="ROS 2 simulator, fault injection, test scenarios and datasets for the E-Club autopilot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge = autopilot_sim.bridge_node:main",
            "run_scenarios = autopilot_sim.run_scenarios:main",
            "json_physics_quad = autopilot_sim.json_physics_quad:main",
        ],
    },
)
