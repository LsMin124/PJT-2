import os
from glob import glob

from setuptools import find_packages, setup

package_name = "amr_agent"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "paho-mqtt"],
    zip_safe=True,
    maintainer="lsmin124",
    maintainer_email="leesmin124@gmail.com",
    description="AMR agent (T4): VDA 5050 bridge + order executor + grid primitive controller",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vda5050_bridge = amr_agent.nodes.bridge_node:main",
            "order_executor = amr_agent.nodes.executor_node:main",
            "primitive_controller = amr_agent.nodes.primitive_node:main",
            "fake_robot = amr_agent.nodes.fake_robot_node:main",
            "fake_fms = amr_agent.tools.fake_fms:main",
        ],
    },
)
