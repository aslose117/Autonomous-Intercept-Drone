import os
from glob import glob
from setuptools import setup

package_name = "uav_rl_guidance"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models"),
         glob("models/*.pt") + glob("models/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="uav_dev",
    maintainer_email="developer@uav.local",
    description="Learned guidance law for vision-based UAV interception",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "uav_rl_guidance = uav_rl_guidance.rl_guidance_node:main",
        ],
    },
)
