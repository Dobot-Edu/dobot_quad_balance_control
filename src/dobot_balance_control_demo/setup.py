from glob import glob

from setuptools import setup

package_name = "dobot_balance_control_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyyaml", "numpy"],
    zip_safe=True,
    maintainer="Dobot Demo Team",
    maintainer_email="support@example.com",
    description="IMU attitude balance compensation demo for Dobot quadruped.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "imu_reader_node = dobot_balance_control_demo.imu_reader_node:main",
            "balance_controller_node = dobot_balance_control_demo.balance_controller_node:main",
            "demo_monitor_node = dobot_balance_control_demo.demo_monitor_node:main",
            "attitude_plot_recorder_node = dobot_balance_control_demo.attitude_plot_recorder_node:main",
        ],
    },
)
