from setuptools import find_packages, setup


PACKAGE_NAME = "mmhand_teleop"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MMHand Team",
    maintainer_email="noreply@example.com",
    description="Headless ROS 2 adapters for the existing MMHand teleoperation preview.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "raw_ik_preview_file_bridge = "
            "mmhand_teleop.nodes.raw_ik_preview_file_bridge:main",
        ],
    },
)
