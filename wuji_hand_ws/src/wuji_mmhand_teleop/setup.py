from setuptools import find_packages, setup

PACKAGE_NAME = "wuji_mmhand_teleop"

setup(
    name=PACKAGE_NAME,
    version="0.2.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "wuji-sdk"],
    include_package_data=True,
    package_data={
        PACKAGE_NAME: [
            "assets/hku_hand_v2_urdf/urdf/*.urdf",
            "assets/hku_hand_v2_urdf/meshes/*",
            "assets/hku_hand_v2_urdf/urdf_contract.json",
            "_vendor/wuji_sdk/*",
            "_vendor/wuji_sdk/**/*",
            "_vendor/wuji_sdk.libs/*",
            "_vendor/wuji_sdk-*.dist-info/*",
        ]
    },
    zip_safe=True,
    maintainer="MMHand Team",
    maintainer_email="noreply@example.com",
    description="Standalone Wuji Glove skeleton to MMHand URDF IK ROS 2 node.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "wuji_raw_ik_target_sender = wuji_mmhand_teleop.nodes.sender:main",
            "wuji_raw_ik_teleop_ui = wuji_mmhand_teleop.ui.teleop_ui:main",
        ],
    },
)
