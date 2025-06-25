from setuptools import find_packages, setup

package_name = "gripper_sub"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pi",
    maintainer_email="pi@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # <CLI名稱> = <Python模組路徑>:<函數名稱>
            "listener = gripper_sub.gripper_sub_main:main",
            "socket_test = gripper_sub.only_socket_receive_test:main",
            "ros_socket_test = gripper_sub.ros_socket_receive:main",
        ],
    },
)
