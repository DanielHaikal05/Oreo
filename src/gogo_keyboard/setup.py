from setuptools import find_packages, setup

package_name = "gogo_keyboard"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],
    package_data={
        'gogo_keyboard': [
            'icons/*.png',
        ],
    },
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel",
    maintainer_email="daniel@todo.todo",
    description="Keyboard input publisher for ROS 2.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ros_node = gogo_keyboard.ros_node:main",
        ],
    },
)
