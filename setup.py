from glob import glob
import os

from setuptools import find_packages, setup


package_name = "openmanipulatorlab"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models"), glob("models/*")),
        (os.path.join("share", package_name), ["README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenManipulator Lab",
    maintainer_email="maintainers@example.com",
    description="ROS 2 teaching package for the OpenManipulator pick-and-sort challenge.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "run_challenge = openmanipulatorlab.run_challenge:main",
        ],
    },
)
