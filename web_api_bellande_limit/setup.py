# Copyright (C) 2024 Bellande Robotics Sensors Research Innovation Center, Ronaldson Bellande
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os
import sys

# Determine ROS version
ros_version = os.getenv("ROS_VERSION", "1")  # Default to ROS 1 if not set

if ros_version == "1":
    # ROS 1: Use catkin_pkg and distutils
    from catkin_pkg.python_setup import generate_distutils_setup
    from distutils.core import setup

    setup_args = generate_distutils_setup(
        packages=[],
        package_dir={"": "src"},
        py_modules=["bellande_limit_api"],
    )
    setup(**setup_args)

elif ros_version == "2":
    # ROS 2: Use setuptools
    from setuptools import setup

    setup(
        name="web_api_bellande_limit",
        version="0.1.0",
        packages=[],
        package_dir={"": "src"},
        py_modules=["bellande_limit_api"],
        data_files=[
            (
                os.path.join("share", "web_api_bellande_limit", "config", "json"),
                ["config/json/http_configs.json"],
            ),
        ],
        install_requires=[
            "setuptools",
        ],
        zip_safe=True,
        author="Ronaldson Bellande",
        author_email="ronaldsonbellande@gmail.com",
        description="Bellande ROS/ROS2 package with a JSON config file for making HTTP requests for Bellande Limit",
        license="GPL-3.0",
    )

else:
    sys.stderr.write(f"Unsupported ROS_VERSION: {ros_version}\n")
    sys.exit(1)
