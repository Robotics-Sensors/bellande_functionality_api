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

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    scripts=['src/bellande_2d_computer_vision_prediction.py', 'src/bellande_2d_computer_vision_face_detection.py', 'src/bellande_2d_computer_vision_object_detection.py', 'src/bellande_2d_computer_vision_instance_segmentation.py', 'src/bellande_2d_computer_vision_semantic_segmentation.py'],
    packages=['web_api_bellande_2d_computer_vision'],
    package_dir={'': 'src'},
)

setup(**setup_args)
