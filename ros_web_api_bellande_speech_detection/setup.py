from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    scripts=['src/bellande_speech_detection.py'],
    packages=['ros_web_api_bellande_speech_detection'],
    package_dir={'': 'src'},
)

setup(**setup_args)
