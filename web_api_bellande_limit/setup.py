from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    scripts=['src/bellande_step_api_2d.py'],
    packages=['web_api_bellande_step'],
    package_dir={'': 'src'},
)

setup(**setup_args)
