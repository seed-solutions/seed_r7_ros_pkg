## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
## See https://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# ferch values from package.xml
setup_args = generate_distutils_setup(
    packages=[''],
    package_dir={'': 'scripts'}
)

setup(**setup_args)