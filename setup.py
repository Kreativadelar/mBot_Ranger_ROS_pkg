## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(packages=['mBot_Ranger_ROS_pkg'],
                                      package_dir={'': 'nodes'}, )

setup(**setup_args)
