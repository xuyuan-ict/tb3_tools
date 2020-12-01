#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['tb3_safe_teleop'],
    package_dir={'': 'src'},
    scripts=['scripts/tb3-safe-teleop']
)

setup(**setup_args)
