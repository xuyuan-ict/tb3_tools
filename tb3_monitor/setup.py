#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['tb3_monitor'],
    package_dir={'': 'src'},
    scripts=['scripts/tb3-monitor']
)

setup(**setup_args)
