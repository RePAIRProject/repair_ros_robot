#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['repair_interface'],
   package_dir={'repair_interface': 'src/repair_interface'}
)

setup(**d)