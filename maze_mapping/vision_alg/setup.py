#!/usr/bin/env python3

from os.path import dirname, abspath, basename
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages = ['vision_alg'],
    package_dir = {'': 'sift.py'}
)

setup(**setup_args)