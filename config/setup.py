#!/usr/bin/env python

from setuptools import setup, find_packages, extension


setup(
    name = "DEVINE_config_package",    
    version = "0.0.1",
    license = "BSD",
    keywords = "DEVINE",
    author = "DEVINE team",
    packages = find_packages('src'),
    package_dir = {'':'src'},
    py_modules = ['DEVINEParameters'],
    include_package_data = True,
)
