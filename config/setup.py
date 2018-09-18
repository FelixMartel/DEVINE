#!/usr/bin/env python

from setuptools import setup, find_packages

setup(
    name = "config",    
    version = "0.0.1",
    license = "BSD",
    keywords = "example documentation tutorial",
    packages = find_packages(),
    package_data = {'parameters' : ['*.ini']},
    include_package_data = True,
)
