from setuptools import find_packages
from setuptools import setup

setup(
    name='boat_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('boat_interfaces', 'boat_interfaces.*')),
)
