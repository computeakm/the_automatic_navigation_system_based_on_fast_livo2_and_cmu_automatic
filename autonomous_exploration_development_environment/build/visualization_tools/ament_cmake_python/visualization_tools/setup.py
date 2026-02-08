from setuptools import find_packages
from setuptools import setup

setup(
    name='visualization_tools',
    version='0.0.1',
    packages=find_packages(
        include=('visualization_tools', 'visualization_tools.*')),
)
