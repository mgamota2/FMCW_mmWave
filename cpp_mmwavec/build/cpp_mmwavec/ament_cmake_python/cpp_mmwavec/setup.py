from setuptools import find_packages
from setuptools import setup

setup(
    name='cpp_mmwavec',
    version='0.0.0',
    packages=find_packages(
        include=('cpp_mmwavec', 'cpp_mmwavec.*')),
)
