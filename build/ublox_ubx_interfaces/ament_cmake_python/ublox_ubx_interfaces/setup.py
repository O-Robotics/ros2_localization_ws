from setuptools import find_packages
from setuptools import setup

setup(
    name='ublox_ubx_interfaces',
    version='0.5.3',
    packages=find_packages(
        include=('ublox_ubx_interfaces', 'ublox_ubx_interfaces.*')),
)
