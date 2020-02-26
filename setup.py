#!/usr/bin/env python3

from setuptools import setup

# Installation:
# pip install -e .

setup(name='pprz_data',
      version='0.1',
      description='Data class from Paparazzi System',
      url='http://github.com/mrtbrnz/pprz_data',
      author='Murat Bronz',
      author_email='muratbronz@gmail.com',
      license='GPL v2',
      install_requires=["numpy >= 1.13.3", "scipy", "pandas"],
      packages=['pprz_data'],
      zip_safe=False)