
# -*- coding: utf-8 -*-

# Learn more: https://github.com/sequgen/accelsim

from setuptools import setup, find_packages

with open('README.md') as f:
    readme = f.read()

#with open('LICENSE') as f:
#    license = f.read()

setup(
    name='accelsim',
    version='0.1.0',
    description="Simulate accelerometer's readings",
    long_description=readme,
    author='Pablo Rodríguez-Sánchez',
    author_email='pablo.rodriguez.sanchez@gmail.com',
    url='https://github.com/sequgen/accelsim',
    # license=license,
    install_requires=[
        'numpy',
    ],
    packages=find_packages(
        exclude=('tests', 'docs', 'vignettes', 'scripts', 'drafts')
        )
)
