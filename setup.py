from setuptools import setup, find_packages

setup(
    name='panda_polymetis',
    version='0.1.0',
    description='Package for accessings sensors and controlling the panda with polymetis.',
    author='Trevor Ablett',
    packages=find_packages(),
    install_requires=[
        'polymetis',
        'transform_utils',
        'pymodbus==2.5.3'  # should have been installed by polymetis
    ],
    include_package_data=True
)