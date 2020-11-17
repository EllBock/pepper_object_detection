from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['pepper_object_detection'],
    package_dir={'':'src'},
)

setup(**setup_args)