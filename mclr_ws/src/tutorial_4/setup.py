from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['tutorial_4'],
    package_dir={'': 'src'}
)
setup(**d)