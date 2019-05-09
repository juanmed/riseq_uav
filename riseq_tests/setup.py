from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['riseq_tests'],
    #scripts=['scripts/toy_trajectory_generator/df_flat.py'],
    package_dir={'': 'src'}
)

setup(**d)