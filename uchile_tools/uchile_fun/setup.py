from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['uchile_fun'],
    scripts=[''],
    # obs: this should point to my_script_1.py ... (not folders)
    #scripts=['src/bender_macros'],
    package_dir={'': 'src'}
)

setup(**d)
