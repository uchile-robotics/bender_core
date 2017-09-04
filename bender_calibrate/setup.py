from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['bender_calibrate'],
    scripts=['src/cameracalibrator.py', 'src/cameracheck.py', 'src/tarfile_calibration.py'],
    # obs: this should point to my_script_1.py ... (not folders)
    #scripts=['src/bender_macros'],
    package_dir={'': 'src'}
)

setup(**d)
