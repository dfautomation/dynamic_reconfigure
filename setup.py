from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['dynamic_reconfigure'],
    package_dir={'': 'src'},
    requires=['roslib', 'rospkg', 'rospy', 'rosservice']
)

setup(**d)
