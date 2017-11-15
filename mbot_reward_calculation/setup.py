#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['mbot_reward_calculation', 'mbot_reward_calculation_ros'],
 package_dir={'mbot_reward_calculation': 'common/src/mbot_reward_calculation', 'mbot_reward_calculation_ros': 'ros/src/mbot_reward_calculation_ros'}
)

setup(**d)
