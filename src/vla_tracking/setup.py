from setuptools import find_packages, setup

package_name = 'vla_tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='y_ricky',
    maintainer_email='shit6333@users.noreply.github.com',
    description=(
        'Model-independent ROS 2 runtime for vision-language tracking.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        # The console scripts are declared in Phase 3 and Phase 4, when
        # inference_node and trajectory_executor_node exist. Declaring them
        # against missing modules would install launchable executables that
        # fail at import time.
        'console_scripts': [],
    },
)
