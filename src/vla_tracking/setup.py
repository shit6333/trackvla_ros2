from glob import glob

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
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py'),
        ),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
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
        'console_scripts': [
            'vla_inference_node = vla_tracking.inference_node:main',
            'trajectory_executor_node = '
            'vla_tracking.trajectory_executor_node:main',
        ],
        # Backends advertise themselves here so that this package never
        # imports a model adapter. See vla_tracking/backend_loader.py.
        'vla_tracking.backends': [
            'fake = vla_tracking.fake_backend:FakeBackend',
        ],
    },
)
