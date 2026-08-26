from setuptools import find_packages, setup

package_name = 'vla_tracking_omtrackvla'

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
    description='OmTrackVLA adapter for vla_tracking.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        # The adapter ships no executable of its own; it is loaded through
        # vla_tracking's backend loader.
        'console_scripts': [],
        'vla_tracking.backends': [
            'omtrackvla = '
            'vla_tracking_omtrackvla.omtrackvla_backend:OmTrackVLABackend',
        ],
    },
)
