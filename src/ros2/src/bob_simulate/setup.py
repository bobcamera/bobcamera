from setuptools import setup

package_name = 'bob_simulate'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='michael.groenewald@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'object_simulator = bob_simulate.object_simulator_node:main',
            'simulated_video_provider = bob_simulate.simulated_video_provider_node:main',
            'simulation_overlay_provider = bob_simulate.simulation_overlay_provider_node:main',
        ],
    },
)