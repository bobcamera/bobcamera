import os
from glob import glob
from setuptools import setup

package_name = 'bob_monitor'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            source_file_path = os.path.join(path, filename)
            # Destination path is relative to `share/<package_name>`
            destination_path = os.path.join('share', package_name, os.path.relpath(path, directory))
            paths.append((destination_path, [source_file_path]))
    return paths

wsdl_files = package_files('resource/wsdl')

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include wsdl files with their directory structure
    ] + wsdl_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='michael.groenewald@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'prometheus_metrics = bob_monitor.prometheus_node:main',
            'onvif_service = bob_monitor.onvif_service_node:main',
            'monitoring_status_aggregator = bob_monitor.monitoring_status_aggregator_node:main',
            'ptz_manager = bob_monitor.ptz_manager_node:main',
            'config_manager = bob_monitor.config_manager_node:main',
        ],
    },
)
