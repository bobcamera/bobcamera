import os
from glob import glob
from setuptools import setup

package_name = 'bob_ptz'

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
            # 'onvif_absolute_move = bob_ptz.onvif_absolute_move_ros:main',
            'raster_ptz_client = bob_ptz.raster_ptz_client:main',
            'raster_image_acquisition_service = bob_ptz.raster_image_acquisition_service:main',
        ],
    },
)
