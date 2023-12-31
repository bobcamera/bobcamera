from setuptools import setup

package_name = 'bob_observer'

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
            'cloud_estimator = bob_observer.cloud_estimator_node:main',
            'day_night_classifier = bob_observer.day_night_classifier_node:main',            
        ],
    },
)
