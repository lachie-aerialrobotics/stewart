import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'stewart'


def model_data_files():
    # install models/ (sdf, config, meshes) to share/stewart/models, keeping the folder layout
    files = []
    for root, _, names in os.walk('models'):
        if names:
            files.append((os.path.join('share', package_name, root),
                          [os.path.join(root, n) for n in names]))
    return files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ] + model_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lachie',
    maintainer_email='lachie_orr@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematics_node = stewart.kinematics_node:main',
            'dynamixel_servo_node = stewart.servo_controller:main',
            'msg_converter_node = stewart.msg_converter:main',
        ],
    },
)
