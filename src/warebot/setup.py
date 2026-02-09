from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'warebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'description'), glob('description/*.xacro')),
        (os.path.join('share', package_name, 'description', 'a200'), glob('description/a200/*.xacro')),
        (os.path.join('share', package_name, 'description', 'a200', 'attachments'), glob('description/a200/attachments/*.xacro')),
        (os.path.join('share', package_name, 'description', 'a200', 'drivetrain'), glob('description/a200/drivetrain/*.xacro')),
        (os.path.join('share', package_name, 'description', 'a200', 'drivetrain', 'wheels'), glob('description/a200/drivetrain/wheels/*.xacro')),
        (os.path.join('share', package_name, 'meshes', 'a200'), glob('meshes/a200/*.dae') + glob('meshes/a200/*.stl')),
        (os.path.join('share', package_name, 'meshes', 'a200', 'attachments'), glob('meshes/a200/attachments/*.dae') + glob('meshes/a200/attachments/*.stl')),
        (os.path.join('share', package_name, 'meshes', 'a200', 'drivetrain'), glob('meshes/a200/drivetrain/*.dae') + glob('meshes/a200/drivetrain/*.stl')),
        (os.path.join('share', package_name, 'meshes', 'a200', 'wheels'), glob('meshes/a200/wheels/*.dae') + glob('meshes/a200/wheels/*.stl')),
        (os.path.join('share', package_name, 'world'), glob('world/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaneki',
    maintainer_email='manavsikka5@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
