from setuptools import setup
import os
from glob import glob
from setuptools import setup

package_name = 'holons'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carter',
    maintainer_email='carter@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'product = holons.product:main',
        	'R_LinearConveyor = holons.R_LinearConveyor:main',
        	'R_KR10 = holons.R_KR10:main',
        	'R_KR16 = holons.R_KR16:main',
        	'R_CircConveyor = holons.R_CircConveyor:main',
        	'R_CNC = holons.R_CNC:main',
        	'R_BasketA = holons.R_BasketA:main',
        	'R_BasketB = holons.R_BasketB:main',
        ],
    },
    
)
