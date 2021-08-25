from glob import glob
import os
from setuptools import setup

package_name = 'yasmin_viewer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'yasmin_viewer_web_client'),
         glob('yasmin_viewer_web_client/build/*.*')),
        (os.path.join('share', package_name, 'yasmin_viewer_web_client/static/css'),
         glob('yasmin_viewer_web_client/build/static/css/*.*')),
        (os.path.join('share', package_name, 'yasmin_viewer_web_client/static/js'),
         glob('yasmin_viewer_web_client/build/static/js/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miguel',
    maintainer_email='mgons@unileon.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yasmin_viewer_node = yasmin_viewer.yasmin_viewer_node:main',
        ],
    },
)
