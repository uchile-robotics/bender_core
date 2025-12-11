from setuptools import setup
import os
from glob import glob

package_name = 'bender_emotion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # opcional si tienes launch/
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuel',
    maintainer_email='tu_email@example.com',
    description='Paquete para control emocional del robot Bender',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_server = bender_emotion.action_server:main',
            'speak_action_server = bender_emotion.speak_action_server:main',
            'emo_serial_propio = bender_emotion.emo_serial_propio:main',
            'emo_serial = bender_emotion.emo_serial:main',
            'emotion_publisher_node = bender_emotion.emotion_publisher_node:main',
            'neck = bender_emotion.neck:main',
            'prueba = bender_emotion.prueba:main',
        ],
    },
)


