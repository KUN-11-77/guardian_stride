from glob import glob
import os

from setuptools import setup

package_name = 'guardian_voice'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Guardian Stride Team',
    maintainer_email='dev@guardian-stride.com',
    description='Voice interaction: BOYA mic ASR + RealSense obstacle query + TTS',
    license='MIT',
    entry_points={
        'console_scripts': [
            'voice_asr_node = guardian_voice.voice_asr_node:main',
            'depth_obstacle_node = guardian_voice.depth_obstacle_node:main',
            'voice_assistant_node = guardian_voice.voice_assistant_node:main',
            'tts_node = guardian_voice.tts_node:main',
        ],
    },
)
