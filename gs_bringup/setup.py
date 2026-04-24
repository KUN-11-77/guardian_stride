from setuptools import setup

setup(
    name='gs_bringup',
    version='1.0.0',
    packages=['gs_bringup'],
    data_files=[
        ('share/gs_bringup', ['package.xml']),
        ('share/gs_bringup/launch', [
            'launch/guardian_stride.launch.py',
            'launch/perception_only.launch.py',
            'launch/hardware_only.launch.py',
        ]),
        ('share/gs_bringup/config', ['config/system_params.yaml']),
    ],
)
