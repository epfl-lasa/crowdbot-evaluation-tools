from setuptools import setup
import os, sys

sys.path.append(os.path.dirname(__file__))

setup(
    name='qolo',
    version=0.2,
    description='Package of data processing, evaluation, and visulization for Crowdbot Dataset',
    author='Yujie He, Diego F. Paez G.',
    author_email='yujie.he@epfl.ch',
    packages=[
        'qolo',
        'qolo.ros',
        'qolo.utils',
        'qolo.metrics',
        'qolo.external',
        'qolo.external.trajectory_smoothing',
        'qolo.test',
    ],
)
