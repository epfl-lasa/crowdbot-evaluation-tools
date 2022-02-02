from setuptools import setup
import os, sys

sys.path.append(os.path.dirname(__file__))

setup(
    name='qolo',
    version=0.1,
    packages=[
        'qolo',
        'qolo.utils',
        'qolo.metrics',
        'qolo.external',
    ],
)
