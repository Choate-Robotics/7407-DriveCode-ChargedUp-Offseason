'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

from pyfrc.tests import *
import runpy

def test_path():
    assert runpy.run_path('../robot.py') is not None

