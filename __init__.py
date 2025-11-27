"""
Python Gocator - Python wrapper for LMI Technologies Gocator SDK

SDK-based implementation using official Gocator SDK via ctypes.
Requires Gocator SDK installation from https://www.lmi3d.com/
"""

from .gocator import GocatorScanner

__version__ = '1.0.0'

__all__ = ['GocatorScanner']
