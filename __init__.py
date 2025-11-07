"""
Gocator Interface - Simple library to get point clouds from Gocator sensors

Separate from pin_scanner - can be used independently.

SDK-based implementation using official Gocator SDK via ctypes.
Requires Gocator SDK installation from https://www.lmi3d.com/
"""

from .gocator import GocatorScanner

__version__ = '1.0.0'

__all__ = ['GocatorScanner']
