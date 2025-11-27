# Gocator Interface

Python wrapper for LMI Technologies Gocator SDK. Acquire 3D point clouds from Gocator laser scanners. Tested with snapshot sensor Gocator 3506.

## Installation

1. Install [Gocator SDK](https://www.lmi3d.com/) to standard location
2. Install dependencies (requires Python 3.9+):

   **Simple installation:**
   ```bash
   pip install numpy
   ```

   **With virtual environment using uv.lock file (recommended):**
   ```bash
   uv sync
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   ```

## Quick Start

```python
from gocator import GocatorScanner

# Basic acquisition with software trigger
with GocatorScanner("192.168.1.10") as scanner:
    scanner.start()
    scanner.trigger()
    point_cloud = scanner.get_point_cloud()  # Returns Nx3 NumPy array (X, Y, Z in meters)
    scanner.stop()
```

## Features

- Automatic SDK detection (Linux, Windows, macOS)
- Context manager support
- Software triggering
- Configurable exposure and spacing
- NumPy array output

## API Reference

**Connection:**
- `connect() -> bool` - Connect to sensor
- `disconnect()` - Disconnect from sensor
- `get_sensor_info() -> dict` - Get model, serial number, IP

**Acquisition:**
- `start() -> bool` - Start acquisition system
- `stop() -> bool` - Stop acquisition system
- `trigger() -> bool` - Send software trigger
- `get_point_cloud(timeout_us=20000000) -> NDArray` - Receive point cloud data
- `scan_and_get() -> NDArray` - Convenience method (start → trigger → get → stop)

**Configuration:**
- `get_exposure() -> float` - Get exposure time (microseconds)
- `set_exposure(exposure_us: float) -> bool` - Set exposure time
- `get_spacing_interval() -> float` - Get spacing interval (mm)
- `set_spacing_interval(interval_mm: float) -> bool` - Set spacing interval
- `get_scan_mode() -> tuple[int, str]` - Get current scan mode

## Examples

See [example.py](example.py) for complete examples including:
- Context manager usage
- Parameter configuration
- Multiple scans
- Saving point clouds (PLY, CSV, NumPy)

## SDK Installation Paths

The library auto-detects SDK from these locations:
- **Linux**: `/opt/lmi/gocator/lib/libGoSdk.so`
- **Windows**: `C:\Program Files\LMI Technologies\Gocator\lib\GoSdk.dll`
- **macOS**: `/usr/local/lib/libGoSdk.dylib`

Or specify manually:
```python
scanner = GocatorScanner("192.168.1.10", sdk_path="/path/to/libGoSdk.so")
```

## Requirements

- Python 3.9+
- NumPy 1.20+
- Gocator SDK (from LMI Technologies)

## License

MIT License - Copyright (c) 2025 Maximilian Eder
