# Gocator Interface

Python ctypes wrapper for LMI Gocator SDK. Acquires 3D point clouds from Gocator sensors.

## Installation

1. Download Gocator SDK from https://www.lmi3d.com/
2. Install to standard location (auto-detected)
3. Install Python dependencies: `pip install numpy`

## Usage

### Basic Acquisition

```python
with GocatorScanner("192.168.1.10") as scanner:
    scanner.start()
    point_cloud = scanner.get_point_cloud()
    scanner.stop()
```

### Configure Parameters

```python
with GocatorScanner("192.168.1.10") as scanner:
    scanner.set_exposure(100.0)          # microseconds
    scanner.set_spacing_interval(0.5)    # mm
    point_cloud = scanner.scan_and_get()
```

### Multiple Scans

```python
with GocatorScanner("192.168.1.10") as scanner:
    scanner.start()
    for i in range(10):
        point_cloud = scanner.get_point_cloud()
    scanner.stop()
```

## API

**Connection:**
- `connect() -> bool`
- `disconnect()`
- `get_sensor_info() -> dict`

**Acquisition:**
- `start() -> bool`
- `stop() -> bool`
- `get_point_cloud(timeout_us=20000000) -> NDArray`
- `scan_and_get() -> NDArray`

**Configuration:**
- `get_exposure() -> float`
- `set_exposure(exposure_us: float) -> bool`
- `get_spacing_interval() -> float`
- `set_spacing_interval(interval_mm: float) -> bool`

## Point Cloud Format

NumPy array with shape (N, 3):
- Column 0: X (meters)
- Column 1: Y (meters)
- Column 2: Z (meters)

Invalid points automatically filtered.

## Examples

See [example.py](example.py) for complete examples including parameter configuration and saving point clouds.

## Details

- Full ctypes bindings for 30+ GoSDK functions
- Context manager support
