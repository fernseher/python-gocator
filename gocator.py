import ctypes
import numpy as np
from numpy.typing import NDArray
from typing import Optional
import os


# GoSDK Constants
class kStatus:
    """GoSDK status codes"""
    kOK = 0
    kERROR = -1
    kERROR_PARAMETER = -2
    kERROR_UNIMPLEMENTED = -3
    kERROR_MEMORY = -4
    kERROR_TIMEOUT = -5
    kERROR_INCOMPLETE = -6
    kERROR_STREAM = -7
    kERROR_CLOSED = -8
    kERROR_VERSION = -9
    kERROR_ABORT = -10
    kERROR_ALREADY_EXISTS = -11
    kERROR_NETWORK = -12
    kERROR_HEAP = -13
    kERROR_FORMAT = -14
    kERROR_READ_ONLY = -15
    kERROR_WRITE_ONLY = -16
    kERROR_BUSY = -17
    kERROR_CONFLICT = -18
    kERROR_OS = -19
    kERROR_DEVICE = -20
    kERROR_FULL = -21
    kERROR_IN_PROGRESS = -22


# GoSDK Role constants
GO_ROLE_MAIN = 0
GO_ROLE_BUDDY = 1

# GoSDK message types
GO_DATA_MESSAGE_TYPE_UNKNOWN = 0
GO_DATA_MESSAGE_TYPE_STAMP = 1
GO_DATA_MESSAGE_TYPE_HEALTH = 2
GO_DATA_MESSAGE_TYPE_VIDEO = 3
GO_DATA_MESSAGE_TYPE_RANGE = 4
GO_DATA_MESSAGE_TYPE_RANGE_INTENSITY = 5
GO_DATA_MESSAGE_TYPE_PROFILE = 6
GO_DATA_MESSAGE_TYPE_PROFILE_INTENSITY = 7
GO_DATA_MESSAGE_TYPE_RESAMPLED_PROFILE = 8
GO_DATA_MESSAGE_TYPE_SURFACE = 9
GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY = 10
GO_DATA_MESSAGE_TYPE_SECTION = 11
GO_DATA_MESSAGE_TYPE_SECTION_INTENSITY = 12
GO_DATA_MESSAGE_TYPE_MEASUREMENT = 13
GO_DATA_MESSAGE_TYPE_ALIGNMENT = 14
GO_DATA_MESSAGE_TYPE_EXPOSURE_CAL = 15
GO_DATA_MESSAGE_TYPE_EDGE_MATCH = 16
GO_DATA_MESSAGE_TYPE_TRACHEID = 17
GO_DATA_MESSAGE_TYPE_GENERIC = 18

# Receive timeout in microseconds
RECEIVE_TIMEOUT = 20000000  # 20 seconds

# kNULL pointer
kNULL = None

# kTRUE/kFALSE boolean values
kTRUE = 1
kFALSE = 0

# Invalid/special values
k16U_NULL = 65535  # 0xFFFF - invalid 16-bit unsigned value
k16S_MIN = -32768  # Invalid 16-bit signed minimum


# IP Address structure
class kIpAddress(ctypes.Structure):
    """kApi IP address structure"""
    _fields_ = [
        ("address", ctypes.c_uint32),  # IP address as 32-bit integer
    ]


class GocatorScanner:
    """
    Wrapper for Gocator C SDK using ctypes.

    Requires Gocator SDK installation from https://www.lmi3d.com/
    """

    def __init__(self, ip_address: str = "192.168.1.10", sdk_path: Optional[str] = None):
        """
        Initialize Gocator interface.

        Args:
            ip_address: Gocator IP address
            sdk_path: Path to SDK library (libGoSdk.so / GoSdk.dll)
                     If None, searches common install locations
        """
        self.ip_address = ip_address
        self.sdk = None
        self.system = None
        self.sensor = None
        self.connected = False

        # Load SDK
        if not self._load_sdk(sdk_path):
            raise RuntimeError(
                "Gocator SDK not found. Install from https://www.lmi3d.com/\n"
                f"Or specify sdk_path parameter."
            )

    def _load_sdk(self, sdk_path: Optional[str] = None) -> bool:
        """Load the Gocator SDK library."""
        if sdk_path:
            paths = [sdk_path]
        else:
            # Common SDK installation paths
            paths = [
                # Linux
                "/opt/lmi/gocator/lib/libGoSdk.so",
                "/usr/local/lib/libGoSdk.so",
                "/usr/lib/libGoSdk.so",
                # Windows
                "C:/Program Files/LMI Technologies/Gocator/lib/GoSdk.dll",
                "C:/Program Files (x86)/LMI Technologies/Gocator/lib/GoSdk.dll",
                # macOS
                "/usr/local/lib/libGoSdk.dylib",
                # Relative paths
                "./libGoSdk.so",
                "./GoSdk.dll",
            ]

        for path in paths:
            if os.path.exists(path):
                try:
                    self.sdk = ctypes.CDLL(path)
                    print(f"Loaded Gocator SDK from: {path}")
                    self._setup_functions()
                    return True
                except Exception as e:
                    print(f"Failed to load {path}: {e}")

        return False

    def _setup_functions(self):
        """
        Setup ctypes function signatures for SDK calls.
        Based on GoSDK C API documentation.
        """
        if not self.sdk:
            return

        # === Initialization Functions ===

        # kStatus GoSdk_Construct(kAlloc allocator, kObject* system)
        self.sdk.GoSdk_Construct.argtypes = [
            ctypes.POINTER(ctypes.c_void_p)     # system
        ]
        self.sdk.GoSdk_Construct.restype = ctypes.c_int

        # kStatus GoSystem_Construct(kObject* system, kAlloc allocator)
        self.sdk.GoSystem_Construct.argtypes = [
            ctypes.POINTER(ctypes.c_void_p),    # system
            ctypes.c_void_p                      # allocator (can be NULL)
        ]
        self.sdk.GoSystem_Construct.restype = ctypes.c_int

        # === IP Address Functions ===

        # kStatus kIpAddress_Parse(kIpAddress* address, const kChar* text)
        self.sdk.kIpAddress_Parse.argtypes = [
            ctypes.POINTER(kIpAddress),         # address
            ctypes.c_char_p                      # text
        ]
        self.sdk.kIpAddress_Parse.restype = ctypes.c_int

        # === Sensor Discovery ===

        # kStatus GoSystem_FindSensorByIpAddress(kObject system, const kIpAddress* address, kObject* sensor)
        self.sdk.GoSystem_FindSensorByIpAddress.argtypes = [
            ctypes.c_void_p,                    # system
            ctypes.POINTER(kIpAddress),         # address
            ctypes.POINTER(ctypes.c_void_p)     # sensor
        ]
        self.sdk.GoSystem_FindSensorByIpAddress.restype = ctypes.c_int

        # === Connection Functions ===

        # kStatus GoSensor_Connect(kObject sensor)
        self.sdk.GoSensor_Connect.argtypes = [ctypes.c_void_p]
        self.sdk.GoSensor_Connect.restype = ctypes.c_int

        # kStatus GoSensor_Disconnect(kObject sensor)
        self.sdk.GoSensor_Disconnect.argtypes = [ctypes.c_void_p]
        self.sdk.GoSensor_Disconnect.restype = ctypes.c_int

        # === System Configuration ===

        # kStatus GoSystem_EnableData(kObject system, kBool enable)
        self.sdk.GoSystem_EnableData.argtypes = [
            ctypes.c_void_p,                    # system
            ctypes.c_int                         # enable (kTRUE/kFALSE)
        ]
        self.sdk.GoSystem_EnableData.restype = ctypes.c_int

        # kObject GoSensor_Setup(kObject sensor)
        self.sdk.GoSensor_Setup.argtypes = [ctypes.c_void_p]
        self.sdk.GoSensor_Setup.restype = ctypes.c_void_p

        # === Sensor Information ===

        # kStatus GoSensor_Model(kObject sensor, kChar* model, kSize capacity)
        self.sdk.GoSensor_Model.argtypes = [
            ctypes.c_void_p,                    # sensor
            ctypes.c_char_p,                    # model buffer
            ctypes.c_size_t                     # capacity
        ]
        self.sdk.GoSensor_Model.restype = ctypes.c_int

        # k32u GoSensor_Id(kObject sensor)
        self.sdk.GoSensor_Id.argtypes = [ctypes.c_void_p]
        self.sdk.GoSensor_Id.restype = ctypes.c_uint32

        # === Parameter Configuration ===

        # k64f GoSetup_Exposure(kObject setup, GoRole role)
        self.sdk.GoSetup_Exposure.argtypes = [
            ctypes.c_void_p,                    # setup
            ctypes.c_int                         # role
        ]
        self.sdk.GoSetup_Exposure.restype = ctypes.c_double

        # kStatus GoSetup_SetExposure(kObject setup, GoRole role, k64f value)
        self.sdk.GoSetup_SetExposure.argtypes = [
            ctypes.c_void_p,                    # setup
            ctypes.c_int,                        # role
            ctypes.c_double                      # value
        ]
        self.sdk.GoSetup_SetExposure.restype = ctypes.c_int

        # k64f GoSetup_SpacingInterval(kObject setup, GoRole role)
        self.sdk.GoSetup_SpacingInterval.argtypes = [
            ctypes.c_void_p,                    # setup
            ctypes.c_int                         # role
        ]
        self.sdk.GoSetup_SpacingInterval.restype = ctypes.c_double

        # kStatus GoSetup_SetSpacingInterval(kObject setup, GoRole role, k64f value)
        self.sdk.GoSetup_SetSpacingInterval.argtypes = [
            ctypes.c_void_p,                    # setup
            ctypes.c_int,                        # role
            ctypes.c_double                      # value
        ]
        self.sdk.GoSetup_SetSpacingInterval.restype = ctypes.c_int

        # === Acquisition Control ===

        # kStatus GoSystem_Start(kObject system)
        self.sdk.GoSystem_Start.argtypes = [ctypes.c_void_p]
        self.sdk.GoSystem_Start.restype = ctypes.c_int

        # kStatus GoSystem_Stop(kObject system)
        self.sdk.GoSystem_Stop.argtypes = [ctypes.c_void_p]
        self.sdk.GoSystem_Stop.restype = ctypes.c_int

        # === Data Acquisition ===

        # kStatus GoSystem_ReceiveData(kObject system, kObject* dataset, k64u timeout)
        self.sdk.GoSystem_ReceiveData.argtypes = [
            ctypes.c_void_p,                    # system
            ctypes.POINTER(ctypes.c_void_p),    # dataset
            ctypes.c_uint64                      # timeout
        ]
        self.sdk.GoSystem_ReceiveData.restype = ctypes.c_int

        # === Dataset Functions ===

        # kSize GoDataSet_Count(kObject dataset)
        self.sdk.GoDataSet_Count.argtypes = [ctypes.c_void_p]
        self.sdk.GoDataSet_Count.restype = ctypes.c_size_t

        # kObject GoDataSet_At(kObject dataset, kSize index)
        self.sdk.GoDataSet_At.argtypes = [
            ctypes.c_void_p,                    # dataset
            ctypes.c_size_t                     # index
        ]
        self.sdk.GoDataSet_At.restype = ctypes.c_void_p

        # === Message Type Functions ===

        # GoDataMessageType GoDataMsg_Type(kObject message)
        self.sdk.GoDataMsg_Type.argtypes = [ctypes.c_void_p]
        self.sdk.GoDataMsg_Type.restype = ctypes.c_int

        # === Surface Message Functions ===

        # k32u GoSurfaceMsg_Length(kObject message)
        self.sdk.GoSurfaceMsg_Length.argtypes = [ctypes.c_void_p]
        self.sdk.GoSurfaceMsg_Length.restype = ctypes.c_uint32

        # k32u GoSurfaceMsg_Width(kObject message)
        self.sdk.GoSurfaceMsg_Width.argtypes = [ctypes.c_void_p]
        self.sdk.GoSurfaceMsg_Width.restype = ctypes.c_uint32

        # k64f GoSurfaceMsg_XResolution(kObject message)
        self.sdk.GoSurfaceMsg_XResolution.argtypes = [ctypes.c_void_p]
        self.sdk.GoSurfaceMsg_XResolution.restype = ctypes.c_double

        # k64f GoSurfaceMsg_YResolution(kObject message)
        self.sdk.GoSurfaceMsg_YResolution.argtypes = [ctypes.c_void_p]
        self.sdk.GoSurfaceMsg_YResolution.restype = ctypes.c_double

        # k64f GoSurfaceMsg_ZResolution(kObject message)
        self.sdk.GoSurfaceMsg_ZResolution.argtypes = [ctypes.c_void_p]
        self.sdk.GoSurfaceMsg_ZResolution.restype = ctypes.c_double

        # k64f GoSurfaceMsg_XOffset(kObject message)
        self.sdk.GoSurfaceMsg_XOffset.argtypes = [ctypes.c_void_p]
        self.sdk.GoSurfaceMsg_XOffset.restype = ctypes.c_double

        # k64f GoSurfaceMsg_YOffset(kObject message)
        self.sdk.GoSurfaceMsg_YOffset.argtypes = [ctypes.c_void_p]
        self.sdk.GoSurfaceMsg_YOffset.restype = ctypes.c_double

        # k64f GoSurfaceMsg_ZOffset(kObject message)
        self.sdk.GoSurfaceMsg_ZOffset.argtypes = [ctypes.c_void_p]
        self.sdk.GoSurfaceMsg_ZOffset.restype = ctypes.c_double

        # k64f GoSurfaceMsg_Exposure(kObject message)
        self.sdk.GoSurfaceMsg_Exposure.argtypes = [ctypes.c_void_p]
        self.sdk.GoSurfaceMsg_Exposure.restype = ctypes.c_double

        # k16s* GoSurfaceMsg_RowAt(kObject message, k32u rowIndex)
        self.sdk.GoSurfaceMsg_RowAt.argtypes = [
            ctypes.c_void_p,                    # message
            ctypes.c_uint32                      # rowIndex
        ]
        self.sdk.GoSurfaceMsg_RowAt.restype = ctypes.POINTER(ctypes.c_int16)

        # === Cleanup Functions ===

        # kStatus GoDestroy(kObject object)
        self.sdk.GoDestroy.argtypes = [ctypes.c_void_p]
        self.sdk.GoDestroy.restype = ctypes.c_int

    def connect(self) -> bool:
        """
        Connect to Gocator sensor.

        Returns:
            True if successful

        Raises:
            RuntimeError: If SDK not loaded or connection fails
        """
        if not self.sdk:
            raise RuntimeError("SDK not loaded")

        try:
            # 1. Initialize SDK API
            api_obj = ctypes.c_void_p()
            status = self.sdk.GoSdk_Construct(ctypes.byref(api_obj))
            if status != kStatus.kOK:
                raise RuntimeError(f"GoSdk_Construct failed with status {status}")

            # 2. Construct system object
            system_obj = ctypes.c_void_p()
            status = self.sdk.GoSystem_Construct(ctypes.byref(system_obj), None)
            if status != kStatus.kOK:
                raise RuntimeError(f"GoSystem_Construct failed with status {status}")
            self.system = system_obj

            # 3. Parse IP address
            ip_addr = kIpAddress()
            status = self.sdk.kIpAddress_Parse(
                ctypes.byref(ip_addr),
                self.ip_address.encode('utf-8')
            )
            if status != kStatus.kOK:
                raise RuntimeError(f"Failed to parse IP address: {self.ip_address}")

            # 4. Find sensor by IP address
            sensor_obj = ctypes.c_void_p()
            status = self.sdk.GoSystem_FindSensorByIpAddress(
                self.system,
                ctypes.byref(ip_addr),
                ctypes.byref(sensor_obj)
            )
            if status != kStatus.kOK:
                raise RuntimeError(
                    f"Failed to find sensor at {self.ip_address}. "
                    f"Status: {status}"
                )
            self.sensor = sensor_obj

            # 5. Connect to sensor
            status = self.sdk.GoSensor_Connect(self.sensor)
            if status != kStatus.kOK:
                raise RuntimeError(f"Failed to connect to sensor. Status: {status}")

            # 6. Enable data channel
            status = self.sdk.GoSystem_EnableData(self.system, kTRUE)
            if status != kStatus.kOK:
                raise RuntimeError(f"Failed to enable data channel. Status: {status}")

            # 7. Get sensor info
            model_buffer = ctypes.create_string_buffer(64)
            self.sdk.GoSensor_Model(self.sensor, model_buffer, 64)
            sensor_id = self.sdk.GoSensor_Id(self.sensor)

            print(f"Connected to Gocator sensor:")
            print(f"  IP: {self.ip_address}")
            print(f"  Model: {model_buffer.value.decode('utf-8')}")
            print(f"  Serial: {sensor_id}")

            self.connected = True
            return True

        except Exception as e:
            print(f"Connection failed: {e}")
            # Cleanup on failure
            if self.system:
                self.sdk.GoDestroy(self.system)
                self.system = None
            return False

    def disconnect(self):
        """Disconnect from sensor and cleanup resources."""
        if self.sdk and self.connected:
            try:
                # Stop system if running
                self.sdk.GoSystem_Stop(self.system)

                # Disconnect sensor
                if self.sensor:
                    self.sdk.GoSensor_Disconnect(self.sensor)
                    self.sdk.GoDestroy(self.sensor)
                    self.sensor = None

                # Destroy system
                if self.system:
                    self.sdk.GoDestroy(self.system)
                    self.system = None

                print("Disconnected from Gocator sensor")
            except Exception as e:
                print(f"Error during disconnect: {e}")
            finally:
                self.connected = False

    def start(self) -> bool:
        """
        Start the sensor acquisition system.

        Returns:
            True if successful
        """
        if not self.connected:
            raise ConnectionError("Not connected to sensor")

        status = self.sdk.GoSystem_Start(self.system)
        if status != kStatus.kOK:
            print(f"Failed to start system. Status: {status}")
            return False
        return True

    def stop(self) -> bool:
        """
        Stop the sensor acquisition system.

        Returns:
            True if successful
        """
        if not self.connected:
            raise ConnectionError("Not connected to sensor")

        status = self.sdk.GoSystem_Stop(self.system)
        if status != kStatus.kOK:
            print(f"Failed to stop system. Status: {status}")
            return False
        return True

    def get_point_cloud(self, timeout_us: int = RECEIVE_TIMEOUT) -> Optional[NDArray[np.float64]]:
        """
        Receive and extract point cloud data from sensor.

        Args:
            timeout_us: Timeout in microseconds (default: 20 seconds)

        Returns:
            Nx3 numpy array of (X, Y, Z) points in meters, or None if no data

        Raises:
            ConnectionError: If not connected to sensor
        """
        if not self.connected:
            raise ConnectionError("Not connected to sensor")

        dataset = ctypes.c_void_p()
        try:
            # 1. Receive data from system
            status = self.sdk.GoSystem_ReceiveData(
                self.system,
                ctypes.byref(dataset),
                timeout_us
            )

            if status != kStatus.kOK:
                if status == kStatus.kERROR_TIMEOUT:
                    print("Timeout waiting for data")
                    return None
                print(f"Failed to receive data. Status: {status}")
                return None

            # 2. Process all messages in dataset
            msg_count = self.sdk.GoDataSet_Count(dataset)
            if msg_count == 0:
                print("No messages in dataset")
                return None

            # 3. Find surface message and extract point cloud
            for i in range(msg_count):
                msg = self.sdk.GoDataSet_At(dataset, i)
                msg_type = self.sdk.GoDataMsg_Type(msg)

                if msg_type == GO_DATA_MESSAGE_TYPE_SURFACE:
                    points = self._extract_surface_points(msg)
                    return points

            print("No surface message found in dataset")
            return None

        except Exception as e:
            print(f"Failed to get point cloud: {e}")
            import traceback
            traceback.print_exc()
            return None

        finally:
            # Cleanup dataset
            if dataset:
                self.sdk.GoDestroy(dataset)

    def scan_and_get(self) -> Optional[NDArray[np.float64]]:
        """
        Convenience method: start acquisition, get point cloud, then stop.

        Returns:
            Nx3 point cloud or None
        """
        if self.start():
            import time
            time.sleep(0.1)  # Wait for scan to complete
            return self.get_point_cloud()
        return None

    def _extract_surface_points(self, surface_msg) -> NDArray[np.float64]:
        """
        Extract XYZ point cloud from GoSurfaceMsg.

        The surface message contains a 2D grid of elevation (Z) values.
        X and Y coordinates are computed based on resolution and offsets.

        Args:
            surface_msg: GoSurfaceMsg object pointer

        Returns:
            Nx3 numpy array of (X, Y, Z) coordinates in meters
        """
        # Get dimensions
        length = self.sdk.GoSurfaceMsg_Length(surface_msg)  # rows (Y direction)
        width = self.sdk.GoSurfaceMsg_Width(surface_msg)    # cols (X direction)

        # Get resolution and offset
        x_resolution = self.sdk.GoSurfaceMsg_XResolution(surface_msg)
        y_resolution = self.sdk.GoSurfaceMsg_YResolution(surface_msg)
        z_resolution = self.sdk.GoSurfaceMsg_ZResolution(surface_msg)

        x_offset = self.sdk.GoSurfaceMsg_XOffset(surface_msg)
        y_offset = self.sdk.GoSurfaceMsg_YOffset(surface_msg)
        z_offset = self.sdk.GoSurfaceMsg_ZOffset(surface_msg)

        # Pre-allocate point cloud array
        # We'll filter invalid points later
        points_list = []

        # Iterate through rows (Y direction)
        for row_idx in range(length):
            # Get pointer to elevation data for this row
            row_data = self.sdk.GoSurfaceMsg_RowAt(surface_msg, row_idx)

            # Calculate Y coordinate for this row
            y = y_offset + row_idx * y_resolution

            # Iterate through columns (X direction)
            for col_idx in range(width):
                # Get elevation value (16-bit signed integer)
                z_raw = row_data[col_idx]

                # Skip invalid points
                # k16U_NULL (65535 cast to signed = -1) or k16S_MIN (-32768)
                if z_raw == k16S_MIN or z_raw == -1:
                    continue

                # Calculate X coordinate
                x = x_offset + col_idx * x_resolution

                # Calculate Z coordinate (convert to meters)
                z = z_offset + z_raw * z_resolution

                # Add point to list
                points_list.append([x, y, z])

        # Convert to numpy array
        if not points_list:
            print("Warning: No valid points in surface message")
            return np.array([]).reshape(0, 3)

        points = np.array(points_list, dtype=np.float64)

        print(f"Extracted {len(points)} points from {length}x{width} surface")
        print(f"  X range: [{points[:, 0].min():.6f}, {points[:, 0].max():.6f}] m")
        print(f"  Y range: [{points[:, 1].min():.6f}, {points[:, 1].max():.6f}] m")
        print(f"  Z range: [{points[:, 2].min():.6f}, {points[:, 2].max():.6f}] m")

        return points

    def get_exposure(self, role: int = GO_ROLE_MAIN) -> float:
        """
        Get current exposure time in microseconds.

        Args:
            role: Sensor role (GO_ROLE_MAIN or GO_ROLE_BUDDY)

        Returns:
            Exposure time in microseconds
        """
        if not self.connected:
            raise ConnectionError("Not connected to sensor")

        setup = self.sdk.GoSensor_Setup(self.sensor)
        exposure = self.sdk.GoSetup_Exposure(setup, role)
        return exposure

    def set_exposure(self, exposure_us: float, role: int = GO_ROLE_MAIN) -> bool:
        """
        Set exposure time in microseconds.

        Args:
            exposure_us: Exposure time in microseconds
            role: Sensor role (GO_ROLE_MAIN or GO_ROLE_BUDDY)

        Returns:
            True if successful
        """
        if not self.connected:
            raise ConnectionError("Not connected to sensor")

        setup = self.sdk.GoSensor_Setup(self.sensor)
        status = self.sdk.GoSetup_SetExposure(setup, role, exposure_us)

        if status != kStatus.kOK:
            print(f"Failed to set exposure. Status: {status}")
            return False

        print(f"Set exposure to {exposure_us} µs")
        return True

    def get_spacing_interval(self, role: int = GO_ROLE_MAIN) -> float:
        """
        Get spacing interval (distance between scan lines).

        Args:
            role: Sensor role (GO_ROLE_MAIN or GO_ROLE_BUDDY)

        Returns:
            Spacing interval in mm
        """
        if not self.connected:
            raise ConnectionError("Not connected to sensor")

        setup = self.sdk.GoSensor_Setup(self.sensor)
        interval = self.sdk.GoSetup_SpacingInterval(setup, role)
        return interval

    def set_spacing_interval(self, interval_mm: float, role: int = GO_ROLE_MAIN) -> bool:
        """
        Set spacing interval (distance between scan lines).

        Args:
            interval_mm: Spacing interval in mm
            role: Sensor role (GO_ROLE_MAIN or GO_ROLE_BUDDY)

        Returns:
            True if successful
        """
        if not self.connected:
            raise ConnectionError("Not connected to sensor")

        setup = self.sdk.GoSensor_Setup(self.sensor)
        status = self.sdk.GoSetup_SetSpacingInterval(setup, role, interval_mm)

        if status != kStatus.kOK:
            print(f"Failed to set spacing interval. Status: {status}")
            return False

        print(f"Set spacing interval to {interval_mm} mm")
        return True

    def get_sensor_info(self) -> dict:
        """
        Get sensor information.

        Returns:
            Dictionary with sensor model, serial number, etc.
        """
        if not self.connected:
            raise ConnectionError("Not connected to sensor")

        model_buffer = ctypes.create_string_buffer(64)
        self.sdk.GoSensor_Model(self.sensor, model_buffer, 64)
        sensor_id = self.sdk.GoSensor_Id(self.sensor)

        return {
            "ip_address": self.ip_address,
            "model": model_buffer.value.decode('utf-8'),
            "serial_number": sensor_id,
        }

    def __enter__(self):
        """Context manager support."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):  # noqa: ARG002
        """Context manager support."""
        self.disconnect()
