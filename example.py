#!/usr/bin/env python3
"""
Example usage of Gocator Interface with GoSDK

This example demonstrates how to:
1. Connect to a Gocator sensor
2. Configure sensor parameters
3. Acquire point cloud data
4. Process and save point clouds
"""

from gocator import GocatorScanner
import numpy as np

IP = "192.168.100.125"
SDK_PATH = "/home/max/dev/GO_SDK/lib/linux_x64d/libGoSdk.so"


def example_basic_connection():
    """Basic connection and single scan example with software triggering."""
    print("=" * 60)
    print("Example 1: Basic Connection and Software-Triggered Scan")
    print("=" * 60)

    # SDK path is auto-detected from common installation locations
    # Or specify manually: sdk_path="/path/to/libGoSdk.so"
    scanner = GocatorScanner(
        ip_address=IP,
        sdk_path=SDK_PATH
    )

    try:
        if scanner.connect():
            # Get sensor information
            info = scanner.get_sensor_info()
            print("\nSensor Info:")
            print(f"  Model: {info['model']}")
            print(f"  Serial: {info['serial_number']}")
            print(f"  IP: {info['ip_address']}")

            # Start the acquisition system
            print("\nStarting acquisition system...")
            scanner.start()

            # Software trigger the sensor
            print("Sending software trigger...")
            if scanner.trigger():
                print("Trigger sent successfully")

                # Wait for and receive the triggered scan
                print("Waiting for point cloud data...")
                point_cloud = scanner.get_point_cloud()

                if point_cloud is not None:
                    print(f"\nReceived {len(point_cloud)} points")
                    print(f"Shape: {point_cloud.shape}")
                    print(f"Data type: {point_cloud.dtype}")

                    # Save point cloud to files
                    print("\nSaving point cloud to files...")

                    # Save as NumPy binary
                    np.save("point_cloud.npy", point_cloud)
                    print("  ✓ Saved to point_cloud.npy")

                    # Save as CSV
                    np.savetxt(
                        "point_cloud.csv", point_cloud,
                        delimiter=",",
                        header="X,Y,Z",
                        comments="",
                        fmt="%.6f"
                    )
                    print("  ✓ Saved to point_cloud.csv")

                    # Save as PLY
                    save_ply("point_cloud.ply", point_cloud)
                    print("  ✓ Saved to point_cloud.ply")
                else:
                    print("\nNote: If no data received, ensure sensor is configured for:")
                    print("  - Trigger Mode: Software or Time")
                    print("  - Output: Surface data enabled")
            else:
                print("Failed to trigger sensor")

            # Stop the acquisition system
            scanner.stop()
            scanner.disconnect()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


def example_context_manager():
    """Using context manager for automatic connection/disconnection."""
    print("\n" + "=" * 60)
    print("Example 2: Context Manager Usage")
    print("=" * 60)

    with GocatorScanner(IP, SDK_PATH) as scanner:
        # Convenience method: start, get data, and stop
        point_cloud = scanner.scan_and_get()

        if point_cloud is not None:
            print(f"\nReceived {len(point_cloud)} points")

            # Analyze point cloud
            print("\nPoint Cloud Statistics:")
            print(f"  X: min={point_cloud[:, 0].min():.6f}, max={point_cloud[:, 0].max():.6f}")
            print(f"  Y: min={point_cloud[:, 1].min():.6f}, max={point_cloud[:, 1].max():.6f}")
            print(f"  Z: min={point_cloud[:, 2].min():.6f}, max={point_cloud[:, 2].max():.6f}")


def example_configure_parameters():
    """Configure sensor parameters before scanning."""
    print("\n" + "=" * 60)
    print("Example 3: Configure Sensor Parameters")
    print("=" * 60)

    with GocatorScanner(IP, SDK_PATH) as scanner:
        # Get current settings
        current_exposure = scanner.get_exposure()
        current_spacing = scanner.get_spacing_interval()

        print("\nCurrent Settings:")
        print(f"  Exposure: {current_exposure:.2f} µs")
        print(f"  Spacing Interval: {current_spacing:.6f} mm")

        # Set new parameters
        print("\nConfiguring sensor...")
        scanner.set_exposure(100.0)  # 100 microseconds
        scanner.set_spacing_interval(0.5)  # 0.5 mm

        # Verify new settings
        new_exposure = scanner.get_exposure()
        new_spacing = scanner.get_spacing_interval()

        print("\nNew Settings:")
        print(f"  Exposure: {new_exposure:.2f} µs")
        print(f"  Spacing Interval: {new_spacing:.6f} mm")

        # Acquire with new settings
        point_cloud = scanner.scan_and_get()
        if point_cloud is not None:
            print(f"\nAcquired {len(point_cloud)} points with new settings")


def example_multiple_scans():
    """Acquire multiple scans continuously."""
    print("\n" + "=" * 60)
    print("Example 4: Multiple Continuous Scans")
    print("=" * 60)

    with GocatorScanner(IP, SDK_PATH) as scanner:
        scanner.start()

        num_scans = 5
        point_clouds = []

        for i in range(num_scans):
            print(f"\nAcquiring scan {i+1}/{num_scans}...")
            point_cloud = scanner.get_point_cloud(timeout_us=10000000)  # 10 sec timeout

            if point_cloud is not None:
                point_clouds.append(point_cloud)
                print(f"  Points: {len(point_cloud)}")
            else:
                print(f"  Failed to acquire scan {i+1}")

        scanner.stop()

        print(f"\nSuccessfully acquired {len(point_clouds)} scans")

        # Combine all point clouds
        if point_clouds:
            combined = np.vstack(point_clouds)
            print(f"Combined point cloud: {len(combined)} points")


def example_save_point_cloud():
    """Save point cloud to file formats."""
    print("\n" + "=" * 60)
    print("Example 5: Save Point Cloud to Files")
    print("=" * 60)

    with GocatorScanner(IP, SDK_PATH) as scanner:
        point_cloud = scanner.scan_and_get()

        if point_cloud is not None:
            # Save as NumPy binary
            np.save("point_cloud.npy", point_cloud)
            print("Saved to point_cloud.npy")

            # Save as text (CSV format)
            np.savetxt(
                "point_cloud.csv", point_cloud,
                delimiter=",",
                header="X,Y,Z",
                comments="",
                fmt="%.6f"
            )
            print("Saved to point_cloud.csv")

            # Save as PLY (common 3D format)
            save_ply("point_cloud.ply", point_cloud)
            print("Saved to point_cloud.ply")


def save_ply(filename: str, points: np.ndarray):
    """
    Save point cloud to PLY format.

    Args:
        filename: Output file path
        points: Nx3 numpy array of XYZ coordinates
    """
    with open(filename, 'w') as f:
        # PLY header
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")

        # Point data
        for point in points:
            f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")


if __name__ == "__main__":
    print("Gocator SDK Python Interface Examples")
    print("Make sure the Gocator SDK is installed and the sensor is connected")
    print()

    # Run examples (comment out examples you don't want to run)
    try:
        example_basic_connection()
        # example_context_manager()
        # example_configure_parameters()
        # example_multiple_scans()
        # example_save_point_cloud()

    except Exception as e:
        print(f"\nExample failed: {e}")
        import traceback
        traceback.print_exc()
