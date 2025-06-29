#!/usr/bin/env python3
"""
RPLIDAR A1 Real-time Visualization Program
360-degree distance data display with matplotlib
Note: Check /dev/tty.* with 'ls /dev/tty.*' command to find your device
"""

import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

# ==================== CONFIGURATION ====================
# Adjust these parameters as needed

# Serial port configuration
SERIAL_PORT = '/dev/tty.usbserial-0001'  # Update this with your device
BAUDRATE = 115200

# Visualization parameters
UPDATE_RATE = 50  # milliseconds (20 fps)
POINT_SIZE = 1  # size of data points
MAX_DISTANCE = 5000  # maximum distance in mm
ANGLE_RESOLUTION = 0.5  # degrees (0.5 = 720 points, 1.0 = 360 points)
DATA_COLLECTION_TIME = 0.1  # seconds per frame

# Color map settings
COLORMAP = 'jet'  # 'jet' provides blue(near) to red(far) gradient
# Other options: 'viridis', 'plasma', 'turbo', 'rainbow'

# Angle adjustment
MIRROR_HORIZONTALLY = False  # Set to True to swap left/right (90° ↔ 270°)
# True: 90° appears on left, 270° on right (mirrored)
# False: 90° appears on right, 270° on left (normal)

# ======================================================

class RPLidarA1:
    """Simple driver for RPLIDAR A1"""

    SYNC_BYTE = 0xA5
    SYNC_BYTE2 = 0x5A

    GET_INFO = 0x50
    GET_HEALTH = 0x52
    STOP = 0x25
    RESET = 0x40
    SCAN = 0x20
    EXPRESS_SCAN = 0x82

    DESCRIPTOR_LEN = 7
    INFO_LEN = 20
    HEALTH_LEN = 3

    def __init__(self, port, baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None

    def connect(self):
        """Connect to serial port"""
        self.serial = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.timeout,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )

    def disconnect(self):
        """Close connection"""
        if self.serial:
            self.serial.close()

    def _send_command(self, cmd):
        """Send command"""
        req = struct.pack('<BB', self.SYNC_BYTE, cmd)
        self.serial.write(req)

    def _read_descriptor(self):
        """Read descriptor"""
        descriptor = self.serial.read(self.DESCRIPTOR_LEN)
        if len(descriptor) != self.DESCRIPTOR_LEN:
            raise Exception("Failed to read descriptor")
        return descriptor

    def get_info(self):
        """Get device information"""
        self._send_command(self.GET_INFO)
        descriptor = self._read_descriptor()
        data = self.serial.read(self.INFO_LEN)

        if len(data) == self.INFO_LEN:
            model, firmware_minor, firmware_major, hardware, serial_number = struct.unpack('<BBBB16s', data)
            return {
                'model': model,
                'firmware': f"{firmware_major}.{firmware_minor}",
                'hardware': hardware,
                'serial_number': serial_number.hex()
            }
        return None

    def get_health(self):
        """Get health status"""
        self._send_command(self.GET_HEALTH)
        descriptor = self._read_descriptor()
        data = self.serial.read(self.HEALTH_LEN)

        if len(data) == self.HEALTH_LEN:
            status, error_code = struct.unpack('<BH', data)
            return {'status': status, 'error_code': error_code}
        return None

    def start_scan(self):
        """Start scanning"""
        self._send_command(self.SCAN)
        descriptor = self._read_descriptor()
        # Check scan response header
        if descriptor[0] != self.SYNC_BYTE or descriptor[1] != self.SYNC_BYTE2:
            raise Exception("Failed to start scan")

    def stop_scan(self):
        """Stop scanning"""
        self._send_command(self.STOP)
        time.sleep(0.1)  # Wait for command processing
        self.serial.reset_input_buffer()

    def reset(self):
        """Reset device"""
        self._send_command(self.RESET)
        time.sleep(2)  # Wait for reset completion

    def read_scan_data(self):
        """Read scan data (generator)"""
        while True:
            # Read 5-byte data point
            data = self.serial.read(5)
            if len(data) == 5:
                # Parse data
                quality = data[0] >> 2
                start_flag = (data[0] & 0x01) == 1
                # Calculate angle (fixed point format)
                angle = ((data[2] << 8) | data[1]) >> 1
                angle = angle / 64.0  # Convert to degrees
                # Calculate distance
                distance = (data[4] << 8) | data[3]
                distance = distance / 4.0  # Convert to mm

                if quality > 0 and distance > 0:
                    yield {
                        'angle': angle,
                        'distance': distance,
                        'quality': quality,
                        'start_flag': start_flag
                    }


class LidarVisualizer:
    """LIDAR data visualization class"""

    def __init__(self, lidar):
        self.lidar = lidar
        self.angles = []
        self.distances = []
        self.max_points = int(360 / ANGLE_RESOLUTION)  # Calculate based on resolution

        # Plot setup
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111, projection='polar')
        self.ax.set_ylim(0, MAX_DISTANCE)
        self.ax.set_title('RPLIDAR A1 - 360° Distance Data', fontsize=16)
        self.ax.grid(True)

        # Set angle direction and zero location
        self.ax.set_theta_direction(-1)  # Clockwise
        self.ax.set_theta_zero_location('N')  # 0 degrees at top

        # Initialize scatter plot with distance-based coloring
        self.scatter = self.ax.scatter([], [], c=[], cmap=COLORMAP,
                                      s=POINT_SIZE, vmin=0, vmax=MAX_DISTANCE)

        # Initialize colorbar
        self.cbar = plt.colorbar(self.scatter, ax=self.ax, pad=0.1)
        self.cbar.set_label('Distance (mm)', rotation=270, labelpad=20)

        # Initialize text display
        self.text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes,
                                verticalalignment='top', fontsize=10,
                                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    def update(self, frame):
        """Animation update function"""
        # Collect new data
        angle_dict = {}  # Use dict to handle angle resolution

        start_time = time.time()
        point_count = 0

        # Collect data for specified time
        while time.time() - start_time < DATA_COLLECTION_TIME:
            try:
                data = next(self.lidar.read_scan_data())
                # Round angle to nearest resolution step
                angle_key = round(data['angle'] / ANGLE_RESOLUTION) * ANGLE_RESOLUTION
                angle_dict[angle_key] = data['distance']
                point_count += 1
            except StopIteration:
                break

        if angle_dict:
            # Convert to lists
            angles = []
            distances = []
            for angle, distance in angle_dict.items():
                # Apply angle adjustment if needed
                if MIRROR_HORIZONTALLY:
                    # Mirror the angle horizontally (left-right flip)
                    # This swaps 90° and 270°
                    adjusted_angle = (360 - angle) % 360
                else:
                    adjusted_angle = angle

                angles.append(np.radians(adjusted_angle))
                distances.append(distance)

            # Update data
            self.angles = angles
            self.distances = distances

            # Update scatter plot
            self.scatter.set_offsets(np.c_[angles, distances])
            self.scatter.set_array(np.array(distances))  # Color by distance

            # Update statistics
            avg_dist = np.mean(distances) if distances else 0
            min_dist = np.min(distances) if distances else 0
            max_dist = np.max(distances) if distances else 0

            info_text = f'Points: {len(angles)}/{self.max_points}\n'
            info_text += f'Avg Distance: {avg_dist:.0f} mm\n'
            info_text += f'Min Distance: {min_dist:.0f} mm\n'
            info_text += f'Max Distance: {max_dist:.0f} mm\n'
            info_text += f'Update Rate: {1000/UPDATE_RATE:.1f} Hz'
            self.text.set_text(info_text)

        return self.scatter, self.text

    def start(self):
        """Start visualization"""
        # Start animation
        ani = FuncAnimation(self.fig, self.update, interval=UPDATE_RATE,
                           blit=True, cache_frame_data=False)
        plt.show()
        return ani


def main():
    """Main function"""
    # Connect to LIDAR
    lidar = RPLidarA1(SERIAL_PORT, baudrate=BAUDRATE)

    try:
        print("Connecting to RPLIDAR...")
        print(f"Port: {SERIAL_PORT}")
        print(f"Baudrate: {BAUDRATE}")
        lidar.connect()

        # Display device information
        info = lidar.get_info()
        if info:
            print(f"\nDevice Information:")
            print(f"Model: {info['model']}")
            print(f"Firmware: {info['firmware']}")
            print(f"Hardware: {info['hardware']}")
            print(f"Serial Number: {info['serial_number']}")

        # Check health status
        health = lidar.get_health()
        if health:
            print(f"\nHealth Status: {'OK' if health['status'] == 0 else 'ERROR'}")
            print(f"Error Code: {health['error_code']}")

        # Configuration summary
        print(f"\nVisualization Configuration:")
        print(f"Update Rate: {1000/UPDATE_RATE:.1f} Hz")
        print(f"Point Size: {POINT_SIZE}")
        print(f"Max Distance: {MAX_DISTANCE} mm")
        print(f"Angle Resolution: {ANGLE_RESOLUTION}° ({int(360/ANGLE_RESOLUTION)} points max)")
        print(f"Color Map: {COLORMAP}")
        print(f"Mirror Horizontally: {MIRROR_HORIZONTALLY}")

        # Start scanning
        print("\nStarting scan...")
        lidar.start_scan()

        # Start visualization
        print("Opening visualization window...")
        print("Close the window to exit")
        visualizer = LidarVisualizer(lidar)
        ani = visualizer.start()

    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        # Cleanup
        lidar.stop_scan()
        lidar.disconnect()
        print("Connection closed")


if __name__ == "__main__":
    main()
