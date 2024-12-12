import socket
import struct
import threading
import requests
import xml.etree.ElementTree as ET
import time

class ForceTorqueSensorDriver:
    def __init__(self, sensor_ip, sensor_port=49152):
        """
        Initialize the force-torque sensor driver.

        Args:
            sensor_ip (str): IP address of the sensor.
            sensor_port (int): UDP port for the sensor communication.
        """
        self.sensor_ip = sensor_ip
        self.sensor_port = sensor_port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(1.0)  # Set timeout for socket operations

        self.running = False
        self.latest_reading = None
        self.lock = threading.Lock()

        calibration = self.get_calibration_data()

        # Extract counts per force and torque unit
        try:
            self.calcpf = int(calibration.get('calcpf')) 
            self.calcpt = int(calibration.get('calcpt'))
        except (ValueError, TypeError):
            raise ValueError("Calibration data not available or invalid")

    def _send_command(self, command, sample_count=0):
        """
        Send a command to the sensor.

        Args:
            command (int): Command code.
            sample_count (int): Number of samples to output (0 for continuous).
        """
        command_header = 0x1234
        request = struct.pack('!HHI', command_header, command, sample_count)
        self.socket.sendto(request, (self.sensor_ip, self.sensor_port))

    def _read_loop(self):
        """
        Thread loop to read data continuously from the sensor.
        """
        while self.running:
            try:
                start_time = time.time()
                data, _ = self.socket.recvfrom(1024)
                end_time = time.time()
                if len(data) == 36:  # Minimum packet size
                    parsed_data = struct.unpack('!IIIiiiiii', data[:36])
                    reading = {
                        'rdt_sequence': parsed_data[0],
                        'ft_sequence': parsed_data[1],
                        'status': parsed_data[2],
                        'Fx': parsed_data[3],
                        'Fy': parsed_data[4],
                        'Fz': parsed_data[5],
                        'Tx': parsed_data[6],
                        'Ty': parsed_data[7],
                        'Tz': parsed_data[8],
                        'delta_t': end_time - start_time,
                    }
                    with self.lock:
                        self.latest_reading = reading
                else:
                    print(f"Warning: Unexpected packet size {len(data)}")
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error in read loop: {e}")

    def start(self, timeout=3.0):
        """
        Start the data acquisition thread.
        """
        self.running = True
        self._send_command(command=0x0002, sample_count=0)  # Start continuous streaming
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

        # Block until the first reading is available
        start_time = time.time()
        while self.latest_reading is None:
            if time.time() - start_time > timeout:
                raise TimeoutError("Timeout waiting for first reading")
            time.sleep(1/50)  # 50 Hz

    def stop(self):
        """
        Stop the data acquisition thread.
        """
        self.running = False
        self._send_command(command=0x0000)  # Stop streaming
        if self.thread.is_alive():
            self.thread.join()

        # Clear the latest reading
        with self.lock:
            self.latest_reading = None

    def get_latest_reading(self):
        """
        Get the most recent force-torque data, converted to N and Nm if calibration data is available.

        Returns:
            dict: Latest force-torque data or None if no data is available.
        """
        with self.lock:
            if self.latest_reading is None:
                return None

            if self.calcpf and self.calcpt:
                # Convert raw readings to N and Nm
                return {
                    'rdt_sequence': self.latest_reading['rdt_sequence'],
                    'ft_sequence': self.latest_reading['ft_sequence'],
                    'status': self.latest_reading['status'],
                    'Fx': self.latest_reading['Fx'] / self.calcpf,
                    'Fy': self.latest_reading['Fy'] / self.calcpf,
                    'Fz': self.latest_reading['Fz'] / self.calcpf,
                    'Tx': self.latest_reading['Tx'] / self.calcpt,
                    'Ty': self.latest_reading['Ty'] / self.calcpt,
                    'Tz': self.latest_reading['Tz'] / self.calcpt,
                    'delta_t': self.latest_reading['delta_t'],
                }
            else:
                return self.latest_reading
            
    def get_latest_wrench(self):
        with self.lock:
            if self.latest_reading is None:
                return None
            return [
                self.latest_reading['Fx'] / self.calcpf,
                self.latest_reading['Fy'] / self.calcpf,
                self.latest_reading['Fz'] / self.calcpf,
                self.latest_reading['Tx'] / self.calcpt,
                self.latest_reading['Ty'] / self.calcpt,
                self.latest_reading['Tz'] / self.calcpt
            ]

    def get_sensor_configuration(self):
        """
        Retrieve the sensor's configuration data via the XML interface.

        Returns:
            dict: Parsed configuration data.
        """
        url = f"http://{self.sensor_ip}/netftapi2.xml"
        try:
            response = requests.get(url, timeout=5)
            response.raise_for_status()
            root = ET.fromstring(response.content)
            config = {child.tag: child.text for child in root}
            return config
        except requests.RequestException as e:
            print(f"Error fetching configuration data: {e}")
            return None

    def get_calibration_data(self, index=None):
        """
        Retrieve the sensor's calibration data via the XML interface.

        Args:
            index (int, optional): Calibration index. Defaults to None (current calibration).

        Returns:
            dict: Parsed calibration data.
        """
        url = f"http://{self.sensor_ip}/netftcalapi.xml"
        if index is not None:
            url += f"?index={index}"
        try:
            response = requests.get(url, timeout=5)
            response.raise_for_status()
            root = ET.fromstring(response.content)
            calibration = {child.tag: child.text for child in root}

            return calibration
        except requests.RequestException as e:
            print(f"Error fetching calibration data: {e}")
            return None

