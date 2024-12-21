import socket
import struct
import requests
import xml.etree.ElementTree as ET
import time
import multiprocessing

def read_loop(sensor_ip, sensor_port, calpf, calpt, force_torque_data, data_lock, stop_event):
    """
    Thread loop to read data continuously from the sensor.
    Args:
        sensor_ip (str): IP address of the sensor.
        sensor_port (int): UDP port for the sensor communication.
        force_torque_data (multiprocessing.Array): Shared memory array for force-torque data.
        lock (multiprocessing.Lock): Lock for accessing shared memory.
        stop_event (multiprocessing.Event): Event to stop the thread.
    """
    sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sensor_socket.settimeout(1.0)  # Set timeout for socket operations

    def send_command(command, sample_count=0):
        """
        Send a command to the sensor.

        Args:
            command (int): Command code.
            sample_count (int): Number of samples to output (0 for continuous).
        """
        command_header = 0x1234
        request = struct.pack('!HHI', command_header, command, sample_count)
        sensor_socket.sendto(request, (sensor_ip, sensor_port))

    send_command(command=0x0002, sample_count=0)  # Start continuous streaming

    while not stop_event.is_set():
        try:
            data, _ = sensor_socket.recvfrom(1024)

            if len(data) == 36:  # Minimum packet size
                parsed_data = struct.unpack('!IIIiiiiii', data[:36])

                rdt_sequence = parsed_data[0]
                ft_sequence = parsed_data[1]
                status = parsed_data[2]
                Fx = parsed_data[3] / calpf
                Fy = parsed_data[4] / calpf
                Fz = parsed_data[5] / calpf
                Tx = parsed_data[6] / calpt 
                Ty = parsed_data[7] / calpt
                Tz = parsed_data[8] / calpt
                
                with data_lock:
                    force_torque_data[:] = [Fx, Fy, Fz, Tx, Ty, Tz]

            else:
                print(f"Warning: Unexpected packet size {len(data)}")

        except socket.timeout:
            print(f"WARNING: Timeout reading from sensor, retrying...")
            send_command(command=0x0002, sample_count=0) # retry

        except KeyboardInterrupt:
            print(f"Stopping the force-torque sensor driver...")
            break

        except Exception as e:
            print(f"Error in read loop: {e}")

    send_command(command=0x0000)  # Stop streaming
    sensor_socket.close()


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

        calibration = self.get_calibration_data()

        # Extract counts per force and torque unit
        try:
            self.calcpf = int(calibration.get('calcpf')) 
            self.calcpt = int(calibration.get('calcpt'))
        except (ValueError, TypeError):
            raise ValueError("Calibration data not available or invalid")

        self.thread = None
        self.force_torque_data = multiprocessing.Array('d', [float('nan')] * 6)  # 'd' means double (float)
        self.data_lock = multiprocessing.Lock()
        self.stop_event = multiprocessing.Event()

    def start(self, timeout=3.0):
        """
        Start the data acquisition thread.
        """

        # check that the thread is not already running
        if self.thread is not None and self.thread.is_alive():
            raise RuntimeError("Sensor driver is already running")

        # clear the latest reading and event
        with self.data_lock:
            self.force_torque_data[:] = [float('nan')] * 6

        self.stop_event.clear()

        self.thread = multiprocessing.Process(
            target=read_loop,
            args=(self.sensor_ip, self.sensor_port, self.calcpf, self.calcpt, self.force_torque_data, self.data_lock, self.stop_event)
        )
        self.thread.start()

        # Block until the first reading is available
        start_time = time.time()
        while True:

            with self.data_lock:
                if self.force_torque_data[0] != float('nan'):
                    break

            if time.time() - start_time > timeout:
                raise TimeoutError("Timeout waiting for first reading")
            
            time.sleep(1/50)  # 50 Hz

    def stop(self):
        """
        Stop the data acquisition thread.
        """
        self.stop_event.set()
        self.thread.join()

        # Clear the latest reading
        with self.data_lock:
            self.force_torque_data[:] = [float('nan')] * 6

    def get_wrench(self):
        # check if the thread is running
        if self.thread is None or not self.thread.is_alive():
            raise RuntimeError("Sensor driver is not running")
        
        with self.data_lock:
            return list(self.force_torque_data)

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
