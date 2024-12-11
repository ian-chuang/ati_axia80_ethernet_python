from ati_axia80_ethernet_python import ForceTorqueSensorDriver
import time

# Example usage
if __name__ == "__main__":
    sensor_ip = "172.22.22.3" # your ip address probably 192.168.1.1
    driver = ForceTorqueSensorDriver(sensor_ip)

    try:
        driver.start()

        print("Calibration data:")
        print(driver.get_calibration_data())
        print() 
        
        print("Sensor configuration:")
        print(driver.get_sensor_configuration())
        print()

        while True:
            reading = driver.get_latest_reading()
            if reading:
                print(reading)

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Stopping the driver...")
    finally:
        driver.stop()