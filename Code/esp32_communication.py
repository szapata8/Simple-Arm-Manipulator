import serial
import serial.tools.list_ports
import serial.serialutil
import time

BAUD_RATE = 9600
ESP32_ID = "PID=10C4:EA60"
INITIALIZATION_MESSAGE_LENGTH = 40
MAX_BUFFER_LENGTH = 255

def find_esp32_port():
    ports = list(serial.tools.list_ports.comports())
    for i, port in enumerate(ports):
        # print(f"port #{i}: {port}. hwid: {port.hwid}. name: {port.name}. description: {port.description}. device: {port.device}")
        if ESP32_ID in port.hwid:
            return port.name
    print("ERROR: COM port for ESP32 not found")
    return None

class ESP32_communication:
    def __init__(self):
        self.com_port_name = find_esp32_port()
        self.com_port = serial.Serial(port = self.com_port_name, baudrate = BAUD_RATE)
        while self.com_port.inWaiting() < INITIALIZATION_MESSAGE_LENGTH:
            continue
        bytesToRead = self.com_port.inWaiting() 
        response = self.com_port.read(bytesToRead)
        self.com_port.flushInput()
        self.com_port.flushOutput()
        print(f"ESP32 detected: {response}")

    def monitor_serial_port(self):
        while True:
            message_to_send = input("Message for ESP32: ").encode()
            self.com_port.write(message_to_send)
            while self.com_port.inWaiting() < 1:
                pass
            time.sleep(1)
            [bytes_available, response] = self.read_buffer()
            print(f"[bytes_available, response]: \n {[bytes_available, response]}")

    def send_arrays(self):
        arrays_to_send = [
            bytearray([1, 2, 3, 4, 5]),
            bytearray([6, 7, 8, 9, 10]),
            bytearray([2, 4, 6, 8, 10]),
            bytearray([1, 3, 5, 7, 9])
        ]
        for arr in arrays_to_send:
            self.com_port.write(arr)
            time.sleep(1)
            response = self.read_buffer()
            print(response)
            time.sleep(1)

    def read_buffer(self):
        bytesToRead = self.com_port.inWaiting() 
        response = self.com_port.read(bytesToRead)
        return response

    def done(self):
        self.com_port.close()


if __name__ == "__main__":
    esp32 = ESP32_communication()
    # esp32.monitor_serial_port()
    esp32.send_arrays()
    esp32.done()



