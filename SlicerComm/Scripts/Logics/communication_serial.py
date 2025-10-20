
import serial
import time
from typing import Optional, List
import qt
import slicer

class SerialCommunication:
    def __init__(self, id):
        
        self.decode_errors = "ignore"
        self.serial_port = None

        self.last_response = None
        self.all_responses = []
        self.delimiter = None
        self.id = id
        self.port = None
        self.baudrate = None
        self.timeout = None
        self.__setNode()



    def connectSerialPort(self, port: str, baudrate: int = 9600, timeout = None) -> bool:
        """Connect to the specified serial port"""
        try:
            if self.isPortConnected():
                self.serial_port.close()
            self.port = port
            self.baudrate = baudrate
            self.timeout = timeout
            self.serial_port = serial.Serial(port, baudrate, timeout=timeout)
            if self.serial_port.is_open:

                print(f"Connected to {port} at {baudrate} baud.")
                return True
            else:

                print(f"Failed to connect to {port}.")
                return False
        except serial.SerialException as e:
            print(f"Serial connection error: {e}")

            return False
        
    def reconnectSerialPort(self):
        """Reconnect to the serial port"""
        if self.isPortConnected():
            self.serial_port.close()
            self.serial_port = None
        if self.port and self.baudrate:
            self.connectSerialPort(self.port, self.baudrate, self.timeout)
        else:
            print("Port or baudrate is not set.")

    def setTimeout(self, timeout: float):
        """Set the timeout for the serial port"""
        if self.isPortConnected():
            self.serial_port.timeout = timeout
        else:
            print("Serial port is not open.")

    def getSerialPort(self):
        """Get the serial port object"""
        if self.serial_port and self.serial_port.is_open:
            return self.serial_port
        else:
            print("Serial port is not open.")
            return None

    def getNode(self):
        """Get the node for serial port communication"""
        return self.serial_port_node
    
    def setBufferSize(self, rx_size: int = 100, tx_size: int = 100):
        """Set the buffer size for the serial port"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.set_buffer_size(rx_size=rx_size, tx_size=tx_size)
        else:
            print("Serial port is not open.")

    def closeSerialPort(self):
        """Close the serial port connection"""
        if self.isPortConnected():
            self.serial_port.close()
            print("Serial port closed.")

    def sendData(self, data: str, expect_response: bool = False) -> Optional[str]:
        """Send a command to the motor controller"""
        try:
            if self.isPortConnected():
                # print(f"Sending data: {data}")
                self.serial_port.write((data).encode())
                if expect_response:
                    return self.readResponse()
            else:
                raise Exception("Serial port is not open.")
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            return None

    def readResponse(self) -> Optional[str]:
        """Read the response from the serial port"""
        if self.isPortConnected():
                    
            try:
                
                    read = self.serial_port.read_until(self.delimiter) if self.delimiter else self.serial_port.readline()
                    response = self.__decodeResponse(read)
                    current_data = self.serial_port_node.GetParameter("Response")
                    if response != current_data:       
                        self.serial_port_node.SetParameter("Response", response)
                    
                    return response
            except serial.SerialException as e:
                print(f"Error reading response: {e}")
                return None
        else:
            print("Serial port is not open.")   
            return None
        
    def readLastResponse(self) -> Optional[tuple[str, list[str]]]:
        """Read the response from the serial port"""
        if self.isPortConnected():
            self.all_responses = []
            try:
                while self.serial_port.in_waiting > 0:
                    read = self.serial_port.read_until(self.delimiter) if self.delimiter else self.serial_port.readline()
                    response = self.__decodeResponse(read)
                    if response:  # Only add non-empty responses
                        self.all_responses.append(response)
                
                if len(self.all_responses) > 0:
                    # Use the last response as the main response, but log all
                    self.last_response = self.all_responses[-1]
                    # Update node parameter only if response changed
                    current_data = self.serial_port_node.GetParameter("Last Response")
                    if self.last_response != current_data:       
                        self.serial_port_node.SetParameter("Last Response", self.last_response)
                    
                    return self.last_response
            except serial.SerialException as e:
                print(f"Error reading response: {e}")
                return None
        else:
            print("Serial port is not open.")   
            return None

    def setDelimiter(self, delimiter: str):
        """Set the delimiter for the serial port"""
        self.delimiter = delimiter

    def setDecodeErrors(self, error_handler: str):
        """Change error handling behavior"""
        self.decode_errors = error_handler

    def listAvailablePorts(self):
        """List all available serial ports"""
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def isPortConnected(self) -> bool:
        """Check if serial port is connected and open"""
        return  self.serial_port  and self.serial_port.is_open
    
    def __setNode(self):
        """Set the node for serial port communication"""
        self.serial_port_node = slicer.vtkMRMLScriptedModuleNode()
        while True:
            node_name = f"SerialPort[{self.id}]"
            if not slicer.mrmlScene.GetFirstNodeByName(node_name):
                self.serial_port_node.SetName(node_name)
                break
        slicer.mrmlScene.AddNode(self.serial_port_node)
    
    def __decodeResponse(self, raw_data: bytes) -> str:
        """Helper method with configurable error handling"""
        return raw_data.decode("utf-8", errors=self.decode_errors).strip()


    
    def __del__(self):
        """Cleanup when object is destroyed"""
        self.closeSerialPort()

