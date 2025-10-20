from .communication_serial import SerialCommunication
from .communicaton_TCPIP import TCPIPCommunication
from .communication_UDP import UDPCommunication


class CommunicationManager:
    def __init__(self):
        self.communications = {}
    
    def createCommunication(self, communication_id,protocol='serial', port=None, baudrate=9600, ip=None, remote_port=None):
        """Create and configure a communication instance"""
        if protocol == 'TCP/IP':
            if ip is None or port is None:
                raise ValueError("IP and port are required for TCP/IP communication")
        elif protocol == 'UDP':
            if ip is None or port is None:
                raise ValueError("IP and port are required for UDP communication")
        if protocol == 'serial':
            communication = SerialCommunication()
            if communication.connect_serial_port(port, baudrate):
                self.communications[communication_id] = communication
                return communication
            return None
        elif protocol == 'TCP/IP':
            communication = TCPIPCommunication()
            if communication.connect(ip, port):
                self.communications[communication_id] = communication
                return communication
            return None
        elif protocol == 'UDP':
            communication = UDPCommunication()
            if communication.connect(ip, port):
                self.communications[communication_id] = communication
                return communication
            return None
        else:
            raise ValueError(f"Invalid protocol: {protocol}")
    
    def getCommunication(self, communication_id):
        """Get a specific communication instance"""
        return self.communications.get(communication_id)
    
    def cleanupAll(self):
        """Clean up all communications"""
        for communication in self.communications.values():
            communication.close_serial_port()
        self.communications.clear()