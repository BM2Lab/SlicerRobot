import socket
import time
import struct
from typing import Optional, List, Union
import qt
import slicer

class TCPIPCommunication:
    """Generalized TCP/IP communication class for any device"""
    
    def __init__(self, check_buffer=True):
        self.ip = None
        self.port = None
        self.terminator = '\r'  # Default terminator
        self.is_connected = False
        self.socket = None
        self.check_buffer = check_buffer
        self.chunk_size = 1048576
        self.timeout = 5.0
        # Server mode variables
        self.is_server = False
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        
        # Response buffer
        self.response_buffer = None
        
        # Node for Slicer integration
        self._setNode()
    
    def __del__(self):
        """Cleanup when object is destroyed"""
        self.disconnect()

    
    def _setNode(self):
        """Set the node for TCP/IP communication"""
        tcpip_node_count = 0
        self.tcpip_node = slicer.vtkMRMLScriptedModuleNode()
        while True:
            node_name = f"TCP/IP Communication {tcpip_node_count}"
            if not slicer.mrmlScene.GetFirstNodeByName(node_name):
                self.tcpip_node.SetName(node_name)
                break
            tcpip_node_count += 1
        slicer.mrmlScene.AddNode(self.tcpip_node)
    
    def connect(self, ip: str, port: int, timeout: float = 0.1) -> bool:
        """Connect to the specified IP address and port"""
        try:
            if self.socket:
                self.disconnect()
            
            self.ip = ip
            self.port = port
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(timeout)
            self.timeout = timeout
            # Connect to the provided IP address and port
            self.socket.connect((self.ip, self.port))
            self.is_connected = True
            print(f"Connected to {self.ip} on port {self.port}")
            return True
            
        except socket.timeout:
            print(f"Connection to {self.ip} on port {self.port} timed out.")
            self.is_connected = False
            return False
        except Exception as e:
            print(f"Failed to connect: {e}")
            self.is_connected = False
            return False
        
    def checkForServer(self, timeout: float = 0.001) -> bool:
        """Check for a server on the specified IP address and port"""
        if self.is_connected:
            return True
        try:
            self.socket.settimeout(timeout)
            self.timeout = timeout
            self.socket.connect((self.ip, self.port))
            self.is_connected = True
            return True
        except socket.timeout:
            return False
        except Exception as e:
            print(f"Error checking for server: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the TCP/IP device"""
        if self.is_server:
            return self.stopServer()
        
        if self.socket:
            self.socket.close()
            self.socket = None
            self.is_connected = False
            print("Disconnected")
            return True
        else:
            print("No connection to close")
            return False
    
    def startServer(self, port: int, host: str = "0.0.0.0", max_connections: int = 1) -> bool:
        """Start a TCP/IP server and listen for incoming connections"""
        try:
            if self.is_server and self.server_socket:
                self.stopServer()
            
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # Bind to the specified host and port
            self.server_socket.bind((host, port))
            self.server_socket.listen(max_connections)
            
            self.is_server = True
            self.port = port
            self.ip = host
            self.is_connected = False  # No client connected yet
            
            print(f"Server started on {host}:{port}, waiting for connections...")
            return True
            
        except Exception as e:
            print(f"Failed to start server: {e}")
            self.is_server = False
            self.server_socket = None
            return False
    
    def acceptConnection(self, timeout: float = None) -> bool:
        """Accept an incoming client connection"""
        if not self.is_server or not self.server_socket:
            print("Server not started")
            return False
        
        try:
            if timeout:
                self.server_socket.settimeout(timeout)
            
            print("Waiting for client connection...")
            self.client_socket, self.client_address = self.server_socket.accept()
            self.socket = self.client_socket  # Use client socket for communication
            self.is_connected = True
            
            print(f"Client connected from {self.client_address}")
            return True
            
        except socket.timeout:
            print(f"Timeout: No client connection received within {timeout} seconds")
            return False
        except Exception as e:
            print(f"Error accepting connection: {e}")
            return False
    
    def stopServer(self) -> bool:
        """Stop the server and close all connections"""
        try:
            # Close client connection if exists
            if self.client_socket:
                self.client_socket.close()
                self.client_socket = None
                print("Client connection closed")
            
            # Close server socket
            if self.server_socket:
                self.server_socket.close()
                self.server_socket = None
                print("Server stopped")
            
            self.is_server = False
            self.is_connected = False
            self.socket = None
            self.client_address = None
            
            return True
            
        except Exception as e:
            print(f"Error stopping server: {e}")
            return False
    
    def waitForClient(self, timeout: float = None) -> bool:
        """Wait for a client to connect (blocking)"""
        return self.acceptConnection(timeout)
    
    def isServerMode(self) -> bool:
        """Check if running in server mode"""
        return self.is_server
    
    def getClientInfo(self) -> Optional[dict]:
        """Get information about the connected client"""
        if self.is_connected and self.client_address:
            return {
                'address': self.client_address[0],
                'port': self.client_address[1],
                'connected': self.is_connected
            }
        return None
    
    def checkForClient(self, timeout: float = 0.001) -> bool:
        """Check for incoming client connection (non-blocking)"""
        if not self.is_server or not self.server_socket:
            return False
        
        try:
            self.server_socket.settimeout(timeout)
            self.timeout = timeout
            self.client_socket, self.client_address = self.server_socket.accept()
            self.socket = self.client_socket
            self.is_connected = True
            print(f"Client connected from {self.client_address}")
            return True
        except socket.timeout:
            return False
        except Exception as e:
            print(f"Error checking for client: {e}")
            return False

    
    def sendData(self, data) -> Optional[str]:
        """Send data to the device"""
        try:
            if not self.is_connected or not self.socket:
                raise Exception("TCP/IP connection is not open.")
            
            # Handle both text and binary data
            if isinstance(data, str):
                # Text data: add terminator and encode
                data = f'{data}'.encode()
                self.socket.sendall(data)
            elif isinstance(data, bytes):
                # Binary data: send as-is (no terminator)
                self.socket.sendall(data)
            else:
                # Convert to string and handle as text
                data = f'{str(data)}'.encode()
                self.socket.sendall(data)
        except Exception as e:
            print(f"TCP/IP communication error: {e}")
            return None
    
    def readBinaryData(self) -> Optional[bytes]:
        """Read binary data (like images) from the device"""
        if not self.is_connected or not self.socket:
            return None
        
        try:
            # Read in chunks, but don't exceed remaining bytes
            data = self.socket.recv(self.chunk_size)
            return data
    
        except socket.timeout:
            print(f"Timeout: No binary data received within {self.timeout} seconds")
            return None
        except Exception as e:
            print(f"Error reading binary data: {e}")
            return None

    def setTimeout(self, timeout: float):
        """Set the timeout for reading data"""
        if self.socket:
            self.timeout = timeout
            self.socket.settimeout(timeout)
        else:
            print("No socket to set timeout")

    def setChunkSize(self, chunk_size: int):
        """Set the chunk size for reading data"""
        self.chunk_size = chunk_size
    
    def isConnected(self) -> bool:
        """Check if TCP/IP connection is active"""
        return self.is_connected and self.socket is not None
    
    def getConnectionInfo(self) -> dict:
        """Get current connection information"""
        info = {
            'ip': self.ip,
            'port': self.port,
            'connected': self.is_connected,
            'terminator': self.terminator,
            'is_server': self.is_server
        }
        
        # Add client information if in server mode
        if self.is_server and self.client_address:
            info['client_address'] = self.client_address[0]
            info['client_port'] = self.client_address[1]
        
        return info
    
    def __del__(self):
        self.disconnect()