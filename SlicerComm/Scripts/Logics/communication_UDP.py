import socket
import time
from typing import Optional, List, Union, Tuple
import qt
import slicer

class UDPCommunication:
    """Generalized UDP communication class for any device"""
    
    def __init__(self, timer_interval=100, check_buffer=True):
        self.local_ip = '0.0.0.0'  # Listen on all interfaces
        self.local_port = None
        self.remote_ip = None
        self.remote_port = None
        self.terminator = '\r'  # Default terminator
        self.is_connected = False
        self.socket = None
        self.check_buffer = check_buffer
        self.chunk_size = 1024
        self.timeout = 5.0
        # Response buffer
        self.response_buffer = None
        
        # Node for Slicer integration
        self._setNode()
    
    def __del__(self):
        """Cleanup when object is destroyed"""
        self.disconnect()

    def _setNode(self):
        """Set the node for UDP communication"""
        udp_node_count = 0
        self.udp_node = slicer.vtkMRMLScriptedModuleNode()
        while True:
            node_name = f"UDP Communication {udp_node_count}"
            if not slicer.mrmlScene.GetFirstNodeByName(node_name):
                self.udp_node.SetName(node_name)
                break
            udp_node_count += 1
        slicer.mrmlScene.AddNode(self.udp_node)
    
    def bind(self, local_ip: str = '0.0.0.0', local_port: int = 0) -> bool: # bind to a local address and port for receiving
        """Bind to a local address and port for receiving"""
        try:
            if self.socket:
                self.disconnect()
            
            self.local_ip = local_ip
            self.local_port = local_port
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind((self.local_ip, self.local_port))
            
            # Get the actual port if we bound to port 0
            if local_port == 0:
                self.local_port = self.socket.getsockname()[1]
            
            self.is_connected = True
            print(f"UDP bound to {self.local_ip}:{self.local_port}")
            return True
            
        except Exception as e:
            print(f"Failed to bind UDP socket: {e}")
            self.is_connected = False
            return False
    
    def connect(self, remote_ip: str, remote_port: int, local_ip: str = '0.0.0.0', 
                local_port: int = 0) -> bool: # connect to a remote UDP endpoint
        """Connect to a remote UDP endpoint"""
        try:
            if self.socket:
                self.disconnect()
            
            self.remote_ip = remote_ip
            self.remote_port = remote_port
            self.local_ip = local_ip
            self.local_port = local_port
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind((self.local_ip, self.local_port))
            
            # Get the actual port if we bound to port 0
            if local_port == 0:
                self.local_port = self.socket.getsockname()[1]
            
            self.is_connected = True
            print(f"UDP connected to {self.remote_ip}:{self.remote_port} from {self.local_ip}:{self.local_port}")
            return True
            
        except Exception as e:
            print(f"Failed to connect UDP: {e}")
            self.is_connected = False
            return False

    def disconnect(self):
        """Disconnect from the UDP device"""
        if self.socket:
            self.socket.close()
            self.socket = None
            self.is_connected = False
            print("UDP disconnected")
            return True
        else:
            print("No UDP connection to close")
            return False
    
    def sendData(self, data, remote_ip: str, remote_port: int) -> Optional[str]:
        """Send data to the device (with fragmentation for large data)"""
        try:
            if not self.is_connected or not self.socket or not self.remote_ip:
                raise Exception("UDP connection is not properly configured.")
            
            # Convert data to bytes
            if isinstance(data, str):
                data_bytes = data.encode()
            elif isinstance(data, bytes):
                data_bytes = data
            else:
                data_bytes = str(data).encode()
            
            # Check if data needs fragmentation
            max_udp_size = 1400  # Safe UDP datagram size
            
            if len(data_bytes) <= max_udp_size:
                # Small data: send directly
                self.socket.sendto(data_bytes, (self.remote_ip, self.remote_port))

            else:
                # Large data: fragment and send
                self._sendFragmentedData(data_bytes)
            
        except Exception as e:
            print(f"UDP communication error: {e}")
            return None
    
    def _sendFragmentedData(self, data_bytes: bytes):
        """Send large data by fragmenting into smaller UDP packets"""
        max_udp_size = 1400
        total_size = len(data_bytes)
        num_fragments = (total_size + max_udp_size - 1) // max_udp_size
        
        print(f"Fragmenting {total_size} bytes into {num_fragments} UDP packets")
        
        # Send header with fragmentation info
        header = f"FRAG:{num_fragments}:{total_size}:".encode()
        self.socket.sendto(header, (self.remote_ip, self.remote_port))
        
        # Send data fragments
        for i in range(num_fragments):
            start = i * max_udp_size
            end = min(start + max_udp_size, total_size)
            fragment = data_bytes[start:end]
            
            # Add fragment header
            frag_header = f"PART:{i}:{len(fragment)}:".encode()
            packet = frag_header + fragment
            
            self.socket.sendto(packet, (self.remote_ip, self.remote_port))
            print(f"Sent fragment {i+1}/{num_fragments} ({len(fragment)} bytes)")
        
        # Send end marker
        end_marker = "END:".encode()
        self.socket.sendto(end_marker, (self.remote_ip, self.remote_port))
        print("Fragmentation complete")
    
    def readBinaryData(self,verbose: bool = True) -> Optional[bytes]:
        """Read binary data from the device (with fragmentation support)"""
        if not self.is_connected or not self.socket:
            return None
        
        try:
            # Read first packet to check if it's fragmented
            data, addr = self.socket.recvfrom(self.chunk_size)
            return data
            
        except socket.timeout:
            if verbose:
                print(f"UDP read timeout after {self.timeout} seconds")
            return None
            
        except Exception as e:
            if verbose:
                print(f"UDP read error: {e}")
            return None
    
    def _receiveFragmentedData(self, timeout: float) -> Optional[bytes]:
        """Receive and reassemble fragmented data"""
        try:
            # Read fragmentation header
            header_data, addr = self.socket.recvfrom(self.chunk_size)
            header = header_data.decode('utf-8')
            
            if not header.startswith("FRAG:"):
                print("Invalid fragmentation header")
                return None
            
            # Parse header: FRAG:num_fragments:total_size:
            parts = header.split(":")
            num_fragments = int(parts[1])
            total_size = int(parts[2])
            
            print(f"Receiving {num_fragments} fragments, total size: {total_size} bytes")
            
            # Receive all fragments
            fragments = [None] * num_fragments
            fragments_received = 0
            
            while fragments_received < num_fragments:
                try:
                    self.socket.settimeout(timeout)
                    data, addr = self.socket.recvfrom(self.chunk_size)
                    
                    if data.startswith(b"END:"):
                        break
                    
                    if data.startswith(b"PART:"):
                        # Parse fragment header: PART:index:size:
                        header_part = data[:data.index(b":", 5) + 1]
                        header_str = header_part.decode('utf-8')
                        header_parts = header_str.split(":")
                        fragment_index = int(header_parts[1])
                        fragment_size = int(header_parts[2])
                        
                        # Extract fragment data
                        fragment_data = data[len(header_part):]
                        
                        if fragment_index < num_fragments:
                            fragments[fragment_index] = fragment_data
                            fragments_received += 1
                            print(f"Received fragment {fragment_index + 1}/{num_fragments}")
                    
                except socket.timeout:
                    print(f"Timeout waiting for fragment {fragments_received + 1}")
                    return None
            
            # Reassemble data
            if all(frag is not None for frag in fragments):
                reassembled_data = b''.join(fragments)
                print(f"Successfully reassembled {len(reassembled_data)} bytes")
                return reassembled_data
            else:
                print("Missing fragments, reassembly failed")
                return None
                
        except Exception as e:
            print(f"Error receiving fragmented data: {e}")
            return None
    
    
    def broadcast(self, command: str, port: int) -> bool:
        """Broadcast a command to all devices on a port"""
        try:
            if not self.is_connected or not self.socket:
                raise Exception("UDP socket is not bound.")
            
            print(f"Broadcasting UDP command on port {port}: {command}")
            
            # Send the command with the terminator
            cmd = f'{command}{self.terminator}'.encode()
            self.socket.sendto(cmd, ('<broadcast>', port))
            return True
            
        except Exception as e:
            print(f"UDP broadcast error: {e}")
            return False
    
    def setTimeout(self, timeout: float):
        """Set the timeout for reading data"""
        if self.socket:
            self.timeout = timeout
            self.socket.settimeout(timeout)
        else:
            print("No socket to set timeout")

    def setTerminator(self, terminator: str):
        """Set the command terminator"""
        self.terminator = terminator
    
    def setChunkSize(self, chunk_size: int):
        """Set the chunk size for reading data"""
        self.chunk_size = chunk_size
    
    def isConnected(self) -> bool:
        """Check if UDP connection is active"""
        return self.is_connected and self.socket is not None
    
    def getConnectionInfo(self) -> dict:
        """Get current connection information"""
        return {
            'local_ip': self.local_ip,
            'local_port': self.local_port,
            'remote_ip': self.remote_ip,
            'remote_port': self.remote_port,
            'connected': self.is_connected,
            'terminator': self.terminator
        }
    
    def getSocketInfo(self) -> Tuple[str, int]:
        """Get the actual socket address and port"""
        if self.socket:
            return self.socket.getsockname()
        return (None, None)
    
    def __del__(self):
        """Cleanup when object is destroyed"""
        self.disconnect()

