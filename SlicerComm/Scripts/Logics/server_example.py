#!/usr/bin/env python3
"""
Example demonstrating TCP/IP server functionality for SlicerComm.
This server can receive images and data from clients.
"""

import time
import os
from SlicerComm.Scripts.Logics.communicaton_TCPIP import TCPIPCommunication

def server_example():
    """Example of running a TCP/IP server that can receive images"""
    
    # Create server instance
    server = TCPIPCommunication()
    
    # Start server on port 5005 (same as your sender)
    if not server.startServer(port=5005, host="0.0.0.0"):
        print("Failed to start server")
        return
    
    print("Server started successfully!")
    print("Waiting for client connections...")
    
    try:
        # Wait for client connection (blocking)
        if server.waitForClient(timeout=30.0):  # Wait up to 30 seconds
            print("Client connected!")
            
            # Get client information
            client_info = server.getClientInfo()
            print(f"Client connected from: {client_info}")
            
            # Create directory for received images
            os.makedirs("received_images", exist_ok=True)
            
            # Receive images from client
            image_count = 0
            while True:
                print(f"\n--- Waiting for image {image_count + 1} ---")
                
                # Read image from client
                image_data = server.readImage(
                    save_path=f"received_images/image_{image_count + 1}.jpg",
                    timeout=10.0
                )
                
                if image_data:
                    print(f"Successfully received image {image_count + 1} ({len(image_data)} bytes)")
                    image_count += 1
                    
                    # Send acknowledgment to client
                    response = server.sendToClient("IMAGE_RECEIVED", expect_response=False)
                    
                    # Ask if client wants to send more
                    more_data = server.sendToClient("SEND_MORE?", expect_response=True, response_timeout=2.0)
                    if more_data and more_data.lower() in ['no', 'n', 'stop']:
                        print("Client indicated no more images")
                        break
                else:
                    print("No image received or timeout")
                    break
                
                time.sleep(1)  # Brief pause between images
            
            print(f"Total images received: {image_count}")
            
        else:
            print("No client connected within timeout period")
    
    except KeyboardInterrupt:
        print("\nServer stopped by user")
    except Exception as e:
        print(f"Server error: {e}")
    finally:
        # Clean up
        server.stopServer()
        print("Server cleanup completed")

def non_blocking_server_example():
    """Example of non-blocking server that checks for clients periodically"""
    
    server = TCPIPCommunication()
    
    if not server.startServer(port=5006, host="0.0.0.0"):
        print("Failed to start server")
        return
    
    print("Non-blocking server started on port 5006")
    print("Checking for clients every 1 second...")
    
    try:
        check_count = 0
        while check_count < 60:  # Check for 60 seconds
            # Non-blocking check for client
            if server.checkForClient(timeout=0.001):
                print("Client connected!")
                
                # Handle client communication
                handle_client_communication(server)
                break
            
            check_count += 1
            time.sleep(1)
            
            if check_count % 10 == 0:
                print(f"Still waiting for client... ({check_count}s)")
    
    except KeyboardInterrupt:
        print("\nServer stopped by user")
    finally:
        server.stopServer()
        print("Server cleanup completed")

def handle_client_communication(server):
    """Handle communication with a connected client"""
    
    # Create directory for received data
    os.makedirs("received_data", exist_ok=True)
    
    try:
        # Receive binary data (could be image, file, etc.)
        print("Waiting for data from client...")
        data = server.readBinaryData(timeout=10.0)
        
        if data:
            # Save the received data
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"received_data/data_{timestamp}.bin"
            
            with open(filename, 'wb') as f:
                f.write(data)
            
            print(f"Received {len(data)} bytes, saved to {filename}")
            
            # Send acknowledgment
            server.sendToClient("DATA_RECEIVED_OK")
        else:
            print("No data received")
            server.sendToClient("NO_DATA_RECEIVED")
    
    except Exception as e:
        print(f"Error handling client communication: {e}")

def server_with_multiple_clients():
    """Example showing how to handle multiple clients (basic implementation)"""
    
    server = TCPIPCommunication()
    
    if not server.startServer(port=5007, host="0.0.0.0", max_connections=5):
        print("Failed to start server")
        return
    
    print("Multi-client server started on port 5007")
    print("Note: This is a basic implementation - only handles one client at a time")
    
    try:
        while True:
            print("Waiting for client connection...")
            
            if server.waitForClient(timeout=30.0):
                client_info = server.getClientInfo()
                print(f"Client connected: {client_info}")
                
                # Handle this client
                handle_client_communication(server)
                
                # Disconnect this client to accept the next one
                server.disconnect()
                print("Client disconnected, ready for next client")
            else:
                print("No client connected, stopping server")
                break
    
    except KeyboardInterrupt:
        print("\nServer stopped by user")
    finally:
        server.stopServer()

if __name__ == "__main__":
    print("TCP/IP Server Examples")
    print("=" * 50)
    
    print("\n1. Basic server example (receives images)")
    print("2. Non-blocking server example")
    print("3. Multi-client server example")
    
    choice = input("\nSelect example (1-3): ").strip()
    
    if choice == "1":
        server_example()
    elif choice == "2":
        non_blocking_server_example()
    elif choice == "3":
        server_with_multiple_clients()
    else:
        print("Invalid choice, running basic server example")
        server_example()
