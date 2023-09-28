import socket
import struct

# Server address
server_address = ('127.0.0.1', 8888)

# Create a socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the server
try:
    client_socket.connect(server_address)
    print("Connected to the server.")

    while True:
        data = client_socket.recv(4 * 3 * 35)  # Receive 3 subarrays of 35 integers
        if not data:
            break

        # Unpack the received data into arrays
        random_numbers = struct.unpack('105i', data)

        # Convert negative numbers to their correct values (handling signed integers)
        subarrays = [[num if num >= 0 else num + (1 << 32) for num in random_numbers[i:i + 35]] for i in range(0, len(random_numbers), 35)]
        
        print("Received Coordinates:")
        for i, subarray in enumerate(subarrays):
            print(f"Subarray {i + 1}: {subarray}")

finally:
    # Close the socket
    client_socket.close()
