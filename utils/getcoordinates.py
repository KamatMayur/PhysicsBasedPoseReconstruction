import socket
import struct

# Server address
def get_coordinates():
    server_address = ('127.0.0.1', 8888)

    # Create a socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Connect to the server
    try:
        client_socket.connect(server_address)
        print("Connected to the server.")
        data = client_socket.recv(4)  # Receive 3 subarrays of 35 integers
        if not data:
            exit()

        jnt_num = int.from_bytes(data, byteorder='little')

        data = client_socket.recv(4 + 24*jnt_num + 8*jnt_num*3)  # Receive 3 subarrays of 35 integers
        if not data:
            exit()

        print(jnt_num , '\n')
        data = data[4:]

        print(data)

        # jnt_names = []
        # for _ in range(jnt_num):
        #     # Assuming strings are null-terminated
        #     string, data = data.split(b'\0', 1)
        #     jnt_names.append(string.decode('utf-8'))

        # num_doubles = jnt_num * 8  # Each double is 8 bytes
        # x = struct.unpack(f"!{jnt_num}d", data[:num_doubles])
        # data = data[num_doubles:]
        # y = struct.unpack(f"!{jnt_num}d", data[:num_doubles])
        # data = data[num_doubles:]
        # z = struct.unpack(f"!{jnt_num}d", data[:num_doubles])
        # print(jnt_names)
        # print(x, y, z)

            # # Unpack the received data into arrays
            # random_numbers = struct.unpack('105i', data)

            # # Convert negative numbers to their correct values (handling signed integers)
            # subarrays = [[num if num >= 0 else num + (1 << 32) for num in random_numbers[i:i + 35]] for i in range(0, len(random_numbers), 35)]
            
            # print("Received Coordinates:")
            # for i, subarray in enumerate(subarrays):
            #     print(f"Subarray {i + 1}: {subarray}")
            

    finally:
        # Close the socket
        client_socket.close()
