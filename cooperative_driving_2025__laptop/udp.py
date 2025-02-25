# Simple usd server on port 2121 to listen and print out
# the data received from the client

import socket

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the port
server_address = ('0.0.0.0', 2121)
print('starting up on %s port %s' % server_address)
sock.bind(server_address)

while True:
    try:
        print(f'waiting to receive message')
        data, address = sock.recvfrom(4096)

        print(f'received {len(data)} bytes from {address}')
        print(data)
    except KeyboardInterrupt:
        print('done')
        break


