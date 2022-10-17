import socket
import streamlit as st

 
localIP     = "192.168.1.30"
localPort   = 6038
bufferSize  = 1024

msgFromServer       = "Hello UDP Client"
bytesToSend         = str.encode(msgFromServer)

usv_data = 0
print_data = 0

# Create a datagram socket

UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
 
# Bind to address and ip

UDPServerSocket.bind((localIP, localPort))
print("UDP server up and listening")

 
# Listen for incoming datagrams
while(True):

    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    message = bytesAddressPair[0]
    address = bytesAddressPair[1]
    clientMsg = "Message from Client:{}".format(message)
    clientIP  = "Client IP Address:{}".format(address)
    print(clientMsg)
    print(clientIP)

    # Sending a reply to client
    UDPServerSocket.sendto(bytesToSend, address)

    usv_data = int(message[2:4])
    print_data = usv_data * 2 
    st.write(print_data)

    if not message:
        break