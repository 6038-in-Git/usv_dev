# streamlit run .\Desktop\python_tests\udp_server_ui.py

import os
import socket
import streamlit as st
import time

import pandas as pd
import numpy as np
import plost
from PIL import Image

cwd = os.getcwd()

# ------------------------------ Page setting ------------------------------
st.set_page_config( page_title = "THE F.I.R.S.T",
					layout = "wide")
st.title("--- Ground Control Station ---")
st.markdown("#")
st.header("USV Data")

localIP     = "192.168.1.30"
localPort   = 6038
bufferSize  = 1024

msgFromServer       = "Hello UDP Client"
bytesToSend         = str.encode(msgFromServer)

usv_data = 0
print_data = 0

section_1 = st.empty()

apoch = 0

# ------------------------------ Create a datagram socket ------------------------------

UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
 
# Bind to address and ip

UDPServerSocket.bind((localIP, localPort))
print("UDP server up and listening")


# ------------------------------ Listen for incoming datagrams ------------------------------
while(True):

	apoch = apoch + 1

	bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
	message = bytesAddressPair[0]
	address = bytesAddressPair[1]
	clientMsg = "Message from Client:{}".format(message)
	clientIP  = "Client IP Address:{}".format(address)
	#print(clientMsg)
	#print(clientIP)

	# Sending a reply to client
	UDPServerSocket.sendto(bytesToSend, address)

	usv_data = int(message[2:5])
	print_data = usv_data * 2 

	section_1.empty()
	with section_1.container():
		st.subheader("Motor 1 = {}".format(print_data))

	os.chdir(cwd)
	with open('.\Desktop\python_tests\motor_data.txt', 'a') as log_file:
		log_file.write('{}. '.format(apoch))
		log_file.write('Motor 1 = {}'.format(print_data))
		log_file.write("\n")
		print(log_file.tell())

	if not message:
		break