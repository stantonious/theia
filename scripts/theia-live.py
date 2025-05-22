import socket
import struct
import lz4.block
import time
import csv
import os
import math
import threading
import sys
import pathlib


# Define the server's address and port
SERVER_ADDRESS = ('10.0.0.62', 57345)

SAMPLE_DEPTH = 8
# Create a UDP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    print(f"Starting capture {time.time()}")
    capturing = False
    rec_time = 0.
    capture_cnt=0

    message = sys.argv[1]
    label = sys.argv[2]
    client_socket.sendto(message.encode('utf-8'), SERVER_ADDRESS)
    client_socket.setblocking(0)
    print(f"Sent: {message}")
    # Receive response from the server
    out_dir = f'./data'
    new_dir = f'{out_dir}/live/{label}/{round(time.time(),5)}/'
    pathlib.Path(new_dir).mkdir(parents=True, exist_ok=True)
    h = ['# Time (seconds)'] 
    h.extend([f'f0_f0_f{_n}' for _n in range(SAMPLE_DEPTH)])
    time_step = 0.



    with open(f'{new_dir}/chirp.data','w') as f:
        w = csv.writer(f)
        w.writerow(h)
        t = 0.
        last_capture_time = None
        while 1:
            try:
                data, server_address = client_socket.recvfrom(2048)
                print ('recv d ')
                num_samples = int(len(data)/4) - 1
                samples = list(struct.unpack(f'<L{num_samples}f',data))
                capture_time = samples[0]

                if last_capture_time is None:
                    last_capture_time = capture_time
                    continue
                samples = samples[1:]
                ms_per_sample = (capture_time - last_capture_time) / (num_samples / SAMPLE_DEPTH)
                print ('data rate', (len(samples)/SAMPLE_DEPTH)/((capture_time-last_capture_time)/1000.))

                for _n in range(0,num_samples,SAMPLE_DEPTH):
                    time_step += ms_per_sample
                    n_time = last_capture_time + (_n * ms_per_sample/1000.)
                    w.writerow([time_step/1000.] + samples[_n:_n+SAMPLE_DEPTH])
                last_capture_time = capture_time

            except socket.error as e:
                if e.errno == 11 or e.errno == 10035:
                    continue
                else:
                    print ("exiting ", e)
                    time.sleep(.01)
                    continue

finally:
    # Close the socket
    client_socket.close()
