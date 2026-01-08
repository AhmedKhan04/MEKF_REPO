import serial
import time 
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading
import Gyro


#Script to plot data from serial port in real-time

def update_plot(fig, t, ax, ay, az, gx, gy, gz, ax1):
    for row in ax1:
        for ax_subplot in row:
            ax_subplot.clear()

    ax1[0,0].plot(t, ax)
    ax1[0,0].set_xlabel('Time')
    ax1[0,0].set_ylabel('Acceleration X')
    ax1[0,1].plot(t, ay)
    ax1[0,1].set_xlabel('Time')
    ax1[0,1].set_ylabel('Acceleration Y')
    ax1[0,2].plot(t, az)
    ax1[0,2].set_xlabel('Time')
    ax1[0,2].set_ylabel('Acceleration Z')
    ax1[1,0].plot(t, gx)
    ax1[1,0].set_xlabel('Time')
    ax1[1,0].set_ylabel('Gyroscope X')
    ax1[1,1].plot(t, gy)
    ax1[1,1].set_xlabel('Time')
    ax1[1,1].set_ylabel('Gyroscope Y')
    ax1[1,2].plot(t, gz)
    ax1[1,2].set_xlabel('Time')
    ax1[1,2].set_ylabel('Gyroscope Z')
    return 

if __name__ == "__main__":
    port = "COM11" 
    baud = 115200 
    #read_serial_port(port, baud)

    gyro = Gyro.Gyro(port, baud)
    gyro.read_serial_port()
    
    fig, ax1 = plt.subplots(nrows=2, ncols=3, figsize=(10, 8))
    plt.tight_layout()
    ani = animation.FuncAnimation(fig, update_plot, fargs=(gyro.t_data, gyro.ax_data, gyro.ay_data, gyro.az_data, gyro.gx_data, gyro.gy_data, gyro.gz_data, ax1), interval=5)
    plt.show()
    