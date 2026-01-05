import serial
import time 
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading

MAX_LEN = 1000
t_data = deque(maxlen=MAX_LEN)
ax_data = deque(maxlen=MAX_LEN)
ay_data = deque(maxlen=MAX_LEN)
az_data = deque(maxlen=MAX_LEN)
gx_data = deque(maxlen=MAX_LEN)
gy_data = deque(maxlen=MAX_LEN)
gz_data = deque(maxlen=MAX_LEN)


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

def read_serial_port(port_name, baud_rate): 
    

    try:
    
        ser = serial.Serial(port_name, baud_rate, timeout=1)
        time.sleep(3)  # Wait for the serial connection to initialize
        print(f"Connected to {ser.portstr}")

        while True:

            encoded_data = ser.read(1)
            print(f"Encoded data: {encoded_data}")
            if (encoded_data == b'\xAA') :
                raw = ser.read(28)
                if len(raw) != 28:
                    print("Incomplete data packet received.")
                    continue
                t, ax, ay, az, gx, gy, gz = struct.unpack('<I6f', raw)
                t_data.append(t)
                ax_data.append(ax)
                ay_data.append(ay)
                az_data.append(az)
                gx_data.append(gx)
                gy_data.append(gy)
                gz_data.append(gz)
                #ani = animation.FuncAnimation(fig, update_plot, fargs=( t_data, ax_data, ay_data, az_data, gx_data, gy_data, gz_data, ax1), interval=1000)
                #plt.show()
                print(f"t={t}, ax={ax:.3f}, ay={ay:.3f}, az={az:.3f}, "f"gx={gx:.3f}, gy={gy:.3f}, gz={gz:.3f}")
        
        

    except serial.SerialException as e:
        print(f"Error opening or communicating with serial port: {e}")
    except KeyboardInterrupt:
        print("Program terminated by user.")
    finally:
        if ser and ser.is_open:
            ser.close()
            print(f"Serial port {ser.port} closed.")

if __name__ == "__main__":
    port = "COM11" 
    baud = 115200 
    #read_serial_port(port, baud)
    
    fig, ax1 = plt.subplots(nrows=2, ncols=3, figsize=(10, 8))
    plt.tight_layout()

   

    thread = threading.Thread(target=read_serial_port, args=(port, baud), daemon=True)
    thread.start()


    ani = animation.FuncAnimation(fig, update_plot, fargs=( t_data, ax_data, ay_data, az_data, gx_data, gy_data, gz_data, ax1), interval=5)
    plt.show()
    