from collections import deque
import serial
import struct
import threading
import time

# Gyroscope class to read data from serial port

class Gyro:
    def __init__(self, port_name, baud_rate):
        MAX_LEN = 1000
        self.t_data = deque(maxlen=MAX_LEN)
        self.ax_data = deque(maxlen=MAX_LEN)
        self.ay_data = deque(maxlen=MAX_LEN)
        self.az_data = deque(maxlen=MAX_LEN)
        self.gx_data = deque(maxlen=MAX_LEN)
        self.gy_data = deque(maxlen=MAX_LEN)
        self.gz_data = deque(maxlen=MAX_LEN)
        
        self.port_name = port_name
        self.baud_rate = baud_rate
    
    def read_serial_port(self):
        
        def read_serial_port_val(port_name, baud_rate):
            try:
            
                ser = serial.Serial(self.port_name, self.baud_rate, timeout=1)
                time.sleep(3)  # Wait for the serial connection to initialize
                print(f"Connected to {ser.portstr}")

                while True:

                    encoded_data = ser.read(1)
                    #print(f"Encoded data: {encoded_data}")
                    if (encoded_data == b'\xAA') :
                        raw = ser.read(28)
                        if len(raw) != 28:
                            print("Incomplete data packet received.")
                            continue
                        t, ax, ay, az, gx, gy, gz = struct.unpack('<I6f', raw)
                        self.t_data.append(t)
                        self.ax_data.append(ax)
                        self.ay_data.append(ay)
                        self.az_data.append(az)
                        self.gx_data.append(gx)
                        self.gy_data.append(gy)
                        self.gz_data.append(gz)
                        #ani = animation.FuncAnimation(fig, update_plot, fargs=( t_data, ax_data, ay_data, az_data, gx_data, gy_data, gz_data, ax1), interval=1000)
                        #plt.show()
                        #print(f"t={t}, ax={ax:.3f}, ay={ay:.3f}, az={az:.3f}, "f"gx={gx:.3f}, gy={gy:.3f}, gz={gz:.3f}")
                
                

            except serial.SerialException as e:
                print(f"Error opening or communicating with serial port: {e}")
            except KeyboardInterrupt:
                print("Program terminated by user.")
            finally:
                if ser and ser.is_open:
                    ser.close()
                    print(f"Serial port {ser.port} closed.")

        thread = threading.Thread(target=read_serial_port_val, args=(self.port_name, self.baud_rate), daemon=True)
        thread.start()
