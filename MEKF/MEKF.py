import numpy as np
import quaternion as qt
import Gyro
import time 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Where to find quoternion information: 
# https://www.reddit.com/r/gamedev/comments/atzyhc/i_put_together_a_easy_to_understand_explanation/#lightbox
# q = w + xi + yj + zk

class MEKF:
    def __init__(self):
        self.dt = 5   # Time step in milliseconds 
        self.q_ref = qt.quaternion(1, 0, 0, 0)  # Reference quaternion
        self.b_hat = np.array([0.0, 0.0, 0.0])  # Estimated bias
        self.x = np.zeros(6)  # State vector: [error angles; bias] 
        self.estimate = qt.quaternion(1, 0, 0, 0)  # Initial orientation estimate
        # ----------- 
        # L = 56 /1000
        # W = 84/1000
        # H = 9.5 / 1000
        # M = 0.01  # Mass in kg
        # Ixx = (1/12) * M * (H**2 + W**2)
        # Iyy = (1/12) * M * (L**2 + H**2)
        # Izz = (1/12) * M * (L**2 + W**2)
        # -------------

        self.a_hat = np.array([0.0, 0.0, 0.0])  # Estimated accelerometer bias

        #self.inertia = np.diag([Ixx, Iyy, Izz])
        self.sigma_gyro = 0.01  # gyro noise rad/s
        self.sigma_bias = 1e-5  # bias random walk
        self.P= np.eye(6) 
        
        self.P[0:3] *=  (self.sigma_gyro**2)
        self.P[3:6] *=  (self.sigma_bias**2)

        sigma_acc = 0.05
        self.R = np.eye(3) * (sigma_acc**2)  # Measurement noise covariance
        self.g_ref = np.array([0, 0, 9.81]).T  # down is +Z
        self.G = np.eye(6)  # Process noise covariance
        #self.G [0:3, 0:3] *= -1 

        self.Phi = np.eye(6)  # State transition matrix
        #self.Phi[0:3, 0:3] = np.eye(3) - self.skew(np.array([0,0,0])) * (self.dt / 1000.0)
         # Initialize Gyro object

        self.R = np.eye(3) * (sigma_acc**2)  # Measurement noise covariance
        
        self.measured_accel = np.array([0.0, 0.0, 0.0])  # Placeholder for measured acceleration

        self.baud = 115200
        self.port = "COM11"
        self.gyro = Gyro.Gyro(self.port, self.baud)  # Initialize Gyro object
        self.gyro.read_serial_port() # Start reading data from serial port


    def skew(self, v):
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])
      

    def predict_state(self):
        # Predict the next state based on gyro measurements
        t, ax, ay, az, gx, gy, gz = self.state_measurements()
        measured_omega = np.array([gx, gy, gz]) - self.b_hat
        self.measured_accel = np.array([ax, ay, az]) - self.a_hat 
        #print(self.measured_accel)
        self.estimate = self.estimate + 0.5 * qt.quaternion(0, *measured_omega) * self.estimate * (self.dt / 1000.0)
        self.estimate = self.estimate.normalized()

        self.Phi[0:3, 0:3] = np.eye(3) - self.skew(measured_omega) * (self.dt / 1000.0)
        self.Phi[0:3, 3:6] = -np.eye(3) * (self.dt / 1000.0)
        self.Phi[3:6, 3:6] = np.eye(3)

        Q_d = np.eye(6)  # Discrete process noise covariance
        Q_d[0:3, 0:3] *= (self.dt / 1000.0) * (self.sigma_gyro**2)
        Q_d[3:6, 3:6] *= (self.dt / 1000.0) * (self.sigma_bias**2)

        self.P = self.Phi @ self.P @ self.Phi.T + self.G @ Q_d @ self.G.T





        
    def update_state(self):
        R_body_to_inertial = qt.as_rotation_matrix(self.estimate)
        self.a_hat = R_body_to_inertial.T @ self.g_ref  # m/s²
        
        # Measurement Jacobian
        self.H = np.zeros((3, 6))
        self.H[0:3, 0:3] = -self.skew(self.a_hat)
        # Kalman gain
        S = self.H @ self.P @ self.H.T + self.R
        self.K = self.P @ self.H.T @ np.linalg.inv(S)
        
        
        r = self.measured_accel - self.a_hat  # Residual
        
        # Check if we should trust this measurement (no dynamic accel)
        # accel_magnitude = np.linalg.norm(self.measured_accel)
        # if abs(accel_magnitude - 9.81) > 3.0:  # More than 3 m/s² off
        #     print(f"Skipping update - dynamic accel detected: {accel_magnitude:.2f}")
        #     return  # Don't update, just use prediction
        
        # Apply correction
        self.x = self.K @ r
        delta_theta = self.x[0:3]
        delta_b = self.x[3:6]
        
        delta_q = qt.quaternion(1, *(0.5 * delta_theta))
        self.estimate = (self.estimate * delta_q).normalized()
        self.b_hat += delta_b
        
        # Covariance update
        self.P = (np.eye(6) - self.K @ self.H) @ self.P @ (np.eye(6) - self.K @ self.H).T + self.K @ self.R @ self.K.T
            


        

      

    
    def state_measurements(self):
        # Read measurements from sensors
        return self.gyro.t_data[-1], self.gyro.ax_data[-1], self.gyro.ay_data[-1], self.gyro.az_data[-1], self.gyro.gx_data[-1], self.gyro.gy_data[-1], self.gyro.gz_data[-1]

    def MEKF_compiler(self):
        while True:
            self.predict_state()
            self.update_state()
            print(self.estimate)
            time.sleep(self.dt / 1000.0)  # Sleep for dt milliseconds
        

    def animate_compiler(self, MEKF_instance):
            fig = plt.figure(figsize=(7, 7))
            ax = fig.add_subplot(111, projection='3d')
            
            def animate(i):
                # Run one MEKF step
                MEKF_instance.predict_state()
                MEKF_instance.update_state()

                q_est = MEKF_instance.estimate
                
                
                a_pred = MEKF_instance.a_hat
                
                
                a_meas_norm = MEKF_instance.measured_accel / np.linalg.norm(MEKF_instance.measured_accel)

                
                R_mat = qt.as_rotation_matrix(q_est)

               
                body_x_vec = R_mat[:, 0]
                body_y_vec = R_mat[:, 1]
                body_z_vec = R_mat[:, 2]

                # Clear and redraw
                ax.cla()
                ax.set_xlim([-1, 1])
                ax.set_ylim([-1, 1])
                ax.set_zlim([-1, 1])
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.set_title('MEKF 3D Orientation')

                # Draw body frame axes (RED=X, GREEN=Y, BLUE=Z)
                ax.quiver(0, 0, 0, *body_x_vec, color='r', length=0.5, normalize=True, label='Body X')
                ax.quiver(0, 0, 0, *body_y_vec, color='g', length=0.5, normalize=True, label='Body Y')
                ax.quiver(0, 0, 0, *body_z_vec, color='b', length=0.5, normalize=True, label='Body Z')

                # Predicted gravity (ORANGE)
                ax.quiver(0, 0, 0, *a_pred, color='orange', length=0.6, normalize=True, 
                        arrow_length_ratio=0.2, linewidth=2, label='Pred Gravity')

                # Measured accelerometer (MAGENTA)
                ax.quiver(0, 0, 0, *a_meas_norm, color='magenta', length=0.6, normalize=True, 
                        arrow_length_ratio=0.2, linewidth=2, label='Meas Accel')
                
                ax.legend(loc='upper right', fontsize=8)
                
                return ax,

            anim = FuncAnimation(fig, animate, interval=MEKF_instance.dt, blit=False)
            plt.show()


MEKF_instance = MEKF()
time.sleep(15)  # Wait for some data to be collected
MEKF_instance.animate_compiler(MEKF_instance) # Start the animation
