# 🧭 Multiplicative Extended Kalman Filter (MEKF) for IMU Orientation Estimation

This repo serves as the implementation of the Multiplicative Extended Kalman Filter for robust 3D orientation tracking using IMU (Inertial Measurement Unit) sensor data. This implementation fuses gyroscope and accelerometer measurements from an MPU6050 to provide accurate attitude estimates while compensating for sensor biases. Please see the repo: https://github.com/AhmedKhan04/Arduino-MEKF-Logger, to see the embedded code onboard the Arduino Nano and MPU6050 to stream values. 

## 🎥 Demo

https://github.com/user-attachments/assets/061fe5ba-ee56-49d2-8dc1-e96d47a8d8da

*Real-time MEKF orientation tracking with 3D visualization*

---

## 🌟 Features

- **Real-time orientation estimation** using quaternion-based MEKF
- **Gyroscope bias estimation** for improved long-term accuracy
- **3D visualization** showing body frame axes, predicted gravity, and measured acceleration
- **Tunable filter parameters** for different sensor characteristics and application needs
- **Serial communication interface plotting** for direct IMU hardware integration

---

## 📐 Theory

The Multiplicative Extended Kalman Filter (MEKF) is a variant of the standard Extended Kalman Filter (EKF) designed specifically for attitude estimation. Unlike the EKF, MEKF represents orientation using quaternions and estimates small perturbations from the reference quaternion, which:

- Avoids singularities inherent in Euler angle representations
- Maintains quaternion normalization naturally
- Reduces computational complexity
- Provides better numerical stability

### State Vector

The MEKF estimates a 6-dimensional state vector:

```
x = [δθ; δb]
```

Where:
- `δθ` (3×1): Small-angle attitude error vector
- `δb` (3×1): Gyroscope bias error vector

### Measurement Model

The accelerometer measurement model assumes that, when stationary or in uniform motion, the only acceleration measured is gravity:

```
a_measured = R_body_to_inertial^T * g_inertial + noise
```

Where `R` is the rotation matrix derived from the estimated quaternion and `g_inertial = [0, 0, -9.81]^T`.

### Filter Cycle

1. **Prediction**: Propagate quaternion and covariance using gyroscope measurements
2. **Update**: Correct estimate using accelerometer measurements
3. **Reset**: Apply corrections to quaternion and reset error state

---

### For a detailed mathematical treatment, see:
📄 Cortiella, Alexandre & Vidal, David & Jane, Jaume & Juan, Enric & Olivé, Roger & Amézaga, Adrià & Munoz-Martin, Joan & Carreno-Luengo, Hugo & Camps, Adriano. (2017). 3Cat-2: Attitude Determination and Control System for a GNSS-R Earth Observation 6U CubeSat Mission. European Journal of Remote Sensing. 49. 759-776. 10.5721/EuJRS20164940. 

---

## 🚀 Getting Started

### Prerequisites

```bash
pip install numpy numpy-quaternion matplotlib pyserial
```

### Hardware Requirements

- IMU sensor with serial output (e.g., MPU6050, BNO055, ICM-20948)
- USB-to-Serial adapter (if needed)
- Arduino or microcontroller for sensor interface

### Installation

```bash
git clone https://github.com/AhmedKhan04/MEKF_REPO.git
cd MEKF_REPO
```

### Configuration

Edit the serial port and baud rate in `MEKF.py`:

```python
self.baud = 115200
self.port = "COM11"  # Change to your port (e.g., /dev/ttyUSB0 on Linux)
```

### Running

```python
python MEKF.py
```

The visualization will start after a 15-second initialization period to collect sensor data.

---

## ⚙️ Tuning Parameters

### Process Noise (Gyroscope)

```python
self.sigma_gyro = 0.05  # Gyroscope noise (rad/s)
self.sigma_bias = 1e-4  # Bias random walk
```

**Higher values** → Filter trusts gyro less, responds faster to accelerometer corrections  
**Lower values** → Filter trusts gyro more, smoother but slower to correct

### Measurement Noise (Accelerometer)

```python
sigma_acc = 0.01  # Accelerometer noise (m/s²)
```

**Higher values** → Filter ignores accelerometer more, slower convergence  
**Lower values** → Filter trusts accelerometer more, faster convergence but more noise


## 📊 Understanding the Visualization

The 3D plot shows:

- **🔴 Red Arrow (Body X)**: IMU's X-axis direction
- **🟢 Green Arrow (Body Y)**: IMU's Y-axis direction  
- **🔵 Blue Arrow (Body Z)**: IMU's Z-axis direction
- **🟠 Orange Arrow (Pred Gravity)**: Where the filter expects gravity based on current orientation estimate
- **🟣 Magenta Arrow (Meas Accel)**: Actual accelerometer reading (normalized)

When stationary, orange and magenta arrows should align perfectly. 

## 🔧 Troubleshooting

### Slow Convergence
- Decrease `sigma_acc` (trust accelerometer more)
- Increase `sigma_gyro` (trust gyro less)

### Jittery/Noisy Output
- Increase `sigma_acc` (filter accelerometer noise)
- Decrease `sigma_gyro` (trust gyro more)

### Body Axes Point Wrong Direction
- Check coordinate frame conventions between your IMU and the code
- Try transposing the rotation matrix or flipping gravity reference sign

