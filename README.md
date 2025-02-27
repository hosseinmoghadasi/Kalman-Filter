# Kalman Filter from Scratch in C++

## What is a Kalman Filter?  
A **Kalman Filter** is an algorithm that estimates the state of a system (e.g., position and velocity) by combining predictions from a mathematical model with noisy measurements. It is widely used in **tracking, robotics, computer vision, and navigation**.

#### How It Works  
The Kalman Filter operates in two main steps:  

1. **Prediction**: Estimates the next state based on a motion model.  
2. **Update**: Adjusts the estimate using a new measurement, weighted by the **Kalman Gain**, which balances uncertainty from the model and sensor noise.  

### Why Use It?  
✅ **Reduces noise** in sensor data  
✅ **Optimally combines** multiple sources of information  
✅ **Efficient** and works in real-time  

....................................................................
## Implemetations 

#### 1. 1D Kalman Filter for Position Estimation with Constant Velocity  
This C++ program implements a **1D Kalman Filter** to estimate the position of an object moving with a **constant velocity**.  

#### Key Points:  
- The **velocity (\( v \)) remains constant** throughout the process.  
- The **covariance (\( P \)) increases** due to **process noise (\( Q \))**.  
- The **Kalman Gain (\( K \))** determines how much the measurement influences the estimate.  

#### 🚀 Code Implementation  
Check out the full **C++ implementation** in the [`KF_1D_Position_Tracking.cpp`](./KF_1D_Position_Tracking.cpp) file.  

....................................................................

#### 2. 2D Kalman Filter for Motion Tracking
This version extends the Kalman Filter to 2D space, tracking both X and Y coordinates along with their respective velocities.

#### Key Points:  
- Tracks position and velocity in X and Y directions.  
- Uses a constant velocity motion model.
- Measurement updates improve accuracy by reducing sensor noise.

#### 🚀 Code Implementation  
Check out the full **C++ implementation** in the [`KF_2D_Motion_Tracking.cpp`](./KF_2D_Motion_Tracking.cpp) file.  
