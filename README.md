# Kalman Filter From Scratch In C++


##What is a Kalman Filter?
A Kalman Filter is an algorithm that estimates the state of a system (e.g., position and velocity) by combining predictions from a mathematical model with noisy measurements. It is widely used in tracking, robotics, computer vision, and navigation.

##How It Works:
Prediction: Estimates the next state based on a motion model.
Update: Adjusts the estimate using a new measurement, weighted by the Kalman Gain, which balances uncertainty from the model and sensor noise.

##Why Use It?
Reduces noise in sensor data.
Optimally combines multiple sources of information.
Efficient and works in real-time.
In your case, the Kalman Filter is tracking the position of an object moving with a constant velocity.
