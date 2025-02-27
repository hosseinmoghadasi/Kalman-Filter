#include <iostream>
using namespace std;

class KalmanFilter {
public:
    float x;  // Estimated position
    float v;  // Estimated velocity (constant in this model)
    float P;  // Covariance (uncertainty)
    float K;  // Kalman Gain
    float Q;  // Process noise
    float R;  // Measurement noise
    float z;  // New measurement
    float T;  // Time step (e.g., 1 second)

    // Constructor to initialize the Kalman Filter with given parameters
    KalmanFilter(float initial_x, float initial_v, float initial_P, float process_noise, float measurement_noise, float time_step) {
        x = initial_x;
        v = initial_v;
        P = initial_P;
        Q = process_noise;
        R = measurement_noise;
        T = time_step;
    }

    // Prediction step: Estimates the next state based on motion model
    void predict() {
        // Update position estimate using constant velocity model
        x = x + v * T;  // New position
        v = v;  // Velocity remains constant

        // Update the uncertainty (covariance) due to process noise
        P = P + Q;  
    }

    // Update step: Corrects the estimate using a new measurement
    void update(float measurement) {
        z = measurement;  // New measurement

        // Compute Kalman Gain
        K = P / (P + R);

        // Update the position estimate based on the measurement
        x = x + K * (z - x);
        
        // Velocity remains unchanged (constant velocity assumption)
        v = v;

        // Update covariance (uncertainty) after measurement update
        P = (1 - K) * P;

        // Print updated state
        cout << "Updated state (x, v): " << x << ", " << v << endl;
    }
};

int main() {
    // Initialize the Kalman Filter with:
    // Initial position = 0, Initial velocity = 1, Initial uncertainty = 1
    // Process noise = 0.1, Measurement noise = 0.5, Time step = 1 second
    KalmanFilter kf(0, 1, 1, 0.1, 0.5, 1);

    // Example measurements (simulated sensor readings)
    float measurements[] = {1.1, 2.2, 3.1, 3.9, 4.8};

    // Run the Kalman Filter through the measurements
    for (int i = 0; i < 5; ++i) {
        kf.predict();  // Predict next state
        kf.update(measurements[i]);  // Correct state using the measurement
    }

    return 0;
}
