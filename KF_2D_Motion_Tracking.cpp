#include <iostream>  // Standard I/O Library
using namespace std;

// Class definition for a 2D Kalman Filter
class KalmanFilter2D {
public:
    double x, vx, y, vy; // Estimated position (x, y) and velocity (vx, vy)
    double P[4][4];      // Covariance matrix
    double A[4][4];      // State transition matrix
    double H[2][4];      // Measurement matrix
    double Q[4][4];      // Process noise covariance
    double R;           // Measurement noise covariance
    double K[4][2];     // Kalman Gain
    double dt;          // Time step

    // Constructor to initialize the Kalman Filter
    KalmanFilter2D(double initial_x, double initial_y, double initial_vx, double initial_vy, double initial_cov, double time_step) {
        x = initial_x;
        y = initial_y;
        vx = initial_vx;
        vy = initial_vy;
        dt = time_step;

        // Initialize covariance matrix
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                P[i][j] = (i == j) ? initial_cov : 0.1;

        // Define state transition matrix (Assumes constant velocity)
        A[0][0] = 1.0; A[0][1] = dt; A[0][2] = 0.0; A[0][3] = 0.0;
        A[1][0] = 0.0; A[1][1] = 1.0; A[1][2] = 0.0; A[1][3] = 0.0;
        A[2][0] = 0.0; A[2][1] = 0.0; A[2][2] = 1.0; A[2][3] = dt;
        A[3][0] = 0.0; A[3][1] = 0.0; A[3][2] = 0.0; A[3][3] = 1.0;

        // Measurement matrix (Extracts position)
        H[0][0] = 1.0; H[0][1] = 0.0; H[0][2] = 0.0; H[0][3] = 0.0;
        H[1][0] = 0.0; H[1][1] = 0.0; H[1][2] = 1.0; H[1][3] = 0.0;

        // Process noise covariance
        Q[0][0] = 0.1; Q[1][1] = 0.5; Q[2][2] = 0.1; Q[3][3] = 0.5;
        R = 0.5; // Measurement noise covariance
    }

    // Prediction Step
    void predict() {
        x = A[0][0] * x + A[0][1] * vx;
        vx = A[1][0] * x + A[1][1] * vx;
        y = A[2][0] * y + A[2][3] * vy;
        vy = A[3][2] * y + A[3][3] * vy;
    }

    // Update Step
    void update(double measured_x, double measured_y) {
        // Compute Kalman Gain
        double Sx = P[0][0] + R;
        double Sy = P[2][2] + R;
        K[0][0] = P[0][0] / Sx;
        K[1][0] = P[1][0] / Sx;
        K[2][1] = P[2][2] / Sy;
        K[3][1] = P[3][2] / Sy;

        // Compute Innovation (Measurement Residual)
        double yx = measured_x - x;
        double yy = measured_y - y;

        // Update Estimates
        x += K[0][0] * yx;
        vx += K[1][0] * yx;
        y += K[2][1] * yy;
        vy += K[3][1] * yy;
    }

    // Get Estimates
    void getEstimates(double &pos_x, double &vel_x, double &pos_y, double &vel_y) {
        pos_x = x;
        vel_x = vx;
        pos_y = y;
        vel_y = vy;
    }
};

// Main function to test the Kalman Filter
int main() {
    double time_step = 1.0;
    KalmanFilter2D kf(0.0, 0.0, 0.0, 0.0, 1.0, time_step);

    // Simulated sensor measurements (x, y)
    double measurements[][2] = {{1.0, 1.2}, {2.1, 2.3}, {2.9, 3.1}, {4.1, 4.3}, {5.2, 5.1}};
    int n = sizeof(measurements) / sizeof(measurements[0]);

    // Loop through each measurement
    for (int i = 0; i < n; ++i) {
        kf.predict();
        kf.update(measurements[i][0], measurements[i][1]);

        double pos_x, vel_x, pos_y, vel_y;
        kf.getEstimates(pos_x, vel_x, pos_y, vel_y);

        cout << "Estimate " << i + 1 << ": X = " << pos_x << ", Vx = " << vel_x
             << ", Y = " << pos_y << ", Vy = " << vel_y << endl;
    }

    return 0;
}
