#include <math.h>
#include <Arduino.h>

#define EPSILON 1e-6  // Convergence tolerance
#define MAX_ITER 100  // Maximum iterations

const double L1 = 140.0;  // Length of first arm (mm)
const double L2 = 140.0;  // Length of second arm (mm)

// Define theta1 and theta2 limits (in radians)
const double THETA1_MIN = -90.0 * M_PI / 180.0;
const double THETA1_MAX = 120.0 * M_PI / 180.0;
const double THETA2_MIN = 0.0 * M_PI / 180.0;
const double THETA2_MAX = 160.0 * M_PI / 180.0;

// Structure for position
struct Position {
    double x;
    double y;
    double z;
};

Position dee5 {-15, 205, 20};
Position home {-100, 0, 0};

// Function to evaluate F(x) = [f1, f2]
void evaluateF(double theta, double theta2, double r, double z, double *F) {
    F[0] = L1 * sin(theta) + L2 * sin(theta + theta2) - r;
    F[1] = L1 * cos(theta) + L2 * cos(theta + theta2) - z;
}

// Jacobian matrix J(x)
void evaluateJacobian(double theta, double theta2, double J[2][2]) {
    J[0][0] = L1 * cos(theta) + L2 * cos(theta + theta2);
    J[0][1] = L2 * cos(theta + theta2);
    J[1][0] = -L1 * sin(theta) - L2 * sin(theta + theta2);
    J[1][1] = -L2 * sin(theta + theta2);
}

// Solve 2x2 linear system J * dx = -F using Cramer's rule
void solveLinearSystem(double J[2][2], double F[2], double *dx) {
    double detJ = J[0][0] * J[1][1] - J[0][1] * J[1][0];

    if (fabs(detJ) < 1e-9) {  // Avoid division by zero
        Serial.println("Jacobian is singular. Newton's method fails.");
        dx[0] = NAN;
        dx[1] = NAN;
        return;
    }

    dx[0] = (-F[0] * J[1][1] + F[1] * J[0][1]) / detJ;
    dx[1] = (-F[1] * J[0][0] + F[0] * J[1][0]) / detJ;
}

// Newton's Method to solve for theta1 and theta2 with bounds
void newtonsMethod(double r, double z, double theta0, double theta2_0, double *theta, double *theta2) {
    *theta = theta0;
    *theta2 = theta2_0;
    int iterations = 0;

    while (iterations < MAX_ITER) {
        double F[2], J[2][2], dX[2];

        evaluateF(*theta, *theta2, r, z, F);
        evaluateJacobian(*theta, *theta2, J);
        solveLinearSystem(J, F, dX);

        if (isnan(dX[0]) || isnan(dX[1])) {
            Serial.println("Newton's method failed to converge.");
            return;
        }

        *theta += dX[0];
        *theta2 += dX[1];

        // Apply constraints (clamping values)
        if (*theta < THETA1_MIN) *theta = THETA1_MIN;
        if (*theta > THETA1_MAX) *theta = THETA1_MAX;
        if (*theta2 < THETA2_MIN) *theta2 = THETA2_MIN;
        if (*theta2 > THETA2_MAX) *theta2 = THETA2_MAX;

        // Check convergence
        if (fabs(dX[0]) < EPSILON && fabs(dX[1]) < EPSILON) {
            break;
        }

        iterations++;
    }
}

void setup() {
    Serial.begin(115200);

    double r = sqrt(sq(dee5.x) + sq(dee5.y));
    double theta0 = 0.5;   // Initial guess for theta
    double theta2_0 = 0.5; // Initial guess for theta2
    double theta, theta2;

    newtonsMethod(r, dee5.z, theta0, theta2_0, &theta, &theta2);

    if (!isnan(theta)) {
        Serial.print("\n\nr = ");
        Serial.print(r);
        Serial.print("\ntheta1 = ");
        Serial.print(theta * 180 / M_PI); // Convert to degrees
        Serial.print("\ntheta2 = ");
        Serial.print(theta2 * 180 / M_PI); // Convert to degrees
        Serial.print("\n\n");
    } else {
        Serial.println("Failed to find a solution.");
    }
}

void loop() {
    // No loop action needed
}


