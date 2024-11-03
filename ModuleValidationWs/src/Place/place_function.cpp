#include <Eigen/Dense>
#include <iostream>

// Function to compute the controllability matrix
Eigen::MatrixXd controllabilityMatrix(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B) {
    int n = A.rows();
    Eigen::MatrixXd controllability(n, n);
    controllability.col(0) = B;
    for (int i = 1; i < n; ++i) {
        controllability.col(i) = A * controllability.col(i - 1);
    }
    return controllability;
}

// Function to compute the characteristic polynomial coefficients given desired eigenvalues
Eigen::VectorXd characteristicPolynomial(const Eigen::VectorXd& eigenvalues) {
    int n = eigenvalues.size();
    Eigen::VectorXd coeffs(n + 1);
    coeffs(0) = 1.0;  // Leading coefficient
    for (int i = 1; i <= n; ++i) {
        coeffs(i) = 0.0;
    }

    // Compute polynomial coefficients based on desired eigenvalues
    for (int i = 0; i < n; ++i) {
        for (int j = n; j > 0; --j) {
            coeffs(j) -= eigenvalues(i) * coeffs(j - 1);
        }
    }

    return coeffs;
}

// Helper function to compute A^i (matrix power)
Eigen::MatrixXd matrixPower(const Eigen::MatrixXd& A, int power) {
    if (power == 0) {
        return Eigen::MatrixXd::Identity(A.rows(), A.cols());
    }
    Eigen::MatrixXd result = A;
    for (int i = 1; i < power; ++i) {
        result = result * A;
    }
    return result;
}

// Function to perform pole placement using Ackermann's formula
Eigen::MatrixXd polePlacement(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::VectorXd& desired_eigenvalues) {
    int n = A.rows();
    
    // Step 1: Compute controllability matrix
    Eigen::MatrixXd C = controllabilityMatrix(A, B);
    
    // Step 2: Compute the characteristic polynomial P(A) using desired eigenvalues
    Eigen::VectorXd alpha = characteristicPolynomial(desired_eigenvalues);

    // Step 3: Calculate Ackermann's gain
    Eigen::MatrixXd P_A = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < n; ++i) {
        P_A += alpha(i) * matrixPower(A, i);
    }
    P_A += alpha(n) * Eigen::MatrixXd::Identity(n, n);  // Constant term

    // Step 4: Use Ackermann's formula to compute feedback gain matrix K
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(1, n);  // 1xN for gain matrix
    K.row(0) = Eigen::MatrixXd::Identity(1, n) * C.inverse() * P_A;

    return K;
}


/*
int main() {
    // Define your system matrices (A and B) and the desired eigenvalues
    Eigen::MatrixXd A(3, 3);
    Eigen::MatrixXd B(3, 1);
    Eigen::VectorXd desired_eigenvalues(3);

    // Example values (replace these with your specific values)
    A << 0, 1, 0,
         0, 0, 1,
        -1, -2, -3;
    
    B << 0, 0, 1;

    desired_eigenvalues << -2, -3, -4;

    // Perform pole placement using Ackermann's formula
    Eigen::MatrixXd K = polePlacement(A, B, desired_eigenvalues);

    // Display the result
    std::cout << "Feedback Gain Matrix (K): \n" << K << std::endl;

    return 0;
}
*/