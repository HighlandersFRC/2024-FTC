package org.firstinspires.ftc.teamcode.Tools;

import java.util.Arrays;

public class KalmanFilter {
    private int n; // State dimension
    private double[][] Q; // Process noise covariance
    private double[][] R; // Measurement noise covariance
    private double[][] P; // Estimate error covariance
    private double[] x; // State estimate

    private double alpha = 1e-3; // Small positive constant
    private double kappa = 0; // Secondary scaling parameter
    private double beta = 2; // Optimal for Gaussian distributions
    private double lambda; // Scaling factor
    private double gamma; // Square root of (n + lambda)
    private double[] Wm; // Weights for mean
    private double[] Wc; // Weights for covariance

    public KalmanFilter(int stateDim, double[][] processNoiseCov, double[][] measurementNoiseCov, double[][] initialCovariance, double[] initialState) {
        this.n = stateDim;
        this.Q = processNoiseCov;
        this.R = measurementNoiseCov;
        this.P = initialCovariance;
        this.x = initialState;

        this.lambda = alpha * alpha * (n + kappa) - n;
        this.gamma = Math.sqrt(n + lambda);

        this.Wm = new double[2 * n + 1];
        this.Wc = new double[2 * n + 1];

        Arrays.fill(Wm, 1.0 / (2 * (n + lambda)));
        Arrays.fill(Wc, 1.0 / (2 * (n + lambda)));
        Wm[0] = lambda / (n + lambda);
        Wc[0] = lambda / (n + lambda) + (1 - alpha * alpha + beta);
    }

    private double[][] sigmaPoints(double[] x, double[][] P) {
        double[][] sigmaPoints = new double[2 * n + 1][n];
        sigmaPoints[0] = Arrays.copyOf(x, x.length);
        double[][] sqrtP = choleskyDecomposition(scaleMatrix(addMatrices(P, Q), n + lambda));

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                sigmaPoints[i + 1][j] = x[j] + sqrtP[j][i];
                sigmaPoints[n + i + 1][j] = x[j] - sqrtP[j][i];
            }
        }
        return sigmaPoints;
    }
    public interface ProcessModel {
        double[] process(double[] state);
    }
    public void predict(ProcessModel f) {
        double[][] sigmaPoints = sigmaPoints(x, P);
        double[][] sigmaPointsPred = new double[sigmaPoints.length][n];

        for (int i = 0; i < sigmaPoints.length; i++) {
            sigmaPointsPred[i] = f.process(sigmaPoints[i]);
        }

        x = new double[n];
        for (int i = 0; i < sigmaPointsPred.length; i++) {
            for (int j = 0; j < n; j++) {
                x[j] += Wm[i] * sigmaPointsPred[i][j];
            }
        }

        P = addMatrices(Q, covarianceUpdate(sigmaPointsPred, x, Wc));
    }
    public interface MeasurementModel {
        double[] measure(double[] state);
    }

    public void update(double[] z, MeasurementModel h) {
        double[][] sigmaPoints = sigmaPoints(x, P);
        double[][] sigmaPointsMeas = new double[sigmaPoints.length][z.length];

        for (int i = 0; i < sigmaPoints.length; i++) {
            sigmaPointsMeas[i] = h.measure(sigmaPoints[i]);
        }

        double[] zPred = new double[z.length];
        for (int i = 0; i < sigmaPointsMeas.length; i++) {
            for (int j = 0; j < z.length; j++) {
                zPred[j] += Wm[i] * sigmaPointsMeas[i][j];
            }
        }

        double[][] S = addMatrices(R, covarianceUpdate(sigmaPointsMeas, zPred, Wc));
        double[][] Tc = covarianceUpdate(sigmaPoints, x, sigmaPointsMeas, zPred, Wc);

        double[][] K = matrixMultiply(Tc, inverse(S));
        x = addVectors(x, matrixMultiply(K, subtractVectors(z, zPred)));
        P = subtractMatrices(P, matrixMultiply(matrixMultiply(K, S), transpose(K)));
    }

    public double[] currentState() {
        return x;
    }

    private double[][] covarianceUpdate(double[][] sigmaPoints, double[] mean, double[] Wc) {
        double[][] cov = new double[n][n];
        for (int i = 0; i < sigmaPoints.length; i++) {
            double[] diff = subtractVectors(sigmaPoints[i], mean);
            cov = addMatrices(cov, scaleMatrix(outerProduct(diff, diff), Wc[i]));
        }
        return cov;
    }

    private double[][] covarianceUpdate(double[][] sigmaPoints1, double[] mean1, double[][] sigmaPoints2, double[] mean2, double[] Wc) {
        double[][] cov = new double[n][sigmaPoints2[0].length];
        for (int i = 0; i < sigmaPoints1.length; i++) {
            double[] diff1 = subtractVectors(sigmaPoints1[i], mean1);
            double[] diff2 = subtractVectors(sigmaPoints2[i], mean2);
            cov = addMatrices(cov, scaleMatrix(outerProduct(diff1, diff2), Wc[i]));
        }
        return cov;
    }

    // Utility functions (matrix and vector operations)

    private double[][] addMatrices(double[][] A, double[][] B) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < A[0].length; j++) {
                C[i][j] = A[i][j] + B[i][j];
            }
        }
        return C;
    }

    private double[] addVectors(double[] A, double[] B) {
        double[] C = new double[A.length];
        for (int i = 0; i < A.length; i++) {
            C[i] = A[i] + B[i];
        }
        return C;
    }

    private double[][] subtractMatrices(double[][] A, double[][] B) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < A[0].length; j++) {
                C[i][j] = A[i][j] - B[i][j];
            }
        }
        return C;
    }

    private double[] subtractVectors(double[] A, double[] B) {
        double[] C = new double[A.length];
        for (int i = 0; i < A.length; i++) {
            C[i] = A[i] - B[i];
        }
        return C;
    }

    private double[][] scaleMatrix(double[][] A, double scale) {
        double[][] B = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < A[0].length; j++) {
                B[i][j] = A[i][j] * scale;
            }
        }
        return B;
    }

    private double[][] outerProduct(double[] A, double[] B) {
        double[][] C = new double[A.length][B.length];
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < B.length; j++) {
                C[i][j] = A[i] * B[j];
            }
        }
        return C;
    }

    private double[][] transpose(double[][] A) {
        double[][] B = new double[A[0].length][A.length];
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < A[0].length; j++) {
                B[j][i] = A[i][j];
            }
        }
        return B;
    }

    private double[][] matrixMultiply(double[][] A, double[][] B) {
        int m = A.length;
        int n = A[0].length;
        int p = B[0].length;
        double[][] C = new double[m][p];
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < p; j++) {
                for (int k = 0; k < n; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    private double[] matrixMultiply(double[][] A, double[] B) {
        double[] C = new double[A.length];
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < B.length; j++) {
                C[i] += A[i][j] * B[j];
            }
        }
        return C;
    }

    // Cholesky decomposition
    private double[][] choleskyDecomposition(double[][] A) {
        int n = A.length;
        double[][] L = new double[n][n];

        for (int i = 0; i < n; i++) {
            for (int j = 0; j <= i; j++) {
                double sum = 0;

                for (int k = 0; k < j; k++) {
                    sum += L[i][k] * L[j][k];
                }

                if (i == j) {
                    L[i][j] = Math.sqrt(A[i][i] - sum);
                } else {
                    L[i][j] = (A[i][j] - sum) / L[j][j];
                }
            }
        }
        return L;
    }

    // Matrix inversion using Gauss-Jordan elimination
    private double[][] inverse(double[][] A) {
        int n = A.length;
        double[][] I = new double[n][n];
        double[][] C = Arrays.copyOf(A, n);

        for (int i = 0; i < n; i++) {
            I[i][i] = 1;
        }

        for (int i = 0; i < n; i++) {
            double pivot = C[i][i];
            for (int j = 0; j < n; j++) {
                C[i][j] /= pivot;
                I[i][j] /= pivot;
            }

            for (int k = 0; k < n; k++) {
                if (k != i) {
                    double factor = C[k][i];
                    for (int j = 0; j < n; j++) {
                        C[k][j] -= factor * C[i][j];
                        I[k][j] -= factor * I[i][j];
                    }
                }
            }
        }
        return I;
    }
}
