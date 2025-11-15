#ifndef LINEAR_SOLUTOR_H
#define LINEAR_SOLUTOR_H

#include <vector>
#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>

// Classe semplificata per la gestione dei vettori
class Vector {
public:
    std::vector<double> data;
    int size;

    Vector(int s) : size(s), data(s, 0.0) {}

    double& operator[](int index) { return data[index]; }
    const double& operator[](int index) const { return data[index]; }

    void setZero() { std::fill(data.begin(), data.end(), 0.0); }
};

// Classe semplificata per la gestione delle matrici dense
class Matrix {
public:
    std::vector<std::vector<double>> data;
    int rows, cols;

    Matrix(int r, int c) : rows(r), cols(c), data(r, std::vector<double>(c, 0.0)) {}

    void setZero() {
        for (int i = 0; i < rows; ++i) {
            std::fill(data[i].begin(), data[i].end(), 0.0);
        }
    }

    double& operator()(int row, int col) { return data[row][col]; }
    const double& operator()(int row, int col) const { return data[row][col]; }
};

class LinearSolver {
    
public:
    
    Eigen::VectorXd solveLinearSystemEigen(Eigen::MatrixXd G, Eigen::VectorXd I) {
        int n = G.rows();
        if (n != G.cols() || n != I.size()) {
            throw std::runtime_error("Dimensioni non corrispondenti per la risoluzione lineare.");
        }
        
        // Costruisce la matrice aumentata [G|I]
        Eigen::MatrixXd augmented(n, n + 1);
        augmented.leftCols(n) = G;
        augmented.rightCols(1) = I;
        
        // Eliminazione in avanti con pivoting parziale
        for (int k = 0; k < n; ++k) {
            int max_i;
            augmented.col(k).tail(n - k).cwiseAbs().maxCoeff(&max_i);
            max_i += k;
            
            if (max_i != k) {
                augmented.row(k).swap(augmented.row(max_i));
            }
            
            if (augmented(k, k) == 0.0) {
                throw std::runtime_error("Matrice singolare");
            }
            
            for (int i = k + 1; i < n; ++i) {
                double factor = augmented(i, k) / augmented(k, k);
                augmented.row(i) -= factor * augmented.row(k);
            }
        }
        
        // Sostituzione all'indietro
        Eigen::VectorXd V_new(n);
        for (int i = n - 1; i >= 0; --i) {
            double sum = 0.0;
            for (int j = i + 1; j < n; ++j) {
                sum += augmented(i, j) * V_new(j);
            }
            V_new(i) = (augmented(i, n) - sum) / augmented(i, i);
        }
        
        return V_new;
    }
    
};

#endif