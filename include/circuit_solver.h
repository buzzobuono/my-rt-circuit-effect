#ifndef CIRCUIT_SOLVER_H
#define CIRCUIT_SOLVER_H

#include "circuit.h"
#include "linear_solutor.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <memory>
#include <map>
#include <Eigen/Dense>

class CircuitSolver {

private:
    Circuit& circuit;
    Eigen::MatrixXd G;
    Eigen::VectorXd I, V, V_new;
    
   // Eigen::PartialPivLU<Eigen::MatrixXd> lu_solver;
    LinearSolver dumbSolver;
    
    uint64_t sample_count = 0;
    uint64_t failed_count = 0;
    uint64_t iteration_count = 0;
    
    double dt;
    int max_iterations;
    double tolerance_sq;
    double source_g;
    int max_non_convergence_warning;
    std::ofstream logFile;
    bool logging_enabled = false;
    
    void warmUp(double warmup_duration) {
        std::cout << "Circuit WarmUp" << std::endl;
        int warmup_samples = static_cast<int>(warmup_duration / dt);
        for (int i = 0; i < warmup_samples; i++) {
            if (!solve(0.0)) {
                std::cerr << "   WarmUp sample " << i << " convergence issue" << std::endl;
            }
        }
        std::cout << "   Circuit stabilized after " << (warmup_samples * dt * 1000) << " ms" << std::endl;
        std::cout << std::endl;
    }
    
public:
    CircuitSolver(Circuit& ckt, double sample_rate, int source_impedance, int max_iterations, double tolerance, int max_non_convergence_warning = 50) 
        : circuit(ckt), 
          dt(1.0 / sample_rate),
          source_g(1.0 / source_impedance),
          max_iterations(max_iterations),
          tolerance_sq(tolerance*tolerance),
          max_non_convergence_warning(max_non_convergence_warning) {
              
              G.resize(circuit.num_nodes, circuit.num_nodes);
              I.resize(circuit.num_nodes);
              V.resize(circuit.num_nodes);
              V_new.resize(circuit.num_nodes);
              V.setZero();
    }
    
    ~CircuitSolver() {
        if (circuit.hasProbes()) {
            closeProbeFile();
        }
    }

    bool initialize() {
        if (circuit.hasInitialConditions()) {
            circuit.applyInitialConditions();
        }
        if (circuit.hasWarmUp()) {
            warmUp(circuit.warmup_duration);
        }
        if (!circuit.hasInitialConditions() && !circuit.hasWarmUp() ) {
            std::cout << "Starting from zero state" << std::endl;
            std::cout << std::endl;
        }
        if (circuit.hasProbes()) {
            openProbeFile();
        }
        printDCOperatingPoint();
        return true;
    }
    
    bool solveDC() {
        using namespace Eigen;
        std::cout << "Calcolo punto di lavoro DC" << std::endl;

        MatrixXd Gdc = MatrixXd::Zero(circuit.num_nodes, circuit.num_nodes);
        VectorXd Idc = VectorXd::Zero(circuit.num_nodes);
        
        // Iterazione Newton-Raphson per componenti non lineari
        VectorXd Vdc = VectorXd::Zero(circuit.num_nodes);
        
        for (int iter = 0; iter < max_iterations; iter++) {
            Gdc.setZero();
            Idc.setZero();
            
            for (auto& component : circuit.components) {
                // dt=0 for DC analysis
                component->stamp(Gdc, Idc, Vdc, 0.0);
            }

            // Nodo di massa forzato
            Gdc.row(0).setZero();
            Gdc.col(0).setZero();
            Gdc(0, 0) = 1.0;
            Idc(0) = 0.0;
            
            VectorXd Vdc_new = Gdc.lu().solve(Idc);
            
            double error_sq = (V_new - V).squaredNorm();
            Vdc = Vdc_new;
            
            if (error_sq < tolerance_sq) {
                // Convergenza raggiunta
                std::cout << "   Convergenza raggiunta dopo " << (iter + 1) << " iterazioni" << std::endl;
                std::cout << "   Tensioni nodali:" << std::endl;
                for (int i = 0; i < circuit.num_nodes; ++i) {
                    std::cout << "      Nodo " << i << ": " << Vdc(i) << " V" << std::endl;
                }
                
                std::cout << std::endl;
                
                // Copia le tensioni DC come condizioni iniziali
                V = Vdc;

                return true;
            }
        }
        
        // Non convergenza
        std::cerr << "ERRORE: Analisi DC non convergente dopo " << max_iterations << " iterazioni" << std::endl;
        return false;
    }

    void openProbeFile() {
        std::string filename = circuit.getProbeFile();
        logFile.open(filename);
        if (!logFile.is_open()) {
            throw std::runtime_error("Cannot open probe file: " + filename);
        }
        
        logging_enabled = true;
        
        logFile << "time";
        for (auto& p : circuit.probes) {
            if (p.type == ProbeTarget::Type::VOLTAGE) {
                logFile << ";V(" << p.name << ")";
            } else if (p.type == ProbeTarget::Type::CURRENT) {
                logFile << ";I(" << p.name << ")";
            }
        }
        logFile << "\n";
        
        std::cout << "Probe file opened: " << filename << std::endl;
    }
    
    void logProbes() {
        if (!logging_enabled) return;
        double time = sample_count * dt;
        logFile << std::fixed << std::setprecision(9) << time;
        
        for (auto& p : circuit.probes) {
            if (p.type == ProbeTarget::Type::VOLTAGE) {
                int node = std::stoi(p.name);
                if (node < circuit.num_nodes) {
                    logFile << ";" << V(node);
                } else {
                    logFile << ";NaN";
                }
            } else if (p.type == ProbeTarget::Type::CURRENT) {
                double current = 0.0;
                bool found = false;
                for (auto& comp : circuit.components) {
                    if (comp->name == p.name) {
                        current = comp->getCurrent();
                        found = true;
                        break;
                    }
                }
                logFile << ";" << (found ? current : NAN);
            }
        }
        
        logFile << "\n";
    }
    
    void closeProbeFile() {
        if (logging_enabled && logFile.is_open()) {
            logFile.close();
            logging_enabled = false;
            std::cout << "Probe file closed." << std::endl;
        }
    }
    
    bool solve(double input_voltage) {
        sample_count++;
        double alpha = 0.8;  // Start with higher damping
        double prev_error = 1e10;
        
        // Newton-Raphson
        for (int iter = 0; iter < max_iterations; iter++) {
            G.setZero();
            I.setZero();
            
            for (auto& component : circuit.components) {
                component->stamp(G, I, V, dt);
            }
            
            // Ground node constraint
            G.row(0).setZero();
            G.col(0).setZero();
            G(0, 0) = 1.0;
            I(0) = 0.0;
            
            if (circuit.input_node > 0) {
                G(circuit.input_node, circuit.input_node) += source_g;
                I(circuit.input_node) += input_voltage * source_g;
            }
            
            /*if (sample_count == 10 && iter == 0) {
                std::cout << "Matrice G:\n" << G << "\n\n";
                std::cout << "Vettore I:\n" << I << "\n\n";
                std::cout << "Determinante: " << G.determinant() << std::endl;
            }
            */
            
            // Solve linear system
           // lu_solver.compute(G);
           // V_new = lu_solver.solve(I);
            V_new = dumbSolver.solveLinearSystemEigen(G, I);
            
            double error_sq = (V_new - V).squaredNorm();
            if (error_sq < tolerance_sq) {
                V = V_new;
                
                for (auto& comp : circuit.components) {
                    comp->updateHistory(V, dt);
                }
                
                logProbes();
                iteration_count += iter + 1;
                return true;
            }
            
            if (iter > 0 && error_sq > prev_error) {
                // Errore aumentato -> riduci alpha (più damping)
                alpha *= 0.5;
                alpha = std::max(alpha, 0.1);  // minimo 0.1
            } else if (iter > 2 && error_sq < prev_error * 0.5) {
                // Convergenza buona -> aumenta alpha (meno damping)
                alpha = std::min(alpha * 1.2, 1.0);  // massimo 1.0
            }
            
            V = alpha * V_new + (1.0 - alpha) * V;
           // V = V_new;
        }
        
        logProbes();
        failed_count++;
        iteration_count += max_iterations;
        return false;
    }
    
    double getFailurePercentage() {
        return (sample_count > 0) ? (100.0 * failed_count / sample_count) : 0.0;
    }
    
    double getTotalIterations() {
        return iteration_count;
    }
    
    double getTotalSamples() {
        return sample_count;
    }
    
    double getMeanIterations() {
        return (sample_count > 0) ? (1.0 * iteration_count / sample_count) : 0.0;
    }
    
    double getOutputVoltage() const {
        return V(circuit.output_node);
    }
    
    void printDCOperatingPoint() {
        std::cout << "DC Operating Point" << std::endl;
        for (int i = 0; i < circuit.num_nodes; i++) {
            std::cout << "   Node " << i << ": " << V(i) << " V" << std::endl;
        }
        std::cout << std::endl;
    }

    void reset() {
        V.setZero();
        circuit.reset();
    }
    
};

#endif