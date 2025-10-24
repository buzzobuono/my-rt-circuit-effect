#ifndef CIRCUIT_SOLVER_H
#define CIRCUIT_SOLVER_H

#include <iostream>
#include <fstream>
#include <memory>
#include <map>
#include <Eigen/Dense>

#include "circuit.h"

class CircuitSolver {
private:
    Circuit& circuit;
    Eigen::MatrixXd G;
    Eigen::VectorXd I, V;
    double dt;
    int max_iterations;
    double tolerance;
    int input_impedance;
    
public:
    CircuitSolver(Circuit& ckt, double sample_rate, int input_impedance,
                  int max_iter = 10, double tol = 1e-5) 
        : circuit(ckt), 
          dt(1.0 / sample_rate),
          input_impedance(input_impedance),
          max_iterations(max_iter),
          tolerance(tol) {
        
        G.resize(circuit.num_nodes, circuit.num_nodes);
        I.resize(circuit.num_nodes);
        V.resize(circuit.num_nodes);
        V.setZero();
    }
    
    bool solve(double input_voltage) {
        static bool first_call = true;
        static int sample_count = 0;
        
        // Set input voltage
        //if (circuit.voltage_sources.count("VIN") > 0) {
        //    circuit.voltage_sources["VIN"]->setVoltage(input_voltage);
        //}

        // Newton-Raphson iteration
        double final_error = 0.0;
        
        for (int iter = 0; iter < max_iterations; iter++) {
            G.setZero();
            I.setZero();
            
            if (sample_count == 0 && iter == 0) {
                std::cout << "=== Component stamping order ===" << std::endl;
                for (size_t i = 0; i < circuit.components.size(); i++) {
                    std::cout << i << ": " << circuit.components[i]->name 
                            << " (type " << (int)circuit.components[i]->type << ")" 
                            << std::endl;
                }
            }
            // Stamp all components
            for (auto& comp : circuit.components) {
                comp->stamp(G, I, V, dt);
            }

            double input_g = 1.0 / input_impedance;
            
            if (circuit.input_node > 0) {
                G(circuit.input_node, circuit.input_node) += input_g;
                I(circuit.input_node) += input_voltage * input_g;
            }
            // ================================================
            
            //std::cout << "---" << std::endl;
            //for (size_t i = 0; i < circuit.components.size(); ++i) {
            //    std::cout << "Component " << i << ": " 
            //            << circuit.components[i]->name << std::endl;
            //}
            
            // Ground node constraint (DOPO stamp per sicurezza)
            G(0, 0) = 1.0;
            I(0) = 0.0;
            
            // Solve linear system (usa LU per robustezza)
            Eigen::VectorXd V_new = G.lu().solve(I);
            
            // Check convergence
            double error = (V_new - V).norm();
            final_error = error;  // Salva per debug
            
            // Debug output
            /*if (sample_count < 5) {
                std::cout << "Sample " << sample_count << ", Iter " << iter 
                        << ": error = " << error << std::endl;
                if (iter == 0) {
                    std::cout << "  Node voltages: ";
                    for (int i = 0; i < std::min(10, (int)V.size()); i++) {
                        std::cout << "V[" << i << "]=" << V(i) << "V ";
                    }
                    std::cout << std::endl;
                }
            }
            */
            // Convergence check
            if (error < tolerance) {
                V = V_new;  // ✓ Aggiorna SOLO quando converge
                
                // Update component history
                for (auto& comp : circuit.components) {
                    comp->updateHistory(V, dt);
                }
                
                // DC operating point on first convergence
                if (first_call) {
                    first_call = false;
                    std::cout << "\n=== DC Operating Point (first sample) ===" << std::endl;
                    for (int i = 0; i < circuit.num_nodes; i++) {
                        std::cout << "  Node " << i << ": " << V(i) << " V" << std::endl;
                    }
                    std::cout << "  Converged in " << (iter + 1) << " iterations" << std::endl;
                    std::cout << "=========================================\n" << std::endl;
                }
                
                sample_count++;
                return true;
            }
            
            // Update voltage guess for next iteration
            V = V_new;
        }
        
        // Non-convergence warning
        if (sample_count < 10) {
            std::cerr << "WARNING: Sample " << sample_count 
                    << " did not converge after " << max_iterations 
                    << " iterations" << std::endl;
            std::cerr << "  Final error: " << final_error << std::endl;
            std::cerr << "  Node voltages: ";
            for (int i = 0; i < std::min(10, (int)V.size()); i++) {
                std::cerr << "V[" << i << "]=" << V(i) << " ";
            }
            std::cerr << std::endl;
        }
        
        sample_count++;
        return false;
    }
    
    // Get output voltage
    double getOutputVoltage() const {
        return V(circuit.output_node);
    }
    
    // Get any node voltage (for debugging/analysis)
    double getNodeVoltage(int node) const {
        if (node >= 0 && node < circuit.num_nodes) {
            return V(node);
        }
        return 0.0;
    }
    
    // Reset state
    void reset() {
        V.setZero();
        circuit.reset();
    }
    
    const Eigen::VectorXd& getVoltages() const { return V; }

    // Configuration
    void setMaxIterations(int iter) { max_iterations = iter; }
    void setTolerance(double tol) { tolerance = tol; }
};

#endif