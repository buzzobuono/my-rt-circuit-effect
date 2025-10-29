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
    int max_non_convergence_warning;
    
    void solveDCOperatingPoint() {
        using namespace Eigen;
        std::cout << "Calcolo punto di lavoro DC" << std::endl;

        MatrixXd Gdc = MatrixXd::Zero(circuit.num_nodes, circuit.num_nodes);
        VectorXd Idc = VectorXd::Zero(circuit.num_nodes);
        // Stamp dei componenti con dt=0
        for (auto& comp : circuit.components) {
            comp->stamp(Gdc, Idc, VectorXd::Zero(circuit.num_nodes), 0.0);
        }

        // Nodo di massa forzato
        Gdc(0, 0) = 1.0;
        Idc(0) = 0.0;

        // Soluzione del sistema
        VectorXd Vdc = Gdc.lu().solve(Idc);
        
        // Output di debug
        std::cout << "  Node Voltage" << std::endl;
        for (int i = 0; i < circuit.num_nodes; ++i) {
            std::cout << "      Nodo " << i << ": " << Vdc(i) << " V" << std::endl;
        }

        // Inizializza i condensatori
        
        std::cout << "  Capacitor Initialization" << std::endl;
        for (auto& comp : circuit.components) {
            if (comp->type == ComponentType::CAPACITOR) {
                auto* c = dynamic_cast<Capacitor*>(comp.get());
                if (c) {
                    int n1 = c->nodes[0];
                    int n2 = c->nodes[1];
                    double v1 = (n1 > 0 && n1 < circuit.num_nodes) ? Vdc[n1] : 0.0;
                    double v2 = (n2 > 0 && n2 < circuit.num_nodes) ? Vdc[n2] : 0.0;
                    double v_cap = v1 - v2;
                    c->setInitialVoltage(v_cap);
                    std::cout << "     " << c->name << " inizializzato a " << v_cap << " V" << std::endl;
                }
            }
        }

        std::cout << std::endl;
        // Copia nei voltages principali (così la simulazione parte da lì)
        V = Vdc;
    }

public:
    CircuitSolver(Circuit& ckt, double sample_rate, int input_impedance, int max_iterations = 50, double tolerance = 1e-8, int max_non_convergence_warning = 50) 
        : circuit(ckt), 
          dt(1.0 / sample_rate),
          input_impedance(input_impedance),
          max_iterations(max_iterations),
          tolerance(tolerance),
          max_non_convergence_warning(max_non_convergence_warning) {
        
        G.resize(circuit.num_nodes, circuit.num_nodes);
        I.resize(circuit.num_nodes);
        V.resize(circuit.num_nodes);
        V.setZero();

        //solveDCOperatingPoint();
    }

    
    bool solve(double input_voltage, bool dc) {
        //static bool first_call = true;
        static int sample_count = 0;

        // Newton-Raphson iteration
        double final_error = 0.0;
        
        for (int iter = 0; iter < max_iterations; iter++) {
            G.setZero();
            I.setZero();
            
            if (sample_count == 0 && iter == 0) {
                std::cout << "Component Stamping Order" << std::endl;
            }
            for (auto& component : circuit.components) {
                if (sample_count == 0 && iter == 0) {
                    std::cout << "   " << component->name << std::endl;
                }
                component->stamp(G, I, V, dt);
            }
            if (sample_count == 0 && iter == 0) {
                std::cout << std::endl;
            }

            double input_g = 1.0 / input_impedance;
            
            if (circuit.input_node > 0) {
                G(circuit.input_node, circuit.input_node) += input_g;
                I(circuit.input_node) += input_voltage * input_g;
            }
            // Ground node constraint (DOPO stamp per sicurezza)
            G(0, 0) = 1.0;
            I(0) = 0.0;
            
            // Solve linear system (usa LU per robustezza)
            Eigen::VectorXd V_new = G.lu().solve(I);
            
            // Check convergence
            double error = (V_new - V).norm();
            final_error = error;  // Salva per debug
            
            // Convergence check
            if (error < tolerance) {
                V = V_new;  // ✓ Aggiorna SOLO quando converge
                
                // Update component history
                for (auto& comp : circuit.components) {
                    comp->updateHistory(V, dt);
                }
                
                if (dc) {
                    //first_call = false;
                    std::cout << "DC Operating Point" << std::endl;
                    for (int i = 0; i < circuit.num_nodes; i++) {
                        std::cout << "  Node " << i << ": " << V(i) << " V" << std::endl;
                    }
                    std::cout << "  Converged in " << (iter + 1) << " iterations" << std::endl;
                    std::cout << std::endl;
                }
                
                sample_count++;
                return true;
            }
            
            V = V_new;
        }
        
        if (sample_count < max_non_convergence_warning) {
            std::cerr << "WARNING: Sample " << sample_count << " did not converge after " << max_iterations  << " iterations" << std::endl;
            std::cerr << "  Final error: " << final_error << std::endl;
            std::cerr << "  Node voltages: ";
            for (int i = 0; i < V.size(); i++) {
                std::cerr << "V[" << i << "]=" << V(i) << " ";
            }
            std::cerr << std::endl << std::endl;
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