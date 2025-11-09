#ifndef CIRCUIT_SOLVER_H
#define CIRCUIT_SOLVER_H

#include "circuit.h"

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
    Eigen::VectorXd I, V;
    double dt;
    int max_iterations;
    double tolerance;
    int input_impedance;
    int max_non_convergence_warning;
    std::ofstream logFile;
    bool logging_enabled = false;
    
    bool warmUp(double warmup_duration) {
        std::cout << "Circuit WarmUp" << std::endl;
        int warmup_samples = static_cast<int>(warmup_duration / dt);
        for (int i = 0; i < warmup_samples; i++) {
            if (!solve(0.0)) {
                std::cerr << "   WarmUp sample " << i << " convergence issue" << std::endl;
                return 1;
            }
        }
        std::cout << "   Circuit stabilized after " << (warmup_samples * dt * 1000) << " ms" << std::endl;
        std::cout << std::endl;
        return 0;
    }

    
    
public:
    CircuitSolver(Circuit& ckt, double sample_rate, int input_impedance, int max_iterations, double tolerance, int max_non_convergence_warning = 50) 
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
        
        // Stamp dei componenti con dt=infinito (DC)
        // Per l'analisi DC:
        // - Condensatori = circuito aperto (non contribuiscono)
        // - Induttori = cortocircuito (contribuiscono come resistenze con R=0)
        for (auto& comp : circuit.components) {
            if (comp->type == ComponentType::CAPACITOR) {
                // I condensatori non contribuiscono in DC
                continue;
            } else if (comp->type == ComponentType::INDUCTOR) {
                // Gli induttori sono cortocircuiti in DC
                // Trattali come resistenze con valore molto piccolo
/*auto* ind = dynamic_cast<Inductor*>(comp.get());
                if (ind) {
                    int n1 = ind->nodes[0];
                    int n2 = ind->nodes[1];
                    double g = 1e6; // Conduttanza molto alta (R ≈ 0)
                    
                    if (n1 >= 0) Gdc(n1, n1) += g;
                    if (n2 >= 0) Gdc(n2, n2) += g;
                    if (n1 >= 0 && n2 >= 0) {
                        Gdc(n1, n2) -= g;
                        Gdc(n2, n1) -= g;
                    }
                }*/
            } else {
                // Altri componenti (resistori, diodi, ecc.) stampano normalmente
                // Passa dt=0 per indicare analisi DC
                comp->stamp(Gdc, Idc, Vdc, 0.0);
            }
        }

        // Nodo di massa forzato
        Gdc.row(0).setZero();
        Gdc.col(0).setZero();
        Gdc(0, 0) = 1.0;
        Idc(0) = 0.0;

        // Soluzione del sistema
        VectorXd Vdc_new = Gdc.lu().solve(Idc);
        
        // Check convergenza (importante per componenti non lineari)
        double error = (Vdc_new - Vdc).norm();
        Vdc = Vdc_new;
        
        if (error < tolerance) {
            // Convergenza raggiunta
            std::cout << "  Convergenza raggiunta dopo " << (iter + 1) << " iterazioni" << std::endl;
            std::cout << "  Tensioni nodali:" << std::endl;
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

    void openProbeFile(const std::string& filename = "probe.csv") {
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
    
    void logProbes(double time) {
        if (!logging_enabled) return;
        
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
        static int sample_count = 0;

        // Newton-Raphson iteration
        double final_error = 0.0;
        
        for (int iter = 0; iter < max_iterations; iter++) {
            G.setZero();
            I.setZero();
            
            for (auto& component : circuit.components) {   
                component->stamp(G, I, V, dt);
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
    
    const Eigen::VectorXd& getVoltages() const { return V; }

    // Configuration
    void setMaxIterations(int iter) { max_iterations = iter; }
    void setTolerance(double tol) { tolerance = tol; }
};

#endif