#ifndef CIRCUIT_SOLVER_H
#define CIRCUIT_SOLVER_H

#include <vector>
#include <memory>
#include <map>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

// ============================================================================
// COMPONENT INTERFACE
// ============================================================================

enum class ComponentType {
    RESISTOR, CAPACITOR, INDUCTOR,
    DIODE, BJT,
    VOLTAGE_SOURCE
};

class Component {
public:
    ComponentType type;
    std::string name;
    std::vector<int> nodes;
    
    virtual ~Component() = default;
    virtual void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, 
                      const Eigen::VectorXd& V, double dt) = 0;
    virtual void updateHistory(const Eigen::VectorXd& V, double dt) {}
    virtual void reset() {}
};

// ============================================================================
// COMPONENTS IMPLEMENTATION
// ============================================================================

class Resistor : public Component {
    
public:
    double R;

    Resistor(const std::string& comp_name, int node1, int node2, double resistance) {
        if (resistance <= 0) {
            throw std::runtime_error(std::string("Resistance must be positive"));
        }
        if (node1 == node2) {
            throw std::runtime_error(std::string("Resistor nodes must be different"));
        }
        type = ComponentType::RESISTOR;
        name = comp_name;
        nodes = { node1, node2 };
        R = resistance;
    }
    
    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, const Eigen::VectorXd& V, double dt) override {
        const double R_MIN = 1e-12; // Resistenza minima
        const double R_MAX = 1e12; // Oltre questa soglia, consideri circuito aperto
        
        if (R > R_MAX) return; // Non stampare nulla
        
        double g = 1.0 / std::max(R, R_MIN);
        
        int n1 = nodes[0], n2 = nodes[1];
        
        if (n1 != 0) {
            G(n1, n1) += g;
            if (n2 != 0) G(n1, n2) -= g;
        }
        if (n2 != 0) {
            G(n2, n2) += g;
            if (n1 != 0) G(n2, n1) -= g;
        }
    }
    
    double getResistance() const { 
        return R; 
    }

};

class Capacitor : public Component {
    double C;
    double v_prev;   // tensione al passo precedente
    double i_prev;   // corrente equivalente al passo precedente

public:
    Capacitor(const std::string& comp_name, int node1, int node2, double capacitance) {
        if (capacitance <= 0) {
            throw std::runtime_error(std::string("Capacitance must be positive"));
        }
        if (node1 == node2) {
            std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
            throw std::runtime_error(std::string("Capacitor nodes must be different"));
        }
        type = ComponentType::CAPACITOR;
        name = comp_name;
        nodes = { node1, node2 };
        C = capacitance;
        v_prev = 0.0;
        i_prev = 0.0;
    }

    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, const Eigen::VectorXd& V, double dt) override {
        if (dt <= 0) {
            throw std::runtime_error("Time step must be positive");
        }
        // Trapezoidal rule
        double geq = 2.0 * C / dt;          // equivalente conductance
        double ieq = geq * v_prev + i_prev; // corrente equivalente

        int n1 = nodes[0], n2 = nodes[1];

        if (n1 != 0) {
            G(n1, n1) += geq;
            if (n2 != 0) G(n1, n2) -= geq;
            I(n1) += ieq;
        }
        if (n2 != 0) {
            G(n2, n2) += geq;
            if (n1 != 0) G(n2, n1) -= geq;
            I(n2) -= ieq;
        }
    }

    void updateHistory(const Eigen::VectorXd& V, double dt) override {
        double v_n1 = (nodes[0] != 0) ? V(nodes[0]) : 0.0;
        double v_n2 = (nodes[1] != 0) ? V(nodes[1]) : 0.0;
        double v = v_n1 - v_n2;
        
        double geq = 2.0 * C / dt;
        i_prev = geq * (v - v_prev) - i_prev;
        v_prev = v;
    }

    void setInitialVoltage(double v0) {
        v_prev = v0;
        i_prev = 0.0; // Assumendo che parta da regime
    }

    void reset() override {
        v_prev = 0.0;
        i_prev = 0.0;
    }
};


class Diode : public Component {
    
private: 
    // Model parameters
    double IS;   // Saturation current
    double VT;   // Thermal voltage
    double N;    // Ideality factor

public:
    // Costruttore con parametri personalizzabili
    Diode(const std::string& nm, int anode, int cathode,
               double Is, double n, double Vt) {
        type = ComponentType::DIODE;
        if (Is <= 0) {
            throw std::runtime_error("Diode: Saturation current IS must be positive");
        }
        if (n <= 0) {
            throw std::runtime_error("Diode: ideality factor must be positive");
        }
        if (Vt <= 0) {
            throw std::runtime_error("Diode: Thermal voltage VT must be positive");
        }
        name = nm;
        nodes = {anode, cathode};
        IS = Is;
        N = n;
        VT = Vt;
    }

    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I,
               const Eigen::VectorXd& V, double dt) override {
        double vd = V(nodes[0]) - V(nodes[1]);
        double id = IS * (std::exp(vd / (N * VT)) - 1.0);
        double gd = (IS / (N * VT)) * std::exp(vd / (N * VT));
        double ieq = id - gd * vd;

        int n1 = nodes[0], n2 = nodes[1];

        if (n1 != 0) {
            G(n1, n1) += gd;
            if (n2 != 0) G(n1, n2) -= gd;
            I(n1) -= ieq;
        }
        if (n2 != 0) {
            G(n2, n2) += gd;
            if (n1 != 0) G(n2, n1) -= gd;
            I(n2) += ieq;
        }
    }
};

class BJT : public Component {
public:
    enum BJTType { NPN, PNP };
    
private:
    BJTType bjt_type;
    int nc, nb, ne;  // collector, base, emitter nodes
    
    // Model parameters
    double IS;   // Saturation current (default: 1e-14 A)
    double BF;   // Forward beta (default: 100)
    double BR;   // Reverse beta (default: 1)
    double VT;   // Thermal voltage = kT/q ≈ 26mV
    
    // Previous iteration values (for Newton-Raphson)
    double vbe_prev, vbc_prev;
    
    static constexpr double V_LIMIT = 0.5;  // Voltage limiting for convergence
    
public:
    BJT(const std::string& comp_name, int collector, int base, int emitter, 
        double bf, double br, double is, double Vt) {
        if (is <= 0) {
            throw std::runtime_error("BJT: Saturation current IS must be positive");
        }
        if (bf <= 0) {
            throw std::runtime_error("BJT: Forward beta BF must be positive");
        }
        if (br <= 0) {
            throw std::runtime_error("BJT: Reverse beta BR must be positive");
        }
        if (Vt <= 0) {
            throw std::runtime_error("BJT: Thermal voltage VT must be positive");
        }
        this->type = ComponentType::BJT;
        name = comp_name;
        bjt_type = NPN;
        nc = collector;
        nb = base;
        ne = emitter;
        nodes = {nc, nb, ne};
        
        IS = is;
        BF = bf;
        BR = br;  // Typically much smaller than BF
        VT = Vt;  // 26mV at 300K
        
        vbe_prev = 0.0;
        vbc_prev = 0.0;
        
        // Validation
        if (nc == nb || nb == ne || nc == ne) {
            throw std::runtime_error(std::string("BJT: All three nodes must be different"));
        }
        if (BF <= 0 || IS <= 0) {
            throw std::runtime_error(std::string("BJT: BF and IS must be positive"));
        }
    }
    
    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, 
               const Eigen::VectorXd& V, double dt) override {
        
        // Read node voltages (handle ground)
        double vc = (nc != 0) ? V(nc) : 0.0;
        double vb = (nb != 0) ? V(nb) : 0.0;
        double ve = (ne != 0) ? V(ne) : 0.0;
        
        // Junction voltages
        double vbe = vb - ve;
        double vbc = vb - vc;
        
        // Voltage limiting for convergence
        vbe = limitJunction(vbe, vbe_prev);
        vbc = limitJunction(vbc, vbc_prev);
        
        // For PNP, invert polarities
        if (bjt_type == PNP) {
            vbe = -vbe;
            vbc = -vbc;
        }
        
        // Clamp exponentials to prevent overflow
        double exp_vbe = std::exp(std::min(vbe / VT, 80.0));
        double exp_vbc = std::exp(std::min(vbc / VT, 80.0));
        
        // Ebers-Moll diode currents
        double if_diode = IS * (exp_vbe - 1.0);  // Forward BE junction
        double ir_diode = IS * (exp_vbc - 1.0);  // Reverse BC junction
        
        // Terminal currents (Ebers-Moll equations)
        double ib = if_diode / BF + ir_diode / BR;
        double ic = if_diode - ir_diode;
        double ie = -(ib + ic);
        
        // Conductances (derivatives for linearization - Jacobian)
        double gbe = (IS / (BF * VT)) * exp_vbe;  // ∂ib/∂vbe
        double gbc = (IS / (BR * VT)) * exp_vbc;  // ∂ib/∂vbc
        double gce = (IS / VT) * exp_vbe;         // ∂ic/∂vbe
        double gcc = -(IS / VT) * exp_vbc;        // ∂ic/∂vbc
        
        // Equivalent current sources (companion model)
        // i = g*v + ieq  =>  ieq = i - g*v
        double ieq_b = ib - (gbe * vbe + gbc * vbc);
        double ieq_c = ic - (gce * vbe + gcc * vbc);
        double ieq_e = ie - (-(gbe + gce) * vbe - (gbc + gcc) * vbc);
        
        // For PNP, flip signs
        double sign = (bjt_type == PNP) ? -1.0 : 1.0;
        
        // ========================================
        // STAMPING - Complete 3x3 submatrix
        // ========================================
        
        // Base-Emitter junction (BE diode contribution)
        if (nb != 0 && ne != 0) {
            G(nb, nb) += gbe;
            G(nb, ne) -= gbe;
            G(ne, nb) -= gbe;
            G(ne, ne) += gbe;
        } else if (nb != 0) {
            G(nb, nb) += gbe;
        } else if (ne != 0) {
            G(ne, ne) += gbe;
        }
        
        // Base-Collector junction (BC diode contribution)
        if (nb != 0 && nc != 0) {
            G(nb, nb) += gbc;
            G(nb, nc) -= gbc;
            G(nc, nb) -= gbc;
            G(nc, nc) += gbc;
        } else if (nb != 0) {
            G(nb, nb) += gbc;
        } else if (nc != 0) {
            G(nc, nc) += gbc;
        }
        
        // Collector-Emitter controlled source (transistor action)
        if (nc != 0 && ne != 0) {
            G(nc, nb) += gce;
            G(nc, ne) -= gce;
            G(ne, nb) -= gce;
            G(ne, ne) += gce;
            
            G(nc, nb) += gcc;
            G(nc, nc) -= gcc;
            G(ne, nb) -= gcc;
            G(ne, nc) += gcc;
        } else if (nc != 0) {
            G(nc, nb) += gce + gcc;
            G(nc, nc) -= gcc;
        } else if (ne != 0) {
            G(ne, nb) -= gce + gcc;
            G(ne, nc) += gcc;
        }
        
        // Current sources (equivalent sources from companion model)
        if (nb != 0) I(nb) -= sign * ieq_b;
        if (nc != 0) I(nc) -= sign * ieq_c;
        if (ne != 0) I(ne) -= sign * ieq_e;
        
        // Save for next iteration
        vbe_prev = (bjt_type == PNP) ? -vbe : vbe;
        vbc_prev = (bjt_type == PNP) ? -vbc : vbc;
    }
    
    void updateHistory(const Eigen::VectorXd& V, double dt) override {
        // For BJT, history is updated during stamp
        // But we can update vbe_prev, vbc_prev here for clarity
        double vc = (nc != 0) ? V(nc) : 0.0;
        double vb = (nb != 0) ? V(nb) : 0.0;
        double ve = (ne != 0) ? V(ne) : 0.0;
        
        vbe_prev = vb - ve;
        vbc_prev = vb - vc;
        
        if (bjt_type == PNP) {
            vbe_prev = -vbe_prev;
            vbc_prev = -vbc_prev;
        }
    }
    
    void reset() override {
        vbe_prev = 0.0;
        vbc_prev = 0.0;
    }
    
private:
    // Voltage limiting to help Newton-Raphson convergence
    double limitJunction(double vnew, double vold) {
        double dv = vnew - vold;
        
        // Limit the change in junction voltage
        if (std::abs(dv) > V_LIMIT) {
            return vold + std::copysign(V_LIMIT, dv);
        }
        
        // Also limit absolute voltage for uninitialized case
        if (std::abs(vnew) > 1.0 && std::abs(vold) < 0.1) {
            return std::copysign(0.7, vnew);  // Start near typical Vbe
        }
        
        return vnew;
    }
};


class VoltageSource : public Component {
    
    
public:

    double voltage;
    
    VoltageSource(const std::string& nm, int np, int nn, double v) {
        type = ComponentType::VOLTAGE_SOURCE;
        name = nm;
        nodes = {np, nn};
        voltage = v;
        std::cout << "[VoltageSource] Created " << name
              << " (nm=" << nm << ", np=" << np << ", nn=" << nn << ")"
              << "  v=" << v << std::endl;
    }
    
    void setVoltage(double v) { voltage = v; }
    double getVoltage() const { return voltage; }
    
    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, 
               const Eigen::VectorXd& V, double dt) override {
        double Rs = 1.0;
        double g = 1.0 / Rs;
        int n1 = nodes[0], n2 = nodes[1];
        
        if (n1 != 0) {
            G(n1, n1) += g;
            if (n2 != 0) G(n1, n2) -= g;
            I(n1) += voltage * g;
        }
        if (n2 != 0) {
            G(n2, n2) += g;
            if (n1 != 0) G(n2, n1) -= g;
            I(n2) -= voltage * g;
        }
    }
};

// ============================================================================
// CIRCUIT
// ============================================================================

class Circuit {
public:
    std::vector<std::unique_ptr<Component>> components;
    int num_nodes;
    int input_node;
    int output_node;
    std::map<std::string, VoltageSource*> voltage_sources;
    
    Circuit() : num_nodes(0), input_node(-1), output_node(-1) {}
    
    bool loadNetlist(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Cannot open netlist: " << filename << std::endl;
            return false;
        }
        
        std::string line;
        int max_node = 0;
        
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '*' || line[0] == '#') continue;
            
            std::istringstream iss(line);
            std::string comp_name;
            iss >> comp_name;
            
            if (comp_name.empty()) continue;
            
            char type = std::toupper(comp_name[0]);
            
            
            switch(type) {
                case 'R': {
                    int n1, n2;
                    double value;
                    std::string unit;
                    iss >> n1 >> n2 >> value >> unit;
                    value *= parseUnit(unit);
                    components.push_back(std::make_unique<Resistor>(comp_name, n1, n2, value));
                    max_node = std::max(max_node, std::max(n1, n2));
                    break;
                }
                case 'C': {
                    int n1, n2;
                    double value;
                    std::string unit;
                    iss >> n1 >> n2 >> value >> unit;
                    value *= parseUnit(unit);
                    components.push_back(std::make_unique<Capacitor>(comp_name, n1, n2, value));
                    max_node = std::max(max_node, std::max(n1, n2));
                    break;
                }
                case 'D': {
                    int n1, n2;
                    std::string model;
                    iss >> n1 >> n2 >> model;
                    double Is, n, Vt;
                    std::string token;
                    while (iss >> token) {
                        if (token.find("Is=") == 0) Is = std::stod(token.substr(3));
                        else if (token.find("N=") == 0) n = std::stod(token.substr(2));
                        else if (token.find("Vt=") == 0) Vt = std::stod(token.substr(3));
                    }
                    components.push_back(std::make_unique<Diode>(comp_name, n1, n2, Is, n, Vt));
                    
                    max_node = std::max(max_node, std::max(n1, n2));
                    break;
                }
                case 'Q': {
                    int nc, nb, ne;
                    std::string model;
                    iss >> nc >> nb >> ne >> model;

                    // Default parameters
                    double Is, Bf, Br, Vt;

                    // Parse optional parameters
                    std::string token;
                    while (iss >> token) {
                        if (token.find("Is=") == 0)
                            Is = std::stod(token.substr(3));
                        else if (token.find("Bf=") == 0)
                            Bf = std::stod(token.substr(3));
                        else if (token.find("Br=") == 0)
                            Br = std::stod(token.substr(3));
                        else if (token.find("Vt=") == 0)
                            Vt = std::stod(token.substr(3));
                    }

                    components.push_back(std::make_unique<BJT>(
                        comp_name,           // name
                        nc,                  // collector node
                        nb,                  // base node  
                        ne,                  // emitter node
                        Bf,                  // beta (forward beta)
                        Br,                  // beta (reverse beta)
                        Is,                  // saturation current
                        Vt                   // Thermal voltage
                    ));
                    max_node = std::max({max_node, nc, nb, ne});
                    break;
                }
                case 'V': {
                    int n1, n2;
                    std::string dcstr;
                    double value;
                    iss >> n1 >> n2 >> dcstr >> value;
                    auto vs = std::make_unique<VoltageSource>(comp_name, n1, n2, value);
                    voltage_sources[comp_name] = vs.get();
                    components.push_back(std::move(vs));
                    max_node = std::max(max_node, std::max(n1, n2));
                    break;
                }
                case '.': {
                    if (comp_name == ".INPUT") {
                        iss >> input_node;
                    } else if (comp_name == ".OUTPUT") {
                        iss >> output_node;
                    }
                    break;
                }
                default: {
                    throw std::runtime_error("Component type unknown " + std::string(1, type));
                }
            }

        }
        
        num_nodes = max_node + 1;
        
        std::cout << "Loaded circuit: " << components.size() << " components, "
                  << num_nodes << " nodes" << std::endl;
        
        return input_node >= 0 && output_node >= 0;
    }
    
    void reset() {
        for (auto& comp : components) {
            comp->reset();
        }
    }
    
private:
    double parseUnit(const std::string& unit) {
        if (unit.empty()) return 1.0;
        switch(unit[0]) {
            case 'p': return 1e-12;
            case 'n': return 1e-9;
            case 'u': return 1e-6;
            case 'm': return 1e-3;
            case 'k': return 1e3;
            case 'M': return 1e6;
            default: 
                throw std::runtime_error("Unit cannot be determined: " + std::string(1, unit[0]));
            ;
        }
    }
};

// ============================================================================
// CIRCUIT SOLVER - Pure computational engine
// ============================================================================

class CircuitSolver {
private:
    Circuit& circuit;
    Eigen::MatrixXd G;
    Eigen::VectorXd I, V;
    double dt;
    int max_iterations;
    double tolerance;
    
public:
    CircuitSolver(Circuit& ckt, double sample_rate, 
                  int max_iter = 10, double tol = 1e-4) 
        : circuit(ckt), 
          dt(1.0 / sample_rate),
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
        if (circuit.voltage_sources.count("VIN") > 0) {
            circuit.voltage_sources["VIN"]->setVoltage(input_voltage);
        }
        
        // Newton-Raphson iteration
        double final_error = 0.0;
        
        for (int iter = 0; iter < max_iterations; iter++) {
            G.setZero();
            I.setZero();
            
            // Stamp all components
            for (auto& comp : circuit.components) {
                comp->stamp(G, I, V, dt);
            }
            
            // Ground node constraint (DOPO stamp per sicurezza)
            G(0, 0) = 1.0;
            I(0) = 0.0;
            
            // Solve linear system (usa LU per robustezza)
            Eigen::VectorXd V_new = G.lu().solve(I);
            
            // Check convergence
            double error = (V_new - V).norm();
            final_error = error;  // Salva per debug
            
            // Debug output
            if (sample_count < 5) {
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

// ============================================================================
// AUDIO PROCESSOR - Generic interface
// ============================================================================

class AudioProcessor {
protected:
    Circuit circuit;
    std::unique_ptr<CircuitSolver> solver;
    double sample_rate;
    
    // Realistic voltage levels (in Volts)
    double input_voltage_max;   // Max input voltage to circuit
    double output_voltage_max;  // Expected max output voltage
    
    // Audio normalization
    double input_headroom;   // How much of digital range to use
    double output_headroom;
    
    // Statistics
    float max_input_digital = 0.0f;
    float max_input_voltage = 0.0f;
    float max_output_voltage = 0.0f;
    float max_output_digital = 0.0f;
    size_t clip_count = 0;
    
public:
    AudioProcessor(const std::string& netlist_file, 
                   double sr,
                   double input_v_max = 0.2,      // 200mV (realistic guitar)
                   double output_v_max = 2.0,     // 2V expected output
                   double in_headroom = 0.8,      // Use 80% of digital range
                   double out_headroom = 0.8)
        : sample_rate(sr), 
          input_voltage_max(input_v_max),
          output_voltage_max(output_v_max),
          input_headroom(in_headroom),
          output_headroom(out_headroom) {
        
        std::cout << "\n=== Audio Processor Configuration ===" << std::endl;
        std::cout << "Input scaling:  ±" << input_headroom << " digital → ±" 
                  << input_voltage_max << "V" << std::endl;
        std::cout << "Output scaling: ±" << output_voltage_max << "V → ±" 
                  << output_headroom << " digital" << std::endl;
        
        if (!circuit.loadNetlist(netlist_file)) {
            throw std::runtime_error("Failed to load netlist");
        }
        
        solver = std::make_unique<CircuitSolver>(circuit, sample_rate);
        
        std::cout << "\n=== DC Operating Point ===" << std::endl;
        for (int n = 0; n < circuit.num_nodes; n++) {
            std::cout << "  V(" << n << ") = " << solver->getNodeVoltage(n) << " V";
            if (n == circuit.output_node) {
                std::cout << " ← OUTPUT DC BIAS";
            }
            std::cout << std::endl;
        }
        
        std::cout << "\nReady to process audio.\n" << std::endl;
    }
    
    virtual ~AudioProcessor() = default;
    
    void printStats() const {
        std::cout << "\n=== Processing Statistics ===" << std::endl;
        std::cout << "\nInput:" << std::endl;
        std::cout << "  Digital peak:  " << max_input_digital << " (max 1.0)" << std::endl;
        std::cout << "  Voltage peak:  " << max_input_voltage << " V" << std::endl;
        std::cout << "  Using:         " << (max_input_voltage / input_voltage_max * 100) 
                  << "% of range" << std::endl;
        
        std::cout << "\nCircuit Output:" << std::endl;
        std::cout << "  Voltage peak:  " << max_output_voltage << " V (DC coupled)" << std::endl;
        std::cout << "  Digital peak:  " << max_output_digital << " (after AC coupling)" << std::endl;
        
        if (clip_count > 0) {
            std::cout << "\n⚠ WARNING: " << clip_count << " samples clipped (soft)" << std::endl;
            std::cout << "  Consider:" << std::endl;
            std::cout << "  - Reduce input level (currently " << input_voltage_max << "V)" << std::endl;
            std::cout << "  - Increase output_voltage_max (currently " << output_voltage_max << "V)" << std::endl;
        }
        
        if (max_output_digital < 0.1) {
            std::cout << "\n⚠ WARNING: Output very quiet!" << std::endl;
            std::cout << "  Consider:" << std::endl;
            std::cout << "  - Increase input level" << std::endl;
            std::cout << "  - Decrease output_voltage_max" << std::endl;
            std::cout << "  - Check circuit gain is correct" << std::endl;
        }
        
        // Effective gain calculation (voltage domain)
        if (max_input_voltage > 0) {
            double voltage_gain = max_output_voltage / max_input_voltage;
            double gain_db = 20.0 * std::log10(voltage_gain);
            std::cout << "\nCircuit Gain:" << std::endl;
            std::cout << "  Voltage gain: " << voltage_gain << "x (" 
                      << gain_db << " dB)" << std::endl;
        }
        
    
    }

    // Process single sample - pure function
    float processSample(float input_digital) {
        // Track digital input level
        float abs_in = std::abs(input_digital);
        if (abs_in > max_input_digital) max_input_digital = abs_in;
        
        // Convert digital audio [-1, +1] to voltage
        // Use only 'input_headroom' of the range to avoid clipping input
        double v_in = (input_digital * input_headroom) * input_voltage_max;
        
        // Track voltage input
        double abs_v_in = std::abs(v_in);
        if (abs_v_in > max_input_voltage) max_input_voltage = abs_v_in;
        
        // Solve circuit
        if (solver->solve(v_in)) {
            double v_out = solver->getOutputVoltage();
            
            // Track output voltage (before DC removal)
            double abs_v_out = std::abs(v_out);
            if (abs_v_out > max_output_voltage) max_output_voltage = abs_v_out;
            
            // Remove DC bias (AC coupling simulation)
            static double dc_bias = v_out;  // Initialize on first sample
            const double alpha = 0.9995;    // High-pass filter (fc ~ 3Hz at 44.1kHz)
            dc_bias = alpha * dc_bias + (1 - alpha) * v_out;
            double v_ac = v_out - dc_bias;
            
            // Convert voltage to digital audio
            // Scale so that output_voltage_max → output_headroom
            float output_digital = static_cast<float>(
                (v_ac / output_voltage_max) * output_headroom
            );
            
            // Track output digital level
            float abs_out = std::abs(output_digital);
            if (abs_out > max_output_digital) max_output_digital = abs_out;
            
            // Soft clipping only if exceeds ±1.0 (prevent hard clipping)
            if (std::abs(output_digital) > 1.0) {
                clip_count++;
                return std::tanh(output_digital);  // Soft clip
            }
            
            return output_digital;
        }
        
        return 0.0f;
    }
    
    // Process block - generic interface for any audio source
    void processBlock(const float* input, float* output, size_t num_samples) {
        for (size_t i = 0; i < num_samples; i++) {
            output[i] = processSample(input[i]);
        }
    }
    
    void reset() {
        solver->reset();
        max_input_digital = 0.0f;
        max_input_voltage = 0.0f;
        max_output_voltage = 0.0f;
        max_output_digital = 0.0f;
        clip_count = 0;
    }
    
    void printAllNodeVoltages() const {
        if (!solver) return;

        const auto& V = solver->getVoltages();

        std::cout << "\n=== Node Voltages ===" << std::endl;
        for (int n = 0; n < circuit.num_nodes; n++) {
            double v = (n < V.size()) ? V(n) : 0.0;
            std::cout << "Node " << n << ": " << v << " V";
            if (n == 0) std::cout << " (GND)";
            std::cout << std::endl;
        }
        std::cout << "====================\n" << std::endl;
    }

    void setInputVoltageMax(double v) { 
        if (v > 0 && v < 10.0) {
            input_voltage_max = v;
            std::cout << "Input voltage range: ±" << v << "V" << std::endl;
        }
    }
    
    void setOutputVoltageMax(double v) { 
        if (v > 0 && v < 50.0) {
            output_voltage_max = v;
            std::cout << "Output voltage scaling: " << v << "V → ±1.0" << std::endl;
        }
    }
    
    double getInputVoltageMax() const { return input_voltage_max; }
    double getOutputVoltageMax() const { return output_voltage_max; }
};

#endif // CIRCUIT_SOLVER_H