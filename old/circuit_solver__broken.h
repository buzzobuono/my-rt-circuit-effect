// circuit_solver.h - Modular circuit simulation engine
// Independent from audio I/O source

#ifndef CIRCUIT_SOLVER_H
#define CIRCUIT_SOLVER_H
#include <iomanip>
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
    DIODE, BJT_NPN, BJT_PNP,
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
    double R;
    
public:
    Resistor(const std::string& n, int n1, int n2, double resistance) {
        type = ComponentType::RESISTOR;
        name = n;
        nodes = {n1, n2};
        R = resistance;
    }
    
    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, 
               const Eigen::VectorXd& V, double dt) override {
        double g = 1.0 / R;
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
};

class Capacitor : public Component {
    double C;
    double v_prev, i_prev;
    
public:
    Capacitor(const std::string& n, int n1, int n2, double capacitance) {
        type = ComponentType::CAPACITOR;
        name = n;
        nodes = {n1, n2};
        C = capacitance;
        v_prev = 0.0;
        i_prev = 0.0;
    }
    
    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, 
               const Eigen::VectorXd& V, double dt) override {
        double geq = 2.0 * C / dt;
        double ieq = geq * v_prev + i_prev;
        
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
        double v = V(nodes[0]) - V(nodes[1]);
        double geq = 2.0 * C / dt;
        i_prev = geq * (v - v_prev) - i_prev;
        v_prev = v;
    }
    
    void reset() override {
        v_prev = 0.0;
        i_prev = 0.0;
    }
};

class Diode : public Component {
    double Is;   // Saturation current
    double Vt;   // Thermal voltage
    double n;    // Ideality factor
    
public:
    // Costruttore con parametri personalizzabili
    Diode(const std::string& nm, int anode, int cathode,
               double is = 1e-14, double ideality = 1.0, double vt = 0.026) {
        type = ComponentType::DIODE;
        name = nm;
        nodes = {anode, cathode};
        Is = is;
        n = ideality;
        Vt = vt;
    }

    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I,
               const Eigen::VectorXd& V, double dt) override {
        double vd = V(nodes[0]) - V(nodes[1]);
        double id = Is * (std::exp(vd / (n * Vt)) - 1.0);
        double gd = (Is / (n * Vt)) * std::exp(vd / (n * Vt));
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

class BJT_NPN : public Component {
    double Is, Bf, Br, Vaf, Vt;
    
public:
    BJT_NPN(const std::string& nm, int c, int b, int e, 
            double is = 1e-14, double beta_f = 100.0, double beta_r = 1.0, double va = 100.0) {
        type = ComponentType::BJT_NPN;
        name = nm;
        nodes = {c, b, e};
        Is = is;
        Bf = beta_f;
        Br = beta_r;      // Reverse beta
        Vaf = va;         // Early voltage
        Vt = 0.026;       // Thermal voltage at 300K
    }
    
    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, 
               const Eigen::VectorXd& V, double dt) override {
        int nc = nodes[0], nb = nodes[1], ne = nodes[2];
        
        double vbe = V(nb) - V(ne);
        double vbc = V(nb) - V(nc);
        double vce = V(nc) - V(ne);
        
        // Limit voltages to prevent numerical overflow
        const double v_max = 0.7;  // ~27*Vt
        if (vbe > v_max) vbe = v_max;
        if (vbe < -v_max) vbe = -v_max;
        if (vbc > v_max) vbc = v_max;
        if (vbc < -v_max) vbc = -v_max;
        
        // Ebers-Moll with Early effect
        double exp_vbe = std::exp(vbe / Vt);
        double exp_vbc = std::exp(vbc / Vt);
        
        // Forward and reverse currents
        double Ibe = Is * (exp_vbe - 1.0);
        double Ibc = Is * (exp_vbc - 1.0);
        
        // Early effect factor
        double early_factor = 1.0;
        if (std::abs(Vaf) > 1e-6) {
            early_factor = 1.0 + vce / Vaf;
            if (early_factor < 0.1) early_factor = 0.1;  // Prevent negative
        }
        
        // Collector and base currents with Early effect
        double Ic = (Bf * Ibe - Ibc) * early_factor;
        double Ib = Ibe / (Bf + 1.0) + Ibc / (Br + 1.0);
        
        // Transconductances
        double gm_f = (Bf * Is / Vt) * exp_vbe * early_factor;
        double gm_r = (Is / Vt) * exp_vbc;
        double gib = (Is / ((Bf + 1.0) * Vt)) * exp_vbe + (Is / ((Br + 1.0) * Vt)) * exp_vbc;
        
        // Early effect conductance
        double go = 0.0;
        if (std::abs(Vaf) > 1e-6) {
            go = (Bf * Ibe - Ibc) / Vaf;
            if (go < 0) go = 0;  // Only positive output conductance
        }
        
        // Equivalent current sources
        double ieq_c = Ic - gm_f * vbe + gm_r * vbc - go * vce;
        double ieq_b = Ib - gib * vbe + gm_r * vbc;
        
        // Stamp collector current (C->E)
        if (nc != 0) {
            G(nc, nb) += gm_f - gm_r;
            G(nc, nc) += go + gm_r;
            if (ne != 0) {
                G(nc, ne) -= gm_f + go;
            }
            I(nc) -= ieq_c;
        }
        if (ne != 0) {
            if (nb != 0) G(ne, nb) -= gm_f - gm_r;
            G(ne, ne) += gm_f + go;
            if (nc != 0) G(ne, nc) -= go + gm_r;
            I(ne) += ieq_c;
        }
        
        // Stamp base current (B->E)
        if (nb != 0) {
            G(nb, nb) += gib + gm_r;
            if (ne != 0) G(nb, ne) -= gib;
            if (nc != 0) G(nb, nc) -= gm_r;
            I(nb) -= ieq_b;
        }
        if (ne != 0) {
            if (nb != 0) G(ne, nb) -= gib + gm_r;
            G(ne, ne) += gib;
            if (nc != 0) G(ne, nc) += gm_r;
            I(ne) += ieq_b;
        }
    }
};

class BJT_NPN___ : public Component {
    double Is, Bf, Vt;
    
public:
    BJT_NPN___(const std::string& nm, int c, int b, int e, 
            double is = 1e-14, double beta_f = 100.0, double beta_r = 1.0, double va = 100.0) {
        type = ComponentType::BJT_NPN;
        name = nm;
        nodes = {c, b, e};
        Is = is;
        Bf = beta_f;
        Vt = 0.026;
    }
    
    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, 
               const Eigen::VectorXd& V, double dt) override {
        int nc = nodes[0], nb = nodes[1], ne = nodes[2];
        
        double vbe = V(nb) - V(ne);
        
        // CRITICAL: Hard clamp Vbe to prevent exponential explosion
        // Real BJT: Vbe never exceeds ~0.75V in normal operation
        const double vbe_max = 0.75;
        const double vbe_min = 0.0;
        
        if (vbe > vbe_max) vbe = vbe_max;
        if (vbe < vbe_min) vbe = vbe_min;
        
        // Below threshold: BJT is OFF
        if (vbe < 0.4) {
            const double g_min = 1e-12;
            if (nb != 0 && ne != 0) {
                G(nb, nb) += g_min;
                G(nb, ne) -= g_min;
                G(ne, nb) -= g_min;
                G(ne, ne) += g_min;
            }
            return;
        }
        
        // Exponential (now safe because vbe <= 0.75V)
        // exp(0.75/0.026) = exp(28.8) ≈ 3e12
        double exp_vbe = std::exp(vbe / Vt);
        
        // Currents
        double Ibe = Is * (exp_vbe - 1.0);
        double Ic = Bf * Ibe;
        double Ib = Ic / Bf;
        
        // ADDITIONAL SAFETY: Clamp currents to reasonable values
        const double Ic_max = 0.1;  // 100mA max
        const double Ib_max = Ic_max / Bf;
        
        if (Ic > Ic_max) {
            Ic = Ic_max;
            Ib = Ib_max;
            Ibe = Ic / Bf;
        }
        
        // Transconductances
        double gm = (Bf * Is / Vt) * exp_vbe;
        double gib = (Is / Vt) * exp_vbe;
        
        // Clamp transconductances too
        const double gm_max = 10.0;  // 10 Siemens max
        if (gm > gm_max) gm = gm_max;
        if (gib > gm_max / Bf) gib = gm_max / Bf;
        
        // Floor
        const double g_min = 1e-12;
        gm = std::max(gm, g_min);
        gib = std::max(gib, g_min);
        
        // Norton equivalents
        double ieq_c = Ic - gm * vbe;
        double ieq_b = Ib - gib * vbe;
        
        // Stamp collector current (C → E)
        if (nc != 0) {
            G(nc, nb) += gm;
            if (ne != 0) G(nc, ne) -= gm;
            I(nc) -= ieq_c;
        }
        if (ne != 0) {
            if (nb != 0) G(ne, nb) -= gm;
            G(ne, ne) += gm;
            I(ne) += ieq_c;
        }
        
        // Stamp base current (B → E)
        if (nb != 0) {
            G(nb, nb) += gib;
            if (ne != 0) G(nb, ne) -= gib;
            I(nb) -= ieq_b;
        }
        if (ne != 0) {
            if (nb != 0) G(ne, nb) -= gib;
            G(ne, ne) += gib;
            I(ne) += ieq_b;
        }
    }
};

class VoltageSource : public Component {
    double voltage;
    
public:
    VoltageSource(const std::string& nm, int np, int nn, double v) {
        type = ComponentType::VOLTAGE_SOURCE;
        name = nm;
        nodes = {np, nn};
        voltage = v;
    }
    
    void setVoltage(double v) { voltage = v; }
    double getVoltage() const { return voltage; }
    
    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, 
               const Eigen::VectorXd& V, double dt) override {
        double Rs = 0.01;  // Era 1.0, ora 0.01
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
            
            try {
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
                        double Is = 1e-14, n = 1.0, Vt = 0.026;
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
                    //case 'Q': {
                    //    int nc, nb, ne;
                    //    std::string model;
                    //    iss >> nc >> nb >> ne >> model;
                    //    components.push_back(std::make_unique<BJT_NPN>(comp_name, nc, nb, ne));
                    //    max_node = std::max({max_node, nc, nb, ne});
                    //    break;
                    //}
                    case 'Q': {
                        int nc, nb, ne;
                        std::string model;
                        iss >> nc >> nb >> ne >> model;
                        
                        // Default parameters
                        double Is = 1e-14, Bf = 100.0, Br = 1.0, Vaf = 100.0;
                        
                        // Parse optional parameters
                        std::string token;
                        while (iss >> token) {
                            if (token.find("Is=") == 0) Is = std::stod(token.substr(3));
                            else if (token.find("Bf=") == 0) Bf = std::stod(token.substr(3));
                            else if (token.find("Br=") == 0) Br = std::stod(token.substr(3));
                            else if (token.find("Vaf=") == 0) Vaf = std::stod(token.substr(4));
                        }
                        
                        components.push_back(std::make_unique<BJT_NPN>(comp_name, nc, nb, ne, Is, Bf, Br, Vaf));
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
                        std::cerr << "Tipo di componente sconosciuto: " << type << std::endl;
                        exit(EXIT_FAILURE);
                    }
                }
            } catch (...) {
                std::cerr << "Error parsing line: " << line << std::endl;
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
            default: return 1.0;
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
    
    // Debug stats
    size_t total_samples = 0;
    size_t failed_samples = 0;
    
public:
    CircuitSolver(Circuit& ckt, double sample_rate, 
                  int max_iter = 20, double tol = 1e-3)  // Relaxed!
        : circuit(ckt), 
          dt(1.0 / sample_rate),
          max_iterations(max_iter),
          tolerance(tol) {
        
        G.resize(circuit.num_nodes, circuit.num_nodes);
        I.resize(circuit.num_nodes);
        V.resize(circuit.num_nodes);
        
        // Smart initialization
        V.setConstant(1.0);
        V(0) = 0.0;  // Ground
        
        // Initialize voltage source nodes to their voltages
        for (const auto& [name, vs] : circuit.voltage_sources) {
            int np = vs->nodes[0];
            if (np != 0) {
                V(np) = vs->getVoltage();
            }
        }
    }
    
    bool solve(double input_voltage) {
        total_samples++;
        
        // Set input
        if (circuit.voltage_sources.count("VIN") > 0) {
            circuit.voltage_sources["VIN"]->setVoltage(input_voltage);
        }
        
        // Newton-Raphson with damping
        for (int iter = 0; iter < max_iterations; iter++) {
            G.setZero();
            I.setZero();
            
            // Stamp all components
            for (auto& comp : circuit.components) {
                comp->stamp(G, I, V, dt);
            }
            
            // Ground node constraint
            G(0, 0) = 1e12;
            I(0) = 0;
            
            // Add g_min to all nodes for numerical stability (nodal conductance to ground)
            const double g_min = 1e-12;
            for (int n = 0; n < G.rows(); n++) {
                G(n, n) += g_min;
            }
            
            // Solve
            Eigen::VectorXd dV = G.ldlt().solve(I);
            
            // Check for NaN/Inf
            if (!dV.allFinite()) {
                failed_samples++;
                return false;
            }
            
            // Damped update for stability
            double alpha = 1.0;
            if (iter < 3) alpha = 0.5;  // Slow start
            
            Eigen::VectorXd V_new = V + alpha * (dV - V);
            
            // Voltage limiting (prevent runaway)
            for (int i = 0; i < V_new.size(); i++) {
                if (V_new(i) > 20.0) V_new(i) = 20.0;
                if (V_new(i) < -20.0) V_new(i) = -20.0;
            }
            
            // Check convergence
            double error = (V_new - V).norm();
            V = V_new;
            
            if (error < tolerance) {
                // Update history
                for (auto& comp : circuit.components) {
                    comp->updateHistory(V, dt);
                }
                return true;
            }
        }
        
        failed_samples++;
        return false;
    }
    
    double getOutputVoltage() const {
        return V(circuit.output_node);
    }
    
    double getNodeVoltage(int node) const {
        if (node >= 0 && node < circuit.num_nodes) {
            return V(node);
        }
        return 0.0;
    }
    
    void reset() {
        V.setZero();
        circuit.reset();
    }
    
    void printStats() const {
        std::cout << "\n=== Solver Statistics ===" << std::endl;
        std::cout << "Total samples: " << total_samples << std::endl;
        std::cout << "Failed samples: " << failed_samples 
                  << " (" << (100.0 * failed_samples / total_samples) << "%)" << std::endl;
        
        // Print DC operating point
        std::cout << "\n=== DC Operating Point ===" << std::endl;
        for (int i = 0; i < circuit.num_nodes; i++) {
            std::cout << "Node " << i << ": " << V(i) << " V" << std::endl;
        }
    }
    
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
    double input_gain;
    double output_gain;
    
    // Statistics
    float max_input = 0.0f;
    float max_output = 0.0f;
    float min_output = 0.0f;
    
public:
    AudioProcessor(const std::string& netlist_file, 
               double sr,
               double in_gain = 0.1,
               double out_gain = 5.0) 
        : sample_rate(sr), 
        input_gain(in_gain), 
        output_gain(out_gain) {
        
        // Load netlist
        if (!circuit.loadNetlist(netlist_file)) {
            throw std::runtime_error("Failed to load netlist");
        }
        
        // Create solver
        solver = std::make_unique<CircuitSolver>(circuit, sample_rate, 50, 1e-3);
        
        // ============================================================
        // SOURCE STEPPING DC INITIALIZATION
        // ============================================================
        std::cout << "\n=== SOURCE STEPPING DC ANALYSIS ===" << std::endl;
        
        // Find main power supply
        VoltageSource* main_supply = nullptr;
        double target_voltage = 9.0;
        
        if (circuit.voltage_sources.count("V1") > 0) {
            main_supply = circuit.voltage_sources["V1"];
            target_voltage = main_supply->getVoltage();
        } else if (circuit.voltage_sources.count("VCC") > 0) {
            main_supply = circuit.voltage_sources["VCC"];
            target_voltage = main_supply->getVoltage();
        }
        
        if (main_supply) {
            std::cout << "Ramping supply from 0V to " << target_voltage << "V..." << std::endl;
            
            // Source stepping: 20 steps from 0 to Vcc
            for (int step = 0; step <= 20; step++) {
                double v = (step / 20.0) * target_voltage;
                main_supply->setVoltage(v);
                
                // Solve multiple times at each voltage
                for (int iter = 0; iter < 50; iter++) {
                    solver->solve(0.0);
                }
                
                // Print every 4 steps
                if (step % 4 == 0) {
                    double vb = (circuit.num_nodes > 2) ? solver->getNodeVoltage(2) : 0;
                    std::cout << "  Step " << std::setw(2) << step 
                            << ": Vcc=" << std::fixed << std::setprecision(1) << v << "V"
                            << ", Vbase=" << std::setprecision(3) << vb << "V" << std::endl;
                }
            }
            
            // Restore full voltage and settle
            main_supply->setVoltage(target_voltage);
            std::cout << "\nSettling DC operating point..." << std::flush;
            for (int i = 0; i < 500; i++) {
                solver->solve(0.0);
            }
            std::cout << " done!" << std::endl;
            
            // Final bias check
            std::cout << "\n=== FINAL DC OPERATING POINT ===" << std::endl;
            for (int n = 0; n < circuit.num_nodes && n < 10; n++) {
                std::cout << "  Node " << n << ": " 
                        << std::setprecision(4) << solver->getNodeVoltage(n) << " V";
                if (n == 0) std::cout << " (ground)";
                else if (n == circuit.input_node) std::cout << " (input)";
                else if (n == circuit.output_node) std::cout << " (output)";
                std::cout << std::endl;
            }
            
            // BJT bias analysis
            if (circuit.num_nodes > 4) {
                double vb = solver->getNodeVoltage(2);
                double vc = solver->getNodeVoltage(3);
                double ve = solver->getNodeVoltage(4);
                
                std::cout << "\n=== BJT ANALYSIS ===" << std::endl;
                std::cout << "  Vbe = " << (vb - ve) << " V (should be ~0.65-0.75V)" << std::endl;
                std::cout << "  Vce = " << (vc - ve) << " V (should be >1V for active region)" << std::endl;
                
                if (vb < 0.5) {
                    std::cerr << "\n*** ERROR: Base voltage too low! BJT is OFF! ***" << std::endl;
                } else if (vc < 2.0 || vc > 7.0) {
                    std::cerr << "*** WARNING: Collector not at mid-rail, may clip! ***" << std::endl;
                } else {
                    std::cout << "\n*** DC bias looks GOOD! Ready to process audio. ***" << std::endl;
                }
            }
            
        } else {
            std::cerr << "Warning: No V1 or VCC found, skipping source stepping" << std::endl;
            for (int i = 0; i < 1000; i++) {
                solver->solve(0.0);
            }
        }
        std::cout << std::endl;
    }
    
    virtual ~AudioProcessor() = default;
    
    float processSample(float input_sample) {
        // Track input level
        float abs_in = std::abs(input_sample);
        if (abs_in > max_input) max_input = abs_in;
        
        double v_in = input_sample * input_gain;
        
        if (solver->solve(v_in)) {
            double v_out = solver->getOutputVoltage();
            float output = static_cast<float>(v_out * output_gain);
            
            // Track output without clipping
            if (output > max_output) max_output = output;
            if (output < min_output) min_output = output;
            
            // Soft clipping
            return std::tanh(output);
        }
        
        return 0.0f;
    }
    
    void processBlock(const float* input, float* output, size_t num_samples) {
        for (size_t i = 0; i < num_samples; i++) {
            output[i] = processSample(input[i]);
        }
    }
    
    void reset() {
        solver->reset();
        max_input = 0.0f;
        max_output = 0.0f;
        min_output = 0.0f;
    }
    
    void printStats() const {
        std::cout << "\n=== Audio Statistics ===" << std::endl;
        std::cout << "Max input: " << max_input << std::endl;
        std::cout << "Output range: [" << min_output << ", " << max_output << "]" << std::endl;
        std::cout << "Effective gain: " << (max_output / max_input) << "x" << std::endl;
        
        solver->printStats();
    }
    
    void setInputGain(double gain) { input_gain = gain; }
    void setOutputGain(double gain) { output_gain = gain; }
    double getInputGain() const { return input_gain; }
    double getOutputGain() const { return output_gain; }
};

#endif // CIRCUIT_SOLVER_H