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
    
    
public:
    double R;

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
    
    double getResistance() const { return R; }
};

class Capacitor : public Component {
    double C;
    double v_prev;   // tensione al passo precedente
    double i_prev;   // corrente equivalente al passo precedente

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
        // tensione attuale
        double v = V(nodes[0]) - V(nodes[1]);

        // aggiorna corrente equivalente per il prossimo passo
        i_prev = i_prev + 2.0 * C / dt * (v - v_prev);

        // salva tensione corrente
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
    
public:

    double Is, Bf, Vt;

    BJT_NPN(const std::string& nm, int c, int b, int e,
            double is = 1e-14, double beta = 200.0)
    {
        type = ComponentType::BJT_NPN;
        name = nm;
        nodes = {c, b, e};
        Is = is;
        Bf = beta;
        Vt = 0.026; // thermal voltage at 300K
        std::cout << "[BJT_NPN] Created " << name
              << " (C=" << c << ", B=" << b << ", E=" << e << ")"
              << "  Is=" << Is << "  Bf=" << Bf << std::endl;
    }

    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I,
           const Eigen::VectorXd& V, double /*dt*/) override
    {
        int nc = nodes[0], nb = nodes[1], ne = nodes[2];

        auto safeV = [&](int idx){ return (idx >= 0 && idx < V.size()) ? V(idx) : 0.0; };
        double vbe = safeV(nb) - safeV(ne);
        vbe = std::clamp(vbe, -0.2, 0.8);
        double exp_vbe = std::exp(std::min(vbe / Vt, 40.0));

        // Correct currents
        double Ic = Is * (exp_vbe - 1.0);   // collector current (physical)
        double Ib = Ic / Bf;                // base current
        // small-signal derivatives
        double gm = (Is / Vt) * exp_vbe;    // dIc/dVbe
        double gib = gm / Bf;               // dIb/dVbe  (CORRECT)

        const double gmin = 1e-12;
        gm  = std::max(gm, gmin);
        gib = std::max(gib, gmin);

        #ifdef DEBUG
        std::cout << "[BJT_NPN] stamp " << name
                << " vbe=" << vbe
                << " Ic=" << Ic << " Ib=" << Ib
                << " gm=" << gm << " gib=" << gib
                << " nodes(C,B,E)=("<<nc<<","<<nb<<","<<ne<<")\n";
        #endif
        
        if (nc != 0) {
            if (nb != 0) G(nc, nb) += gm;
            if (ne != 0) G(nc, ne) -= gm;
            I(nc) -= Ic - gm * vbe;
        }
        if (ne != 0) {
            if (nb != 0) G(ne, nb) -= gm;
            G(ne, ne) += gm;
            I(ne) += Ic - gm * vbe;
        }

        // Base-emitter branch
        if (nb != 0) {
            G(nb, nb) += gib;
            if (ne != 0) G(nb, ne) -= gib;
            I(nb) -= Ib - gib * vbe;
        }
        if (ne != 0) {
            if (nb != 0) G(ne, nb) -= gib;
            G(ne, ne) += gib;
            I(ne) += Ib - gib * vbe;
        }
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
                    case 'Q': {
                        int nc, nb, ne;
                        std::string model;
                        iss >> nc >> nb >> ne >> model;

                        // Default parameters
                        double Is = 1e-14;
                        double Bf = 100.0;

                        // Parse optional parameters
                        std::string token;
                        while (iss >> token) {
                            if (token.find("Is=") == 0)
                                Is = std::stod(token.substr(3));
                            else if (token.find("Bf=") == 0)
                                Bf = std::stod(token.substr(3));
                            // Ignora Br, Vaf ecc. se presenti
                        }

                        // Crea il transistor con il nuovo costruttore
                        components.push_back(std::make_unique<BJT_NPN>(comp_name, nc, nb, ne, Is, Bf));

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
    
    // Main solve method - completely independent from audio source
    bool solve(double input_voltage) {
        // Set input voltage if VIN source exists
        if (circuit.voltage_sources.count("VIN") > 0) {
            circuit.voltage_sources["VIN"]->setVoltage(input_voltage);
        }
        
        // Newton-Raphson iteration
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
            
            // Solve linear system
            Eigen::VectorXd V_new = G.ldlt().solve(I);
            
            // Check convergence
            double error = (V_new - V).norm();
            V = V_new;
            
            if (error < tolerance) {
                // Update component history (capacitors, etc.)
                for (auto& comp : circuit.components) {
                    comp->updateHistory(V, dt);
                }
                return true;
            }
        }
        
        return false; // Did not converge
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

    void printAllComponentCurrents() const {
        if (!solver) return;

        const auto& V = solver->getVoltages();  // usa la reference al vettore dei nodi

        std::cout << "\n=== Component Currents ===" << std::endl;

        for (const auto& comp : circuit.components) {

            switch (comp->type) {

                // 🔹 RESISTOR
                case ComponentType::RESISTOR: {
                    auto* r = dynamic_cast<Resistor*>(comp.get());
                    if (!r) break;

                    int n1 = r->nodes[0];
                    int n2 = r->nodes[1];
                    double v1 = (n1 < V.size()) ? V(n1) : 0.0;
                    double v2 = (n2 < V.size()) ? V(n2) : 0.0;
                    double i = (r->R != 0.0) ? (v1 - v2) / r->R : 0.0;

                    std::cout << r->name << " (R): " << i << " A" << std::endl;
                    break;
                }

                // 🔹 VOLTAGE SOURCE
                case ComponentType::VOLTAGE_SOURCE: {
                    auto* v = dynamic_cast<VoltageSource*>(comp.get());
                    if (!v) break;

                    int n1 = v->nodes[0];
                    int n2 = v->nodes[1];
                    double v1 = (n1 < V.size()) ? V(n1) : 0.0;
                    double v2 = (n2 < V.size()) ? V(n2) : 0.0;

                    std::cout << v->name << " (Vsrc): V=" << v1 - v2
                            << " (nominale " << v->voltage << " V)" << std::endl;
                    break;
                }

                // 🔹 BJT_NPN
                case ComponentType::BJT_NPN: {
                    auto* q = dynamic_cast<BJT_NPN*>(comp.get());
                    if (!q) break;

                    int nc = q->nodes[0];
                    int nb = q->nodes[1];
                    int ne = q->nodes[2];
                    double vc = (nc < V.size()) ? V(nc) : 0.0;
                    double vb = (nb < V.size()) ? V(nb) : 0.0;
                    double ve = (ne < V.size()) ? V(ne) : 0.0;

                    double vbe = vb - ve;
                    double exp_vbe = std::exp(std::clamp(vbe / q->Vt, -40.0, 40.0));

                    double ic = q->Bf * q->Is * (exp_vbe - 1.0);
                    double ib = ic / q->Bf;
                    double ie = ic + ib;

                    std::cout << q->name << " (BJT): Ic=" << ic
                            << " Ib=" << ib
                            << " Ie=" << ie
                            << " Vbe=" << vbe << std::endl;
                    break;
                }

                default:
                    std::cout << comp->name << " (tipo non gestito)" << std::endl;
                    break;
            }
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