// circuit_solver_clean.h - Cleaned modular circuit simulation engine
// Removed development/test instrumentation and debug prints.

#ifndef CIRCUIT_SOLVER_CLEAN_H
#define CIRCUIT_SOLVER_CLEAN_H

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
        double i_new = geq * v - (geq * v_prev + i_prev);
        i_prev = i_new;
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
    double Is, Bf, Vt;

public:
    BJT_NPN(const std::string& nm, int c, int b, int e,
            double is = 1e-14, double beta_f = 100.0, double /*beta_r*/ = 1.0, double /*va*/ = 100.0) {
        type = ComponentType::BJT_NPN;
        name = nm;
        nodes = {c, b, e};
        Is = is;
        Bf = beta_f;
        Vt = 0.026;
    }

    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I,
               const Eigen::VectorXd& V, double /*dt*/) override {
        int nc = nodes[0], nb = nodes[1], ne = nodes[2];

        double vbe = V(nb) - V(ne);

        // Limit exponent to avoid overflow in exp
        double safe_vbe = std::clamp(vbe, 0.0, 0.8);
        double exp_vbe = std::exp(std::min(safe_vbe / Vt, 40.0));

        // Transistor model (simplified Ebers-Moll forward-mode approximation)
        double Ibe = Is * (exp_vbe - 1.0);
        double Ic = Bf * Ibe;
        double Ib = Ibe;

        // Small-signal transconductances
        double gib = (Is / Vt) * exp_vbe;   // dIbe/dVbe
        double gm = Bf * gib;               // dIc/dVbe

        // Norton equivalents (linearized around operating point)
        double ieq_c = Ic - gm * vbe;
        double ieq_b = Ib - gib * vbe;

        // Stamp collector nodes (collector <-> base/emitter contributions)
        if (nc != 0) {
            if (nb != 0) G(nc, nb) += gm;
            if (ne != 0) G(nc, ne) -= gm;
            I(nc) -= ieq_c;
        }
        if (ne != 0) {
            if (nb != 0) G(ne, nb) -= gm;
            G(ne, ne) += gm;
            I(ne) += ieq_c;
        }

        // Stamp base-emitter branch
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

            // Stamp components
            for (auto& comp : circuit.components) comp->stamp(G, I, V, dt);

            // Ground node enforcement (strong conductance to ground)
            if (G.rows() > 0) {
                G(0, 0) += 1e12;
                I(0) = 0.0;
            }

            // small conductance to ground for numerical stability
            const double g_min = 1e-12;
            for (int n = 0; n < G.rows(); ++n) G(n, n) += g_min;

            // Solve linear system G * V_sol = I
            Eigen::VectorXd V_sol;
            Eigen::LDLT<Eigen::MatrixXd> ldlt(G);
            if (ldlt.isPositive()) {
                V_sol = ldlt.solve(I);
            } else {
                // fallback
                V_sol = G.fullPivLu().solve(I);
            }

            if (!V_sol.allFinite()) return false;

            // Damped update
            double alpha = (iter < 3) ? 0.5 : 1.0;
            Eigen::VectorXd V_new = V + alpha * (V_sol - V);

            // clamp voltages to reasonable limits
            for (int i = 0; i < V_new.size(); ++i) {
                if (V_new(i) > 100.0) V_new(i) = 100.0;
                if (V_new(i) < -100.0) V_new(i) = -100.0;
            }

            double error = (V_new - V).norm();
            V = V_new;

            if (error < tolerance) {
                for (auto& comp : circuit.components) comp->updateHistory(V, dt);
                return true;
            }
        }

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
    double input_gain;
    double output_gain;

public:
    AudioProcessor(const std::string& netlist_file,
                   double sr,
                   double in_gain = 0.5, 
                   double out_gain = 2.0) 
        : sample_rate(sr), 
          input_gain(in_gain), 
          output_gain(out_gain) {

        if (!circuit.loadNetlist(netlist_file)) {
            throw std::runtime_error("Failed to load netlist");
        }

        solver = std::make_unique<CircuitSolver>(circuit, sample_rate);
    }

    virtual ~AudioProcessor() = default;

    // Process single sample - pure function
    float processSample(float input_sample) {
        double v_in = input_sample * input_gain;

        if (solver->solve(v_in)) {
            double v_out = solver->getOutputVoltage();
            float output = static_cast<float>(v_out * output_gain);
            return std::tanh(output); // Soft clipping
        }

        return 0.0f; // Mute on non-convergence
    }

    // Process block - generic interface for any audio source
    void processBlock(const float* input, float* output, size_t num_samples) {
        for (size_t i = 0; i < num_samples; i++) {
            output[i] = processSample(input[i]);
        }
    }

    void reset() {
        solver->reset();
    }

    // Parameter access
    void setInputGain(double gain) { input_gain = gain; }
    void setOutputGain(double gain) { output_gain = gain; }
    double getInputGain() const { return input_gain; }
    double getOutputGain() const { return output_gain; }
    double getSampleRate() const { return sample_rate; }
};

#endif // CIRCUIT_SOLVER_H