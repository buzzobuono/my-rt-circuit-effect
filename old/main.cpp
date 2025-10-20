// wav_processor.cpp - Offline WAV file circuit simulator
// Compile: g++ -std=c++17 wav_processor.cpp -o wav_processor -O3
// Usage: ./wav_processor input.wav output.wav circuit.cir

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <memory>
#include <map>
#include <cmath>
#include <string>
#include <cstring>
#include <Eigen/Dense>

// ============================================================================
// WAV FILE HANDLER
// ============================================================================

struct WavHeader {
    char riff[4];           // "RIFF"
    uint32_t file_size;     // File size - 8
    char wave[4];           // "WAVE"
    char fmt[4];            // "fmt "
    uint32_t fmt_size;      // 16 for PCM
    uint16_t audio_format;  // 1 for PCM
    uint16_t num_channels;  // 1 = mono, 2 = stereo
    uint32_t sample_rate;   // 44100, 48000, etc.
    uint32_t byte_rate;     // sample_rate * num_channels * bits_per_sample/8
    uint16_t block_align;   // num_channels * bits_per_sample/8
    uint16_t bits_per_sample; // 16, 24, 32
    char data[4];           // "data"
    uint32_t data_size;     // Number of bytes in data
};

class WavFile {
private:
    WavHeader header;
    std::vector<float> samples;
    
public:
    bool load(const std::string& filename) {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "Cannot open file: " << filename << std::endl;
            return false;
        }
        
        // Read header
        file.read(reinterpret_cast<char*>(&header), sizeof(WavHeader));
        
        // Validate
        if (std::strncmp(header.riff, "RIFF", 4) != 0 ||
            std::strncmp(header.wave, "WAVE", 4) != 0) {
            std::cerr << "Not a valid WAV file" << std::endl;
            return false;
        }
        
        // Read samples
        int num_samples = header.data_size / (header.bits_per_sample / 8);
        samples.resize(num_samples);
        
        if (header.bits_per_sample == 16) {
            std::vector<int16_t> raw_samples(num_samples);
            file.read(reinterpret_cast<char*>(raw_samples.data()), header.data_size);
            
            // Convert to float [-1.0, 1.0]
            for (int i = 0; i < num_samples; i++) {
                samples[i] = raw_samples[i] / 32768.0f;
            }
        } else if (header.bits_per_sample == 32) {
            file.read(reinterpret_cast<char*>(samples.data()), header.data_size);
        } else {
            std::cerr << "Unsupported bit depth: " << header.bits_per_sample << std::endl;
            return false;
        }
        
        file.close();
        
        std::cout << "Loaded WAV: " << num_samples << " samples, "
                  << header.sample_rate << " Hz, "
                  << header.num_channels << " channel(s), "
                  << header.bits_per_sample << " bit" << std::endl;
        
        return true;
    }
    
    bool save(const std::string& filename, const std::vector<float>& output_samples) {
        std::ofstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "Cannot create file: " << filename << std::endl;
            return false;
        }
        
        // Update header for output
        WavHeader out_header = header;
        out_header.data_size = output_samples.size() * sizeof(int16_t);
        out_header.file_size = out_header.data_size + sizeof(WavHeader) - 8;
        
        // Write header
        file.write(reinterpret_cast<char*>(&out_header), sizeof(WavHeader));
        
        // Convert float to int16 and write
        std::vector<int16_t> raw_samples(output_samples.size());
        for (size_t i = 0; i < output_samples.size(); i++) {
            float clamped = std::max(-1.0f, std::min(1.0f, output_samples[i]));
            raw_samples[i] = static_cast<int16_t>(clamped * 32767.0f);
        }
        
        file.write(reinterpret_cast<char*>(raw_samples.data()), out_header.data_size);
        file.close();
        
        std::cout << "Saved WAV: " << output_samples.size() << " samples" << std::endl;
        
        return true;
    }
    
    const std::vector<float>& getSamples() const { return samples; }
    uint32_t getSampleRate() const { return header.sample_rate; }
    uint16_t getChannels() const { return header.num_channels; }
};

// ============================================================================
// COMPONENT BASE CLASS (same as before)
// ============================================================================

enum class ComponentType {
    RESISTOR, CAPACITOR, INDUCTOR,
    DIODE, BJT_NPN, BJT_PNP,
    VOLTAGE_SOURCE, INPUT, OUTPUT
};

class Component {
public:
    ComponentType type;
    std::string name;
    std::vector<int> nodes;
    
    virtual ~Component() = default;
    virtual double getCurrent(const Eigen::VectorXd& V, double dt) = 0;
    virtual double getAdmittance(const Eigen::VectorXd& V, double dt) = 0;
    virtual void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, 
                      const Eigen::VectorXd& V, double dt) = 0;
};

// ============================================================================
// RESISTOR
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
    
    double getCurrent(const Eigen::VectorXd& V, double dt) override {
        return (V(nodes[0]) - V(nodes[1])) / R;
    }
    
    double getAdmittance(const Eigen::VectorXd& V, double dt) override {
        return 1.0 / R;
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

// ============================================================================
// CAPACITOR
// ============================================================================

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
    
    double getCurrent(const Eigen::VectorXd& V, double dt) override {
        double v = V(nodes[0]) - V(nodes[1]);
        double i = (2.0 * C / dt) * (v - v_prev) - i_prev;
        return i;
    }
    
    double getAdmittance(const Eigen::VectorXd& V, double dt) override {
        return 2.0 * C / dt;
    }
    
    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, 
               const Eigen::VectorXd& V, double dt) override {
        double geq = 2.0 * C / dt;
        double v = V(nodes[0]) - V(nodes[1]);
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
        
        v_prev = v;
        i_prev = getCurrent(V, dt);
    }
};

// ============================================================================
// DIODE
// ============================================================================

class Diode : public Component {
    double Is, Vt, n;
    
public:
    Diode(const std::string& nm, int anode, int cathode, 
          double is = 1e-14, double ideality = 1.0) {
        type = ComponentType::DIODE;
        name = nm;
        nodes = {anode, cathode};
        Is = is;
        n = ideality;
        Vt = 0.026;
    }
    
    double getCurrent(const Eigen::VectorXd& V, double dt) override {
        double vd = V(nodes[0]) - V(nodes[1]);
        return Is * (std::exp(vd / (n * Vt)) - 1.0);
    }
    
    double getAdmittance(const Eigen::VectorXd& V, double dt) override {
        double vd = V(nodes[0]) - V(nodes[1]);
        return (Is / (n * Vt)) * std::exp(vd / (n * Vt));
    }
    
    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, 
               const Eigen::VectorXd& V, double dt) override {
        double vd = V(nodes[0]) - V(nodes[1]);
        double id = getCurrent(V, dt);
        double gd = getAdmittance(V, dt);
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

// ============================================================================
// BJT NPN
// ============================================================================

class BJT_NPN : public Component {
    double Is, Bf, Vt;
    
public:
    BJT_NPN(const std::string& nm, int c, int b, int e, 
            double is = 1e-16, double beta = 100.0) {
        type = ComponentType::BJT_NPN;
        name = nm;
        nodes = {c, b, e};
        Is = is;
        Bf = beta;
        Vt = 0.026;
    }
    
    double getCurrent(const Eigen::VectorXd& V, double dt) override {
        double vbe = V(nodes[1]) - V(nodes[2]);
        return Is * (std::exp(vbe / Vt) - 1.0);
    }
    
    double getAdmittance(const Eigen::VectorXd& V, double dt) override {
        double vbe = V(nodes[1]) - V(nodes[2]);
        return (Is / Vt) * std::exp(vbe / Vt);
    }
    
    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, 
               const Eigen::VectorXd& V, double dt) override {
        int nc = nodes[0], nb = nodes[1], ne = nodes[2];
        
        double vbe = V(nb) - V(ne);
        double vbc = V(nb) - V(nc);
        
        double ic = Bf * Is * (std::exp(vbe / Vt) - 1.0);
        double gm = (Bf * Is / Vt) * std::exp(vbe / Vt);
        
        double ib = (Is / Bf) * (std::exp(vbe / Vt) - 1.0);
        double gib = (Is / (Bf * Vt)) * std::exp(vbe / Vt);
        
        if (nc != 0) {
            G(nc, nb) += gm;
            G(nc, ne) -= gm;
            I(nc) -= ic - gm * vbe;
        }
        if (ne != 0) {
            G(ne, nb) -= gm;
            G(ne, ne) += gm;
            I(ne) += ic - gm * vbe;
        }
        
        if (nb != 0) {
            G(nb, nb) += gib;
            if (ne != 0) G(nb, ne) -= gib;
            I(nb) -= ib - gib * vbe;
        }
        if (ne != 0 && nb != 0) {
            G(ne, nb) -= gib;
            G(ne, ne) += gib;
            I(ne) += ib - gib * vbe;
        }
    }
};

// ============================================================================
// VOLTAGE SOURCE
// ============================================================================

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
    
    double getCurrent(const Eigen::VectorXd& V, double dt) override {
        return 0.0;
    }
    
    double getAdmittance(const Eigen::VectorXd& V, double dt) override {
        return 1e9;
    }
    
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
// CIRCUIT & NETLIST PARSER
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
                        iss >> n1 >> n2;
                        components.push_back(std::make_unique<Diode>(comp_name, n1, n2));
                        max_node = std::max(max_node, std::max(n1, n2));
                        break;
                    }
                    case 'Q': {
                        int nc, nb, ne;
                        std::string model;
                        iss >> nc >> nb >> ne >> model;
                        components.push_back(std::make_unique<BJT_NPN>(comp_name, nc, nb, ne));
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
                }
            } catch (...) {
                std::cerr << "Error parsing line: " << line << std::endl;
            }
        }
        
        num_nodes = max_node + 1;
        
        std::cout << "Loaded circuit: " << components.size() << " components, "
                  << num_nodes << " nodes" << std::endl;
        std::cout << input_node;
        std::cout << output_node;

        return input_node >= 0 && output_node >= 0;
    }
    
private:
    double parseUnit(const std::string& unit) {
        if (unit.empty()) return 1.0;
        char c = std::tolower(unit[0]);
        switch(c) {
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
// CIRCUIT SOLVER
// ============================================================================

class CircuitSolver {
private:
    Circuit& circuit;
    Eigen::MatrixXd G;
    Eigen::VectorXd I, V;
    double dt;
    
public:
    CircuitSolver(Circuit& ckt, double sample_rate) 
        : circuit(ckt), dt(1.0 / sample_rate) {
        
        G.resize(circuit.num_nodes, circuit.num_nodes);
        I.resize(circuit.num_nodes);
        V.resize(circuit.num_nodes);
        V.setZero();
    }
    
    bool solve(double input_voltage, int max_iterations = 10) {
        const double tolerance = 1e-4;
        
        if (circuit.voltage_sources.count("VIN") > 0) {
            circuit.voltage_sources["VIN"]->setVoltage(input_voltage);
        }
        
        for (int iter = 0; iter < max_iterations; iter++) {
            G.setZero();
            I.setZero();
            
            for (auto& comp : circuit.components) {
                comp->stamp(G, I, V, dt);
            }
            
            G(0, 0) = 1e12;
            I(0) = 0;
            
            Eigen::VectorXd V_new = G.ldlt().solve(I);
            
            double error = (V_new - V).norm();
            V = V_new;
            
            if (error < tolerance) {
                return true;
            }
        }
        
        return false;
    }
    
    double getOutputVoltage() {
        return V(circuit.output_node);
    }
    
    void reset() {
        V.setZero();
    }
};

// ============================================================================
// WAV PROCESSOR
// ============================================================================

class WavProcessor {
private:
    Circuit circuit;
    std::unique_ptr<CircuitSolver> solver;
    double input_gain;
    double output_gain;
    
public:
    WavProcessor(const std::string& netlist_file, double sample_rate,
                 double in_gain = 0.5, double out_gain = 2.0) 
        : input_gain(in_gain), output_gain(out_gain) {
        
        if (!circuit.loadNetlist(netlist_file)) {
            throw std::runtime_error("Failed to load netlist");
        }
        
        solver = std::make_unique<CircuitSolver>(circuit, sample_rate);
    }
    
    std::vector<float> process(const std::vector<float>& input) {
        std::vector<float> output(input.size());
        
        int total = input.size();
        int progress_step = total / 20;
        
        std::cout << "Processing " << total << " samples..." << std::endl;
        
        for (size_t i = 0; i < input.size(); i++) {
            // Progress indicator
            if (i % progress_step == 0) {
                int percent = (i * 100) / total;
                std::cout << "\rProgress: " << percent << "%" << std::flush;
            }
            
            // Convert audio sample to voltage
            double v_in = input[i] * input_gain;
            
            // Solve circuit
            bool converged = solver->solve(v_in);
            
            if (converged) {
                double v_out = solver->getOutputVoltage();
                output[i] = static_cast<float>(v_out * output_gain);
                
                // Soft clipping on output
                output[i] = std::tanh(output[i]);
            } else {
                output[i] = 0.0f;
            }
        }
        
        std::cout << "\rProgress: 100%" << std::endl;
        
        return output;
    }
    
    void reset() {
        solver->reset();
    }
};

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char* argv[]) {
    std::cout << "=== WAV Circuit Processor ===" << std::endl;
    
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <input.wav> <output.wav> <circuit.cir> [input_gain] [output_gain]" << std::endl;
        std::cerr << "Example: " << argv[0] << " guitar.wav processed.wav fuzz_face.cir 0.5 2.0" << std::endl;
        return 1;
    }
    
    std::string input_file = argv[1];
    std::string output_file = argv[2];
    std::string netlist_file = argv[3];
    
    double input_gain = 0.5;
    double output_gain = 2.0;
    
    if (argc >= 5) input_gain = std::atof(argv[4]);
    if (argc >= 6) output_gain = std::atof(argv[5]);
    
    try {
        // Load input WAV
        WavFile wav;
        if (!wav.load(input_file)) {
            return 1;
        }
        
        // Create processor
        WavProcessor processor(netlist_file, wav.getSampleRate(), 
                              input_gain, output_gain);
        
        // Process audio
        std::vector<float> output_samples = processor.process(wav.getSamples());
        
        // Save output WAV
        if (!wav.save(output_file, output_samples)) {
            return 1;
        }
        
        std::cout << "Success! Processed audio saved to: " << output_file << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}