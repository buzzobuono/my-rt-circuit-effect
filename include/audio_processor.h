#ifndef AUDIO_PROCESSOR_H
#define AUDIO_PROCESSOR_H

#include "circuit_solver.h"

class AudioProcessor {
protected:
    Circuit circuit;
    std::unique_ptr<CircuitSolver> solver;
    double sample_rate;
    
    // Realistic voltage levels (in Volts)
    double input_voltage_max;   // Max input voltage to circuit
    int input_impedance;
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
                   double input_v_max,
                   int input_impedance,
                   double output_v_max,
                   double in_headroom = 0.8,      // Use 80% of digital range
                   double out_headroom = 0.8)
        : sample_rate(sr), 
          input_voltage_max(input_v_max),
          input_impedance(input_impedance),
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
        
        solver = std::make_unique<CircuitSolver>(circuit, sample_rate, input_impedance);
        
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

#endif