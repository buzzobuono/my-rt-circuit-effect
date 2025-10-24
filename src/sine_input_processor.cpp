#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>

#include "audio_processor.h"

class SineInputProcessor {
private:
    AudioProcessor processor;
    double sample_rate;
    float amplitude;
    bool soft_clip;
    float frequency;
    float duration;

public:
    SineInputProcessor(const std::string& netlist,
                       double sample_rate,
                       float amplitude,
                       int input_impedance,
                       float frequency,
                       float duration,
                       bool soft_clip)
        : processor(netlist, sample_rate, input_impedance, 1.0, 1.0),
          sample_rate(sample_rate),
          amplitude(amplitude),
          frequency(frequency),
          duration(duration),
          soft_clip(soft_clip)
    {
        std::cout << "=== Sine Test Signal Generator ===" << std::endl;
        std::cout << "  Amplitude: ±" << amplitude << " V" << std::endl;
        std::cout << "  Clipping mode: " << (soft_clip ? "Soft (tanh)" : "Hard") << std::endl;
        std::cout << "  Sample rate: " << sample_rate << " Hz" << std::endl;
        std::cout << "  Frequency: " << frequency << " Hz" << std::endl;
        std::cout << "  Duration:  " << duration << " s" << std::endl;
    }

    bool process(const std::string& output_file) {
        size_t total_samples = static_cast<size_t>(sample_rate * duration);
        std::ofstream fout(output_file);
        if (!fout.is_open()) {
            std::cerr << "Error: cannot open output file " << output_file << "\n";
            return false;
        }

        fout << "# index;voltage_out\n";

        for (size_t i = 0; i < total_samples; ++i) {
            double t = i / sample_rate;
            float voltage_in = amplitude * std::sin(2.0 * M_PI * frequency * t);

            float voltage_out = processor.processSample(voltage_in);

            // Handle numerical issues
            if (std::isnan(voltage_out) || std::isinf(voltage_out))
            {
                std::cerr << "\nNumerical instability at sample " << i << std::endl;
                voltage_out = 0.0f;
            }


            // Optional soft clipping (analog-like saturation at output stage)
            if (soft_clip)
            {
                // Tanh soft clipping - simulates output stage saturation
                //voltage_out = std::tanh(voltage_out);
            }
            else
            {
                // Hard clipping to [-1, +1] to prevent digital overflow
                voltage_out = std::max(-1.0f, std::min(1.0f, voltage_out));
            }

            fout << i << ";" << voltage_out << "\n";

            
        }

        fout.close();
        std::cout << "✓ Output written to " << output_file << " (" 
                  << total_samples << " samples)\n";
        return true;
    }
};

// === MAIN TEST ===
int main(int argc, char* argv[]) {
    if (argc < 7) {
        std::cerr << "Usage:\n"
                  << argv[0] << " <output.csv> <circuit.cir> <amplitude[V]> <input_impedance[Ohm]> <freq[Hz]> <duration[s]> <samplerate[Hz]>\n\n"
                  << "Example:\n"
                  << argv[0] << " output.csv fuzzface.cir 0.150 10000 440 1 44100\n";
        return 1;
    }

    std::string output_file = argv[1];
    std::string netlist_file = argv[2];
    float amplitude = std::atof(argv[3]);
    float input_impedance = std::atoi(argv[4]);
    float frequency = std::atof(argv[5]);
    float duration = std::atof(argv[6]);
    double sample_rate = std::atof(argv[7]);

    try {
        SineInputProcessor test_proc(netlist_file, sample_rate, amplitude, input_impedance, frequency, duration, true);
        test_proc.process(output_file);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
