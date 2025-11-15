#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <sndfile.h>

#include "external/CLI11.hpp"

#include "circuit.h"
#include "circuit_solver.h"

class SpicePedalProcessor
{
private:
    Circuit circuit;
    std::unique_ptr<CircuitSolver> solver;
    std::string analysis_type;
    double sample_rate;
    int input_frequency;
    int input_duration;
    float max_input_voltage;
    int source_impedance;
    bool bypass;
    int max_iterations;
    double tolerance;

public:
    SpicePedalProcessor(std::string analysis_type,
                     const std::string &netlist_file,
                     double sample_rate,
                     int input_frequency,
                     int input_duration,
                     float max_input_voltage,
                     int source_impedance,
                     bool bypass,
                     int max_iterations,
                     double tolerance
                    )
        : analysis_type(analysis_type),
          sample_rate(sample_rate),
          input_frequency(input_frequency),
          input_duration(input_duration),
          max_input_voltage(max_input_voltage),
          source_impedance(source_impedance),
          bypass(bypass),
          max_iterations(max_iterations),
          tolerance(tolerance)
    {
        if (!circuit.loadNetlist(netlist_file)) {
            throw std::runtime_error("Failed to load netlist");
        }
        
        solver = std::make_unique<CircuitSolver>(circuit, sample_rate, source_impedance, max_iterations, tolerance);
    }

    bool process(const std::string &input_file, const std::string &output_file)
    {
        if (analysis_type == "DC") {
            solver->solveDC();
            return true;
        } else if (analysis_type == "TRAN") {
            std::vector<float> signalIn;
            if (!input_file.empty()) { 
                // Open Input WAV
                SF_INFO sfInfo;
                sfInfo.format = 0;
                SNDFILE* file = sf_open(input_file.c_str(), SFM_READ, &sfInfo);
                
                if (!file) {
                    std::cerr << "Errore apertura WAV: " << sf_strerror(file) << std::endl;
                    return false;
                }
                
                std::cout << "Input File Format" << std::endl;
                printFileFormat(input_file);
                
                if (sample_rate) sample_rate = sfInfo.samplerate;
                
                // Leggi tutti i sample
                std::vector<float> buffer(sfInfo.frames * sfInfo.channels);
                sf_count_t numFrames = sf_readf_float(file, buffer.data(), sfInfo.frames);
                sf_close(file);
                
                if (numFrames != sfInfo.frames) {
                    std::cerr << "Errore lettura sample" << std::endl;
                    return false;
                }
                
                // Estrai canale sinistro
                signalIn.resize(numFrames, 0.0f);
                for (sf_count_t i = 0; i < numFrames; i++) {
                    signalIn[i] = buffer[i * sfInfo.channels];
                }
            } else if (input_frequency != 0) {
                size_t total_samples = static_cast<size_t>(sample_rate * input_duration);
                signalIn.resize(total_samples, 0.0f);
                for (size_t i = 0; i < total_samples; ++i) {
                    double t = i / sample_rate;
                    signalIn[i] = std::sin(2.0 * M_PI * input_frequency * t);
                }
            }
            
            // Rimuovi DC offset
            float mean = 0.0f;
            for (float s : signalIn) mean += s;
            mean /= signalIn.size();
            
            for (float& s : signalIn) s -= mean;
            
            // Normalizza in Volt
            float maxNormalized = 0.0f;
            for (float s : signalIn) {
                maxNormalized = std::max(maxNormalized, std::abs(s));
            }
            
            float scale = 1;
            if (maxNormalized > 1e-10f) {
                scale = max_input_voltage / maxNormalized;
            }
            
            for (float& s : signalIn) s *= scale;
            
            if (!bypass) {
                solver->initialize();
            }
            
            std::vector<float> signalOut(signalIn.size());
            
            float peak_in = 0.0f, peak_out = 0.0f;
            float rms_in = 0.0f, rms_out = 0.0f;
            
            auto start = std::chrono::high_resolution_clock::now();
    
            for (size_t i = 0; i < signalIn.size(); i++) {
                if (!bypass) {
                    signalOut[i] = 0;
                    if (solver->solve(signalIn[i])) {
                        signalOut[i] = solver->getOutputVoltage();
                    }
                } else {
                    signalOut[i] = signalIn[i];
                }
                 
                // Update statistics
                peak_in = std::max(peak_in, std::abs(signalIn[i]));
                peak_out = std::max(peak_out, std::abs(signalOut[i]));
                rms_in += signalIn[i] * signalIn[i];
                rms_out += signalOut[i] * signalOut[i];
            }
            
            auto end = std::chrono::high_resolution_clock::now();
            
            rms_in = std::sqrt(rms_in / signalIn.size());
            rms_out = std::sqrt(rms_out / signalIn.size());
            
            float outputPeak = 0.0f;
            for (float v : signalOut) {
                outputPeak = std::max(outputPeak, std::abs(v));
            }
            
            solver->printDCOperatingPoint();
            
            // Print statistics
            std::cout << "Audio Statistics:" << std::endl;
            std::cout << "  Mean Input Signal " << mean << std::endl;
            std::cout << "  Max Normalized " << maxNormalized << " V, Scale Factor " << scale << std::endl;
            std::cout << "  Input Peak: " << peak_in << " V, " << 20 * std::log10(peak_in) << " dBFS, RMS: " << 20 * std::log10(rms_in) << " dBFS" << std::endl;
            std::cout << "  Output Peak: " << peak_out << " V, " << 20 * std::log10(peak_out) << " dBFS, RMS: " << 20 * std::log10(rms_out) << " dBFS" << std::endl;
            std::cout << "  Circuit gain: " << 20 * std::log10(rms_out / rms_in) << " dB" << std::endl;
            std::cout << std::endl;
            
            std::cout << "Process Statistics:" << std::endl;
            std::cout << "  Solver's Execution Time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " us" << std::endl;
            std::cout << "  Solver's Failure Percentage: " << solver->getFailurePercentage() << " %" << std::endl;
            std::cout << "  Solver's Tolal Samples: " << solver->getTotalSamples() << std::endl;
            std::cout << "  Solver's Total Iterations: " << solver->getTotalIterations() << std::endl;
            std::cout << "  Solver's Mean Iterations: " << solver->getMeanIterations() << std::endl;
            std::cout << std::endl;
            
            if (!output_file.empty()) {
                writeWav(signalOut, output_file, sample_rate);
            }
            return true;
        } else {
            std::cerr << "Analysis Type not valid: " << analysis_type << std::endl;
            return false;
        }
    }

    bool writeWav(std::vector<float> signalOut,
              const std::string& output_file,
              int sample_rate,
              int bitDepth = 24) {
        
        // Configura WAV
        SF_INFO sfInfo;
        sfInfo.samplerate = sample_rate;
        sfInfo.channels = 1;
        
        switch (bitDepth) {
            case 16: sfInfo.format = SF_FORMAT_WAV | SF_FORMAT_PCM_16; break;
            case 24: sfInfo.format = SF_FORMAT_WAV | SF_FORMAT_PCM_24; break;
            case 32: sfInfo.format = SF_FORMAT_WAV | SF_FORMAT_FLOAT; break;
            default:
                std::cerr << "Bit depth non supportato" << std::endl;
                return false;
        }
        
        if (!sf_format_check(&sfInfo)) {
            std::cerr << "Formato WAV invalido" << std::endl;
            return false;
        }
        
        // Scrivi WAV
        SNDFILE* file = sf_open(output_file.c_str(), SFM_WRITE, &sfInfo);
        if (!file) {
            std::cerr << "Errore apertura WAV: " << sf_strerror(file) << std::endl;
            return false;
        }
        
        sf_count_t written = sf_writef_float(file, signalOut.data(), signalOut.size());
        sf_close(file);
        
        if (written != (sf_count_t)signalOut.size()) {
            std::cerr << "Errore scrittura WAV" << std::endl;
            return false;
        }
        
        std::cout << "Output File Format" << std::endl;
        std::cout << "   File Name: " << output_file << std::endl;
        std::cout << "   Duration: " << (float)signalOut.size() / sample_rate << "s" << std::endl;
        
        printFileFormat(output_file);
        
        return true;
    }

    void printFileFormat(const std::string &file) {
        SF_INFO sfInfo;
        sfInfo.format = 0;
        SNDFILE* sound_file = sf_open(file.c_str(), SFM_READ, &sfInfo);
        sf_close(sound_file);
        std::cout << "   Channels: " << sfInfo.channels << std::endl;
        std::cout << "   Sample Rate: " << sfInfo.samplerate << "Hz" << std::endl;
        std::cout << "   Frames: " << sfInfo.frames << std::endl;
        std::cout << "   Numeric Format (bitmask): 0x" << std::hex << sfInfo.format << std::dec << std::endl;

        // Decodifica del formato audio (opzionale)
        int major_format = sfInfo.format & SF_FORMAT_TYPEMASK;
        int subtype = sfInfo.format & SF_FORMAT_SUBMASK;

        std::cout << "   Container Format: ";
        switch (major_format) {
            case SF_FORMAT_WAV:      std::cout << "WAV"; break;
            case SF_FORMAT_AIFF:     std::cout << "AIFF"; break;
            case SF_FORMAT_FLAC:     std::cout << "FLAC"; break;
            default:                 std::cout << "Other (" << std::hex << major_format << std::dec << ")"; break;
        }
        std::cout << std::endl;

        std::cout << "   Subtype (PCM, float, etc.): ";
        switch (subtype) {
            case SF_FORMAT_PCM_16:   std::cout << "PCM 16-bit"; break;
            case SF_FORMAT_PCM_24:   std::cout << "PCM 24-bit"; break;
            case SF_FORMAT_PCM_32:   std::cout << "PCM 32-bit"; break;
            case SF_FORMAT_FLOAT:    std::cout << "Float"; break;
            case SF_FORMAT_DOUBLE:   std::cout << "Double"; break;
            default:                 std::cout << "Other (" << std::hex << subtype << std::dec << ")"; break;
        }
        std::cout << std::endl;

        std::cout << std::endl;
    }

};

int main(int argc, char *argv[]) {
    CLI::App app{"SpicePedal: a realtime simple spice-like simulator for audio"};
    
    std::string analysis_type = "TRAN";
    std::string input_file;
    float input_frequency = 440;
    int input_duration = 2;
    float max_input_voltage = 0.15;
    int source_impedance = 25000;
    int sample_rate = 44100;
    std::string output_file;
    std::string netlist_file;
    bool bypass = false;
    int max_iterations = 20;
    double tolerance = 1e-6;
    
    app.add_option("-a,--analysis-type", analysis_type, "Analysis Type")->check(CLI::IsMember({"TRAN", "DC"}))->default_val(analysis_type);
    
    app.add_option("-i,--input-file", input_file, "Input File")->check(CLI::ExistingFile);
    app.add_option("-f,--input-frequency", input_frequency, "Input Frequency")->default_val(input_frequency);
    app.add_option("-d,--input-duration", input_duration, "Input Duration")->default_val(input_duration);
    app.add_option("-v,--input-voltage-amplitude", max_input_voltage, "Max Input Voltage")->check(CLI::Range(0.0f, 5.0f))->default_val(max_input_voltage);
    app.add_option("-I,--source_impedance", source_impedance, "Source Impedance")->check(CLI::Range(0, 30000))->default_val(source_impedance);
    app.add_option("-s,--sample-rate", sample_rate, "Sample Rate");
    
    app.add_option("-o,--output-file", output_file, "Output File");
    
    app.add_option("-c,--circuit", netlist_file, "Netlist File")->check(CLI::ExistingFile)->required();
    app.add_flag("-b,--bypass", bypass, "Bypass Circuit")->default_val(bypass);
    
    app.add_option("-m,--max-iterations", max_iterations, "Max Solver's Iterations")->default_val(max_iterations);
    app.add_option("-t,--tolerance", tolerance, "Solver's Tolerance")->default_val(tolerance);
    
    CLI11_PARSE(app, argc, argv);

    std::cout << "Input Parameters" << std::endl;
    std::cout << "   Analysis Type: " << analysis_type << std::endl;
    std::cout << "   Input File: " << input_file << std::endl;
    std::cout << "   Input Frequency: " << input_frequency << "Hz" << std::endl;
    std::cout << "   Input Duration: " << input_duration << "s" << std::endl;
    std::cout << "   Input Voltage Amplitude: " << max_input_voltage << "V" << std::endl;
    std::cout << "   Source Impedance: " << source_impedance << "Ω" << std::endl;
    std::cout << "   Sample Rate: " << sample_rate << "Hz" << std::endl;
    std::cout << "   Output File: " << output_file << std::endl;
    std::cout << "   Circuit File: " << netlist_file << std::endl;
    std::cout << "   Bypass Circuit: " << (bypass ? "True" : "False") << std::endl;
    std::cout << "   Max Iterations: " << max_iterations << std::endl;
    std::cout << "   Tolerance: " << tolerance << std::endl;
    std::cout << std::endl;

    try {
        SpicePedalProcessor processor(analysis_type, netlist_file, sample_rate, input_frequency, input_duration, max_input_voltage, source_impedance, bypass, max_iterations, tolerance);
        if (!processor.process(input_file, output_file)) {
            return 1;
        }
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
    
}

