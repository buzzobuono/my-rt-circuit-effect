// wav_app.cpp - Corrected WAV file processor
// Compile: g++ -std=c++17 -O3 -I/usr/include/eigen3 wav_app.cpp -o wav_processor -lsndfile

#include "circuit_solver.h"
#include <sndfile.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

/*
    # Basso passivo (Fender P-Bass)
    ./wav_processor bass.wav output.wav circuit.cir 0.3 9

    # Basso attivo (Ibanez SR con preamp)
    ./wav_processor bass.wav output.wav circuit.cir 1.5 9

    # Chitarra hot pickup (EMG)
    ./wav_processor guitar.wav output.wav circuit.cir 1.0 18
*/

class WavFileProcessor {
private:
    AudioProcessor processor;
    float input_voltage_scale;   // WAV [-1,+1] → Voltage
    float output_voltage_scale;  // Voltage → WAV [-1,+1]
    float supply_voltage;        // Circuit power supply rails
    bool use_soft_clipping;
    bool auto_normalize_output;
    
public:
    WavFileProcessor(const std::string& netlist, double sample_rate,
                     float input_peak_voltage = 0.5f,    // Typical passive bass
                     float supply_v = 0.0f,              // Typical pedal
                     bool soft_clip = true,
                     bool auto_normalize = true)
        : processor(netlist, sample_rate, 1.0, 1.0),    // No gain in processor
          input_voltage_scale(input_peak_voltage),
          supply_voltage(supply_v),
          use_soft_clipping(soft_clip),
          auto_normalize_output(auto_normalize) {
        
        // Output scale: normalize back from supply voltage to [-1, +1]
        output_voltage_scale = 1.0f / supply_voltage;
        
        std::cout << "Audio scaling configuration:" << std::endl;
        std::cout << "  Input peak voltage: ±" << input_voltage_scale << " V" << std::endl;
        std::cout << "  Supply voltage: ±" << supply_voltage << " V" << std::endl;
        std::cout << "  Clipping mode: " << (soft_clip ? "Soft (tanh)" : "Hard") << std::endl;
    }
    
    bool process(const std::string& input_file, const std::string& output_file) {
        // Open input file
        SF_INFO sf_info;
        std::memset(&sf_info, 0, sizeof(sf_info));
        
        SNDFILE* infile = sf_open(input_file.c_str(), SFM_READ, &sf_info);
        if (!infile) {
            std::cerr << "Error opening input file: " << sf_strerror(nullptr) << std::endl;
            return false;
        }
        
        std::cout << "\nInput WAV:" << std::endl;
        std::cout << "  Channels: " << sf_info.channels << std::endl;
        std::cout << "  Sample rate: " << sf_info.samplerate << " Hz" << std::endl;
        std::cout << "  Frames: " << sf_info.frames << std::endl;
        std::cout << "  Duration: " << (double)sf_info.frames / sf_info.samplerate << " s" << std::endl;
        
        // Read samples (interleaved if stereo)
        std::vector<float> interleaved(sf_info.frames * sf_info.channels);
        sf_count_t read_count = sf_read_float(infile, interleaved.data(), interleaved.size());
        sf_close(infile);

        if (read_count != sf_info.frames * sf_info.channels) {
            std::cerr << "Warning: Read " << read_count << " of " 
                      << sf_info.frames * sf_info.channels << " samples" << std::endl;
        }

        // Extract first channel only (mono processing)
        std::vector<float> input_samples(sf_info.frames);
        for (sf_count_t i = 0; i < sf_info.frames; ++i) {
            input_samples[i] = interleaved[i * sf_info.channels]; // Channel 0
        }
        
        std::cout << "\nProcessing audio through circuit..." << std::endl;
        
        // Process samples with proper voltage scaling
        std::vector<float> output_samples(input_samples.size());
        
        size_t total = input_samples.size();
        size_t progress_step = std::max(size_t(1), total / 50);
        
        // Statistics for monitoring
        float peak_in = 0.0f, peak_out = 0.0f;
        float rms_in = 0.0f, rms_out = 0.0f;
        size_t clip_count = 0;
        
        for (size_t i = 0; i < total; i++) {
            // Progress bar
            if (i % progress_step == 0) {
                int percent = (i * 100) / total;
                std::cout << "\rProgress: [";
                int bars = percent / 2;
                for (int b = 0; b < 50; b++) {
                    std::cout << (b < bars ? "=" : " ");
                }
                std::cout << "] " << percent << "%" << std::flush;
            }
            
            // 1. Convert WAV sample [-1, +1] to voltage
            float wav_in = input_samples[i];
            float voltage_in = wav_in * input_voltage_scale;
            
            // 2. Process through circuit
            float voltage_out = processor.processSample(voltage_in);
            
            // 3. Handle numerical issues
            if (std::isnan(voltage_out) || std::isinf(voltage_out)) {
                std::cerr << "\nNumerical instability at sample " << i << std::endl;
                voltage_out = 0.0f;
            }
            
            // 4. Clip to supply rails (hard limit)
            bool clipped = false;
            if (voltage_out > supply_voltage) {
                voltage_out = supply_voltage;
                clipped = true;
            } else if (voltage_out < -supply_voltage) {
                voltage_out = -supply_voltage;
                clipped = true;
            }
            if (clipped) clip_count++;
            
            // 5. Convert voltage to WAV scale
            float wav_out = voltage_out * output_voltage_scale;
            
            // 6. Optional soft clipping (analog-like saturation)
            if (use_soft_clipping) {
                // Tanh soft clipping (smooth saturation)
                wav_out = std::tanh(wav_out * 1.2f);
            } else {
                // Hard clipping to [-1, +1]
                wav_out = std::max(-1.0f, std::min(1.0f, wav_out));
            }
            
            output_samples[i] = wav_out;
            
            // Update statistics
            peak_in = std::max(peak_in, std::abs(wav_in));
            peak_out = std::max(peak_out, std::abs(wav_out));
            rms_in += wav_in * wav_in;
            rms_out += wav_out * wav_out;
        }
        
        std::cout << "\rProgress: [==================================================] 100%" << std::endl;
        
        // Calculate RMS
        rms_in = std::sqrt(rms_in / total);
        rms_out = std::sqrt(rms_out / total);
        
        // Auto-normalization: boost output to use full dynamic range
        // Leave 1dB headroom to prevent clipping
        if (auto_normalize_output && peak_out > 1e-6f) {
            const float target_peak = 0.89f;  // -1dBFS
            float normalize_gain = target_peak / peak_out;
            
            // Apply normalization
            for (auto& sample : output_samples) {
                sample *= normalize_gain;
            }
            
            // Update statistics
            float old_peak = peak_out;
            peak_out *= normalize_gain;
            rms_out *= normalize_gain;
            
            std::cout << "\nAuto-normalization applied:" << std::endl;
            std::cout << "  Peak boost: " << 20*std::log10(normalize_gain) << " dB" << std::endl;
            std::cout << "  Before: " << 20*std::log10(old_peak) << " dBFS" << std::endl;
            std::cout << "  After:  " << 20*std::log10(peak_out) << " dBFS" << std::endl;
        }
        
        // Print statistics
        std::cout << "\nAudio statistics:" << std::endl;
        std::cout << "  Input  - Peak: " << 20*std::log10(peak_in) << " dBFS, RMS: " 
                  << 20*std::log10(rms_in) << " dBFS" << std::endl;
        std::cout << "  Output - Peak: " << 20*std::log10(peak_out) << " dBFS, RMS: " 
                  << 20*std::log10(rms_out) << " dBFS" << std::endl;
        std::cout << "  Circuit gain: " << 20*std::log10(rms_out/rms_in) << " dB" << std::endl;
        
        if (clip_count > 0) {
            std::cout << "  ⚠ Supply clipping: " << clip_count << " samples ("
                      << (100.0 * clip_count / total) << "%)" << std::endl;
        }
        
        // Setup output file info
        SF_INFO out_info;
        out_info.samplerate = sf_info.samplerate;
        out_info.channels = 1;  // Mono output
        
        // Use 32-bit float for maximum quality
        out_info.format = SF_FORMAT_WAV | SF_FORMAT_FLOAT;
        
        // Open output file
        SNDFILE* outfile = sf_open(output_file.c_str(), SFM_WRITE, &out_info);
        if (!outfile) {
            std::cerr << "Error opening output file: " << sf_strerror(nullptr) << std::endl;
            return false;
        }
        
        // Write samples
        sf_count_t written = sf_write_float(outfile, output_samples.data(), 
                                            output_samples.size());
        sf_close(outfile);
        
        if (written != static_cast<sf_count_t>(output_samples.size())) {
            std::cerr << "Warning: Wrote " << written << " of " 
                      << output_samples.size() << " frames" << std::endl;
            return false;
        }
        
        std::cout << "\n✓ Success! Output written to: " << output_file << std::endl;
        
        return true;
    }
};

int main(int argc, char* argv[]) {
    std::cout << "=== Bass/Guitar Circuit Simulator ===" << std::endl;
    std::cout << "Realistic voltage-level audio processing\n" << std::endl;
    
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] 
                  << " <input.wav> <output.wav> <circuit.cir> [input_voltage] [supply_voltage]" 
                  << std::endl;
        std::cerr << "\nExamples:" << std::endl;
        std::cerr << "  Passive bass:  " << argv[0] 
                  << " bass.wav fuzz.wav fuzzface.cir 0.3 9" << std::endl;
        std::cerr << "  Active bass:   " << argv[0] 
                  << " bass.wav fuzz.wav fuzzface.cir 1.5 9" << std::endl;
        std::cerr << "  Hot pickup:    " << argv[0] 
                  << " guitar.wav dist.wav distortion.cir 1.0 18" << std::endl;
        std::cerr << "\nVoltage parameters:" << std::endl;
        std::cerr << "  input_voltage  - Peak voltage from instrument (default: 0.5V)" << std::endl;
        std::cerr << "                   Passive bass/guitar: 0.2-0.5V" << std::endl;
        std::cerr << "                   Active bass: 0.5-2.0V" << std::endl;
        std::cerr << "                   Hot humbuckers: 0.8-1.5V" << std::endl;
        std::cerr << "  supply_voltage - Circuit power supply (default: 9V)" << std::endl;
        std::cerr << "                   Battery pedals: 9V" << std::endl;
        std::cerr << "                   Rack units: 12V, 15V, or 18V" << std::endl;
        return 1;
    }
    
    std::string input_file = argv[1];
    std::string output_file = argv[2];
    std::string netlist_file = argv[3];
    
    // Defaults for passive bass
    float input_voltage = 0.5f;    // ±0.5V peak
    float supply_voltage = 9.0f;   // Standard 9V pedal
    
    if (argc >= 5) input_voltage = std::atof(argv[4]);
    if (argc >= 6) supply_voltage = std::atof(argv[5]);
    
    // Validation
    if (input_voltage <= 0 || input_voltage > 10) {
        std::cerr << "Error: input_voltage must be between 0 and 10V" << std::endl;
        return 1;
    }
    if (supply_voltage <= 0 || supply_voltage > 50) {
        std::cerr << "Error: supply_voltage must be between 0 and 50V" << std::endl;
        return 1;
    }
    
    try {
        // Get sample rate from input file
        SF_INFO sf_info;
        std::memset(&sf_info, 0, sizeof(sf_info));
        SNDFILE* tmp = sf_open(input_file.c_str(), SFM_READ, &sf_info);
        if (!tmp) {
            std::cerr << "Cannot open input file: " << input_file << std::endl;
            return 1;
        }
        double sample_rate = sf_info.samplerate;
        sf_close(tmp);
        
        // Create processor with proper voltage scaling
        WavFileProcessor wav_processor(netlist_file, sample_rate, 
                                       input_voltage, supply_voltage, true);
        
        if (!wav_processor.process(input_file, output_file)) {
            return 1;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception type: " << typeid(e).name()
              << ", message: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}