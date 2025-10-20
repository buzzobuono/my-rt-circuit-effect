
// wav_app.cpp - WAV file processor using libsndfile
// Compile: g++ -std=c++17 -O3 -I/usr/include/eigen3 wav_app.cpp -o wav_processor -lsndfile

#include "circuit_solver.h"
#include <sndfile.h>
#include <iostream>
#include <vector>

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
    
public:
    WavFileProcessor(const std::string& netlist, double sample_rate,
                     double in_gain = 0.5, double out_gain = 2.0)
        : processor(netlist, sample_rate, in_gain, out_gain) {}
    
    bool process(const std::string& input_file, const std::string& output_file) {
        // Open input file
        SF_INFO sf_info;
        std::memset(&sf_info, 0, sizeof(sf_info));
        
        SNDFILE* infile = sf_open(input_file.c_str(), SFM_READ, &sf_info);
        if (!infile) {
            std::cerr << "Error opening input file: " << sf_strerror(nullptr) << std::endl;
            return false;
        }
        
        std::cout << "Input WAV:" << std::endl;
        std::cout << "  Channels: " << sf_info.channels << std::endl;
        std::cout << "  Sample rate: " << sf_info.samplerate << " Hz" << std::endl;
        std::cout << "  Frames: " << sf_info.frames << std::endl;
        std::cout << "  Format: 0x" << std::hex << sf_info.format << std::dec << std::endl;
        
    //    // Verify mono
    //    if (sf_info.channels != 1) {
    //        std::cerr << "Error: Only mono files supported (use channel 1)" << std::endl;
    //        sf_close(infile);
    //        return false;
    //    }
    //    
    //    // Read all samples
    //    std::vector<float> input_samples(sf_info.frames);
    //    sf_count_t read_count = sf_read_float(infile, input_samples.data(), sf_info.frames);
    //    sf_close(infile);
    
    
        // Read samples (always use first channel)
        std::vector<float> interleaved(sf_info.frames * sf_info.channels);
        sf_count_t read_count = sf_read_float(infile, interleaved.data(), interleaved.size());
        sf_close(infile);

        if (read_count != sf_info.frames * sf_info.channels) {
            std::cerr << "Warning: Read " << read_count << " of " << sf_info.frames * sf_info.channels << " samples" << std::endl;
        }

        // Extract first channel only
        std::vector<float> input_samples(sf_info.frames);
        for (sf_count_t i = 0; i < sf_info.frames; ++i) {
            input_samples[i] = interleaved[i * sf_info.channels]; // take channel 0
        }

        if (read_count != sf_info.frames) {
            std::cerr << "Warning: Read " << read_count << " of " << sf_info.frames << " frames" << std::endl;
        }
        
        std::cout << "\nProcessing..." << std::endl;
        
        // Process samples
        std::vector<float> output_samples(input_samples.size());
        
        size_t total = input_samples.size();
        size_t progress_step = total / 50;
        processor.printStats();
        for (size_t i = 0; i < total; i++) {
            if (progress_step > 0 && i % progress_step == 0) {
                int percent = (i * 100) / total;
                std::cout << "\rProgress: [";
                int bars = percent / 2;
                for (int b = 0; b < 50; b++) {
                    std::cout << (b < bars ? "=" : " ");
                }
                std::cout << "] " << percent << "%" << std::flush;
            }
            
            float out = processor.processSample(input_samples[i]);
            
            #ifdef DEBUG
            processor.printAllNodeVoltages();
            #endif
            
            // clamp to avoid numerical blowup
            if (std::isnan(out) || std::isinf(out)) out = 0.0f;
            if (out > 10.0f) out = 10.0f;
            if (out < -10.0f) out = -10.0f;

            output_samples[i] = out;

        }
        
        std::cout << "\rProgress: [==================================================] 100%" << std::endl;
        
        // Setup output file info
        SF_INFO out_info;
        out_info.samplerate = sf_info.samplerate;
        out_info.channels = 1;
        out_info.format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;
        
        // Open output file
        SNDFILE* outfile = sf_open(output_file.c_str(), SFM_WRITE, &out_info);
        if (!outfile) {
            std::cerr << "Error opening output file: " << sf_strerror(nullptr) << std::endl;
            return false;
        }
        
        // Write samples
        sf_count_t written = sf_write_float(outfile, output_samples.data(), output_samples.size());
        sf_close(outfile);
        
        if (written != static_cast<sf_count_t>(output_samples.size())) {
            std::cerr << "Warning: Wrote " << written << " of " << output_samples.size() << " frames" << std::endl;
        }
        
        std::cout << "\nSuccess! Written to: " << output_file << std::endl;
        
        return true;
    }
};

int main(int argc, char* argv[]) {
    std::cout << "=== Circuit WAV Processor ===" << std::endl << std::endl;
    
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <input.wav> <output.wav> <circuit.cir> [input_gain] [output_gain]" << std::endl;
        std::cerr << "\nExample:" << std::endl;
        std::cerr << "  " << argv[0] << " guitar.wav fuzz.wav fuzz_face.cir 0.5 2.0" << std::endl;
        std::cerr << "\nGain parameters:" << std::endl;
        std::cerr << "  input_gain  - Scale audio to voltage (default: 0.5 = ±0.5V)" << std::endl;
        std::cerr << "  output_gain - Scale voltage to audio (default: 2.0)" << std::endl;
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
        // Get sample rate from input file first
        SF_INFO sf_info;
        std::memset(&sf_info, 0, sizeof(sf_info));
        SNDFILE* tmp = sf_open(input_file.c_str(), SFM_READ, &sf_info);
        if (!tmp) {
            std::cerr << "Cannot open input file" << std::endl;
            return 1;
        }
        double sample_rate = sf_info.samplerate;
        sf_close(tmp);
        
        // Create processor and process
        WavFileProcessor wav_processor(netlist_file, sample_rate, input_gain, output_gain);
        
        if (!wav_processor.process(input_file, output_file)) {
            return 1;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}

