#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <sndfile.h>

#include "audio_processor.h"

class WavFileProcessor
{
private:
    AudioProcessor processor;
    double sample_rate;
    float amplitude;
    bool soft_clip;

public:
    WavFileProcessor(const std::string &netlist,
                     double sample_rate,
                     float amplitude,
                     int input_impedance,
                     bool soft_clip)
        : processor(netlist, sample_rate, input_impedance),
          sample_rate(sample_rate),
          amplitude(amplitude),
          soft_clip(soft_clip)
    {
        std::cout << "=== Audio scaling configuration ===" << std::endl;
        std::cout << "  Amplitude: ±" << amplitude << " V" << std::endl;
        std::cout << "  Clipping mode: " << (soft_clip ? "Soft (tanh)" : "Hard") << std::endl;
        std::cout << "  Sample rate: " << sample_rate << " Hz" << std::endl;
    }

    bool process(const std::string &input_file, const std::string &output_file)
    {
        // Open input file
        SF_INFO sf_info;
        std::memset(&sf_info, 0, sizeof(sf_info));

        SNDFILE *infile = sf_open(input_file.c_str(), SFM_READ, &sf_info);
        if (!infile)
        {
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

        if (read_count != sf_info.frames * sf_info.channels)
        {
            std::cerr << "Warning: Read " << read_count << " of "
                      << sf_info.frames * sf_info.channels << " samples" << std::endl;
        }

        // Extract first channel only (mono processing)
        std::vector<float> input_samples(sf_info.frames);
        for (sf_count_t i = 0; i < sf_info.frames; ++i)
        {
            input_samples[i] = interleaved[i * sf_info.channels]; // Channel 0
        }

        std::cout << "\nProcessing audio through circuit..." << std::endl;

        // Process samples with proper voltage scaling
        std::vector<float> output_samples(input_samples.size());

        size_t total = input_samples.size();

        // Statistics for monitoring
        float peak_in = 0.0f, peak_out = 0.0f;
        float rms_in = 0.0f, rms_out = 0.0f;
        size_t clip_count = 0;

        for (size_t i = 0; i < total; i++)
        {
            // Convert WAV sample [-1, +1] to voltage
            float wav_in = input_samples[i];
            float voltage_in = wav_in * amplitude;

            // Process through circuit
            float voltage_out = processor.processSample(voltage_in);

            // Handle numerical issues
            if (std::isnan(voltage_out) || std::isinf(voltage_out))
            {
                std::cerr << "\nNumerical instability at sample " << i << std::endl;
                voltage_out = 0.0f;
            }

            // 5. Convert voltage to WAV - PRESERVE CIRCUIT GAIN!
            // If circuit amplifies 2V→6V (3x), then wav should also do 3x
            float wav_out;
            if (std::abs(voltage_in) > 1e-6f)
            {
                // Preserve gain ratio: wav_out/wav_in = voltage_out/voltage_in
                float circuit_gain = voltage_out / voltage_in;
                wav_out = wav_in * circuit_gain;
            }
            else
            {
                wav_out = voltage_out / amplitude;
            }

            // Optional soft clipping (analog-like saturation at output stage)
            if (soft_clip)
            {
                // Tanh soft clipping - simulates output stage saturation
                wav_out = std::tanh(wav_out);
            }
            else
            {
                // Hard clipping to [-1, +1] to prevent digital overflow
                wav_out = std::max(-1.0f, std::min(1.0f, wav_out));
            }

            output_samples[i] = wav_out;

            // Update statistics
            peak_in = std::max(peak_in, std::abs(wav_in));
            peak_out = std::max(peak_out, std::abs(wav_out));
            rms_in += wav_in * wav_in;
            rms_out += wav_out * wav_out;
        }

        // Calculate RMS
        rms_in = std::sqrt(rms_in / total);
        rms_out = std::sqrt(rms_out / total);

        // Print statistics
        std::cout << "Audio statistics:" << std::endl;
        std::cout << "  Input  - Peak: " << 20 * std::log10(peak_in) << " dBFS, RMS: "
                  << 20 * std::log10(rms_in) << " dBFS" << std::endl;
        std::cout << "  Output - Peak: " << 20 * std::log10(peak_out) << " dBFS, RMS: "
                  << 20 * std::log10(rms_out) << " dBFS" << std::endl;
        std::cout << "  Circuit gain: " << 20 * std::log10(rms_out / rms_in) << " dB" << std::endl;

        if (clip_count > 0)
        {
            std::cout << "  ⚠ Supply clipping: " << clip_count << " samples ("
                      << (100.0 * clip_count / total) << "%)" << std::endl;
        }

        // Setup output file info
        SF_INFO out_info;
        out_info.samplerate = sf_info.samplerate;
        out_info.channels = 1; // Mono output

        // Use 32-bit float for maximum quality
        out_info.format = SF_FORMAT_WAV | SF_FORMAT_FLOAT;

        // Open output file
        SNDFILE *outfile = sf_open(output_file.c_str(), SFM_WRITE, &out_info);
        if (!outfile)
        {
            std::cerr << "Error opening output file: " << sf_strerror(nullptr) << std::endl;
            return false;
        }

        // Write samples
        sf_count_t written = sf_write_float(outfile, output_samples.data(),
                                            output_samples.size());
        sf_close(outfile);

        if (written != static_cast<sf_count_t>(output_samples.size()))
        {
            std::cerr << "Warning: Wrote " << written << " of "
                      << output_samples.size() << " frames" << std::endl;
            return false;
        }

        std::cout << "\n✓ Success! Output written to: " << output_file << std::endl;

        return true;
    }
};

int main(int argc, char *argv[])
{
    std::cout << "=== Bass/Guitar Circuit Simulator ===" << std::endl;
    std::cout << "Realistic voltage-level audio processing\n"
              << std::endl;

    if (argc < 5)
    {
        std::cerr << "Usage: " << argv[0]
                  << " <input.wav> <output.wav> <circuit.cir> <amplitude[V]> <input_impedance[Ohm]>"
                  << std::endl;
        std::cerr << "\n=== Examples ===" << std::endl;
        std::cerr << "  Gibson Les Paul:     " << argv[0]
                  << " guitar.wav out.wav buffer.cir 0.15 15000" << std::endl;
        std::cerr << "  Fender Stratocaster: " << argv[0]
                  << " guitar.wav out.wav buffer.cir 0.10 10000" << std::endl;
        std::cerr << "  Precision Bass:      " << argv[0]
                  << " bass.wav out.wav buffer.cir 0.20 20000" << std::endl;
        std::cerr << "  StingRay Active:     " << argv[0]
                  << " bass.wav out.wav buffer.cir 0.50 600" << std::endl;

        std::cerr << "\n=== Instrument Parameters ===" << std::endl;
        std::cerr << "  input_voltage     - Peak voltage from instrument (Volts)" << std::endl;
        std::cerr << "  source_impedance  - Output impedance of instrument (Ohms)" << std::endl;
        return 1;
    }

    std::string input_file = argv[1];
    std::string output_file = argv[2];
    std::string netlist_file = argv[3];
    float input_voltage = std::atof(argv[4]);
    int input_impedance = std::atoi(argv[5]);

    // Validation
    if (input_voltage <= 0 || input_voltage > 5)
    {
        std::cerr << "Error: input_voltage must be between 0 and 5V" << std::endl;
        return 1;
    }
    if (input_impedance <= 0 || input_impedance > 30000)
    {
        std::cerr << "Error: input_IMPEDANCE must be between 0 and 30k" << std::endl;
        return 1;
    }

    try
    {
        // Get sample rate from input file
        SF_INFO sf_info;
        std::memset(&sf_info, 0, sizeof(sf_info));
        SNDFILE *tmp = sf_open(input_file.c_str(), SFM_READ, &sf_info);
        if (!tmp)
        {
            std::cerr << "Cannot open input file: " << input_file << std::endl;
            return 1;
        }
        double sample_rate = sf_info.samplerate;
        sf_close(tmp);

        WavFileProcessor wav_processor(netlist_file, sample_rate, input_voltage, input_impedance, true);
        if (!wav_processor.process(input_file, output_file))
        {
            return 1;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
