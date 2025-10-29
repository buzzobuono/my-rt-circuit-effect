#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <sndfile.h>
#include "external/CLI11.hpp"
#include "signal_processor.h"

class WavFileProcessor
{
private:
    SignalProcessor processor;
    double sample_rate;
    float input_voltage_max;
    int input_impedance;
    bool bypass;

public:
    WavFileProcessor(const std::string &netlist,
                     double sample_rate,
                     float input_voltage_max,
                     int input_impedance,
                     bool bypass
                    )
        : processor(netlist, sample_rate, input_impedance),
          sample_rate(sample_rate),
          input_voltage_max(input_voltage_max),
          input_impedance(input_impedance),
          bypass(bypass)
    {
        std::cout << "Audio scaling configuration" << std::endl;
        std::cout << "  Input Voltage Max: " << input_voltage_max << std::endl;
        std::cout << "  Input Input Impedance: " << input_impedance << std::endl;
        std::cout << "  Sample rate: " << sample_rate << " Hz" << std::endl;
        std::cout << "  Circuit By Pass: " << bypass << std::endl;
        std::cout << std::endl;
    }

    bool process(const std::string &input_file, const std::string &output_file)
    {
        // Apri WAV
        SF_INFO sfInfo;
        sfInfo.format = 0;
        SNDFILE* file = sf_open(input_file.c_str(), SFM_READ, &sfInfo);
        
        if (!file) {
            std::cerr << "Errore apertura WAV: " << sf_strerror(file) << std::endl;
            return false;
        }
        
        std::cout << "Input File Format" << std::endl;
        printFileFormat(sfInfo);

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
        std::vector<float> signalIn(numFrames);
        for (sf_count_t i = 0; i < numFrames; i++) {
            signalIn[i] = buffer[i * sfInfo.channels];
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
            scale = input_voltage_max / maxNormalized;
        }

        for (float& s : signalIn) s *= scale;


        std::vector<float> signalOut(signalIn.size());

        float peak_in = 0.0f, peak_out = 0.0f;
        float rms_in = 0.0f, rms_out = 0.0f;

        for (size_t i = 0; i < signalIn.size(); i++) {
            if (!bypass) {
                //signalOut[i] = processor.processSample(signalIn[i], false);
                if(i == (signalIn.size() - 1)) {
                    // print DC only for the last sample
                    signalOut[i] = processor.processSample(signalIn[i], true);
                } else {
                    signalOut[i] = processor.processSample(signalIn[i], false);
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
        rms_in = std::sqrt(rms_in / signalIn.size());
        rms_out = std::sqrt(rms_out / signalIn.size());

        float outputPeak = 0.0f;
        for (float v : signalOut) {
            outputPeak = std::max(outputPeak, std::abs(v));
        }

        // Print statistics
        std::cout << "Audio Statistics:" << std::endl;
        std::cout << "  Mean Input Signal " << mean << std::endl;
        std::cout << "  Max Normalized " << maxNormalized << " V, Scale Factor " << scale << std::endl;
        std::cout << "  Input Peak: " << peak_in << " V, " << 20 * std::log10(peak_in) << " dBFS, RMS: " << 20 * std::log10(rms_in) << " dBFS" << std::endl;
        std::cout << "  Output Peak: " << peak_out << " V, " << 20 * std::log10(peak_out) << " dBFS, RMS: " << 20 * std::log10(rms_out) << " dBFS" << std::endl;
        std::cout << "  Circuit gain: " << 20 * std::log10(rms_out / rms_in) << " dB" << std::endl;
        std::cout << std::endl;

        writeWav(signalOut, output_file.c_str(), sample_rate);
        return true;
    }

    bool writeWav(std::vector<float> signalOut,
              const char* output_file,
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
        SNDFILE* file = sf_open(output_file, SFM_WRITE, &sfInfo);
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
        std::cout << "   Duration: " << (float)signalOut.size() / sample_rate << " sec" << std::endl;
        printFileFormat(sfInfo);
        
        return true;
    }

    void printFileFormat(SF_INFO sfInfo) {
        std::cout << "   Canali: " << sfInfo.channels << std::endl;
        std::cout << "   Sample rate: " << sfInfo.samplerate << " Hz" << std::endl;
        std::cout << "   Frames: " << sfInfo.frames << std::endl;
        std::cout << "   Formato numerico (bitmask): 0x" << std::hex << sfInfo.format << std::dec << std::endl;

        // Decodifica del formato audio (opzionale)
        int major_format = sfInfo.format & SF_FORMAT_TYPEMASK;
        int subtype = sfInfo.format & SF_FORMAT_SUBMASK;

        std::cout << "   Formato container: ";
        switch (major_format) {
            case SF_FORMAT_WAV:      std::cout << "WAV"; break;
            case SF_FORMAT_AIFF:     std::cout << "AIFF"; break;
            case SF_FORMAT_FLAC:     std::cout << "FLAC"; break;
            default:                 std::cout << "Altro (" << std::hex << major_format << std::dec << ")"; break;
        }
        std::cout << std::endl;

        std::cout << "   Sottotipo (codifica PCM, float, etc.): ";
        switch (subtype) {
            case SF_FORMAT_PCM_16:   std::cout << "PCM 16-bit"; break;
            case SF_FORMAT_PCM_24:   std::cout << "PCM 24-bit"; break;
            case SF_FORMAT_PCM_32:   std::cout << "PCM 32-bit"; break;
            case SF_FORMAT_FLOAT:    std::cout << "Float"; break;
            case SF_FORMAT_DOUBLE:   std::cout << "Double"; break;
            default:                 std::cout << "Altro (" << std::hex << subtype << std::dec << ")"; break;
        }
        std::cout << std::endl;

        std::cout << std::endl;
    }

};

int main(int argc, char *argv[]) {
    CLI::App app{"Pedal Circuit Simulator"};
    
    
    std::string input_file = "input.wav";
    std::string output_file = "output.wav";
    std::string netlist_file;
    float max_input_voltage;
    int input_impedance;
    bool bypass = false;

    app.add_option("-i,--input", input_file, "File di input")->default_val(input_file)->check(CLI::ExistingFile);
    app.add_option("-o,--output", output_file, "File di output")->default_val(output_file);
    app.add_option("-n,--netlist", netlist_file, "Netlist file")->default_val(netlist_file)->check(CLI::ExistingFile);
    app.add_option("-v,--max-input-voltage", max_input_voltage, "Max Input Voltage")->check(CLI::Range(0.0f, 5.0f))->default_val(0.15);
    app.add_option("-I,--input-impedance", input_impedance, "Input Impedance")->check(CLI::Range(0, 30000))->default_val(25000);
    app.add_flag("-b,--bypass", bypass, "Bypass Circuit")->default_val(false);
    
    CLI11_PARSE(app, argc, argv);

    std::cout << "Input Parameters" << std::endl;
    std::cout << std::left;
    std::cout << "   Input file:" << input_file << std::endl;
    std::cout << "   Output file:" << output_file << std::endl;
    std::cout << "   Netlist file:" << netlist_file << std::endl;
    std::cout << "   Max Input Voltage:" << max_input_voltage << " V" << std::endl;
    std::cout << "   Input Impedance:" << input_impedance << " Ω" << std::endl;
    std::cout << "   Bypass Circuit:" << (bypass ? "Yes" : "No") << std::endl;
    std::cout << std::endl;

    try {
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

        WavFileProcessor processor(netlist_file, sample_rate, max_input_voltage, input_impedance, bypass);
        if (!processor.process(input_file, output_file))
        {
            return 1;
        }
    }
    catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
    
}

int main__(int argc, char *argv[])
{
    if (argc < 6)
    {
        std::cerr << "=== Pedal Circuit Simulator ===" << std::endl;
        std::cerr << "Usage: " << argv[0]
                  << " <input.wav> <output.wav> <circuit.cir> <input voltage max[V]> <input_impedance[Ohm]> <bypass[true|false]"
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
        std::cerr << "  input voltage max - Peak voltage from instrument (Volts)" << std::endl;
        std::cerr << "  source_impedance  - Output impedance of instrument (Ohms)" << std::endl;
        std::cerr << "  bypass            - By Pass circuit" << std::endl;
        return 1;
    }

    std::string input_file = argv[1];
    std::string output_file = argv[2];
    std::string netlist_file = argv[3];
    float input_voltage_max = std::atof(argv[4]);
    int input_impedance = std::atoi(argv[5]);
    int bypass = std::atoi(argv[6]);

    // Validation
    if (input_voltage_max <= 0 || input_voltage_max > 5)
    {
        std::cerr << "Error: input_voltage_max must be between 0 and 5V" << std::endl;
        return 1;
    }
    if (input_impedance <= 0 || input_impedance > 30000)
    {
        std::cerr << "Error: input_impedance must be between 0 and 30k" << std::endl;
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

        WavFileProcessor processor(netlist_file, sample_rate, input_voltage_max, input_impedance, bypass);
        if (!processor.process(input_file, output_file))
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
