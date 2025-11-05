#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <sndfile.h>

#include "external/CLI11.hpp"

#include "circuit.h"
#include "circuit_solver.h"

class WavFileProcessor
{
private:
    Circuit circuit;
    std::unique_ptr<CircuitSolver> solver;
    double sample_rate;
    float max_input_voltage;
    int input_impedance;
    bool bypass;

public:
    WavFileProcessor(const std::string &netlist_file,
                     double sample_rate,
                     float max_input_voltage,
                     int input_impedance,
                     bool bypass
                    )
        : sample_rate(sample_rate),
          max_input_voltage(max_input_voltage),
          input_impedance(input_impedance),
          bypass(bypass)
    {
        if (!circuit.loadNetlist(netlist_file)) {
            throw std::runtime_error("Failed to load netlist");
        }
        
        solver = std::make_unique<CircuitSolver>(circuit, sample_rate, input_impedance);
    }

    bool process(const std::string &input_file, const std::string &output_file)
    {
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
            scale = max_input_voltage / maxNormalized;
        }

        for (float& s : signalIn) s *= scale;

        solver->initialize();
        
        std::vector<float> signalOut(signalIn.size());

        float peak_in = 0.0f, peak_out = 0.0f;
        float rms_in = 0.0f, rms_out = 0.0f;

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

        writeWav(signalOut, output_file, sample_rate);
        return true;
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
        std::cout << "   Duration: " << (float)signalOut.size() / sample_rate << " sec" << std::endl;
        
        printFileFormat(output_file);
        
        return true;
    }

    void printFileFormat(const std::string &file) {
        SF_INFO sfInfo;
        sfInfo.format = 0;
        SNDFILE* sound_file = sf_open(file.c_str(), SFM_READ, &sfInfo);
        sf_close(sound_file);
        std::cout << "   Canali: " << sfInfo.channels << std::endl;
        std::cout << "   Sample Rate: " << sfInfo.samplerate << " Hz" << std::endl;
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
    
    
    std::string input_file;
    std::string output_file;
    std::string netlist_file;
    float max_input_voltage;
    int input_impedance;
    bool bypass = false;

    app.add_option("-i,--input", input_file, "File di input")->check(CLI::ExistingFile);
    app.add_option("-o,--output", output_file, "File di output");
    app.add_option("-c,--circuit", netlist_file, "Netlist file")->check(CLI::ExistingFile);
    app.add_option("-v,--max-input-voltage", max_input_voltage, "Max Input Voltage")->check(CLI::Range(0.0f, 5.0f))->default_val(0.15);
    app.add_option("-I,--input-impedance", input_impedance, "Input Impedance")->check(CLI::Range(0, 30000))->default_val(25000);
    app.add_flag("-b,--bypass", bypass, "Bypass Circuit")->default_val(false);
    
    CLI11_PARSE(app, argc, argv);

    std::cout << "Input Parameters" << std::endl;
    std::cout << std::left;
    std::cout << "   Input file: " << input_file << std::endl;
    std::cout << "   Output file: " << output_file << std::endl;
    std::cout << "   Netlist file: " << netlist_file << std::endl;
    std::cout << "   Max Input Voltage :" << max_input_voltage << " V" << std::endl;
    std::cout << "   Input Impedance: " << input_impedance << " Ω" << std::endl;
    std::cout << "   Bypass Circuit: " << (bypass ? "True" : "False") << std::endl;
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

