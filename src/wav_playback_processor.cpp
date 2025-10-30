#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include <sndfile.h>
#include <portaudio.h>
#include "external/CLI11.hpp"

#include "signal_processor.h"

// =============================================================
// Classe che legge un WAV, lo elabora con SignalProcessor e lo riproduce
// =============================================================
class WavPlaybackProcessor {
private:
    
    Circuit circuit;
    std::unique_ptr<CircuitSolver> solver;
    double sample_rate;
    int input_impedance;

public:
    WavPlaybackProcessor(const std::string& netlist_file,
                     double sample_rate,
                     int input_impedance)
        : sample_rate(sample_rate),
          input_impedance(input_impedance)
    {
        if (!circuit.loadNetlist(netlist_file)) {
            throw std::runtime_error("Failed to load netlist");
        }
        
        solver = std::make_unique<CircuitSolver>(circuit, sample_rate, input_impedance);
    }

    bool process(const std::string& input_file, const std::string& output_file) {
        SF_INFO sfinfo;
        std::memset(&sfinfo, 0, sizeof(sfinfo));

        SNDFILE* infile = sf_open(input_file.c_str(), SFM_READ, &sfinfo);
        if (!infile) {
            std::cerr << "Errore apertura file di input: " << sf_strerror(nullptr) << "\n";
            return false;
        }

        std::vector<float> samples(sfinfo.frames * sfinfo.channels);
        sf_readf_float(infile, samples.data(), sfinfo.frames);
        sf_close(infile);

        std::cout << "WAV Playback Processor\n";
        std::cout << "  Input: " << input_file << "\n";
        std::cout << "  Sample rate: " << sfinfo.samplerate << " Hz\n";
        std::cout << "  Canali: " << sfinfo.channels << "\n";
        std::cout << "  Frame totali: " << sfinfo.frames << "\n";
        std::cout << std::endl;
    
        for (size_t i = 0; i < samples.size(); ++i) {
            float vin = samples[i];
            float vout = 0;
            if (solver->solve(vin)) {
                vout = solver->getOutputVoltage();
            }
            if (std::isnan(vout) || std::isinf(vout))
                vout = 0.0f;
            samples[i] = vout;
        }
    
        // Scrive su file di output
        SNDFILE* outfile = sf_open(output_file.c_str(), SFM_WRITE, &sfinfo);
        if (!outfile) {
            std::cerr << "Errore creazione file di output: " << sf_strerror(nullptr) << "\n";
            return false;
        }
        sf_writef_float(outfile, samples.data(), sfinfo.frames);
        sf_close(outfile);

        std::cout << "✓ File processato salvato in: " << output_file << "\n";

        // Riproduzione (opzionale)
        play(samples, sfinfo.samplerate, sfinfo.channels);

        return true;
    }

private:
    // Funzione per riprodurre con PortAudio
    void play(const std::vector<float>& samples, int sample_rate, int channels) {
        PaError err = Pa_Initialize();
        if (err != paNoError) {
            std::cerr << "Errore PortAudio: " << Pa_GetErrorText(err) << "\n";
            return;
        }

        PaStream* stream;
        err = Pa_OpenDefaultStream(&stream,
                                   0,
                                   channels,
                                   paFloat32,
                                   sample_rate,
                                   256,
                                   nullptr,
                                   nullptr);

        if (err != paNoError) {
            std::cerr << "Errore apertura stream: " << Pa_GetErrorText(err) << "\n";
            Pa_Terminate();
            return;
        }

        Pa_StartStream(stream);
        Pa_WriteStream(stream, samples.data(), samples.size() / channels);
        Pa_StopStream(stream);
        Pa_CloseStream(stream);
        Pa_Terminate();
    }
};

int main(int argc, char* argv[]) {
    CLI::App app{"Pedal Circuit Simulator — WAV Processor"};

    std::string input_file = "input.wav";
    std::string output_file = "output.wav";
    std::string netlist_file;
    int input_impedance;

    app.add_option("-i,--input", input_file, "File di input WAV")
        ->check(CLI::ExistingFile)
        ->default_val(input_file);
    app.add_option("-o,--output", output_file, "File di output WAV")
        ->default_val(output_file);
    app.add_option("-c,--circuit", netlist_file, "Netlist del circuito SPICE")
        ->check(CLI::ExistingFile)
        ->required();
    app.add_option("-I,--input-impedance", input_impedance, "Impedenza d'ingresso [Ohm]")
        ->check(CLI::Range(0, 30000))
        ->default_val(25000);

    CLI11_PARSE(app, argc, argv);

    std::cout << "=== Parametri ===\n";
    std::cout << "Input: " << input_file << "\n";
    std::cout << "Output: " << output_file << "\n";
    std::cout << "Circuito: " << netlist_file << "\n";
    std::cout << "Input Impedance: " << input_impedance << " Ω\n";
    std::cout << std::endl;

    try {
        // Determina il sample rate dal file
        SF_INFO sf_info;
        std::memset(&sf_info, 0, sizeof(sf_info));
        SNDFILE* tmp = sf_open(input_file.c_str(), SFM_READ, &sf_info);
        if (!tmp) {
            std::cerr << "Impossibile aprire il file di input: " << input_file << "\n";
            return 1;
        }
        double sample_rate = sf_info.samplerate;
        sf_close(tmp);

        // Esegui il processore
        WavPlaybackProcessor processor(netlist_file, sample_rate, input_impedance);
        if (!processor.process(input_file, output_file))
            return 1;

    } catch (const std::exception& e) {
        std::cerr << "Errore: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
