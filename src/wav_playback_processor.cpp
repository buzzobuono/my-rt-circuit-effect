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

class WavStreamingProcessor {
private:
    Circuit circuit;
    std::unique_ptr<CircuitSolver> solver;
    double sample_rate;
    int input_impedance;

public:
    WavStreamingProcessor(const std::string& netlist_file,
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

    bool processAndPlay(const std::string& input_file) {
        SF_INFO sfinfo;
        std::memset(&sfinfo, 0, sizeof(sfinfo));

        SNDFILE* infile = sf_open(input_file.c_str(), SFM_READ, &sfinfo);
        if (!infile) {
            std::cerr << "Errore apertura file di input: " << sf_strerror(nullptr) << "\n";
            return false;
        }

        std::cout << "WAV Streaming Processor\n";
        std::cout << "  Input: " << input_file << "\n";
        std::cout << "  Sample rate: " << sfinfo.samplerate << " Hz\n";
        std::cout << "  Canali: " << sfinfo.channels << "\n";
        std::cout << std::endl;

        // Inizializza PortAudio
        PaError err = Pa_Initialize();
        if (err != paNoError) {
            std::cerr << "Errore PortAudio: " << Pa_GetErrorText(err) << "\n";
            sf_close(infile);
            return false;
        }

        PaStream* stream;
        err = Pa_OpenDefaultStream(&stream,
                                   0, // nessun input
                                   sfinfo.channels,
                                   paFloat32,
                                   sfinfo.samplerate,
                                   256, // dimensione del buffer
                                   nullptr,
                                   nullptr);

        if (err != paNoError) {
            std::cerr << "Errore apertura stream: " << Pa_GetErrorText(err) << "\n";
            Pa_Terminate();
            sf_close(infile);
            return false;
        }

        Pa_StartStream(stream);

        constexpr size_t BUFFER_SIZE = 512; // frames per batch
        std::vector<float> buffer(BUFFER_SIZE * sfinfo.channels);

        sf_count_t readcount;
        while ((readcount = sf_readf_float(infile, buffer.data(), BUFFER_SIZE)) > 0) {
            // Processa il buffer
            for (size_t i = 0; i < static_cast<size_t>(readcount * sfinfo.channels); ++i) {
                float vin = buffer[i];
                float vout = 0.0f;

                if (solver->solve(vin)) {
                    vout = solver->getOutputVoltage();
                }

                if (std::isnan(vout) || std::isinf(vout))
                    vout = 0.0f;

                buffer[i] = vout;
            }

            // Scrivi sullo stream audio
            err = Pa_WriteStream(stream, buffer.data(), readcount);
            if (err != paNoError) {
                std::cerr << "Errore scrittura stream: " << Pa_GetErrorText(err) << "\n";
                break;
            }
        }

        // Chiude tutto
        Pa_StopStream(stream);
        Pa_CloseStream(stream);
        Pa_Terminate();
        sf_close(infile);

        std::cout << "✓ Riproduzione completata.\n";
        return true;
    }
};

// =============================================================
// MAIN
// =============================================================
int main(int argc, char* argv[]) {
    CLI::App app{"Pedal Circuit Simulator — WAV Streaming Player"};

    std::string input_file = "input.wav";
    std::string netlist_file;
    int input_impedance;

    app.add_option("-i,--input", input_file, "File di input WAV")
        ->check(CLI::ExistingFile)
        ->default_val(input_file);
    app.add_option("-c,--circuit", netlist_file, "Netlist del circuito SPICE")
        ->check(CLI::ExistingFile)
        ->required();
    app.add_option("-I,--input-impedance", input_impedance, "Impedenza d'ingresso [Ohm]")
        ->check(CLI::Range(0, 30000))
        ->default_val(25000);

    CLI11_PARSE(app, argc, argv);

    std::cout << "=== Parametri ===\n";
    std::cout << "Input: " << input_file << "\n";
    std::cout << "Circuito: " << netlist_file << "\n";
    std::cout << "Input Impedance: " << input_impedance << " Ω\n";
    std::cout << std::endl;

    try {
        SF_INFO sf_info;
        std::memset(&sf_info, 0, sizeof(sf_info));
        SNDFILE* tmp = sf_open(input_file.c_str(), SFM_READ, &sf_info);
        if (!tmp) {
            std::cerr << "Impossibile aprire il file di input: " << input_file << "\n";
            return 1;
        }
        double sample_rate = sf_info.samplerate;
        sf_close(tmp);

        WavStreamingProcessor processor(netlist_file, sample_rate, input_impedance);
        if (!processor.processAndPlay(input_file))
            return 1;

    } catch (const std::exception& e) {
        std::cerr << "Errore: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
