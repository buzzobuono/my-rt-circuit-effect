#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <mutex>
#include <chrono>

#include <sndfile.h>
#include <portaudio.h>
#include "external/CLI11.hpp"
#include "signal_processor.h"

// =============================================================
// Funzioni per input non bloccante su Linux
// =============================================================
void setNonBlocking(bool enable) {
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);

    if (enable) {
        ttystate.c_lflag &= ~ICANON; // disabilita modalità canonica
        ttystate.c_lflag &= ~ECHO;   // disabilita echo
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); // non bloccante
    } else {
        ttystate.c_lflag |= ICANON;
        ttystate.c_lflag |= ECHO;
        fcntl(STDIN_FILENO, F_SETFL, 0); // bloccante
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

int kbhit() {
    char ch;
    int nread = read(STDIN_FILENO, &ch, 1);
    if (nread == 1) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

int getch() {
    return getchar();
}

// =============================================================
// Classe streaming con controllo live
// =============================================================
class WavStreamingProcessor {
private:
    Circuit circuit;
    std::unique_ptr<CircuitSolver> solver;
    double sample_rate;
    int input_impedance;
    float paramValue {1.0f};            // parametro modificabile live
    std::mutex param_mutex;        // mutex per accesso sicuro

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
        using clock = std::chrono::high_resolution_clock;
        double worstBufferLatencyMs = 0.0;
        
        auto lastReportTime = clock::now();

        SF_INFO sfinfo;
        std::memset(&sfinfo, 0, sizeof(sfinfo));

        SNDFILE* infile = sf_open(input_file.c_str(), SFM_READ, &sfinfo);
        if (!infile) {
            std::cerr << "Errore apertura file di input: " << sf_strerror(nullptr) << "\n";
            return false;
        }

        PaError err = Pa_Initialize();
        if (err != paNoError) {
            std::cerr << "Errore PortAudio: " << Pa_GetErrorText(err) << "\n";
            sf_close(infile);
            return false;
        }

        PaStream* stream;
        err = Pa_OpenDefaultStream(&stream, 0, sfinfo.channels, paFloat32, sfinfo.samplerate, 256, nullptr, nullptr);
        if (err != paNoError) { Pa_Terminate(); sf_close(infile); return false; }
        Pa_StartStream(stream);

        constexpr size_t BUFFER_SIZE = 128;
        std::vector<float> buffer(BUFFER_SIZE * sfinfo.channels);

        solver->initialize();
        
        bool running = true;
        setNonBlocking(true);
        std::thread inputThread([&]() {
            while (running) {
                if (kbhit()) {
                    int c = getch();
                    std::lock_guard<std::mutex> lock(param_mutex);
                    if (c == 27) { // ESC per uscire
                        running = false;
                    } else if (c == 'w') {
                        paramValue += 0.05f;
                        if (paramValue > 1) paramValue = 1;
                    } else if (c == 's') {
                        paramValue -= 0.05f;
                        if (paramValue < 0) paramValue = 0;
                    }
                    circuit.setParam(1, paramValue);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

        sf_count_t readcount;
        while (running) {
            readcount = sf_readf_float(infile, buffer.data(), BUFFER_SIZE);

            // Se raggiunge la fine del file, torna all'inizio (loop)
            if (readcount == 0) {
                sf_seek(infile, 0, SEEK_SET);
                continue;
            }

            float localParamValue;
            {
                std::lock_guard<std::mutex> lock(param_mutex);
                localParamValue = paramValue;
            }

            auto bufferStart = clock::now();

            for (size_t i = 0; i < static_cast<size_t>(readcount * sfinfo.channels); ++i) {
                float vin = buffer[i] * localParamValue;
                float vout = 0.0f;
                bool ok = solver->solve(vin);
                if (ok) vout = solver->getOutputVoltage();
                if (std::isnan(vout) || std::isinf(vout)) vout = 0.0f;
                buffer[i] = vout;
            }

            auto bufferEnd = clock::now();
            double bufferTimeMs = std::chrono::duration<double, std::milli>(bufferEnd - bufferStart).count();
            double bufferDurationMs = (static_cast<double>(readcount) / sfinfo.samplerate) * 1000.0;
            double perceivedLatencyMs = bufferTimeMs + bufferDurationMs;

            if (perceivedLatencyMs > worstBufferLatencyMs)
                worstBufferLatencyMs = perceivedLatencyMs;
                
            auto now = clock::now();
            double elapsedMs = std::chrono::duration<double, std::milli>(now - lastReportTime).count();

            if (elapsedMs > 1000.0) { // stampa ogni 1 secondo
                auto now = clock::now();
                double elapsedMs = std::chrono::duration<double, std::milli>(now - lastReportTime).count();

                if (elapsedMs > 1000.0) { // stampa ogni 1 secondo
                    std::cout << "\r[Solver] BufTime: " << bufferTimeMs << " ms | "
                            << "BufDuration: " << bufferDurationMs << " ms | "
                            << "Perceived Latency: " << perceivedLatencyMs << " ms | "
                            << "Worst observed: " << worstBufferLatencyMs << " ms       " << std::flush;
                    lastReportTime = now;
                }
            }

            err = Pa_WriteStream(stream, buffer.data(), readcount);
            if (err != paNoError) break;
        }

        running = false;
        inputThread.join();
        setNonBlocking(false);

        Pa_StopStream(stream);
        Pa_CloseStream(stream);
        Pa_Terminate();
        sf_close(infile);

        std::cout << "\n✓ Riproduzione terminata\n";
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
