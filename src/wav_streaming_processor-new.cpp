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
#include <map>

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
        ttystate.c_lflag &= ~ICANON;
        ttystate.c_lflag &= ~ECHO;
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    } else {
        ttystate.c_lflag |= ICANON;
        ttystate.c_lflag |= ECHO;
        fcntl(STDIN_FILENO, F_SETFL, 0);
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

struct Parameter {
    int id;
    std::atomic<float> value;
    
    Parameter(int id, float defaultValue) : id(id) {
        value.store(std::clamp(defaultValue, 0.0f, 1.0f));
    }
    
    void increase() {
        float newVal = std::min(1.0f, value.load() + 0.05f);
        value.store(newVal);
    }
    
    void decrease() {
        float newVal = std::max(0.0f, value.load() - 0.05f);
        value.store(newVal);
    }
    
    float get() const {
        return value.load(std::memory_order_relaxed);
    }
};

// =============================================================
// Classe streaming con controllo live
// =============================================================
class WavStreamingProcessor {
private:
    Circuit circuit;
    std::unique_ptr<CircuitSolver> solver;
    double sample_rate;
    double input_gain;
    int input_impedance;
    std::vector<Parameter> parameters;
    int currentParamIndex;
   
public:
    WavStreamingProcessor(const std::string& netlist_file,
                          double sample_rate,
                          int input_impedance,
                          double input_gain)
        : sample_rate(sample_rate),
          input_gain(input_gain),
          input_impedance(input_impedance),
          currentParamIndex(0)
    {
        if (!circuit.loadNetlist(netlist_file)) {
            throw std::runtime_error("Failed to load netlist");
        }

        solver = std::make_unique<CircuitSolver>(circuit, sample_rate, input_impedance);
        
        // Inizializza i parametri dal solver
        initializeParameters();
    }

    void initializeParameters() {
        std::vector<int> paramIds = circuit.getParameterIds();
        
        if (paramIds.empty()) {
            std::cout << "⚠ Nessun parametro definito nel circuito\n";
            return;
        }
        
        std::cout << "📋 Parametri trovati:\n";
        for (int id : paramIds) {
            float defaultValue = circuit.getParamValue(id);
            parameters.emplace_back(id, defaultValue);
            std::cout << "  [" << parameters.size() - 1 << "] ID=" << id 
                      << " = " << defaultValue << "\n";
        }
        
        if (!parameters.empty()) {
            currentParamIndex = 0;
        }
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

        constexpr size_t BUFFER_SIZE = 2048;

        PaStream* stream;
        err = Pa_OpenDefaultStream(&stream, 0, sfinfo.channels, paFloat32, 
                                   sfinfo.samplerate, BUFFER_SIZE, nullptr, nullptr);
        if (err != paNoError) { 
            Pa_Terminate(); 
            sf_close(infile); 
            return false; 
        }
        Pa_StartStream(stream);

        std::vector<float> buffer(BUFFER_SIZE * sfinfo.channels);

        solver->initialize();

        bool running = true;
        setNonBlocking(true);
        
        printControls();

        std::thread inputThread([&]() {
            while (running) {
                if (kbhit()) {
                    int c = getch();
                    if (c == 27) { // ESC
                        running = false;
                    } else if (!parameters.empty()) {
                        handleKeyPress(c);
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

        sf_count_t readcount;
        while (running) {
            readcount = sf_readf_float(infile, buffer.data(), BUFFER_SIZE);

            if (readcount == 0) {
                sf_seek(infile, 0, SEEK_SET);
                continue;
            }

            // Sincronizza tutti i parametri con il circuit
            for (auto& param : parameters) {
                circuit.setParamValue(param.id, param.get());
            }

            auto bufferStart = clock::now();

            size_t numSamples = readcount * sfinfo.channels;
            
            if (sfinfo.channels == 1) {
                for (size_t i = 0; i < numSamples; ++i) {
                    float vin = buffer[i] * input_gain;
                    float vout = 0.0f;
                    if (solver->solve(vin)) {
                        vout = solver->getOutputVoltage();
                    }
                    buffer[i] = std::isfinite(vout) ? vout : 0.0f;
                }
            } else if (sfinfo.channels == 2) {
                for (size_t frame = 0; frame < static_cast<size_t>(readcount); ++frame) {
                    size_t idx = frame * 2;
                    float vin = buffer[idx] * input_gain;
                    float vout = 0.0f;
                    if (solver->solve(vin)) {
                        vout = solver->getOutputVoltage();
                    }
                    vout = std::isfinite(vout) ? vout : 0.0f;
                    buffer[idx] = vout;
                    buffer[idx + 1] = vout;
                }
            } else {
                for (size_t frame = 0; frame < static_cast<size_t>(readcount); ++frame) {
                    size_t baseIdx = frame * sfinfo.channels;
                    float vin = buffer[baseIdx] * input_gain;
                    float vout = 0.0f;
                    if (solver->solve(vin)) {
                        vout = solver->getOutputVoltage();
                    }
                    vout = std::isfinite(vout) ? vout : 0.0f;
                    for (int ch = 0; ch < sfinfo.channels; ++ch) {
                        buffer[baseIdx + ch] = vout;
                    }
                }
            }

            auto bufferEnd = clock::now();
            double bufferTimeMs = std::chrono::duration<double, std::milli>(bufferEnd - bufferStart).count();
            double bufferDurationMs = (static_cast<double>(readcount) / sfinfo.samplerate) * 1000.0;
            double perceivedLatencyMs = bufferTimeMs + bufferDurationMs;

            if (perceivedLatencyMs > worstBufferLatencyMs)
                worstBufferLatencyMs = perceivedLatencyMs;
                
            auto now = clock::now();
            double elapsedMs = std::chrono::duration<double, std::milli>(now - lastReportTime).count();

            if (elapsedMs > 1000.0) {
                printStatus(bufferTimeMs, bufferDurationMs, perceivedLatencyMs, worstBufferLatencyMs);
                lastReportTime = now;
            }

            err = Pa_WriteStream(stream, buffer.data(), readcount);
            if (err != paNoError) {
                std::cerr << "\nErrore Pa_WriteStream: " << Pa_GetErrorText(err) << "\n";
                break;
            }
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

private:
    void handleKeyPress(int c) {
        if (parameters.empty()) return;
        
        switch (c) {
            case 'w': case 'W':
                parameters[currentParamIndex].increase();
                break;
            case 's': case 'S':
                parameters[currentParamIndex].decrease();
                break;
            case 'a': case 'A':
                currentParamIndex = (currentParamIndex - 1 + parameters.size()) % parameters.size();
                break;
            case 'd': case 'D':
                currentParamIndex = (currentParamIndex + 1) % parameters.size();
                break;
        }
    }

    void printControls() {
        std::cout << "\n╔══════════════════════════════════════════╗\n";
        std::cout << "║          CONTROLLI LIVE                  ║\n";
        std::cout << "╠══════════════════════════════════════════╣\n";
        std::cout << "║  W/S : Aumenta/Diminuisci parametro     ║\n";
        std::cout << "║  A/D : Parametro precedente/successivo  ║\n";
        std::cout << "║  ESC : Esci                              ║\n";
        std::cout << "╚══════════════════════════════════════════╝\n";
        std::cout << "Buffer size: " << 2048 << " frames\n\n";
    }

    void printStatus(double bufferTimeMs, double bufferDurationMs, 
                     double perceivedLatencyMs, double worstBufferLatencyMs) {
        std::cout << "\r";
        
        if (!parameters.empty()) {
            auto& current = parameters[currentParamIndex];
            std::cout << "[P" << current.id << "=" 
                      << std::fixed << std::setprecision(2) << current.get() << "] ";
        }
        
        std::cout << "BufTime: " << std::setprecision(1) << bufferTimeMs << "ms | "
                  << "Latency: " << perceivedLatencyMs << "ms | "
                  << "Worst: " << worstBufferLatencyMs << "ms";
        
        if (parameters.size() > 1) {
            std::cout << " | [" << (currentParamIndex + 1) 
                      << "/" << parameters.size() << "]";
        }
        
        std::cout << "      " << std::flush;
    }
};

// =============================================================
// MAIN
// =============================================================
int main(int argc, char* argv[]) {
    CLI::App app{"Pedal Circuit Simulator — WAV Streaming Player"};

    std::string input_file = "input.wav";
    std::string netlist_file;
    double input_gain = 1.0;
    int input_impedance = 25000;

    app.add_option("-i,--input", input_file, "File di input WAV")
        ->check(CLI::ExistingFile)
        ->default_val(input_file);
    app.add_option("-g,--input-gain", input_gain, "Input Gain")
        ->check(CLI::Range(0.0, 5.0))
        ->default_val(1.0);
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

        WavStreamingProcessor processor(netlist_file, sample_rate, input_impedance, input_gain);
        if (!processor.processAndPlay(input_file))
            return 1;

    } catch (const std::exception& e) {
        std::cerr << "Errore: " << e.what() << "\n";
        return 1;
    }

    return 0;
}