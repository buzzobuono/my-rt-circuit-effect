#include <sndfile.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>

// ============================================================================
// STEP 1: WAV → CSV (Input chitarra in Volt)
// ============================================================================

/**
 * Converte file WAV in CSV con formato: index;voltage
 * Legge canale sinistro, rimuove DC offset, normalizza al voltaggio specificato
 * 
 * @param wavFilename File WAV di input
 * @param csvFilename File CSV di output
 * @param maxVoltage Tensione massima del segnale (es. 0.25V per pickup passivo)
 * @param sampleRate Puntatore per restituire il sample rate
 * @return true se successo
 */
bool wavToVoltageCsv(const char* wavFilename,
                     const char* csvFilename,
                     float maxVoltage,
                     int* sampleRate = nullptr) {
    
    // Apri WAV
    SF_INFO sfInfo;
    sfInfo.format = 0;
    SNDFILE* file = sf_open(wavFilename, SFM_READ, &sfInfo);
    
    if (!file) {
        std::cerr << "Errore apertura WAV: " << sf_strerror(file) << std::endl;
        return false;
    }
    
    if (sampleRate) *sampleRate = sfInfo.samplerate;
    
    // Leggi tutti i sample
    std::vector<float> buffer(sfInfo.frames * sfInfo.channels);
    sf_count_t numFrames = sf_readf_float(file, buffer.data(), sfInfo.frames);
    sf_close(file);
    
    if (numFrames != sfInfo.frames) {
        std::cerr << "Errore lettura sample" << std::endl;
        return false;
    }
    
    // Estrai canale sinistro
    std::vector<float> signal(numFrames);
    for (sf_count_t i = 0; i < numFrames; i++) {
        signal[i] = buffer[i * sfInfo.channels];
    }
    
    // Rimuovi DC offset
    float mean = 0.0f;
    for (float s : signal) mean += s;
    mean /= signal.size();
    
    for (float& s : signal) s -= mean;
    
    // Normalizza in Volt
    float maxNormalized = 0.0f;
    for (float s : signal) {
        maxNormalized = std::max(maxNormalized, std::abs(s));
    }
    
    if (maxNormalized > 1e-10f) {
        float scale = maxVoltage / maxNormalized;
        for (float& s : signal) s *= scale;
    }
    
    // Scrivi CSV
    std::ofstream csv(csvFilename);
    if (!csv.is_open()) {
        std::cerr << "Errore apertura CSV output" << std::endl;
        return false;
    }
    
    csv << std::fixed << std::setprecision(8);
    csv << "index;voltage" << std::endl;
    
    for (size_t i = 0; i < signal.size(); i++) {
        csv << i << ";" << signal[i] << std::endl;
    }
    
    csv.close();
    
    std::cout << "✅ WAV → CSV completato" << std::endl;
    std::cout << "   File: " << csvFilename << std::endl;
    std::cout << "   Sample: " << signal.size() << std::endl;
    std::cout << "   Range: ±" << maxVoltage << " V" << std::endl;
    
    return true;
}

// ============================================================================
// STEP 2: CSV Input → Simulazione Circuito → CSV Output
// ============================================================================

// Interfaccia astratta per il simulatore di circuito
class CircuitSimulator {
public:
    virtual ~CircuitSimulator() = default;
    
    /**
     * Processa un singolo sample
     * @param inputVoltage Tensione in ingresso (Volt)
     * @return Tensione in uscita (Volt)
     */
    virtual float processSample(float inputVoltage) = 0;
    
    /**
     * Reset dello stato interno del circuito (opzionale)
     */
    virtual void reset() {}
};

/**
 * Simula il circuito leggendo CSV input e scrivendo CSV output
 * 
 * @param inputCsv File CSV input (index;voltage)
 * @param outputCsv File CSV output (index;voltage)
 * @param circuit Puntatore al simulatore di circuito
 * @return true se successo
 */
bool simulateCircuitCsvToCsv(const char* inputCsv,
                             const char* outputCsv,
                             CircuitSimulator* circuit) {
    
    if (!circuit) {
        std::cerr << "Errore: simulatore circuito nullo" << std::endl;
        return false;
    }
    
    // Apri CSV input
    std::ifstream inFile(inputCsv);
    if (!inFile.is_open()) {
        std::cerr << "Errore apertura CSV input" << std::endl;
        return false;
    }
    
    // Apri CSV output
    std::ofstream outFile(outputCsv);
    if (!outFile.is_open()) {
        std::cerr << "Errore apertura CSV output" << std::endl;
        return false;
    }
    
    outFile << std::fixed << std::setprecision(8);
    
    // Reset circuito
    circuit->reset();
    
    // Leggi header
    std::string line;
    std::getline(inFile, line);
    
    // Scrivi header output
    outFile << "index;voltage" << std::endl;
    
    int samplesProcessed = 0;
    float maxInput = 0.0f;
    float maxOutput = 0.0f;
    
    // Processa sample per sample
    while (std::getline(inFile, line)) {
        std::stringstream ss(line);
        std::string indexStr, voltageStr;
        
        if (!std::getline(ss, indexStr, ';') || !std::getline(ss, voltageStr, ';')) {
            continue;
        }
        
        int index = std::stoi(indexStr);
        float inputVoltage = std::stof(voltageStr);
        
        // === SIMULAZIONE CIRCUITO ===
        float outputVoltage = circuit->processSample(inputVoltage);
        
        // Statistiche
        maxInput = std::max(maxInput, std::abs(inputVoltage));
        maxOutput = std::max(maxOutput, std::abs(outputVoltage));
        
        // Scrivi output
        outFile << index << ";" << outputVoltage << std::endl;
        samplesProcessed++;
    }
    
    inFile.close();
    outFile.close();
    
    std::cout << "✅ Simulazione circuito completata" << std::endl;
    std::cout << "   Sample processati: " << samplesProcessed << std::endl;
    std::cout << "   Peak input: " << maxInput << " V" << std::endl;
    std::cout << "   Peak output: " << maxOutput << " V" << std::endl;
    
    return true;
}

// ============================================================================
// STEP 3: CSV → WAV con simulazione amplificatore
// ============================================================================

/**
 * Converte CSV output del circuito in WAV preservando i voltaggi reali
 * 
 * @param csvFilename File CSV input (index;voltage)
 * @param wavFilename File WAV output
 * @param sampleRate Sample rate per il WAV
 * @param referenceVoltage Voltaggio di riferimento per normalizzazione (es. picco max del segnale)
 * @param bitDepth Bit depth WAV (16, 24, 32)
 * @return true se successo
 */
bool csvToWav(const char* csvFilename,
              const char* wavFilename,
              int sampleRate,
              float referenceVoltage,
              int bitDepth = 24) {
    
    // Leggi CSV
    std::ifstream csv(csvFilename);
    if (!csv.is_open()) {
        std::cerr << "Errore apertura CSV" << std::endl;
        return false;
    }
    
    std::vector<float> signalVolts;
    std::string line;
    std::getline(csv, line); // Skip header
    
    while (std::getline(csv, line)) {
        std::stringstream ss(line);
        std::string indexStr, voltageStr;
        
        if (!std::getline(ss, indexStr, ';') || !std::getline(ss, voltageStr, ';')) {
            continue;
        }
        
        float voltage = std::stof(voltageStr);
        signalVolts.push_back(voltage);
    }
    
    csv.close();
    
    if (signalVolts.empty()) {
        std::cerr << "CSV vuoto o invalido" << std::endl;
        return false;
    }
    
    // Trova il picco del segnale
    float peakVoltage = 0.0f;
    for (float v : signalVolts) {
        peakVoltage = std::max(peakVoltage, std::abs(v));
    }
    
    std::cout << "\n=== CONVERSIONE CSV → WAV ===" << std::endl;
    std::cout << "Peak voltage nel segnale: " << peakVoltage << " V" << std::endl;
    std::cout << "Reference voltage: " << referenceVoltage << " V" << std::endl;
    
    // Normalizza: Volt → [-1, 1] usando il riferimento
    std::vector<float> normalized(signalVolts.size());
    for (size_t i = 0; i < signalVolts.size(); i++) {
        normalized[i] = signalVolts[i] / referenceVoltage;
    }
    
    // Verifica se ci sarebbe clipping digitale
    float peakNormalized = peakVoltage / referenceVoltage;
    if (peakNormalized > 1.0f) {
        std::cout << "⚠️  ATTENZIONE: Il segnale supera il reference voltage!" << std::endl;
        std::cout << "    Peak: " << peakVoltage << "V, Reference: " << referenceVoltage << "V" << std::endl;
        std::cout << "    Verrà clippato a ±1.0 nel WAV (come in una registrazione reale)" << std::endl;
        
        // Hard clip (simula ADC saturo)
        for (float& s : normalized) {
            s = std::max(-1.0f, std::min(1.0f, s));
        }
    }
    
    // Configura WAV
    SF_INFO sfInfo;
    sfInfo.samplerate = sampleRate;
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
    SNDFILE* file = sf_open(wavFilename, SFM_WRITE, &sfInfo);
    if (!file) {
        std::cerr << "Errore apertura WAV: " << sf_strerror(file) << std::endl;
        return false;
    }
    
    sf_count_t written = sf_writef_float(file, normalized.data(), normalized.size());
    sf_close(file);
    
    if (written != (sf_count_t)normalized.size()) {
        std::cerr << "Errore scrittura WAV" << std::endl;
        return false;
    }
    
    float actualPeakDB = 20.0f * log10f(std::min(peakNormalized, 1.0f));
    
    std::cout << "\n✅ WAV salvato" << std::endl;
    std::cout << "   File: " << wavFilename << std::endl;
    std::cout << "   Sample: " << normalized.size() << std::endl;
    std::cout << "   Durata: " << (float)normalized.size() / sampleRate << " sec" << std::endl;
    std::cout << "   Peak level: " << actualPeakDB << " dBFS" << std::endl;
    std::cout << "   I voltaggi originali sono preservati nella scala relativa" << std::endl;
    
    return true;
}

// ============================================================================
// ESEMPIO: Simulatore di circuito overdrive semplice
// ============================================================================

class SimpleOverdriveCircuit : public CircuitSimulator {
private:
    float gain;
    float threshold;
    
public:
    SimpleOverdriveCircuit(float gain = 10.0f, float threshold = 0.5f)
        : gain(gain), threshold(threshold) {}
    
    float processSample(float inputVoltage) override {
        // Amplifica
        float boosted = inputVoltage * gain;
        
        // Soft clipping (tanh)
        float output = threshold * tanh(boosted / threshold);
        
        return output;
    }
    
    void reset() override {
        // Nessuno stato da resettare in questo semplice esempio
    }
};

// ============================================================================
// MAIN: Pipeline completa
// ============================================================================

int main() {
    int sampleRate = 48000;
    
    std::cout << "=== PIPELINE SIMULAZIONE CIRCUITO ===" << std::endl;
    
    // STEP 1: WAV chitarra → CSV input in Volt
    std::cout << "\n--- STEP 1: WAV → CSV ---" << std::endl;
    float inputVoltage = 0.25f; // Pickup passivo
    if (!wavToVoltageCsv("chitarra.wav", "input.csv", inputVoltage, &sampleRate)) {
        return 1;
    }
    
    // STEP 2: Simulazione circuito pedale
    std::cout << "\n--- STEP 2: SIMULAZIONE CIRCUITO ---" << std::endl;
    SimpleOverdriveCircuit overdrive(15.0f, 0.7f);
    if (!simulateCircuitCsvToCsv("input.csv", "output.csv", &overdrive)) {
        return 1;
    }
    
    // STEP 3: CSV output → WAV preservando i voltaggi reali
    std::cout << "\n--- STEP 3: CSV → WAV ---" << std::endl;
    // Usa un riferimento ragionevole (es. picco tipico output pedali o calcola dal CSV)
    float referenceVoltage = 1.0f; // 1V reference (line level standard)
    if (!csvToWav("output.csv", "output.wav", sampleRate, referenceVoltage, 24)) {
        return 1;
    }
    
    std::cout << "\n🎸 Pipeline completata!" << std::endl;
    std::cout << "\n💡 NOTE:" << std::endl;
    std::cout << "   - I CSV contengono i voltaggi REALI del circuito" << std::endl;
    std::cout << "   - Il WAV preserva le proporzioni relative dei voltaggi" << std::endl;
    std::cout << "   - Ispeziona i CSV per verificare il comportamento del circuito" << std::endl;
    std::cout << "   - Il lettore audio gestirà volume/clipping in base al tuo sistema" << std::endl;
    
    return 0;
}