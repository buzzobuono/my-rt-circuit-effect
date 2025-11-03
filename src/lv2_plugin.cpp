// lv2_plugin.cpp - Circuit Simulator LV2 Plugin
#include <lv2/core/lv2.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <atomic>

#include "circuit_solver.h"
#include "circuit.h"

#define PLUGIN_URI "http://github.com/buzzobuono/circuit_simulator"

// Port indices
typedef enum {
    PORT_INPUT = 0,
    PORT_OUTPUT = 1,
    PORT_GAIN = 2,
    PORT_BYPASS = 3
} PortIndex;

// Plugin instance
typedef struct {
    // Audio ports (provided by host)
    const float* input;
    float* output;
    
    // Control ports
    const float* gain;
    const float* bypass;
    
    // Circuit simulation
    Circuit* circuit;
    CircuitSolver* solver;
    
    // Sample rate
    double sample_rate;
    
    // State
    std::atomic<float> last_gain;
    std::atomic<bool> last_bypass;
    bool initialized;
    
    // Statistics (for debugging)
    uint64_t total_samples;
    uint32_t non_convergence_count;
    
} CircuitPlugin;

// ============================================
// Instantiate - Called when plugin is loaded
// ============================================
static LV2_Handle instantiate(
    const LV2_Descriptor* descriptor,
    double rate,
    const char* bundle_path,
    const LV2_Feature* const* features)
{
    CircuitPlugin* plugin = new CircuitPlugin();
    
    plugin->sample_rate = rate;
    plugin->initialized = false;
    plugin->total_samples = 0;
    plugin->non_convergence_count = 0;
    plugin->last_gain.store(1.0f);
    plugin->last_bypass.store(false);
    
    // Load circuit from bundle
    plugin->circuit = new Circuit();
    std::string netlist_path = std::string(bundle_path) + "/circuits/default.net";
    
    if (!plugin->circuit->loadNetlist(netlist_path)) {
        std::cerr << "[Circuit Simulator LV2] ERROR: Failed to load netlist: " 
                  << y << std::endl;
        
        // Create a simple passthrough circuit as fallback
        // (You might want to create a minimal working circuit here)
        std::cerr << "[Circuit Simulator LV2] Using passthrough mode" << std::endl;
        plugin->circuit = nullptr;
        plugin->solver = nullptr;
        return (ù)plugin;
    }
    
    try {
        // Create solver with optimized parameters for real-time
        plugin->solver = new CircuitSolver(
            *plugin->circuit,
            rate,
            25000,      // input impedance (25kΩ typical for guitar)
            15,         // max_iterations (reduced for real-time)
            1e-6,       // tolerance (relaxed for speed)
            0           // no warnings in plugin context
        );
        
        std::cerr << "[Circuit Simulator LV2] 77 successfully @ " 
                  << rate << " Hz" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "[Circuit Simulator LV2] ERROR: " << e.what() << std::endl;
        delete plugin->circuit;
        plugin->circuit = nullptr;
        plugin->solver = nullptr;
    }
    
    return (LV2_Handle)plugin;
}

// ============================================
// Connect Port - Host tells us where buffers are
// ============================================
static void connect_port(
    LV2_Handle instance,
    uint32_t port,
    void* data)
{
    CircuitPlugin* plugin = (CircuitPlugin*)instance;
    
    switch ((PortIndex)port) {
        case PORT_INPUT:
            plugin->input = (const float*)data;
            break;
        case PORT_OUTPUT:
            plugin->output = (float*)data;
            break;
        case PORT_GAIN:
            plugin->gain = (const float*)data;
            break;
        case PORT_BYPASS:
            plugin->bypass = (const float*)data;
            break;
    }
}

// ============================================
// Activate - Called when host starts processing
// ============================================
static void activate(LV2_Handle instance)
{
    CircuitPlugin* plugin = (CircuitPlugin*)instance;
    
    if (plugin->solver && !plugin->initialized) {
        // Initialize circuit (warmup)
        try {
            plugin->solver->initialize();
            plugin->initialized = true;
            std::cerr << "[Circuit Simulator LV2] Circuit initialized" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[Circuit Simulator LV2] Initialization error: " 
                      << e.what() << std::endl;
        }
    }
}

// ============================================
// Run - REAL-TIME AUDIO PROCESSING
// THIS MUST COMPLETE IN < n_samples/sample_rate SECONDS!
// ============================================
static void run(LV2_Handle instance, uint32_t n_samples)
{
    CircuitPlugin* plugin = (CircuitPlugin*)instance;
    
    const float* input = plugin->input;
    float* output = plugin->output;
    
    // Read control parameters
    float gain = *(plugin->gain);
    bool bypass = (*(plugin->bypass) > 0.5f);
    
    // Bypass mode - simple passthrough
    if (bypass || !plugin->solver) {
        // Just copy input to output
        std::memcpy(output, input, n_samples * sizeof(float));
        return;
    }
    
    // Process audio through circuit simulator
    for (uint32_t i = 0; i < n_samples; ++i) {
        // Input: scale from [-1, 1] to voltage range
        // Assuming your solver expects [-1, 1] volt range
        double vin = static_cast<double>(input[i]) * gain;
        
        // Solve circuit
        bool converged = plugin->solver->solve(vin);
        
        float vout = 0.0f;
        if (converged) {
            vout = static_cast<float>(plugin->solver->getOutputVoltage());
        } else {
            // Non-convergence: use last good value or zero
            plugin->non_convergence_count++;
            vout = (i > 0) ? output[i-1] : 0.0f;
        }
        
        // Safety: clip output to prevent DAC damage
        if (std::isnan(vout) || std::isinf(vout)) {
            vout = 0.0f;
        }
        
        // Soft clip to [-1, 1] range
        if (vout > 1.0f) vout = 1.0f;
        if (vout < -1.0f) vout = -1.0f;
        
        output[i] = vout;
        plugin->total_samples++;
    }
    
    // Debug: print stats every 10 seconds
    static uint64_t last_report = 0;
    if (plugin->total_samples - last_report > plugin->sample_rate * 10) {
        if (plugin->non_convergence_count > 0) {
            std::cerr << "[Circuit Simulator LV2] Non-convergences: " 
                      << plugin->non_convergence_count 
                      << " / " << plugin->total_samples 
                      << " (" << (100.0 * plugin->non_convergence_count / plugin->total_samples) 
                      << "%)" << std::endl;
        }
        last_report = plugin->total_samples;
    }
}

// ============================================
// Deactivate - Called when host stops processing
// ============================================
static void deactivate(LV2_Handle instance)
{
    CircuitPlugin* plugin = (CircuitPlugin*)instance;
    
    std::cerr << "[Circuit Simulator LV2] Deactivated. Total samples: " 
              << plugin->total_samples << std::endl;
    
    if (plugin->non_convergence_count > 0) {
        std::cerr << "[Circuit Simulator LV2] Total non-convergences: " 
                  << plugin->non_convergence_count << std::endl;
    }
}

// ============================================
// Cleanup - Free all resources
// ============================================
static void cleanup(LV2_Handle instance)
{
    CircuitPlugin* plugin = (CircuitPlugin*)instance;
    
    if (plugin->solver) {
        delete plugin->solver;
    }
    if (plugin->circuit) {
        delete plugin->circuit;
    }
    
    delete plugin;
}

// ============================================
// Extension Data - For advanced features
// ============================================
static const void* extension_data(const char* uri)
{
    return nullptr;  // No extensions for now
}

// ============================================
// Descriptor - Plugin metadata
// ============================================
static const LV2_Descriptor descriptor = {
    PLUGIN_URI,
    instantiate,
    connect_port,
    activate,
    run,
    deactivate,
    cleanup,
    extension_data
};

// ============================================
// Entry point - LV2 discovery
// ============================================
LV2_SYMBOL_EXPORT
const LV2_Descriptor* lv2_descriptor(uint32_t index)
{
    switch (index) {
        case 0:
            return &descriptor;
        default:
            return nullptr;
    }
}