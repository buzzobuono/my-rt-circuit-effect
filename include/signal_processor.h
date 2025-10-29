#ifndef SIGNAL_PROCESSOR_H
#define SIGNAL_PROCESSOR_H

#include "circuit_solver.h"

class SignalProcessor {

protected:
    Circuit circuit;

    std::unique_ptr<CircuitSolver> solver;
    
    double sample_rate;

    int input_impedance;
        
public:
    SignalProcessor(const std::string& netlist_file, 
                   double sample_rate,
                   int input_impedance)
        : sample_rate(sample_rate),
          input_impedance(input_impedance) {
        
        if (!circuit.loadNetlist(netlist_file)) {
            throw std::runtime_error("Failed to load netlist");
        }
        
        solver = std::make_unique<CircuitSolver>(circuit, sample_rate, input_impedance);
        
    }
    
    virtual ~SignalProcessor() = default;
    
    float processSample(float input, bool dc) {
        float out = 0;
        if (solver->solve(input, dc)) {
            out = solver->getOutputVoltage();
        }
        return out;
    }
    
    void reset() {
        solver->reset();
    }

};

#endif