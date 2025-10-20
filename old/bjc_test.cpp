#include "circuit_solver.h"
#include <iostream>

int main() {
    std::cout << "=== BJT Bias Point Test ===" << std::endl;
    
    // Create minimal test circuit
    Circuit test_circuit;
    test_circuit.num_nodes = 5;
    
    // Simple bias: Vcc=9V, Rb=100k, Rc=4.7k, Re=1.5k
    test_circuit.components.push_back(
        std::make_unique<VoltageSource>("VCC", 4, 0, 9.0)
    );
    test_circuit.components.push_back(
        std::make_unique<Resistor>("Rb", 4, 1, 100e3)  // Base bias
    );
    test_circuit.components.push_back(
        std::make_unique<Resistor>("Rc", 4, 2, 4.7e3)  // Collector load
    );
    test_circuit.components.push_back(
        std::make_unique<Resistor>("Re", 3, 0, 1.5e3)  // Emitter resistor
    );
    test_circuit.components.push_back(
        std::make_unique<BJT_NPN>("Q1", 2, 1, 3, 1e-14, 200, 5, 100)
    );
    
    test_circuit.voltage_sources["VCC"] = 
        dynamic_cast<VoltageSource*>(test_circuit.components[0].get());
    test_circuit.input_node = 1;
    test_circuit.output_node = 2;
    
    // Solve
    CircuitSolver solver(test_circuit, 44100);
    
    std::cout << "Initial state (all zeros):" << std::endl;
    for (int n = 0; n < 5; n++) {
        std::cout << "  V(" << n << ") = " << solver.getNodeVoltage(n) << std::endl;
    }
    
    // Ramp up power
    auto vcc = test_circuit.voltage_sources["VCC"];
    for (int step = 0; step <= 20; step++) {
        double v = (step / 20.0) * 9.0;
        vcc->setVoltage(v);
        
        for (int i = 0; i < 50; i++) {
            solver.solve(0.0);
        }
        
        if (step % 5 == 0) {
            std::cout << "\nVcc = " << v << "V:" << std::endl;
            std::cout << "  Vb = " << solver.getNodeVoltage(1) << " V" << std::endl;
            std::cout << "  Vc = " << solver.getNodeVoltage(2) << " V" << std::endl;
            std::cout << "  Ve = " << solver.getNodeVoltage(3) << " V" << std::endl;
            
            double vbe = solver.getNodeVoltage(1) - solver.getNodeVoltage(3);
            double ic = (9.0 - solver.getNodeVoltage(2)) / 4700.0;
            std::cout << "  Vbe = " << vbe << " V" << std::endl;
            std::cout << "  Ic ≈ " << (ic * 1000.0) << " mA" << std::endl;
        }
    }
    
    std::cout << "\n=== Expected values ===" << std::endl;
    std::cout << "Vb should be ~0.7V (one diode drop)" << std::endl;
    std::cout << "Ve should be ~0V (small Re drop)" << std::endl;
    std::cout << "Vc should be ~4V (mid-supply)" << std::endl;
    std::cout << "Vbe should be ~0.7V" << std::endl;
    std::cout << "Ic should be ~1mA" << std::endl;
    
    return 0;
}