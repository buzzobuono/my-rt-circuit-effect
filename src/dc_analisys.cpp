#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>

#include "external/CLI11.hpp"

#include "circuit.h"
#include "circuit_solver.h"

class DCAnalisysProcessor {
private:
    Circuit circuit;
    std::unique_ptr<CircuitSolver> solver;
    double sample_rate;
    int input_impedance;
    
public:
    DCAnalisysProcessor(const std::string &netlist_file,
                        double sample_rate,
                        int input_impedance
                       )
        : sample_rate(sample_rate),
          input_impedance(input_impedance)
    {
        if (!circuit.loadNetlist(netlist_file)) {
            throw std::runtime_error("Failed to load netlist");
        }
        
        solver = std::make_unique<CircuitSolver>(circuit, 44100, 25000);
    }

    bool process() {
        return solver->solveDC();
    }
    
};

int main(int argc, char *argv[]) {
    CLI::App app{"Pedal Circuit Simulator - DC Analisys"};
    
    std::string netlist_file;

    app.add_option("-c,--circuit", netlist_file, "Netlist file")->default_val(netlist_file)->check(CLI::ExistingFile);
    
    CLI11_PARSE(app, argc, argv);

    std::cout << "Input Parameters" << std::endl;
    std::cout << std::left;
    std::cout << "   Netlist file: " << netlist_file << std::endl;
    std::cout << std::endl;

    try {

        DCAnalisysProcessor processor(netlist_file, 44100, 25000);
        if (!processor.process())
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