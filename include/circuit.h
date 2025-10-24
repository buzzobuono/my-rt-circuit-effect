#ifndef CIRCUIT_H
#define CIRCUIT_H

#include <vector>
#include <memory>
#include <string>
#include <map>
#include <fstream>
#include <sstream>
#include <cctype>
#include <algorithm>
#include <iostream>
#include <stdexcept>

#include "component.h"
#include "components/voltage.h"
#include "components/resistor.h"
#include "components/capacitor.h"
#include "components/diode.h"
#include "components/bjt.h"
#include "components/mosfet.h"
#include "components/probe.h"

class Circuit {
public:
    std::vector<std::unique_ptr<Component>> components;
    int num_nodes;
    int input_node;
    int output_node;
    std::vector<int> probe_nodes;

    //std::map<std::string, VoltageSource*> voltage_sources;
    
    Circuit() : num_nodes(0), output_node(-1) {}
    
    bool loadNetlist(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Cannot open netlist: " << filename << std::endl;
            return false;
        }
        
        std::string line;
        int max_node = 0;
        
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '*' || line[0] == '#') continue;
            
            std::istringstream iss(line);
            std::string comp_name;
            iss >> comp_name;
            
            if (comp_name.empty()) continue;
            
            char type = std::toupper(comp_name[0]);
            
            
            switch(type) {
                case 'R': {
                    int n1, n2;
                    double value;
                    std::string unit;
                    iss >> n1 >> n2 >> value >> unit;
                    value *= parseUnit(unit);
                    components.push_back(std::make_unique<Resistor>(comp_name, n1, n2, value));
                    max_node = std::max(max_node, std::max(n1, n2));
                    break;
                }
                case 'C': {
                    int n1, n2;
                    double value;
                    std::string unit;
                    iss >> n1 >> n2 >> value >> unit;
                    value *= parseUnit(unit);
                    components.push_back(std::make_unique<Capacitor>(comp_name, n1, n2, value));
                    max_node = std::max(max_node, std::max(n1, n2));
                    break;
                }
                case 'D': {
                    int n1, n2;
                    std::string model;
                    iss >> n1 >> n2 >> model;
                    double Is, n, Vt;
                    double Cj0;   // Capacità (0 = disabilita)
                    double Vj;
                    double Mj;
                    std::string token;
                    while (iss >> token) {
                        if (token.find("Is=") == 0) Is = std::stod(token.substr(3));
                        else if (token.find("N=") == 0) n = std::stod(token.substr(2));
                        else if (token.find("Vt=") == 0) Vt = std::stod(token.substr(3));
                        else if (token.find("Cj0=") == 0) Cj0 = std::stod(token.substr(4));
                        else if (token.find("Vj=") == 0) Vj = std::stod(token.substr(3));
                        else if (token.find("Mj=") == 0) Mj = std::stod(token.substr(3));
                    }
                    
                    std::cout << "[Diode] name=" << comp_name << " model=" << model << " n1=" << n1 << " n2=" << n2 <<" Is=" << Is << " N=" << n << " Vt=" << Vt << " Cj0=" << Cj0 << " Vj=" << Vj << " Mj=" << Mj << std::endl;
                    
                    components.push_back(std::make_unique<Diode>(comp_name, n1, n2, Is, n, Vt, Cj0, Vj, Mj));
                    
                    max_node = std::max(max_node, std::max(n1, n2));
                    break;
                }
                case 'Q': {
                    int nc, nb, ne;
                    std::string model;
                    iss >> nc >> nb >> ne >> model;

                    // Default parameters
                    double Is, Bf, Br, Vt;

                    // Parse optional parameters
                    std::string token;
                    while (iss >> token) {
                        if (token.find("Is=") == 0)
                            Is = std::stod(token.substr(3));
                        else if (token.find("Bf=") == 0)
                            Bf = std::stod(token.substr(3));
                        else if (token.find("Br=") == 0)
                            Br = std::stod(token.substr(3));
                        else if (token.find("Vt=") == 0)
                            Vt = std::stod(token.substr(3));
                    }

                    components.push_back(std::make_unique<BJT>(
                        comp_name,           // name
                        nc,                  // collector node
                        nb,                  // base node  
                        ne,                  // emitter node
                        Bf,                  // beta (forward beta)
                        Br,                  // beta (reverse beta)
                        Is,                  // saturation current
                        Vt                   // Thermal voltage
                    ));
                    max_node = std::max({max_node, nc, nb, ne});
                    break;
                }
                case 'V': {
                    int n1, n2;
                    std::string dcstr;
                    double value;
                    iss >> n1 >> n2 >> dcstr >> value;
                    auto vs = std::make_unique<VoltageSource>(comp_name, n1, n2, value);
                    //voltage_sources[comp_name] = vs.get();
                    std::cout << "[VoltageSource] name=" << comp_name << " n1=" << n1 << " n2=" << n2 <<" v=" << value << std::endl;
                    components.push_back(std::move(vs));
                    max_node = std::max(max_node, std::max(n1, n2));
                    break;
                }
                //case 'P': {
                //    int n;
                //    iss >> n;
                //    std::cout << "[Probe] name=" << comp_name << " n=" << n << std::endl;
                //    components.push_back(std::make_unique<Probe>(comp_name, n));
                //    max_node = std::max(max_node, n);
                //    break;
                //}
                case '.': {
                    if (comp_name == ".input") {
                        iss >> input_node;
                    } else if (comp_name == ".output") {
                        iss >> output_node;
                    } else if (comp_name == ".probe") {
                        int probe_node;
                        iss >> probe_node;
                        std::cout << "Probe on node " << probe_node << std::endl;
                        probe_nodes.push_back(probe_node);
                    }
                    break;
                }
                default: {
                    throw std::runtime_error("Component type unknown " + std::string(1, type));
                }
            }

        }
        
        num_nodes = max_node + 1;
        
        std::cout << "Loaded circuit: " << components.size() << " components, "
                  << num_nodes << " nodes" << std::endl;
        
        return output_node >= 0;
    }
    
    void reset() {
        for (auto& comp : components) {
            comp->reset();
        }
    }
    
private:
    double parseUnit(const std::string& unit) {
        if (unit.empty()) return 1.0;
        switch(unit[0]) {
            case 'p': return 1e-12;
            case 'n': return 1e-9;
            case 'u': return 1e-6;
            case 'm': return 1e-3;
            case 'k': return 1e3;
            case 'M': return 1e6;
            default: 
                throw std::runtime_error("Unit cannot be determined: " + std::string(1, unit[0]));
            ;
        }
    }
};

#endif