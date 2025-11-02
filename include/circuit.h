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
#include "components/potentiometer.h"

#include <Eigen/Dense>
#include <Eigen/LU>  

class Circuit {
public:
    std::vector<std::unique_ptr<Component>> components;
    int num_nodes;
    int input_node;
    int output_node;
    std::vector<int> probe_nodes;
    double warmup_duration = 0;
    std::map<std::string, double> initial_conditions;
    std::vector<std::pair<int, std::string>> pending_params;
    std::map<int, Potentiometer*> param_map;

    Circuit() : num_nodes(0), output_node(-1) {}
    
    bool loadNetlist(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Cannot open netlist: " << filename << std::endl;
            return false;
        }
        
        std::string line;
        int max_node = 0;
        
        std::cout << "Circuit Creation"<< std::endl;
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
                    double v;
                    std::string unit;
                    iss >> n1 >> n2 >> v >> unit;
                    v *= parseUnit(unit);
                    components.push_back(std::make_unique<Resistor>(comp_name, n1, n2, v));
                    std::cout << "   Component Resistor name=" << comp_name << " n1=" << n1 << " n2=" << n2 <<" v=" << v << std::endl;
                    max_node = std::max(max_node, std::max(n1, n2));
                    break;
                }
                case 'C': {
                    int n1, n2;
                    double v;
                    std::string unit;
                    iss >> n1 >> n2 >> v >> unit;
                    v *= parseUnit(unit);
                    components.push_back(std::make_unique<Capacitor>(comp_name, n1, n2, v));
                    std::cout << "   Component Capacitor name=" << comp_name << " n1=" << n1 << " n2=" << n2 <<" v=" << v << std::endl;
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
                    std::cout << "   Component Diode name=" << comp_name << " model=" << model << " n1=" << n1 << " n2=" << n2 <<" Is=" << Is << " N=" << n << " Vt=" << Vt << " Cj0=" << Cj0 << " Vj=" << Vj << " Mj=" << Mj << std::endl;
                    components.push_back(std::make_unique<Diode>(comp_name, n1, n2, Is, n, Vt, Cj0, Vj, Mj));
                    max_node = std::max(max_node, std::max(n1, n2));
                    break;
                }
                case 'Q': {
                    int nc, nb, ne;
                    std::string model;
                    iss >> nc >> nb >> ne >> model;
                    double Is, Bf, Br, Vt;
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
                    std::cout << "   Component Transistor name=" << comp_name << " model=" << model << " nc=" << nc << " nb=" << nb << " ne=" << ne <<" Is=" << Is << " Bf=" << Bf << " Br=" << Br << " Vt=" << Vt << std::endl;
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
                    std::cout << "   Component VoltageSource name=" << comp_name << " n1=" << n1 << " n2=" << n2 <<" v=" << value << std::endl;
                    components.push_back(std::move(vs));
                    max_node = std::max(max_node, std::max(n1, n2));
                    break;
                }
                case 'P': {
                    int n1, n2, nw;
                    double r_total, pos;
                    std::string taper_str, unit;
                    // P1 1 2 3 10k 0.5 LOG
                    iss >> n1 >> n2 >> nw >> r_total >> unit >> pos >> taper_str;
                    r_total *= parseUnit(unit);

                    Potentiometer::TaperType taper = Potentiometer::TaperType::LINEAR;
                    std::transform(taper_str.begin(), taper_str.end(), taper_str.begin(), ::toupper);
                    if (taper_str == "LOG" || taper_str == "A")
                        taper = Potentiometer::TaperType::LOGARITHMIC;
                    else if (taper_str == "LIN" || taper_str == "B")
                        taper = Potentiometer::TaperType::LINEAR;
                    else
                        std::cerr << "   Warning: Potentiometer taper '" << taper_str << "' not recognized, using LINEAR" << std::endl;
                    components.push_back(std::make_unique<Potentiometer>(comp_name, n1, n2, nw, r_total, pos, taper));
                    std::cout << "   Component Potentiometer name=" << comp_name << " n1=" << n1 << " n2=" << n2 << " nw=" << nw << " Rtot=" << r_total << " pos=" << pos << " taper=" << taper_str << std::endl;
                    max_node = std::max({max_node, n1, n2, nw});
                    break;
                }
                case '.': {
                    std::string directive = comp_name;
                    if (directive == ".input") {
                        iss >> input_node;
                        std::cout << "   Directive Input Node: " << input_node << std::endl;
                    } else if (directive == ".output") {
                        iss >> output_node;
                        std::cout << "   Directive Output Node: " << output_node << std::endl;
                    } else if (directive == ".probe") {
                        int probe_node;
                        iss >> probe_node;
                        std::cout << "   Directive Probe Node: " << probe_node << std::endl;
                        probe_nodes.push_back(probe_node);
                    } else if (directive == ".warmup") {
                        iss >> warmup_duration;
                        std::cout << "   Directive WarmUp Duration: " << warmup_duration << "sec" << std::endl;
                    } else if (directive == ".ic") {
                        std::string cap_name;
                        double v0;
                        iss >> cap_name >> v0;
                        initial_conditions[cap_name] = v0;
                        std::cout << "   Directive Initial Condition: " << cap_name << " = " << v0 << " V" << std::endl;
                    } else if (directive == ".param") {
                        int id;
                        std::string comp_name;
                        iss >> id >> comp_name;
                        pending_params.push_back({id, comp_name});
                        std::cout << "   Directive Param: id=" << id << " comp=" << comp_name << std::endl;
                    }
                    break;
                }
                default: {
                    throw std::runtime_error("Component type unknown " + std::string(1, type));
                }
            }

        }
        std::cout << std::endl;

        num_nodes = max_node + 1;

        std::cout << "Linked Params"<< std::endl;
        for (auto& [id, comp_name] : pending_params) {
        bool found = false;
        for (auto& comp : components) {
            if (comp->name == comp_name) {
                auto* pot = dynamic_cast<Potentiometer*>(comp.get());
                if (!pot)
                    throw std::runtime_error(".param refers to non-potentiometer: " + comp_name);
                param_map[id] = pot;
                found = true;
                std::cout << "   Link Component " << comp_name << " on Param Id " << id << std::endl;
                break;
            }
        }
        if (!found)
            throw std::runtime_error("   .param component not found: " + comp_name);
        std::cout << std::endl;
    }

        return output_node >= 0;
    }

    bool hasInitialConditions() const {
        return !initial_conditions.empty();
    }
    
    bool hasWarmUp() const {
        return warmup_duration > 0;
    }

    void applyInitialConditions() {
        std::cout << "Initial Conditions apply" << std::endl;
        if (initial_conditions.empty()) {
            std::cout << "   No initial conditions to apply" << std::endl;;
            return;
        }
        
        for (auto& comp : components) {
            if (comp->type == ComponentType::CAPACITOR) {
                auto* cap = dynamic_cast<Capacitor*>(comp.get());
                if (cap && initial_conditions.count(cap->name)) {
                    double v0 = initial_conditions[cap->name];
                    cap->setInitialVoltage(v0);
                    std::cout << "   " << cap->name << " = " << v0 << " V" << std::endl;
                }
            }
        }
        
        std::cout << std::endl;
    }

    void setParamValue(int id, double value) {
        auto it = param_map.find(id);
        if (it == param_map.end())
            throw std::runtime_error("Parameter ID not found: " + std::to_string(id));
        
        it->second->setPosition(value);
        std::cout << "   Parameter " << id << " (" << it->second->name << ") set to " << value << std::endl;
    }

    double getParamValue(int id) {
        auto it = param_map.find(id);
        if (it == param_map.end())
            throw std::runtime_error("Parameter ID not found: " + std::to_string(id));
        
        return it->second->getPosition();
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