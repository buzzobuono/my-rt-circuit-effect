#ifndef RESISTOR_H
#define RESISTOR_H

#include <string>
#include <algorithm>
#include <stdexcept>

#include <Eigen/Dense>

#include "component.h"

class Resistor : public Component {
    
public:
    double R;

    Resistor(const std::string& comp_name, int node1, int node2, double resistance) {
        if (resistance <= 0) {
            throw std::runtime_error(std::string("Resistance must be positive"));
        }
        if (node1 == node2) {
            throw std::runtime_error(std::string("Resistor nodes must be different"));
        }
        type = ComponentType::RESISTOR;
        name = comp_name;
        nodes = { node1, node2 };
        R = resistance;
    }
    
    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, const Eigen::VectorXd& V, double dt) override {
        const double R_MIN = 1e-12; // Resistenza minima
        const double R_MAX = 1e12; // Oltre questa soglia, consideri circuito aperto
        
        if (R > R_MAX) return; // Non stampare nulla
        
        double g = 1.0 / std::max(R, R_MIN);
        
        int n1 = nodes[0], n2 = nodes[1];
        
        if (n1 != 0) {
            G(n1, n1) += g;
            if (n2 != 0) G(n1, n2) -= g;
        }
        if (n2 != 0) {
            G(n2, n2) += g;
            if (n1 != 0) G(n2, n1) -= g;
        }
    }
    
    double getResistance() const { 
        return R; 
    }

};

#endif