#ifndef RESISTOR_H
#define RESISTOR_H

#include <string>
#include <algorithm>
#include <stdexcept>

#include <Eigen/Dense>

#include "component.h"

class Resistor : public Component {
private:
    double _r;

public:

    Resistor(const std::string& comp_name, int n1, int n2, double r) {
        if (r <= 0) {
            throw std::runtime_error(std::string("Resistance must be positive"));
        }
        if (n1 == n2) {
            throw std::runtime_error(std::string("Resistor nodes must be different"));
        }
        type = ComponentType::RESISTOR;
        name = comp_name;
        nodes = { n1, n2 };
        _r = r;
    }
    
    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, const Eigen::VectorXd& V, double dt) override {
        const double R_MIN = 1e-12; // Resistenza minima
        const double R_MAX = 1e12; // Oltre questa soglia, consideri circuito aperto
        
        if (_r > R_MAX) return; // Non stampare nulla
        
        double g = 1.0 / std::max(_r, R_MIN);
        
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
        return _r; 
    }

};

#endif