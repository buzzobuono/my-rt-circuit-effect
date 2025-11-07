#ifndef WIRE_H
#define WIRE_H

#include <string>
#include <stdexcept>
#include <Eigen/Dense>

#include "component.h"

class Wire : public Component {
public:
    Wire(const std::string& comp_name, int n1, int n2) {
        if (n1 == n2) {
            throw std::runtime_error("Wire nodes must be different");
        }
        type = ComponentType::WIRE;
        name = comp_name;
        nodes = { n1, n2 };
    }
    
    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, 
               const Eigen::VectorXd& V, double dt) override {
        // Conduttanza molto alta per simulare cortocircuito
        const double g = 1e12;
        
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
};

#endif