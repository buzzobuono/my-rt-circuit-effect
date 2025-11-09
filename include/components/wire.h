#ifndef WIRE_H
#define WIRE_H

#include <string>
#include <stdexcept>
#include <Eigen/Dense>

#include "component.h"
#include "components/resistor.h"

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
                   
        int n1 = nodes[0], n2 = nodes[1];
        Resistor r(name , n1, n2, 1e-3);
        r.stamp(G, I, V, dt);
    }
};

#endif