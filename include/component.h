#ifndef COMPONENT_H
#define COMPONENT_H

#include <string>
#include <vector>
#include <Eigen/Dense>

enum class ComponentType {
    RESISTOR, CAPACITOR, INDUCTOR,
    DIODE, BJT, VOLTAGE_SOURCE,
    POTENTIOMETER,
    WIRE
};

class Component {
public:
    ComponentType type;
    std::string name;
    std::vector<int> nodes;
    
    virtual ~Component() = default;
    virtual void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, 
                      const Eigen::VectorXd& V, double dt) = 0;
    virtual void updateHistory(const Eigen::VectorXd& V, double dt) {}
    virtual void reset() {}
    virtual double getCurrent() const { return 0.0; }
};

#endif
