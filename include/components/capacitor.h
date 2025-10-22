#ifndef CAPACITOR_H
#define CAPACITOR_H

#include <string>
#include <stdexcept>

#include <Eigen/Dense>

#include "component.h"

class Capacitor : public Component {
    double C;
    double v_prev;   // tensione al passo precedente
    double i_prev;   // corrente equivalente al passo precedente

public:
    Capacitor(const std::string& comp_name, int node1, int node2, double capacitance) {
        if (capacitance <= 0) {
            throw std::runtime_error(std::string("Capacitance must be positive"));
        }
        if (node1 == node2) {
            throw std::runtime_error(std::string("Capacitor nodes must be different"));
        }
        type = ComponentType::CAPACITOR;
        name = comp_name;
        nodes = { node1, node2 };
        C = capacitance;
        v_prev = 0.0;
        i_prev = 0.0;
    }

    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, const Eigen::VectorXd& V, double dt) override {
        if (dt <= 0) {
            throw std::runtime_error("Time step must be positive");
        }
        // Trapezoidal rule
        double geq = 2.0 * C / dt;          // equivalente conductance
        double ieq = geq * v_prev + i_prev; // corrente equivalente

        int n1 = nodes[0], n2 = nodes[1];

        if (n1 != 0) {
            G(n1, n1) += geq;
            if (n2 != 0) G(n1, n2) -= geq;
            I(n1) += ieq;
        }
        if (n2 != 0) {
            G(n2, n2) += geq;
            if (n1 != 0) G(n2, n1) -= geq;
            I(n2) -= ieq;
        }
    }

    void updateHistory(const Eigen::VectorXd& V, double dt) override {
        double v_n1 = (nodes[0] != 0) ? V(nodes[0]) : 0.0;
        double v_n2 = (nodes[1] != 0) ? V(nodes[1]) : 0.0;
        double v = v_n1 - v_n2;
        
        double geq = 2.0 * C / dt;
        i_prev = geq * (v - v_prev) - i_prev;
        v_prev = v;
    }

    void setInitialVoltage(double v0) {
        v_prev = v0;
        i_prev = 0.0; // Assumendo che parta da regime
    }

    void reset() override {
        v_prev = 0.0;
        i_prev = 0.0;
    }
};

#endif