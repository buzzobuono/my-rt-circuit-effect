#ifndef PROBE_H
#define PROBE_H

#include <string>
#include <algorithm>
#include <stdexcept>

#include <Eigen/Dense>

#include "component.h"

class Probe : public Component {
    
public:

    Probe(const std::string& comp_name, int n) {
        type = ComponentType::PROBE;
        name = comp_name;
        nodes = { n };
    }
    
    //void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, const Eigen::VectorXd& V, double dt) override {
    //    static int counter = 0;
    //    counter++;
//
    //    if (counter % 44000 == 0) {
    //        std::cout << "Probe name=" << name << " V[" << nodes[0] << "]=" << V(nodes[0]) << std::endl;
    //    }
    //}

    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, const Eigen::VectorXd& V, double dt) override {
        static int counter = 0;
        static double sum = 0.0;
        static double min_val = std::numeric_limits<double>::max();
        static double max_val = std::numeric_limits<double>::lowest();
        
        double v = V(nodes[0]); // Valore del segnale da monitorare

        // Aggiornamento streaming
        sum += v;
        if (v < min_val) min_val = v;
        if (v > max_val) max_val = v;
        
        counter++;

        // Stampa e reset ogni 44k campioni (1 secondo)
        if (counter % 44000 == 0) {
            double mean = sum / 44100.0;  // media sul blocco
            std::cout << "Probe name=" << name 
                    << " V[" << nodes[0] << "] mean=" << mean 
                    << " min=" << min_val 
                    << " max=" << max_val << std::endl;

            // Reset per il prossimo blocco
            sum = 0.0;
            min_val = std::numeric_limits<double>::max();
            max_val = std::numeric_limits<double>::lowest();
        }
    }
};

#endif