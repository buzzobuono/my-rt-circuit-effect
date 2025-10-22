#ifndef DIODE_H
#define DIODE_H

#include <string>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include "component.h"

// ============================================
// VERSIONE MINIMAL - Veloce per 95% dei casi
// ============================================
class Diode : public Component {
private:
    double IS;   // Saturation current
    double N;    // Ideality factor
    double VT;   // Thermal voltage

public:
    Diode(const std::string& nm, int anode, int cathode,
          double Is = 5.3e-9,   // 1N4148 default
          double n = 1.68,
          double Vt = 0.02585)
    {
        type = ComponentType::DIODE;
        if (Is <= 0 || n <= 0 || Vt <= 0) {
            throw std::runtime_error("Diode: parametri non validi");
        }
        name = nm;
        nodes = {anode, cathode};
        IS = Is;
        N = n;
        VT = Vt;
    }

    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I,
               const Eigen::VectorXd& V, double dt) override
    {
        double vd = V(nodes[0]) - V(nodes[1]);
        
        // AGGIUNTA CRITICA: limita tensione per stabilità
        vd = std::clamp(vd, -5.0, 1.0);
        
        double Vt = N * VT;
        double exp_term = std::exp(vd / Vt);
        
        double id = IS * (exp_term - 1.0);
        double gd = (IS / Vt) * exp_term;
        double ieq = id - gd * vd;

        int n1 = nodes[0], n2 = nodes[1];

        if (n1 != 0) {
            G(n1, n1) += gd;
            if (n2 != 0) G(n1, n2) -= gd;
            I(n1) -= ieq;
        }
        if (n2 != 0) {
            G(n2, n2) += gd;
            if (n1 != 0) G(n2, n1) -= gd;
            I(n2) += ieq;
        }
    }
};

#endif


// ============================================
// ESEMPIO USO
// ============================================
/*

// 1. Tube Screamer clipper (veloce, no capacità)
Diode D1("D1", 0, 2);  // Default 1N4148
Diode D2("D2", 2, 0);

// 2. LED clipper (Forward voltage ~1.8V)
Diode LED1("LED1", 0, 2, 1e-12, 3.5);  // IS basso, N alto
Diode LED2("LED2", 2, 0, 1e-12, 3.5);

// 3. Germanium diode (Fuzz Face style)
Diode Ge1("Ge1", 0, 2, 5e-7, 1.2);  // IS alto, N basso → soft clipping
Diode Ge2("Ge2", 2, 0, 5e-7, 1.2);

*/