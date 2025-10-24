#ifndef DIODE_H
#define DIODE_H

#include <string>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include "component.h"

/*
double Is = 5.3e-9,      // 1N4148
double n = 1.68,
double Vt = 0.02585,
double Cj0 = 1.15e-12,   // Capacità (0 = disabilita)
double Vj = 0.74,
double Mj = 0.02)
*/
class Diode : public Component {
    
private:
    double _Is;
    double _n;
    double _Vt;
    double _Cj0;  // Junction capacitance
    double _Vj;   // Built-in potential
    double _Mj;   // Grading coefficient

    double vd_prev = 0.0;

public:
    Diode(const std::string& comp_name, int nn, int np,
                  double Is,
                  double n,
                  double Vt,
                  double Cj0,   // Capacità (0 = disabilita)
                  double Vj,
                  double Mj)
    {
        type = ComponentType::DIODE;
        if (Is <= 0 || n <= 0 || Vt <= 0) {
            throw std::runtime_error("Diode: parametri non validi");
        }
        name = comp_name;
        nodes = { nn, np };
        _Is = Is;
        _n = n;
        _Vt = Vt;
        _Cj0 = Cj0;
        _Vj = Vj;
        _Mj = Mj;
    }

    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I,
               const Eigen::VectorXd& V, double dt) override
    {
        double vd = V(nodes[0]) - V(nodes[1]);
        vd = std::clamp(vd, -5.0, 1.0);
        
        // --- DC PART (identica a versione minimal) ---
        double Vt = _n * _Vt;
        double exp_term = std::exp(vd / Vt);
        
        double id = _Is * (exp_term - 1.0);
        double gd = (_Is / Vt) * exp_term;
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

        // --- CAPACITIVE PART (solo se abilitata e dt > 0) ---
        if (dt > 0 && _Cj0 > 0) {
            // Capacità non lineare
            double vd_cap = std::clamp(vd, -5.0, 0.5);  // Limita forward a 0.5V
            double Cj;
            
            if (vd_cap < 0) {
                // Reverse bias: formula standard
                Cj = _Cj0 * pow(1.0 - vd_cap / _Vj, -_Mj);
            } else {
                // Forward bias: satura a valore costante (stabile!)
                Cj = _Cj0 * 2.0;  // ~2x capacità nominale
            }

            double gC = Cj / dt;
            double iC = gC * vd_prev;

            if (n1 != 0 && n2 != 0) {
                G(n1, n1) += gC;
                G(n1, n2) -= gC;
                G(n2, n1) -= gC;
                G(n2, n2) += gC;
                I(n1) += iC;  // CORRETTO: stesso segno della parte DC
                I(n2) -= iC;
            } else if (n1 != 0) {
                G(n1, n1) += gC;
                I(n1) += iC;
            } else if (n2 != 0) {
                G(n2, n2) += gC;
                I(n2) -= iC;
            }
        }

        vd_prev = vd;
    }

    void reset() override {
        vd_prev = 0.0;
    }
};

#endif
