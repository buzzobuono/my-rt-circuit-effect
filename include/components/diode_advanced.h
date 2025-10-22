#ifndef DIODE_ADVANCED_H
#define DIODE_ADVANCED_H

#include <string>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include "component.h"

// ============================================
// VERSIONE ADVANCED - Per HF accuracy (>96kHz)
// ============================================
class Diode : public Component {
private:
    double IS;
    double N;
    double VT;
    double CJ0;  // Junction capacitance
    double VJ;   // Built-in potential
    double MJ;   // Grading coefficient

    double vd_prev = 0.0;

public:
    Diode(const std::string& nm, int anode, int cathode,
                  double Is = 5.3e-9,      // 1N4148
                  double n = 1.68,
                  double Vt = 0.02585,
                  double Cj0 = 1.15e-12,   // Capacità (0 = disabilita)
                  double Vj = 0.74,
                  double Mj = 0.02)
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
        CJ0 = Cj0;
        VJ = Vj;
        MJ = Mj;
    }

    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I,
               const Eigen::VectorXd& V, double dt) override
    {
        double vd = V(nodes[0]) - V(nodes[1]);
        vd = std::clamp(vd, -5.0, 1.0);
        
        // --- DC PART (identica a versione minimal) ---
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

        // --- CAPACITIVE PART (solo se abilitata e dt > 0) ---
        if (dt > 0 && CJ0 > 0) {
            // Capacità non lineare
            double vd_cap = std::clamp(vd, -5.0, 0.9 * VJ);
            double Cj = (vd_cap < 0) 
                ? CJ0 * pow(1.0 - vd_cap / VJ, -MJ)  // Reverse
                : CJ0 * (1.0 + MJ * vd_cap / VJ);    // Forward (approssimato)

            double gC = Cj / dt;
            double iC = gC * vd_prev;

            if (n1 != 0 && n2 != 0) {
                G(n1, n1) += gC;
                G(n1, n2) -= gC;
                G(n2, n1) -= gC;
                G(n2, n2) += gC;
                I(n1) -= iC;
                I(n2) += iC;
            } else if (n1 != 0) {
                G(n1, n1) += gC;
                I(n1) -= iC;
            } else if (n2 != 0) {
                G(n2, n2) += gC;
                I(n2) += iC;
            }
        }

        vd_prev = vd;
    }

    void reset() override {
        vd_prev = 0.0;
    }
};

#endif


// ============================================
// QUANDO USARE QUALE VERSIONE
// ============================================
/*

USA "Diode" (minimal) per:
✅ Sample rate ≤ 48kHz
✅ Overdrive/distortion standard
✅ Maggior velocità di simulazione
✅ 95% dei casi pratici

USA "DiodeAdvanced" per:
✅ Sample rate ≥ 96kHz
✅ High-gain pedals (Boss MT-2, ProCo RAT)
✅ Analisi spettrale accurata (FFT)
✅ Quando il treble loss è critico

ESEMPIO:
// Tube Screamer @ 48kHz: usa Diode
Diode D1("D1", 0, 2);

// Boss MT-2 @ 192kHz: usa DiodeAdvanced
DiodeAdvanced D1("D1", 0, 2, 5.3e-9, 1.68, 0.02585, 1.15e-12);

// LED clipper (no capacità): usa Diode
Diode LED1("LED1", 0, 2, 1e-12, 3.5);

*/