#ifndef MOSFET_H
#define MOSFET_H

#include <string>
#include <stdexcept>

#include <Eigen/Dense>

#include "component.h"


class MOSFET : public Component {
public:
    enum Type { NMOS, PMOS };

private:
    Type type;
    int nd, ng, ns;

    double K, Vth, lambda;
    double Cgs, Cgd;

    double vgs_prev = 0.0;
    double vgd_prev = 0.0;

public:
    MOSFET(const std::string& comp_name, int drain, int gate, int source, Type t,
           double K_ = 0.2, double Vth_ = 2.0, double lambda_ = 0.01,
           double Cgs_ = 50e-12, double Cgd_ = 10e-12)
    {
        name = comp_name;
        type = t;
        nd = drain; ng = gate; ns = source;
        nodes = {nd, ng, ns};

        K = K_; Vth = Vth_; lambda = lambda_;
        Cgs = Cgs_; Cgd = Cgd_;

        if (K <= 0 || Vth < 0)
            throw std::runtime_error("MOSFET: parametri non validi");
    }

    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I,
               const Eigen::VectorXd& V, double dt) override
    {
        double vd = (nd > 0) ? V(nd - 1) : 0.0;
        double vg = (ng > 0) ? V(ng - 1) : 0.0;
        double vs = (ns > 0) ? V(ns - 1) : 0.0;

        // salva tensioni reali per le capacità (prima dell'inversione PMOS)
        double vgs_real = vg - vs;
        double vgd_real = vg - vd;

        if (type == PMOS) { vd = -vd; vg = -vg; vs = -vs; }

        double vgs = vg - vs;
        double vds = vd - vs;
        double vgd = vg - vd;

        vgs = std::clamp(vgs, -10.0, 10.0);
        vds = std::clamp(vds, -10.0, 10.0);

        double Id = 0.0, gm = 0.0, gds = 0.0;

        if (vgs > Vth) {
            double vgs_eff = vgs - Vth;
            if (vds < vgs_eff) {
                Id = K * (vgs_eff * vds - 0.5 * vds * vds) * (1 + lambda * vds);
                gm = K * vds * (1 + lambda * vds);
                gds = K * (vgs_eff - vds) * (1 + lambda * vds)
                    + K * (vgs_eff * vds - 0.5 * vds * vds) * lambda;
            } else {
                Id = 0.5 * K * vgs_eff * vgs_eff * (1 + lambda * vds);
                gm = K * vgs_eff * (1 + lambda * vds);
                gds = 0.5 * K * vgs_eff * vgs_eff * lambda;
            }
        }

        // forma robusta per corrente equivalente (linearizzazione)
        double Id_eq = Id - gm * vgs - gds * vds;

        // --- G matrix stamping (drain/source rows) ---
        if (nd > 0) {
            int id = nd - 1;
            G(id, id) += gds;
            if (ns > 0) G(id, ns - 1) -= gds;
            if (ng > 0) G(id, ng - 1) += gm;   // gm in colonna gate is OK
            I(id) += Id_eq;
        }

        if (ns > 0) {
            int is = ns - 1;
            G(is, is) += gds;
            if (nd > 0) G(is, nd - 1) -= gds;
            if (ng > 0) G(is, ng - 1) -= gm;   // -gm in colonna gate
            I(is) -= Id_eq;
        }

        // --- Gate non ha conduttanza DC (rimuovere qualsiasi G(ig,...) su gate row) ---
        // (non aggiungere G(ig, ...) relative a gm qui)

        // --- Capacità gate (backward Euler) ---
        if (dt > 0 && (Cgs > 0 || Cgd > 0)) {
            double gCgs = Cgs / dt;
            double gCgd = Cgd / dt;
            double iCgs = gCgs * vgs_prev;
            double iCgd = gCgd * vgd_prev;

            if (ng > 0 && ns > 0) {
                int ig = ng - 1, is = ns - 1;
                G(ig, ig) += gCgs; G(ig, is) -= gCgs;
                G(is, ig) -= gCgs; G(is, is) += gCgs;
                I(ig) += iCgs; I(is) -= iCgs;
            }

            if (ng > 0 && nd > 0) {
                int ig = ng - 1, id = nd - 1;
                G(ig, ig) += gCgd; G(ig, id) -= gCgd;
                G(id, ig) -= gCgd; G(id, id) += gCgd;
                I(ig) += iCgd; I(id) -= iCgd;
            }
        }

        // aggiorna stato usando le tensioni reali (no inversione PMOS)
        vgs_prev = vgs_real;
        vgd_prev = vgd_real;
    }

    void reset() override {
        vgs_prev = 0.0;
        vgd_prev = 0.0;
    }
};

#endif