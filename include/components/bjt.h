#ifndef BJT_H
#define BJT_H

#include <string>
#include <stdexcept>

#include <Eigen/Dense>

#include "component.h"


class BJT : public Component {
public:
    enum BJTType { NPN, PNP };
    
private:
    BJTType bjt_type;
    int nc, nb, ne;  // collector, base, emitter nodes
    
    // Model parameters
    double IS;   // Saturation current (default: 1e-14 A)
    double BF;   // Forward beta (default: 100)
    double BR;   // Reverse beta (default: 1)
    double VT;   // Thermal voltage = kT/q ≈ 26mV
    
    // Previous iteration values (for Newton-Raphson)
    double vbe_prev, vbc_prev;
    
    static constexpr double V_LIMIT = 0.5;  // Voltage limiting for convergence
    
public:
    BJT(const std::string& comp_name, int collector, int base, int emitter, 
        double bf, double br, double is, double Vt) {
        if (is <= 0) {
            throw std::runtime_error("BJT: Saturation current IS must be positive");
        }
        if (bf <= 0) {
            throw std::runtime_error("BJT: Forward beta BF must be positive");
        }
        if (br <= 0) {
            throw std::runtime_error("BJT: Reverse beta BR must be positive");
        }
        if (Vt <= 0) {
            throw std::runtime_error("BJT: Thermal voltage VT must be positive");
        }
        this->type = ComponentType::BJT;
        name = comp_name;
        bjt_type = NPN;
        nc = collector;
        nb = base;
        ne = emitter;
        nodes = {nc, nb, ne};
        
        IS = is;
        BF = bf;
        BR = br;  // Typically much smaller than BF
        VT = Vt;  // 26mV at 300K
        
        vbe_prev = 0.0;
        vbc_prev = 0.0;
        
        // Validation
        if (nc == nb || nb == ne || nc == ne) {
            throw std::runtime_error(std::string("BJT: All three nodes must be different"));
        }
        if (BF <= 0 || IS <= 0) {
            throw std::runtime_error(std::string("BJT: BF and IS must be positive"));
        }
    }
    
    void stamp(Eigen::MatrixXd& G, Eigen::VectorXd& I, 
               const Eigen::VectorXd& V, double dt) override {
        
        // Read node voltages (handle ground)
        double vc = (nc != 0) ? V(nc) : 0.0;
        double vb = (nb != 0) ? V(nb) : 0.0;
        double ve = (ne != 0) ? V(ne) : 0.0;
        
        // Junction voltages
        double vbe = vb - ve;
        double vbc = vb - vc;
        
        // Voltage limiting for convergence
        vbe = limitJunction(vbe, vbe_prev);
        vbc = limitJunction(vbc, vbc_prev);
        
        // For PNP, invert polarities
        if (bjt_type == PNP) {
            vbe = -vbe;
            vbc = -vbc;
        }
        
        // Clamp exponentials to prevent overflow
        double exp_vbe = std::exp(std::min(vbe / VT, 80.0));
        double exp_vbc = std::exp(std::min(vbc / VT, 80.0));
        
        // Ebers-Moll diode currents
        double if_diode = IS * (exp_vbe - 1.0);  // Forward BE junction
        double ir_diode = IS * (exp_vbc - 1.0);  // Reverse BC junction
        
        // Terminal currents (Ebers-Moll equations)
        double ib = if_diode / BF + ir_diode / BR;
        double ic = if_diode - ir_diode;
        double ie = -(ib + ic);
        
        // Conductances (derivatives for linearization - Jacobian)
        double gbe = (IS / (BF * VT)) * exp_vbe;  // ∂ib/∂vbe
        double gbc = (IS / (BR * VT)) * exp_vbc;  // ∂ib/∂vbc
        double gce = (IS / VT) * exp_vbe;         // ∂ic/∂vbe
        double gcc = -(IS / VT) * exp_vbc;        // ∂ic/∂vbc
        
        // Equivalent current sources (companion model)
        // i = g*v + ieq  =>  ieq = i - g*v
        double ieq_b = ib - (gbe * vbe + gbc * vbc);
        double ieq_c = ic - (gce * vbe + gcc * vbc);
        double ieq_e = ie - (-(gbe + gce) * vbe - (gbc + gcc) * vbc);
        
        // For PNP, flip signs
        double sign = (bjt_type == PNP) ? -1.0 : 1.0;
        
        // ========================================
        // STAMPING - Complete 3x3 submatrix
        // ========================================
        
        // Base-Emitter junction (BE diode contribution)
        if (nb != 0 && ne != 0) {
            G(nb, nb) += gbe;
            G(nb, ne) -= gbe;
            G(ne, nb) -= gbe;
            G(ne, ne) += gbe;
        } else if (nb != 0) {
            G(nb, nb) += gbe;
        } else if (ne != 0) {
            G(ne, ne) += gbe;
        }
        
        // Base-Collector junction (BC diode contribution)
        if (nb != 0 && nc != 0) {
            G(nb, nb) += gbc;
            G(nb, nc) -= gbc;
            G(nc, nb) -= gbc;
            G(nc, nc) += gbc;
        } else if (nb != 0) {
            G(nb, nb) += gbc;
        } else if (nc != 0) {
            G(nc, nc) += gbc;
        }
        
        // Collector-Emitter controlled source (transistor action)
        if (nc != 0 && ne != 0) {
            G(nc, nb) += gce;
            G(nc, ne) -= gce;
            G(ne, nb) -= gce;
            G(ne, ne) += gce;
            
            G(nc, nb) += gcc;
            G(nc, nc) -= gcc;
            G(ne, nb) -= gcc;
            G(ne, nc) += gcc;
        } else if (nc != 0) {
            G(nc, nb) += gce + gcc;
            G(nc, nc) -= gcc;
        } else if (ne != 0) {
            G(ne, nb) -= gce + gcc;
            G(ne, nc) += gcc;
        }
        
        // Current sources (equivalent sources from companion model)
        if (nb != 0) I(nb) -= sign * ieq_b;
        if (nc != 0) I(nc) -= sign * ieq_c;
        if (ne != 0) I(ne) -= sign * ieq_e;
        
        // Save for next iteration
        vbe_prev = (bjt_type == PNP) ? -vbe : vbe;
        vbc_prev = (bjt_type == PNP) ? -vbc : vbc;
    }
    
    void updateHistory(const Eigen::VectorXd& V, double dt) override {
        // For BJT, history is updated during stamp
        // But we can update vbe_prev, vbc_prev here for clarity
        double vc = (nc != 0) ? V(nc) : 0.0;
        double vb = (nb != 0) ? V(nb) : 0.0;
        double ve = (ne != 0) ? V(ne) : 0.0;
        
        vbe_prev = vb - ve;
        vbc_prev = vb - vc;
        
        if (bjt_type == PNP) {
            vbe_prev = -vbe_prev;
            vbc_prev = -vbc_prev;
        }
    }
    
    void reset() override {
        vbe_prev = 0.0;
        vbc_prev = 0.0;
    }
    
private:
    // Voltage limiting to help Newton-Raphson convergence
    double limitJunction(double vnew, double vold) {
        double dv = vnew - vold;
        
        // Limit the change in junction voltage
        if (std::abs(dv) > V_LIMIT) {
            return vold + std::copysign(V_LIMIT, dv);
        }
        
        // Also limit absolute voltage for uninitialized case
        if (std::abs(vnew) > 1.0 && std::abs(vold) < 0.1) {
            return std::copysign(0.7, vnew);  // Start near typical Vbe
        }
        
        return vnew;
    }
};

#endif