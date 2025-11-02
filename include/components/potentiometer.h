#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#include <string>
#include <stdexcept>
#include <cmath>
#include <Eigen/Dense>
#include "component.h"

class Potentiometer : public Component
{
public:
    enum class TaperType
    {
        LINEAR,
        LOGARITHMIC
    };

private:
    double _r_total;
    double _position; // [0.0, 1.0]
    TaperType _taper;

    static constexpr double R_MIN = 1e-9;
    static constexpr double R_MAX = 1e9;

    // Applica il tipo di taper
    double applyTaper(double pos) const
    {
        switch (_taper)
        {
        case TaperType::LINEAR:
            return pos; // nessuna trasformazione
        case TaperType::LOGARITHMIC:
        {
            // Curva logaritmica "audio-like"
            // 0.0 → 0.0, 1.0 → 1.0, più compressa nelle zone basse
            constexpr double k = 5.0; // più alto = curva più logaritmica
            return std::pow(pos, k);
        }
        default:
            return pos;
        }
    }

public:
    Potentiometer(const std::string &comp_name,
                  int n1, int nw, int n2,
                  double r_total,
                  double position,
                  TaperType taper)
        : _r_total(r_total), _position(position), _taper(taper)
    {
        if (r_total <= 0)
            throw std::runtime_error("Potentiometer total resistance must be positive");
        if (position < 0.0 || position > 1.0)
            throw std::runtime_error("Potentiometer position must be in [0, 1]");
        if (n1 == n2 || n1 == nw || n2 == nw)
            throw std::runtime_error("Potentiometer nodes must be distinct");

        type = ComponentType::POTENTIOMETER;
        name = comp_name;
        nodes = {n1, n2, nw};
    }

    double getTotalResistance() const { 
        return _r_total;
    }
    
    double getPosition() const { 
        return _position; 
    }

    void setPosition(double pos)
    {
        _position = std::clamp(pos, 0.0, 1.0);
    }

    TaperType getTaper() const { 
        return _taper; 
    }

    void stamp(Eigen::MatrixXd &G, Eigen::VectorXd &I, const Eigen::VectorXd &V, double /*dt*/) override
    {
        if (_r_total > R_MAX)
            return;

        double taperedPos = applyTaper(_position);

        // Divide la resistenza totale in due sezioni
        double r1 = std::max(_r_total * (1.0 - taperedPos), R_MIN);  // n1-nw
        double r2 = std::max(_r_total * taperedPos, R_MIN);          // n2-nw

        double g1 = 1.0 / r1;
        double g2 = 1.0 / r2;

        int n1 = nodes[0], n2 = nodes[1], nw = nodes[2];

        // --- Stamp r1 (n1 - nw) ---
        if (n1 != 0)
        {
            G(n1, n1) += g1;
            if (nw != 0)
            {
                G(n1, nw) -= g1;
                G(nw, n1) -= g1;
                G(nw, nw) += g1;
            }
        }

        // --- Stamp r2 (n2 - nw) ---
        if (n2 != 0)
        {
            G(n2, n2) += g2;
            if (nw != 0)
            {
                G(n2, nw) -= g2;
                G(nw, n2) -= g2;
                G(nw, nw) += g2;
            }
        }
    }
};

#endif
