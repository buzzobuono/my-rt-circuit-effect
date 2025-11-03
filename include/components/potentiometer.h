#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#include <string>
#include <stdexcept>
#include <cmath>
#include <Eigen/Dense>
#include "component.h"
#include "components/resistor.h"

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
    
    // Valori delle resistenze calcolate
    double _r1_val; // n1-nw
    double _r2_val; // n2-nw

    static constexpr double R_MIN = 1e-6;
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
    
    void updateResistances()
    {
        if (_r_total > R_MAX)
        {
            _r1_val = R_MAX;
            _r2_val = R_MAX;
            return;
        }

        double taperedPos = applyTaper(_position);

        // Divide la resistenza totale in due sezioni
        _r1_val = std::max(_r_total * (1.0 - taperedPos), R_MIN);  // n1-nw
        _r2_val = std::max(_r_total * taperedPos, R_MIN);          // n2-nw
    }

public:
    Potentiometer(const std::string &comp_name,
                  int n1, int n2, int nw,
                  double r_total,
                  double position,
                  TaperType taper = TaperType::LINEAR)
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
        
        updateResistances();
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
        updateResistances();
    }

    TaperType getTaper() const { 
        return _taper; 
    }
    
    double getResistance1() const {
        return _r1_val;
    }
    
    double getResistance2() const {
        return _r2_val;
    }

    void stamp(Eigen::MatrixXd &G, Eigen::VectorXd &I, const Eigen::VectorXd &V, double dt) override
    {
        if (_r_total > R_MAX)
            return;

        int n1 = nodes[0], n2 = nodes[1], nw = nodes[2];

        // Crea resistori temporanei e delega lo stamping
        Resistor r1(name + "_r1", n1, nw, _r1_val);
        Resistor r2(name + "_r2", n2, nw, _r2_val);
        
        r1.stamp(G, I, V, dt);
        r2.stamp(G, I, V, dt);
    }
};

#endif