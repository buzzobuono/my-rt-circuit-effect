# SpicePedal

SpicePedal is a realtime simple spice-like simulator for audio.

## TODO

[ ] Optimize performamce for real time *solve()* method
[ ] Implement *getCurrent()* for all component
[ ] Test Mosfet component in a circuit
[ ] Test Inductor model in a circuit
[ ] Adapt project to *c++* best practices
[ ] Refactor *solveDC()* method
[ ] Better manage *.probe* implementation
[ ] implement a circuit generic *lv2* plugin
[ ] enforce netlist number parsing to avoid collision with measure unit
[ ] enforve univocity in circuit directive
[ ] use compile-time param to enable statistics printing 
[ ] use compile-time param to disable .probe directive (no file production at all)
[ ] add convergence statistics

## Performance optimizations

- Original: Solver's Execution Time: 6401402 us
- Pre-allocated V_new: Solver's Execution Time: 6288638 us
- Pre-allocated lu_solver: Solver's Execution Time: 6085518 us
- Tolerance Square: Solver's Execution Time: 6067805 us
- Input g: Solver's Execution Time: 6076243 us
- march=native -DNDEBUG: Solver's Execution Time: 5595021 us

### spicepedal -c circuits/wolly-mammoth.cir -oout.wav -i data/input.wav
- Original
**Process Statistics:**                                  
Solver's Execution Time: 19557967 us                 
Solver's Failure Percentage: 3.39758 %              
Solver's Tolal Samples: 3.37867e+06                  
Solver's Total Iterations: 8.59459e+06               
Solver's Mean Iterations: 2.54378

- Stmp once per sample
**Process Statistics:**


bool solve(double input_voltage) {
        static uint64_t sample_count = 0;
        static uint64_t failed_count = 0;  // conteggio totale dei sample non convergenti
        double alpha = 0.8;  // Start with higher damping
        double prev_error = 1e10;
        
        double final_error_sq;
        
        // Newton-Raphson iteration
        for (int iter = 0; iter < max_iterations; iter++) {
            G.setZero();
            I.setZero();
            
            for (auto& component : circuit.components) {
                component->stamp(G, I, V, dt);
            }
            
            if (circuit.input_node > 0) {
                G(circuit.input_node, circuit.input_node) += source_g;
                I(circuit.input_node) += input_voltage * source_g;
            }
            
            // Ground node constraint
            G.row(0).setZero();
            G.col(0).setZero();
            G(0, 0) = 1.0;
            I(0) = 0.0;
            
            // Shunt resistor per stabilità
            G.diagonal().array() += 1e-12;
            
            // DEBUG solo per un sample specifico
            if (sample_count == 10 && iter == 0) {
                std::cout << "Matrice G:\n" << G << "\n\n";
                std::cout << "Vettore I:\n" << I << "\n\n";
                std::cout << "Determinante: " << G.determinant() << std::endl;
            }
            
            // Solve linear system
            lu_solver.compute(G);
            V_new = lu_solver.solve(I);
            
            double error_sq = (V_new - V).squaredNorm();
            final_error_sq = error_sq;
            if (error_sq < tolerance_sq) {
                V = V_new;
                
                for (auto& comp : circuit.components) {
                    comp->updateHistory(V, dt);
                }
                
                logProbes(sample_count * dt);
                sample_count++;
                return true;  // convergente
            }
            
            // Damping adattativo
            if (iter > 0 && error_sq > prev_error) {
                alpha *= 0.5;
                alpha = std::max(alpha, 0.1);
            } else if (iter > 2 && error_sq < prev_error * 0.5) {
                alpha = std::min(alpha * 1.2, 1.0);
            }
            
            V = alpha * V_new + (1.0 - alpha) * V;
        }
        
        // Se arriviamo qui => non convergente
        failed_count++;
        sample_count++;
        logProbes((sample_count-1) * dt);  // log comunque
        
        return false;
    }
    
    // Funzione ausiliaria per ottenere la percentuale di fallimenti
    double getFailurePercentage() {
        extern uint64_t sample_count;   // o static dentro solve, gestire visibilità
        extern uint64_t failed_count;
        return (sample_count > 0) ? (100.0 * failed_count / sample_count) : 0.0;
    }