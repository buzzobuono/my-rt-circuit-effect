# SpicePedal

SpicePedal is a realtime simple spice-like simulator for audio.

## TODO

- [ ] Undurstsnd convergence issues in *Wolly Mammoth* circuit
- [ ] Optimize performamce for real time *solve()* method
- [ ] Implement *getCurrent()* for all component
- [ ] Test Mosfet component in a circuit
- [ ] Test Inductor model in a circuit
- [ ] Adapt project to *c++* best practices
- [x] Refactor *solveDC()* method
- [ ] Better manage *.probe* implementation
- [ ] implement a circuit generic *lv2* plugin
- [ ] enforce netlist number parsing to avoid collision with measure unit
- [ ] enforve univocity in circuit directive
- [ ] use compile-time param to enable statistics printing 
- [ ] use compile-time param to disable .probe directive (no file production at all)
- [x] add convergence statistics

## Performance optimizations

- Original: Solver's Execution Time: 6401402 us
- Pre-allocated V_new: Solver's Execution Time: 6288638 us
- Pre-allocated lu_solver: Solver's Execution Time: 6085518 us
- Tolerance Square: Solver's Execution Time: 6067805 us
- Input g: Solver's Execution Time: 6076243 us
- march=native -DNDEBUG: Solver's Execution Time: 5595021 us
