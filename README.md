# SpicePedal

SpicePedal is a realtime simple spice-like simulator for audio.

# Performance optimizations

- Original: Solver's Execution Time: 6401402 us
- Pre-allocated V_new: Solver's Execution Time: 6288638 us
- Pre-allocated lu_solver: Solver's Execution Time: 6085518 us
- Tolerance Square: Solver's Execution Time: 6067805 us
- Input g: Solver's Execution Time: 6076243 us
- march=native -DNDEBUG: Solver's Execution Time: 5595021 us
