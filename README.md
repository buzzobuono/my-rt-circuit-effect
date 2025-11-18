# SpicePedal

SpicePedal is a realtime simple spice-like simulator for audio.

## TODO

- [ ] Understand convergence issues in *Wolly Mammoth* circuit
- [x] Add dumpling in Newton-Raphson and measure sistematically performamce: DOESN'T WORK.
- [x] Add voltage clipping to help convergence: DOESN'T WORK.
- [ ] Optimize performamce for real time *solve()* method
- [ ] Implement *getCurrent()* for all component
- [ ] Test Mosfet component in a circuit
- [ ] Test Inductor model in a circuit
- [ ] Adapt project to *c++* best practices
- [x] Refactor *solveDC()* method
- [ ] Better manage *.probe* implementation
- [ ] implement a circuit generic *lv2* plugin
- [ ] enforce netlist number parsing to avoid collision with measure unit
- [ ] enforce univocity in circuit directive
- [ ] use compile-time param to enable statistics printing 
- [ ] use compile-time param to disable .probe directive (no file production at all)
- [x] add convergence statistics
- [ ] potentiometer direct stamping avoiding temporary resistor allocation

## Performance optimizations

- Original: Solver's Execution Time: 6401402 us
- Pre-allocated V_new: Solver's Execution Time: 6288638 us
- Pre-allocated lu_solver: Solver's Execution Time: 6085518 us
- Tolerance Square: Solver's Execution Time: 6067805 us
- Input g: Solver's Execution Time: 6076243 us
- march=native -DNDEBUG: Solver's Execution Time: 5595021 us

### Temptative

spicepedal -i data/input.wav -oout.wav -c circuits/wolly-mammoth.cir

**Pre**

  Solver's Execution Time: 21776558 us                 
  Solver's Failure Percentage: 25.0988 %
  Solver's Tolal Samples: 732672
  Solver's Total Iterations: 6.50744e+06               
  Solver's Mean Iterations: 8.88179

Dumpling

  Solver's Execution Time: 64859292 us
  Solver's Failure Percentage: 87.397 %
  Solver's Tolal Samples: 732672
  Solver's Total Iterations: 1.32889e+07
  Solver's Mean Iterations: 18.1375

MAX_VOLTAGE_STEP = 2.0

  Solver's Execution Time: 57530962 us               
  Solver's Failure Percentage: 99.4418 %
  Solver's Tolal Samples: 732672
  Solver's Total Iterations: 1.45878e+07
  Solver's Mean Iterations: 19.9104
---

spicepedal -i data/input.wav -oout.wav -c circuits/wolly-mammoth-partial.cir
 
**pre**

  Solver's Execution Time: 7698281 us                  
  Solver's Failure Percentage: 5.18308 %               
  Solver's Tolal Samples: 732672                       
  Solver's Total Iterations: 4.04263e+06               
  Solver's Mean Iterations: 5.51765

Dumpling

  never converge

MAX_VOLTAGE_STEP = 2.0

  Solver's Execution Time: 20094462 us
  Solver's Failure Percentage: 46.5526 %
  Solver's Tolal Samples: 732672                      
  Solver's Total Iterations: 8.68135e+06            
  Solver's Mean Iterations: 11.8489

---

spicepedal -i data/input.wav -oout.wav -c circuits/bazz_fuss.cir
 
**pre**

  Solver's Execution Time: 4885487 us                  
  Solver's Failure Percentage: 0.355002 %              
  Solver's Tolal Samples: 732672                      
  Solver's Total Iterations: 4.03762e+06               
  Solver's Mean Iterations: 5.51081

Dumpling

  Solver's Execution Time: 7824216 us
  Solver's Failure Percentage: 5.31984 %
  Solver's Tolal Samples: 732672                    
  Solver's Total Iterations: 4.40219e+06
  Solver's Mean Iterations: 6.0084

MAX_VOLTAGE_STEP = 2.0

  Solver's Execution Time: 7364254 us
  Solver's Failure Percentage: 0.373837 %
  Solver's Tolal Samples: 732672
  Solver's Total Iterations: 4.03984e+06
  Solver's Mean Iterations: 5.51385

---

spicepedal -i data/input.wav -oout.wav -c circuits/booster.cir

**pre**

  Solver's Execution Time: 1695727 us
  Solver's Failure Percentage: 0 %                    
  Solver's Tolal Samples: 732672
  Solver's Total Iterations: 1.64668e+06             
  Solver's Mean Iterations: 2.2475

Dumpling

  Solver's Execution Time: 2375324 us
  Solver's Failure Percentage: 0 %
  Solver's Tolal Samples: 732672
  Solver's Total Iterations: 1.64668e+06
  Solver's Mean Iterations: 2.2475

MAX_VOLTAGE_STEP = 2.0

  Solver's Execution Time: 2581791 us
  Solver's Failure Percentage: 0 %
  Solver's Tolal Samples: 732672
  Solver's Total Iterations: 1.64668e+06
  Solver's Mean Iterations: 2.2475