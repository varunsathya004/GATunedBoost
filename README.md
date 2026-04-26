# Genetic Algorithm-Based PI Tuning of a closed-loop DC-DC Boost Converter 

This project presents a systematic, automated approach for tuning a **PI controller** for a power converter (specifically a DC-DC Boost converter that steps up **12V to 20V**). The PI gains are optimized using a **Genetic Algorithm (GA)** that minimizes a cost function. The result is a well-regulated 20V output with improved dynamic performance.

## 📌 Project Overview

The workflow begins with an open-loop simulation to demonstrate the gap between theoretical and actual output voltages caused by inherent circuit non-idealities, such as switching losses and parasitic resistances. 

To eliminate this steady-state error, a closed-loop PI controller is introduced to dynamically adjust the duty cycle. The control architecture feeds back the output voltage and compares it to a 20V reference. The resulting error signal is then passed through a scaling gain of 1/20 before being sent to the PI controller. 

Mathematically, because the input is scaled, the proportional and integral control actions see an error 20 times smaller than the actual voltage difference:

$$\text{Control Action} = K_{p(GA)} \left(\frac{\text{Error}}{20}\right) + K_{i(GA)} \int \left(\frac{\text{Error}}{20}\right) dt$$

By factoring out the constant, it becomes clear that the effective system-level gains acting on the raw error are exactly one-twentieth of the algorithm's output:

$$\text{Actual } K = \frac{K_{GA}}{20}$$

For instance, if the algorithm optimizes a $K_i$ value of 200, the actual $K_i$ governing the physical circuit is 10.

The core of the project relies on a parallel-pooled Genetic Algorithm to automatically tune these gains. Ultimately, the algorithm optimises system performance by minimising the Integral Time Absolute Error (ITAE) while enforcing strict penalties against instability and excessive overvoltages.

## ⚙️ System Specifications

The boost converter was designed to step up a 12V input to a 20V output at 100W. 

| Parameter | Symbol | Value |
| :--- | :--- | :--- |
| **Input Voltage** | $V_{in}$ | 12 V |
| **Output Voltage (Target)** | $V_{out}$ | 20 V |
| **Switching Frequency** | $f_s$ | 25 kHz |
| **Output Power** | $P$ | 100 W |
| **Inductor** | $L$ | 76.8 µH |
| **Capacitor** | $C$ | 220 µF x2 |
| **Load Resistance** | $R$ | 4 Ω |

## 📊 Results & Performance

Operating the boost converter in an open loop with a theoretical nominal duty cycle of $D=0.4$ yields an actual output of only 18.75V due to inherent circuit losses. 

By implementing the GA-tuned PI controller, the system successfully adapts the duty cycle in real-time, achieving exactly 20V with a soft-start implementation that prevents unsafe voltage transients.

| Metric | Open Loop | Closed Loop (GA PI) |
| :--- | :--- | :--- |
| **Output Voltage** | 18.75 V | $\approx$ 20 V |
| **Steady State Error** | 1.25 V (6.25%) | $\approx$ 0 V |
| **Duty Cycle** | 0.4 (fixed) | 0.4397 (automatic) |
| **Voltage Regulation** | None | Active |
| **Tuning Method** | N/A | Genetic Algorithm + ITAE |

# The Logic Behind the High-Speed GA Tuner

When tuning a control system using a Genetic Algorithm (GA), the algorithm must evaluate thousands of potential PI parameter combinations. A population of 50 evaluated across 100 generations over 10 parallel runs results in **50,000 individual system simulations**. 

If implemented poorly, this takes minutes or hours. The custom script in this repository utilizes multiple mathematical and programmatic optimization techniques to compress 50,000 evaluations into mere seconds.

## 1. Eradicating the Object-Oriented Tax
Standard MATLAB control design relies heavily on LTI (Linear Time-Invariant) objects. Commands like `tf()`, `pid()`, and `feedback()` are incredibly user-friendly, but they carry massive computational overhead. Instantiating, multiplying, and destroying these complex objects 50,000 times brings execution speeds to a crawl.

**The Solution:** The optimization script entirely abandons LTI objects inside the `ga` loop. Instead, the transfer functions of the boost converter and the PI controller are reduced to raw polynomial arrays (lists of coefficients). 

## 2. Calculus vs. Algebra: The $z$-Domain Shift
In continuous time (the $s$-domain), simulating a physical system requires MATLAB to solve complex differential equations. 

To bypass this, the code utilizes `c2d(Plant_s, dt, 'tustin')` **exactly once** before the GA begins. This converts the continuous boost converter physics into a discrete digital system (the $z$-domain). 

In the discrete domain, predicting the future output of a circuit is no longer a differential calculus problem; it is a simple difference equation (basic addition and multiplication):

$$y[n] = b_0 x[n] + b_1 x[n-1] - a_1 y[n-1] ...$$

## 3. MATLAB's `filter()` Engine
Because the entire system is now just arrays of coefficients, the script uses MATLAB's built-in `filter(numerator, denominator, input)` function to simulate the step response. 

`filter()` is not interpreted MATLAB code; it is a highly optimized, pre-compiled C routine operating under the hood. By feeding our polynomial arrays into `filter()`, the 50,000 evaluations are executed at raw machine speeds.

## 4. The Fast Stability Check
In standard control theory, checking for stability requires finding the roots of the characteristic equation (`roots(denominator)`), which involves computationally heavy matrix eigenvalue calculations. Furthermore, floating-point rounding errors during these eigenvalue calculations can sometimes falsely flag a stable controller as unstable.

In a discrete $z$-domain simulation, an unstable system is blindingly obvious: the numerical outputs blow up to infinity within milliseconds. 

Instead of checking roots, the fast code simply simulates the system and applies an instantaneous check:

```matlab
if any(abs(y) > 10 * Vref) || any(isnan(y))
    cost = 1e8; return; 
end
```

If the output exceeds 10 times the target voltage (or hits `NaN` due to computational explosion), the script slaps the GA with a massive penalty ($1\times10^8$) and aborts the evaluation early, saving even more time.

## 5. Pre-Allocation of Memory
Within the objective function loop, array variables like `t_eval` (the time step array) and `step_input` are no longer generated dynamically. They are created once at the top of the script and passed down into the workers. This prevents MATLAB from having to ask the operating system to allocate RAM 50,000 separate times.

## 6. The 10-Pool Parallel Consensus
Due to the stochastic (randomized) nature of Genetic Algorithms, a single run might occasionally get trapped in a "local minimum" rather than finding the absolute best PI combination.

To ensure absolute consistency:
* The search space bounds are heavily constrained based on physical system intuition ($K_p \in [0.001, 50]$, $K_i \in [50, 400]$).
* The `parpool(6)` command spins up CPU cores to execute 10 completely independent GA runs simultaneously.
* The script extracts the minimum cost (`min(all_fval)`) from the pool, guaranteeing that only the mathematically verified, highest-performing controller is chosen to regulate the boost converter.

## 🛠️ Software Requirements

* **MATLAB** (Base)
* **Simulink**
* **Simscape Electrical** (for circuit modelling)
* **Global Optimization Toolbox** (for the `ga` function)

## 🚀 Usage

1. Open MATLAB and navigate to the project directory.
2. Run the `startup.m` function to "pre-warm" the parallel pool.
3. Once the pool is ready, open the `boostconverter_pid.slx` file.
4. Click **Run** in Simulink.

### 🧬 How the Tuning Works
* Upon starting the simulation ($t=0$), Simulink will pause to call the Genetic Algorithm.
* The GA searches for optimal $K_p$ and $K_i$ values using a high-speed discrete-time evaluator.
* Once the optimum gains are found, the simulation resumes automatically and applies the tuned parameters to the physical converter model.

## 🔮 Future Work: Hardware Implementation

Currently, work is underway to transition this simulated closed-loop boost converter into a physical hardware prototype, functioning as a portable laptop charger. Future updates to this repository will include:
* **PCB Schematics:** Complete board designs and routing (developed in KiCad).
* **Microcontroller Integration:** Utilizing an STM32 microcontroller to drive the MOSFET switching. The STM32 will be programmed to generate the 25 kHz PWM frequency, directly applying the optimal duty cycle values outputted by the MATLAB Genetic Algorithm. 

## 👨‍💻 Author

**Varun Sathya** BTech Electrical and Electronics Engineering (EEE)  
College of Engineering Trivandrum (CET)
