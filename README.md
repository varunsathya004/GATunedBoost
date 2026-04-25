# Design and Optimisation of a Closed-Loop DC-DC Boost Converter

This project presents a systematic, automated approach for tuning a power converter controller without manual trial and error. A closed loop PI controller is introduced to regulate the output to the 20V target. The PI gains are optimised using a Genetic Algorithm (GA) that minimises a cost function. The result is a well-regulated 20V output with improved dynamics.

## 📌 Project Overview

The workflow begins with an open-loop simulation to demonstrate the gap between theoretical and actual output voltages caused by inherent circuit non-idealities, such as switching losses and parasitic resistances. To eliminate this steady-state error, a closed-loop PI controller is introduced to dynamically adjust the duty cycle. The core of the project relies on a parallel-pooled Genetic Algorithm to automatically tune the $K_p$ and $K_i$ gains. Ultimately, the algorithm optimises system performance by minimising the Integral Time Absolute Error (ITAE) while enforcing strict penalties against instability and excessive overvoltages.

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

## 🛠️ Software Requirements

* **MATLAB** (Base)
* **Simulink**
* **Simscape Electrical** (for circuit modelling)
* **Global Optimization Toolbox** (for the `ga` function)

## 🚀 Usage

1. Open the Simulink model (`.slx` file).
2. The GA tuner is integrated directly into the simulation via a MATLAB Function block.
3. Upon starting the simulation ($t=0$), Simulink will pause and initialize parallel workers to run the Genetic Algorithm. 
4. The GA searches for optimal $K_p$ and $K_i$ values. Once the optimum gains are found, the Simulink simulation resumes automatically and applies the tuned gains to the PI controller. 

## 🔮 Future Work: Hardware Implementation

Currently, work is underway to transition this simulated closed-loop boost converter into a physical hardware prototype, functioning as a portable laptop charger. Future updates to this repository will include:
* **PCB Schematics:** Complete board designs and routing (developed in KiCad).
* **Microcontroller Integration:** Utilizing an STM32 microcontroller to drive the MOSFET switching. The STM32 will be programmed to generate the 25 kHz PWM frequency, directly applying the optimal duty cycle values outputted by the MATLAB Genetic Algorithm. 

## 👨‍💻 Author

**Varun Sathya** BTech Electrical and Electronics Engineering (EEE)  
College of Engineering Trivandrum (CET)
