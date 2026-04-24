# 7037ENG — Aerodynamics and Vehicle Dynamics Coursework

A MATLAB/Simulink-based vehicle ride dynamics study paired with CFD aerodynamic analysis of a 20% scale DrivAer Notchback model.

---

## Project Overview

This project is split into two parts:

1. **Aerodynamics** — Wind tunnel testing and CFD simulation (STAR-CCM+) of the DrivAer model across three rear-end configurations (Fastback, Notchback, Estate), with smooth vs. detailed underbody comparison.
2. **Vehicle Dynamics** — A 4-DOF half-car ride model built in MATLAB/Simulink for a Toyota Corolla 2.0L Hybrid, analysing ride comfort under half-sine bump and frequency sweep inputs.

---

## Repository Contents

| File | Description |
|---|---|
| `DOF4_parameters.m` | Main MATLAB script — defines all vehicle/suspension parameters, runs both simulations, and generates all output plots |
| `DOF4_sinebump.slx` | Simulink model for half-sine bump road input |
| `DOF4_frequency_input.slx` | Simulink model for chirp (frequency sweep) road input |
| `fft_V.m` | FFT helper function with zero-padding for frequency domain analysis |
| `wide_range_freq.fig` | MATLAB figure — RMS acceleration vs damping ratio (wide frequency sweep: 0.9–1.75 Hz) |
| `narrow_range_freq.fig` | MATLAB figure — RMS acceleration vs damping ratio (narrow frequency sweep: 1.14–1.24 Hz) |
| `7037ENG.pdf` | Full coursework report (aerodynamics + vehicle dynamics) |

---

## Vehicle Dynamics Model

### Vehicle
Toyota Corolla 2.0L Hybrid (2019) — C-segment, 1385 kg kerb mass, 2.64 m wheelbase.

### Model Structure
A 4-degree-of-freedom half-car model capturing:
- Sprung mass vertical displacement (`zs`)
- Sprung mass pitch (`θ`)
- Front unsprung mass displacement (`zusf`)
- Rear unsprung mass displacement (`zusr`)

### Key Parameters (with payload: 79 kg passenger + 10 kg luggage)

| Parameter | Value |
|---|---|
| Total mass | 1474 kg |
| Front ride frequency | 1.2 Hz |
| Rear ride frequency | 1.44 Hz |
| Front spring stiffness | ~49,600 N/m |
| Rear spring stiffness | ~44,700 N/m |
| Tyre stiffness | 290,000 N/m |
| Front damping ratio (baseline) | 0.2 |
| Rear damping ratio (baseline) | 0.2 |

### Optimised Ride Setup
After parametric study, the best balance of ride comfort for this vehicle class is:
- **ζ_front ≈ 0.35**, **ζ_rear ≈ 0.4** at **f = 1.2 Hz**
- Reduces sprung mass RMS acceleration from 0.0969 m/s² to ~0.0808 m/s² (~16% improvement)
- Remains within ISO 2631-1 comfort threshold (< 0.315 m/s²)

---

## Running the Simulation

### Requirements
- MATLAB R2021a or later (with Simulink)
- All four files (`DOF4_parameters.m`, `DOF4_sinebump.slx`, `DOF4_frequency_input.slx`, `fft_V.m`) must be in the **same directory**

### Steps

1. Open MATLAB and set the working directory to the folder containing all project files.
2. Run the main script:
   ```matlab
   run('DOF4_parameters.m')
   ```
3. The script will automatically:
   - Load and simulate `DOF4_sinebump.slx` (half-sine bump response)
   - Load and simulate `DOF4_frequency_input.slx` (frequency sweep response)
   - Generate all time-domain and frequency-domain plots
   - Print key frequencies and RMS values to the command window

### Expected Console Output
```
ridefreq_f =   1.2000
ridefreq_r =   1.4400
fhf        =  10.80xx   (front wheel hop frequency, Hz)
fhr        =  10.72xx   (rear wheel hop frequency, Hz)
rms_sprung =   0.0969   (sprung mass RMS acceleration, m/s²)
```

---

## Output Plots

The script generates the following figures automatically:

| Plot | Description |
|---|---|
| Road Input vs Time | Front and rear half-sine bump profiles |
| Mass Vertical Displacement vs Time | Sprung, front/rear unsprung, and road inputs |
| Masses Vertical Velocity vs Time | Velocity response of all masses |
| Masses Vertical Acceleration vs Time | Acceleration response (key comfort metric) |
| Sprung Mass Vertical Acceleration vs Frequency | FFT — bounce mode (~1.2 Hz) and wheel-hop mode (~10–11 Hz) |
| UnSprung Mass Vertical Acceleration vs Frequency | FFT — front and rear wheel-hop peaks |
| Sprung Mass Pitch Acceleration vs Frequency | FFT — pitch resonance mode |

---

## Aerodynamics Summary

CFD simulations were performed in STAR-CCM+ using steady-state RANS with the SST k-ω turbulence model. Key findings:

| Configuration | Cd | Cl | CMy |
|---|---|---|---|
| Wind Tunnel (Notchback, detailed) | 0.2697 | 0.0946 | −0.138 |
| CFD — Detailed underbody | 0.3045 | 0.0930 | −0.142 |
| CFD — Smooth underbody | 0.2505 | −0.0863 | −0.074 |

- **Smooth underbody** produces lower drag and slight downforce — aerodynamically superior.
- **Detailed underbody** increases separation and wake size, raising drag by ~22%.
- CFD overpredicts drag slightly vs. wind tunnel — consistent with known RANS limitations.

---

## Authors

| Name | Student ID |
|---|---|
| Jagdish Khatu | 16836323 |
| Dhananjay Mahendiratta | 17080424 |
| Sai Tharun Penchala | 17093228 |
| Supunchandra Dassanayake | 17128287 |
| Moses Ajayi | 15809922 |

**Module:** 7037ENG — Aerodynamics and Vehicle Dynamics  
**Institution:** Coventry University
