# Optimisation protocol for NMOR Gradiometer

## OVERVIEW

Our sensor produces two NMOR resonances. We want to overlap these resonances in frequency space. The linear combination of two lorentzians results in one larger resonance (see details https://iopscience.iop.org/article/10.1088/2058-9565/ad3d81). 

In this protol, we permute bias field and laser detuning to reach an optimal gradiometer.

In order to measure optimal resonance, we must fit data to a model. Our models are a single and double lorentzian, with the aim of merging the two resonances into one. We guess model parameters using a differential evolution algorithm, which works iteratively based on the given signal, to minimise sum-of-squared error. Once these parameters are guessed, we fit both a single and double lorentzian to the data. 

With a double and single lorentzian fitted to the data, we use the Akaike Information Criterion (AIC) to compare the models. When the single outperforms the double, we can assume that we are close to overlapping. We can then refine this single lorentzian to optimise the sensor.

## API details

To communicate with the sensor, and perform resonance measurements, we interface with the Zurich Instruments MFLI Python API.

These resonances are then tunable with the following 2 parameters:

  1: Bias field - the bias field will effect the location of the resonance in frequency space. We do this by communicating with the DM-        Technologies Multichannel Current Source (https://dmtechnologies.eu/product/multichannel-current-source/). Interfacing is done along the Serial line.

  2: Laser detuning - Laser detuning will effec the shape of the Lorentzian. Toptica DLC Pro has an API over ethernet, to which we can adjust the voltage on the piezo, effectively changing the frequency of the detuning.


## Code workflow
  1. Coarse search with the bias coil to get some overlap
  2. Refined Bias coil stepped-tuning to maximise signal amplitude (maximum overlap) to find global maximum
  3. Laser detuning stepped-tuning to maximise Amplitude-to-width ratio (we want the maximum signal, for the longest atomic coherence time).

### Example results:

Course search and model selection: 

![image](https://github.com/user-attachments/assets/3d3601b4-284c-45a5-83da-8ad0c81983b4)


Laser optimisation: 
![long_opt](https://github.com/user-attachments/assets/eab6bcb3-011e-49d8-97a3-d1892546b25e)

Example result: 
![laser_optimised_2](https://github.com/user-attachments/assets/a2074697-0a4f-420d-91bb-2fef2a86132e)

### Appendix

setbias.py is a standalone python-executable bias coil setting programme.


