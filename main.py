
###############################################################################
###############################################################################
from toptica.lasersdk.dlcpro.v3_2_0 import DLCpro,NetworkConnection
from zhinst.toolkit import Session
import logging
import sys
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
import numpy as np
import warnings
from scipy.optimize import differential_evolution
import serial
import time

start_time = time.time()

plt.close('all')

session = Session("172.29.168.20")
device = session.connect_device("DEV3994")

OUT_CHANNEL = 0         #Output channel: Sig out 1
OUT_MIXER_CHANNEL = 1   # UHFLI: 3, HF2LI: 6, MFLI: 1
IN_CHANNEL = 1          #0:
CURR_INDEX = 0
DEMOD_INDEX = 0         #Demodulator Index
OSC_INDEX = 0           #Oscillator isndex (I think we only have 1 oscillator)

#C1,C2 bias current in micro-Amps5
b1_fix = 275
b2_start = 290

#Laser detuning
current_voltage = 34.35

#SET NODES
with device.set_transaction():
  
    device.currins[CURR_INDEX].on(True)
    device.currins[CURR_INDEX].autorange()
    device.currins[CURR_INDEX].float(0)                           #Floating ground    OFF
    device.currins[CURR_INDEX].scaling(1)                         #Input scaling      1V/1V
    device.currins[CURR_INDEX].range(1e-6)   
    # device.currins[IN_CHANNEL].range(0.010)                     #Amplifier range    

    # assert device.sigins[IN_CHANNEL].range() <= 0.30000001192092896, (
    #     'Autorange has not settled to the minimum value: check sensor and DC offsets'
    #     )
        
    device.demods[DEMOD_INDEX].adcselect(IN_CHANNEL)            #Select ADC          
    device.demods[DEMOD_INDEX].order(4)                         #LPF order
    device.demods[DEMOD_INDEX].rate(13.39e3)                    #PC sampling rate for data collection
    device.demods[DEMOD_INDEX].oscselect(OSC_INDEX)             #select internal oscillator (only one for us?)
    device.demods[DEMOD_INDEX].harmonic(1)                      #Multiplies ref osc freq by this integer factor (INVESTIGATE NOTE IN THIS ABOUT PLL LOCKING)
    device.demods[DEMOD_INDEX].phaseshift(72)                   #Applied phase shift (deg) to internal ref
    device.demods[DEMOD_INDEX].sinc(0)                          #Sinc filter OFF
    device.demods[DEMOD_INDEX].timeconstant(0.0007)             #Filter time constant
    device.demods[DEMOD_INDEX].enable(True)                     #Enables Data collection for this demodulator, increases load on PC port
    
    device.sigouts[OUT_CHANNEL].on(True)                        #Output                     ON
    device.sigouts[OUT_CHANNEL].imp50(1)                        #50Ohm Imp                  ON 
    device.sigouts[OUT_CHANNEL].diff(0)                         #Single output; diff output OFF
    device.sigouts[OUT_CHANNEL].range(1)                        #Range 5V
    device.sigouts[OUT_CHANNEL].offset(0)                       #DC offset 0V
    device.sigouts[OUT_CHANNEL].enables[OUT_MIXER_CHANNEL](1)   #Enable output amplitudes to drive to sig out 1 (switch 'En' on Block Diagram)
    device.sigouts[OUT_CHANNEL].amplitudes(3)                   #Output amplitude max Vpk
    
    
#Write current values to the Pustelny Power Supply (PPS)
def setbias(vals):
    
    def runit():
        PPS = serial.Serial(port='COM9', 
                            baudrate=115200,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS,
                            timeout=0)
    
        str1 ='!set;1;4mA;{}'.format(vals[0])
        str2 ='!set;2;4mA;{}'.format(vals[1])
    
        PPS.write(bytes(str1+'\r','utf-8'))
        PPS.write(bytes(str2+'\r','utf-8'))
    
        PPS.close()
        
    try:
        runit()
        
    except: #If port is already open, close and reopen
        
        PPS.close()
        runit()
         
    return


vals = [b1_fix,b2_start]
setbias(vals)

host = '172.29.169.14'

def set_scan_offset(value):
    # Value in Volts
    with DLCpro(NetworkConnection(host)) as dlcpro:
        dlcpro.laser1.scan.offset.set(value)
        print(f'Scan Offset adjusted: {value}V')

#Set Laser

set_scan_offset(current_voltage)

with DLCpro(NetworkConnection(host)) as dlcpro:
    current_voltage = dlcpro.laser1.scan.offset.get()

###############################################################################
###############################################################################
#Sweeper function
def sweep_now(device,start,stop,samples,OSC_INDEX,DEMOD_INDEX,save,iteration):
    # Specify the number of sweeps to perform back-to-back.
    LOOPCOUNT = 1
    
    sweeper = session.modules.sweeper
    sweeper.device(device)
    
    sweeper.gridnode(device.oscs[OSC_INDEX].freq)
    sweeper.start(start)
    sweeper.stop(stop)
    sweeper.samplecount(samples)
    sweeper.xmapping(1)
    sweeper.bandwidthcontrol(1) #Changes bandwidth
    sweeper.bandwidthoverlap(0)
    sweeper.scan(0)
    sweeper.loopcount(LOOPCOUNT)
    sweeper.settling.time(0)
    sweeper.settling.inaccuracy(0.001)
    sweeper.averaging.tc(0)
    sweeper.averaging.sample(1)
    
    sample_node = device.demods[DEMOD_INDEX].sample
    sweeper.subscribe(sample_node)
    
    # sweeper.save.filename('sweep_with_save')
    # sweeper.save.fileformat('csv')
    
    handler = logging.StreamHandler(sys.stdout)
    logging.getLogger("zhinst.toolkit").setLevel(logging.WARNING)
    logging.getLogger("zhinst.toolkit").addHandler(handler)
    
    sweeper.execute()
    print(f"Sweeping no. {iteration}")
    sweeper.wait_done(timeout=300)
    
    data = sweeper.read()
    sweeper.unsubscribe(sample_node)
    num_sweeps = len(data[sample_node])
    assert num_sweeps == LOOPCOUNT, (
        f"The sweeper returned an unexpected number of sweeps: "
        f"{num_sweeps}. Expected: {LOOPCOUNT}."
    )
    return data, sample_node

###############################################################################
###############################################################################
#Functions to fit
def Lorentzian_double(x,amp0,amp1,cen0,cen1,wid0,wid1,slope,offset):
    return ((amp0*(wid0)**2/((x-cen0)**2+(wid0)**2)) + (amp1*(wid1)**2/((x-cen1)**2+(wid1)**2))) +slope*x + offset

def Lorentzian_single(x,amp0,cen0,wid0,slope,offset):
    return ((amp0*(wid0)**2/((x-cen0)**2+(wid0)**2))) +slope*x + offset
###############################################################################    
###############################################################################
#Plot swept data
def plot_sweep(node_samples,sample_node):
    
    fig, axs = plt.subplots(3, 1)
    for sample in node_samples:
        frequency = sample[0]["frequency"]
        x = sample[0]["x"]
        y = sample[0]["y"]
        phi = np.angle(sample[0]["x"] + 1j * sample[0]["y"])
        
        axs[0].plot(frequency, x)
        axs[1].plot(frequency, y)
        axs[2].plot(frequency, phi)
        
    axs[0].set_title(f"Results of {len(node_samples)} sweeps.")
    axs[0].grid()
    axs[0].set_ylabel("Quadrature signal (Amps)")
    # axs[0].set_xscale("log")
    axs[0].autoscale()
    
    axs[1].grid()
    axs[1].set_xlabel("Frequency ($Hz$)")
    axs[1].set_ylabel("In-phase Signal (Amps)")
    # axs[1].set_xscale("log")
    axs[1].autoscale()
    
    axs[2].grid()
    axs[2].set_xlabel("Frequency ($Hz$)")
    axs[2].set_ylabel(r"Phi (radians)")
    # axs[2].set_xscale("log")
    axs[2].autoscale()
    
    plt.draw()
    plt.show()
    
    return fig, axs

###############################################################################
###############################################################################
# function for genetic algorithm to minimize (sum of squared error) CITATION
# bounds on parameters are set in generate_Initial_Parameters() below

#Double Lorentzian
def sumOfSquaredError_double(parameterTuple, xData, yData):
    warnings.filterwarnings("ignore") # do not print warnings by genetic algorithm
    return np.sum((yData - Lorentzian_double(xData, *parameterTuple)) ** 2)

def generate_Initial_Parameters_double(xData, yData):
    # min and max used for bounds
    maxX = max(xData)
    minX = min(xData)
    maxY = max(yData)
    minY = min(yData)
    
    parameterBounds = [
        [maxY/1.15, maxY*2],    # parameter bounds for A
        [maxY/1.15, maxY*2],    # parameter bounds for A1
        [minX, maxX],     # parameter bounds for c0
        [minX, maxX],     # parameter bounds for c1
        [50, 110],       # parameter bounds for w0 (HWHM)
        [50, 110],       # parameter bounds for w1 
        [-0.005, 0.005],  # parameter bounds for slope
        [maxY/-0.005, maxY/0.005]  # parameter bounds for offset
    ]
    

    def sumOfSquaredError_wrapper_double(parameterTuple):
        return sumOfSquaredError_double(parameterTuple, xData, yData)

    # "seed" the numpy random number generator for repeatable results
    result = differential_evolution(sumOfSquaredError_wrapper_double, parameterBounds, seed=3)
    return result.x

def sumOfSquaredError_single(parameterTuple, xData, yData):
    warnings.filterwarnings("ignore") # do not print warnings by genetic algorithm
    return np.sum((yData - Lorentzian_single(xData, *parameterTuple)) ** 2)

def generate_Initial_Parameters_single(xData, yData):
    # min and max used for bounds
    maxX = max(xData)
    minX = min(xData)
    maxY = max(yData)
    minY = min(yData)
    
    parameterBounds = [
       [maxY/1.5, maxY*2],      # parameter bounds for A
       [minX, maxX],            # parameter bounds for c0
       [10, 150],               # parameter bounds for w0 (HWHM)
       [-0.005, 0.005],         # parameter bounds for slope
       [maxY/-0.005, maxY/0.005]  # parameter bounds for offset
       ]

    def sumOfSquaredError_wrapper_single(parameterTuple):
        return sumOfSquaredError_single(parameterTuple, xData, yData)

    # "seed" the numpy random number generator for repeatable results
    result = differential_evolution(sumOfSquaredError_wrapper_single, parameterBounds, seed=3)
    return result.x

###############################################################################
###############################################################################
def read_and_fit(data, sample_node, verbose=True, plot=True, ax=None, threshold=80, force_model=None):
    node_samples = data[sample_node]
    xData = node_samples[0][0]["frequency"]
    yData = node_samples[0][0]["x"]
    
    # Ensure xData is valid
    if len(xData) == 0:
        raise ValueError("xData is empty, cannot proceed with fitting.")

    n = len(xData)  # Assign n after xData is initialized
    
    # Fit single
    initial_single = generate_Initial_Parameters_single(xData, yData)
    popt_single, _ = curve_fit(Lorentzian_single, xData, yData, initial_single, maxfev=5000)
    y_fit_single = Lorentzian_single(xData, *popt_single)
    residuals_single = yData - y_fit_single
    ssr_single = np.sum(residuals_single**2)
    k_single = len(popt_single)
    
    # Fit double
    initial_double = generate_Initial_Parameters_double(xData, yData)
    popt_double, _ = curve_fit(Lorentzian_double, xData, yData, initial_double, maxfev=5000)
    y_fit_double = Lorentzian_double(xData, *popt_double)
    residuals_double = yData - y_fit_double
    ssr_double = np.sum(residuals_double**2)
    k_double = len(popt_double)
    
    # Compute AICs
    aic_single = n * np.log(ssr_single / n) + 2 * k_single
    aic_double = n * np.log(ssr_double / n) + 2 * k_double
    
    delta_aic = aic_double - aic_single  # Negative means double is better
    
    # Force the model choice to 'single' if requested
    if force_model == "single":
        chosen = 'single'
        fittedParameters = popt_single
        fit_eval = y_fit_single
    elif delta_aic < -threshold:
        chosen = 'double'
        fittedParameters = popt_double
        fit_eval = y_fit_double
    else:
        chosen = 'single'
        fittedParameters = popt_single
        fit_eval = y_fit_single
    
    if verbose:
        print(f"Model selected: {chosen.upper()} | AIC(single)={aic_single:.2f}  AIC(double)={aic_double:.2f}")

    if plot and ax is not None:
        ax.plot(xData, yData, 'o', markersize=0.5, label='data')
        ax.plot(xData, fit_eval, '--', label=f'{chosen} fit')
        ax.set_title(f"{chosen.upper()} model")
        ax.legend()

    return xData, yData, fittedParameters, fit_eval, chosen, aic_single, aic_double


###############################################################################
###############################################################################
#Coarse sweep

start_b2 = 375
stop_b2 = 400
num = 5

f1 = 1.0e3
f2 = 2.5e3

print(f'Performing Coarse Bias search. Number of sweeps: {num}')

biases = np.linspace(start_b2, stop_b2, num=num)

perr_list = np.zeros((8, len(biases)))
param_list = np.zeros((8, len(biases)))

overlap_list = []
model_choices = []
single_aic_list = []

fig, axs = plt.subplots(nrows=1, ncols=len(biases), figsize=(15, 4), sharey=True)

for i, (bias, ax) in enumerate(zip(biases, axs)):
    setbias([b1_fix, bias])
    data, sample_node = sweep_now(device, f1, f2, 100, OSC_INDEX, DEMOD_INDEX, 0, i+1)

    # read_and_fit should now return aic_single and aic_double too
    xData, yData, fittedParameters, fit_eval, model, aic_single, aic_double = read_and_fit(data, sample_node, ax=ax)
    model_choices.append(model)

    if model == "double":
        c0, c1 = fittedParameters[2], fittedParameters[3]
        w0, w1 = fittedParameters[4], fittedParameters[5]
        overlap_metric = abs(c1 - c0) / ((w0 + w1) / 2)
        overlap_list.append(overlap_metric)
        single_aic_list.append(np.inf)  # placeholder for double models
    else:
        overlap_list.append(np.inf)
        single_aic_list.append(aic_single)

plt.tight_layout()
plt.show()
print("Model choices per bias:", model_choices)
print("Overlap metrics:", overlap_list)

# Check for any single-model fits
single_indices = [i for i, model in enumerate(model_choices) if model == "single"]

if not single_indices:
    raise RuntimeError("No suitable overlap found: all sweeps selected the DOUBLE model.")

# Find best single-model fit (lowest AIC among singles)
best_idx_in_single = np.argmin([single_aic_list[i] for i in single_indices])
best_run_index = single_indices[best_idx_in_single]
best_aic = single_aic_list[best_run_index]

print(f"Best SINGLE model fit found at sweep index {best_run_index} with AIC = {best_aic:.2f}")

###############################################################################
###############################################################################
#%% 
#Swap to single-Lorentzian and optimise amplitude/width around this region.

print('Fitting Single Lorentzian')

# Use bias value from previous AIC-based best single model
bias = biases[best_run_index]              
setbias([b1_fix, bias])
# Plot initial run
fig, axs = plt.subplots(nrows=1, ncols=1)

data, sample_node = sweep_now(device, 1.0e3, 2.2e3, 60, OSC_INDEX, DEMOD_INDEX, 0, 1)
xData_l1, yData_l1, fittedParameters_l1, fit_eval_l1, _, _, _ = read_and_fit(data, sample_node, verbose=False,ax=axs, force_model="single")

axs.plot(xData_l1, yData_l1, 'o', markersize=0.5, label='Initial run')
axs.plot(xData_l1, fit_eval_l1, c='red')
plt.pause(0.005)

# Define parameter indices
a_inx, c_inx, w_inx = 0, 1, 2

# Initialize the second figure for parameter tracking
fig2, ax2 = plt.subplots(3, 1)

# Plot the initial fitted parameters
ax2[0].plot(-1, fittedParameters_l1[a_inx], 'o')
ax2[1].plot(-1, fittedParameters_l1[c_inx], 'o')
ax2[2].plot(-1, fittedParameters_l1[w_inx], 'o')
plt.pause(0.005)

#get central frequency to estimate new sweep range

f1_adj = fittedParameters_l1[c_inx]-300
f2_adj = fittedParameters_l1[c_inx]+300


# Store parameters
a = [fittedParameters_l1[a_inx]]
w = [fittedParameters_l1[w_inx]*2]


def perform_sweep_and_update(bias, i):
    setbias([b1_fix, bias])
    data, sample_node = sweep_now(device, f1_adj, f2_adj, 60, OSC_INDEX, DEMOD_INDEX, 0, i)
    _, yData, fp, fit_eval, _, _, _ = read_and_fit(data, sample_node, ax=axs, verbose=False,force_model="single")
    return fp, fit_eval, yData

# 'Nudge' bias in one direction
bias += 2
fp, fit_eval, _ = perform_sweep_and_update(bias, 2)

a.append(fp[a_inx])
w.append(fp[w_inx])

# Plot the first sweep results
ax2[0].plot(0, fp[a_inx], 'o', c='black')
ax2[0].set_ylabel('Amplitude')

ax2[1].plot(0, fp[c_inx], 'o', c='black')
ax2[1].set_ylabel('Central Frequency')

ax2[2].plot(0, fp[w_inx], 'o', c='black')
ax2[2].set_ylabel('Width (Hz)')
plt.pause(0.005)

# Loop parameters
i = 1
stable_count = 0
stable_threshold = 5
percent_threshold = 0.05
max_iterations = 20
direction = 1.5  # Positive direction initially

print('Optimising bias...')

while True:
    fp, fit_eval, yData = perform_sweep_and_update(bias, i + 2)
    
    current_a = fp[a_inx]  # Change to track `a` instead of `w`
    a.append(current_a)  # Append `a` to the list
    w.append(fp[w_inx]*2)  # Still track `w` if needed for later, but not optimizing for it
    
    ax2[0].plot(i, current_a, 'o', c='black')  # Plot `a` for tracking
    ax2[1].plot(i, fp[c_inx], 'o', c='black')
    ax2[2].plot(i, fp[w_inx]*2, 'o', c='black')
    plt.pause(0.005)

    # Direction logic for `a` optimization (based on `a`)
    if len(a) > 1 and current_a < a[-2]:  # Look for when `a` decreases (adjust if different criteria needed)
        direction = -direction

    bias += direction  # Adjust bias in the identified direction

    # Stopping condition based on `a`
    if len(a) >= 6:
        last_5_avg = np.mean(a[-6:-1])  # Track the last 5 values of `a`
        recent_min = min(a[-6:-1])  # Find minimum of the last 5 values
        if abs(a[-1] - recent_min) / recent_min <= percent_threshold:  # Compare change to percentage threshold
            stable_count += 1
        else:
            stable_count = 0

        if stable_count >= stable_threshold:
            print(f"Stopping condition met at iteration {i}. Amplitude stable.")
            break

    if i >= max_iterations:
        print(f"{i} iterations reached, exiting.")
        break

    i += 1

# Final result
a_over_w = np.array(a)/ np.array(w)
print('Bias optimised.')
print(f'Final amplitude = {a[-1]:.2e} (a) | Final width = {round(w[-1])} Hz | CF = {round(fp[c_inx])} Hz')
slope_inc = (a_over_w[-1] / a_over_w[0] - 1) * 100
print(f'A/W slope increase of {round(slope_inc, 1)}% during optimisation')


###############################################################################
###############################################################################
#%% Optimise Laser
print('Reading Laser..')

def perform_sweep_and_laser(voltage, j):
    set_scan_offset(voltage)
    data, sample_node = sweep_now(device, f1_adj, f2_adj, 60, OSC_INDEX, DEMOD_INDEX, 0, j)
    _, yData, fp, fit_eval, _, _, _ = read_and_fit(data, sample_node, ax=axs, verbose=False, force_model="single")
    return fp, fit_eval, yData

fig4, ax4 = plt.subplots(4, 1)
ax4[0].set_ylabel('Amplitude')
ax4[1].set_ylabel('Central Frequency')
ax4[2].set_ylabel('Width (Hz)')
ax4[3].set_ylabel('A/W')

fp, fit_eval, yData = perform_sweep_and_laser(current_voltage, 0)

current_voltage += 0.1

awl = list([fp[a_inx] / fp[w_inx]])

# Loop parameters
j = 1
stable_count = 0
stable_threshold = 5  # Number of consecutive iterations to confirm stability
percent_threshold = 0.05  # 3% threshold for stability check
max_iterations = 20  # Maximum number of iterations before exiting (timeout condition)
direction = 0.05  # Initial direction (positive bias change)

print('Optimising Laser...')

awl = list()
w = list()
a = list()

while True:
    fp, fit_eval_l, yData_l = perform_sweep_and_laser(current_voltage, j)

    w.append(fp[w_inx])
    a.append(fp[a_inx])

    # Append new width value
    current_aw = a[-1] / w[-1]
    awl.append(current_aw)

    # Plot data
    ax4[0].plot(j, fp[a_inx], 'o', c='black')
    ax4[1].plot(j, fp[c_inx], 'o', c='black')
    ax4[2].plot(j, fp[w_inx], 'o', c='black')
    ax4[3].plot(j, current_aw, 'o', c='black')
    
    plt.pause(0.005)

    # Check the direction of change in awl
    if len(awl) > 1:
        previous_aw = awl[-2]
        if current_aw < previous_aw:
            direction = -direction  # Change direction if awl decreased
    
    # Update bias
    current_voltage += direction

    # Check the stopping condition
    if len(awl) >= 6:  # Ensure we have at least 6 values to check the last 5
        last_5_avg = np.mean(awl[-6:-1])
        recent_max = max(awl[-6:-1])

        if abs(awl[-1] - recent_max) / recent_max <= percent_threshold:
            stable_count += 1
        else:
            stable_count = 0  # Reset the counter if the condition is not met
        
        print(str(stable_count))
        if stable_count >= stable_threshold:
            print(f"Stopping condition met at iteration {j}. Current awl is within {percent_threshold * 100}% of the recent maximum for {stable_threshold} consecutive iterations.")
            break

    if j == max_iterations:
        print(f"{j} iterations reached, check for errors")
        break

    j += 1  # Increment index


fig3, ax3 = plt.subplots()

ax3.plot(xData_l1, yData_l1, 'o', markersize=0.5)
ax3.plot(xData_l1, fit_eval_l1, c='red', label='First Run')

ax3.plot(xData_l1, yData, 'o', markersize=0.5)
ax3.plot(xData_l1, fit_eval, c='blue', label='Bias-Optimised Run')


ax3.plot(xData_l1, yData_l, 'o', markersize=0.5)
ax3.plot(xData_l1, fit_eval_l, c='green', label='Laser-Optimised Run')

ax3.set(ylabel='Amplitude',xlabel='Frequency (Hz)')

ax3.legend()
plt.pause(0.005)

elapsed_time = (time.time() - start_time) / 60
print(f"Elapsed time: {elapsed_time:.4f} minutes")



