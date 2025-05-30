import serial
import argparse
import time

###############################################################################
# Write current values to the Pustelny Power Supply (PCS)

def setbias(vals):
    
    def runit():
        PCS = serial.Serial(port='COM9', 
                            baudrate=115200,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS,
                            timeout=0)
    
        str1 = '!set;1;4mA;{}'.format(vals[0])
        str2 = '!set;2;4mA;{}'.format(vals[1])
    
        PCS.write(bytes(str1 + '\r', 'utf-8'))
        PCS.write(bytes(str2 + '\r', 'utf-8'))
    
        PCS.close()
        
    try:
        runit()
        
    except:  # If port is already open, close it first
        PCS.close()
        runit()

    print(f"Bias values adjusted! C1: {vals[0]}µA, C2: {vals[1]}µA")  # Final message

###############################################################################
# Parse command-line arguments

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Set bias current values for PCS.")
    parser.add_argument("vals", nargs=2, type=float, help="Bias current values for C1 and C2 in micro-Amps")
    
    args = parser.parse_args()

    start_time = time.time()  # Track execution time
    setbias(args.vals)
    elapsed_time = time.time() - start_time

    print(f"Execution completed in {elapsed_time:.2f} seconds.")
