README

# Wearout Sensor Monitoring IP

This repository contains the full HDL design and validation testbench for an on-chip wearout sensor monitoring IP. It is designed to be ported to the output channels of the wear-out monitoring sensors developed at the Ivanov SoC Lab at UBC ECE. 

The IP accommodates ROSC, SILC, EM and various Analog Sensors whose values are converted into 12-bit ADC readings. Up to 8 sensors of each variant is allowed to be ported to the IP.

The IP design is developed in Systemverilog and adapts to Verilog environments. It is verified to be fully compliable and synthesizable using Intel Quartus platform. Netlists could be extracted following successful compilation. However, EDA and post-synthesis optimizations are not conducted as part of this solution.

The RTL source files are available at the following path:

https://github.com/MateoJRendon/Ivanov_SLM_Wearout_IP/tree/main/rtl

The smoke testbenches and UVM testbenches are available at the following path:

https://github.com/MateoJRendon/Ivanov_SLM_Wearout_IP/tree/main/tb


# Notes on Setup and Usage

## Setup
1. Before powering up the Monitoring IP, ensure the desired sensor is ON and ENABLED. Otherwise, the IP would report a reading of 0 or timeout
2. Output from Analog Sensors and SILC Sensors need to go through a 12-bit ADC and converted into a digital value. Ensure the ADC covers the full range of the analog output such that the IP does not process using faulty information
3. Connect the IP interface with sensors and CPU as follows:
    - SensorReadings[0] <-> Reserved, leave open
    - SensorReadings[1] <-> ROSC Frequency Sensor 
    - SensorReadings[2] <-> DC Analog Sensor
    - SensorReadings[3] <-> EM Sensor
    - SensorReadings[4] <-> TDDB Frequency Sensor
    - SensorReadings[5] <-> SILC Slope Sensor
    - SensorReadings[6] <-> Temperature Analog Sensor
    - SensorReadings[7] <-> Voltage Analog Sensor

    - CPUCommands[31:0] <-> AXI2APB bus carrying 32-bit command from MCU
    - ResultforCPU[31:0]      <-> AXI2APB bus porting measurement result back to MCU

    - Clk               <-> Reference clock on chip
    - En                <-> DC High/Low voltage on chip

## Measurement
To initiate a transaction, follow the steps below:

1. Ensure the designated wear-out sensor is active and functional.
2. Set IP Enable signal (En) High.
3. Assert a 32-bit CPU command following the pattern below:
    - CPU Command [31:30]: Mode (00 - Stop; 01 - Fast Mode; 10 - Slow Mode; 11 - Invalid)
    - CPU Command [29:27]: PSELx (0 - None; 1 - ROSC; 2 - DC analog; 3 - 8 * EM Sensors; 4 - TDDB; 5 - SILC Slope; 6 - Temperature; 7 - Voltage)
    - CPU Command [26:24]: SensorIndex (0 - 7, Max 8 sensors of each type)
    - CPU Command [23:20]: ROSC / TDDB Measurement duration in terms of Number of Clock Cycles 
    - CPU Command [19:14]: SILC Measurement duration in terms of Number of slopes
    - CPU Command [13:2]: SILC TimeoutThreshold
    - CPU Command [0]: CPU Reading Complete
4. Wait until the last bit of 32-bit result message becomes High, indicating measurement cycle completion.
5. Parse the result according to the lookup table below. Result is valid if the returned Error Code is 3’b001. 
    - ResultForCPU [31:15]: Measurement result
    - ResultForCPU [14:12]: Error Code (1 - no error; 7 - timeout)
    - ResultForCPU [0]: Measurement Ready for CPU

## Other notes
1. The first measurement for each sensor after the IP powers up needs to be in SLOW Mode. This ensures a successful reading is available for subsequent FAST Mode acquisition.
2. In case of AMBA communication error, reset CPU command and hold for at least 5 clock cycles.
3. For a SILC sensor, Error Code of 3’b111 indicates measurement times out – not enough slope count is captured within the specified Timeout Threshold.
4. In case of AMBA communication error, the IP would return a 32-bit message of all 1’s. Consider power cycling the module and restart the transaction.


# Module hierarchy and Compliation Flow

## Hierarchy
The Wear-out Monitoring IP is broken down into individual modules and designed using a bottom-up approach. 
You can refer to the report attached to this Github Repo for detailed design methodology behind the code.
The modules are interconnected in the following structure, where CPU commands flow from top of hierarchy to bottom and MUX'ed into corresponding sensor; sensor measurement data flows from bottom to top and de-MUX'ed to merge into CPU.

     DataAcquisition IP    <->   AMBA-Read     <-> ROSC
                                 AMBA-Write    <-> SILC
                                               <-> EM
                                               <-> AnalogSensor

The individual sensors can be further decomposed into fundamental HDL design units:
Each sensor processing unit comprises an upper level control FSM lower level ALU units such as counters, timers and data processing units.

                 Control                         ALU
     |ROSC        -> ROSCGetDataFSM   | -> Frequency Counter
     |SILC        -> SILCGetDataFSM   | -> GetSlopeFSM -> Slope Counter
     |EM          -> GetEMReadingFSM  | -> AcquireEMData
     |AnalogSensor-> AnalogGetDataFSM | -> AcquireAnalogData

## Design rules implemented
1. AMBA APB communication uses a packet size of 32.
2. All memory access / reset are implemented as asynchronous.
3. Active High Enable used throughout the design.
4. Internal register initialization / reset implemented using the Default state of the FSM.
5. All output measurement results from the sensor processing units are normalized to 16 bits. If the results are less than 16 bits, they always fill up the LSB's first.
6. In the IP module, only 1 sensor of each variant is included for demonstration purpose of this project. You can add in more sensor modules but also need to modify the bus structure of SensorReadings to reflect the change.

## Tests implemented using Systemverilog Testbench
1. Fast Mode, Slow mode measurement and switching between the two (All sensor data processing modules)
2. Parameter change (e.g NumClkCycles) during an active measurement (should take effect in the next cycle) and after a measurement (All sensor data processinf modules)
3. Timeout for SILC Measurement, should return error code 3'b111
4. AMBA communication failure, should return result for CPU as {32{1'b1}}
5. Full IP level transaction to complete a measurement cycle
6. IP level transaction changing sensors / parameters (should not interrupt current measurement cycle)
7. IP level hard reset during an active measurement

## Compilation flow
The IP is compiled using a bottom-up approach. If any modifications is made to the design, compile the solution in the following order:
1. ROSC.sv, SILC.sv, EM.sv, Analog.sv
2. AMBA_read.sv, AMBA_Write.sv
3. DataAcquisitionIP.sv

When compiling the full solution in Quartus, ensure that all above files are included in the project and DataAcquisitionIP.sv is set to be the top-level entity.


# Open items
1. Add an intermediate data processing module to perform inter-sensor statistics calculation (averaging between selected sensors etc.)
2. Add a processing module for handling ROSC/ TDDB sensor frequencies slower than the Clk frequency (the current design requires the opposite)
3. EDA steps for layout and post-synthesis optimization
