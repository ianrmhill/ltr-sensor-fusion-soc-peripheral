/************************************************************************************************************
Module: DataAcquisitionIP (V1.0)
Author: Alan Peng
Input: Clk, En (Global enable), CPU Command (32 Bits), SensorReadings (8 types, 16 bit each)

Output: ResultForCPU (32 bits)
-------------------------------------------------------------------------------------------------------------
This module implements the HDL logics of a data acquisition IP for on-chip wear-out sensors. It supports all sensor types developed
at the Ivanov SoC Lab and up to 8 sensors of each type. Communication between the CPU and the sensor is implemented using the
AMBA APB protocol. 

For a typical data acquisition cycle, the CPU first selects a sensor that it intends to get data from, by setting PSELx and SensorIndex.
It will then set a 32-bit command in the following format:
CPU Command [31:30]: Mode (00 - Stop; 01 - Fast Mode; 10 - Slow Mode; 11 - Invalid)
CPU Command [29:27]: PSELx (0 - None; 1 - ROSC; 2 - DC analog; 3 - 8 * EM Sensors; 4 - TDDB; 5 - SILC Slope; 6 - Temperature; 7 - Voltage)
CPU Command [26:24]: SensorIndex (0 - 7, Max 8 sensors of each type)
CPU Command [23:20]: ROSC / TDDB Measurement duration in terms of Number of Clock Cycles 
CPU Command [19:14]: SILC Measurement duration in terms of Number of slopes
CPU Command [13:2]: SILC TimeoutThreshold
CPU Command [0]: CPU Reading Complete

The above formatted command would be written to CPU Command Register through AMBA APB, read by the corresponding sensor.
The sensor would undertake a measurement cycle and report a result to the Sensor Measurement Data Register through AMBA APB.
The result is read by the CPU as a 32-bit data block, for ease of AMBA APB communication.

The results are interprated in the following manner:
ResultForCPU [31:16]: Measurement result
ResultForCPU [15:13]: Error Code (1 - no error; 7 - timeout)
ResultForCPU [0]: Measurement Ready for CPU

NOTES:
1. Please use Slow mode during 1st cycle after power up, fast mode would not provide any data as there is no previous measurement in memory
2. Make sure to reset CPUCommands to 0 before starting another measurement cycle
************************************************************************************************************/

module DataAcquisitionIP(input logic Clk,
                         input logic En,  
                         input logic[31:0] CPUCommand,
                         input logic[7:0][15:0] SensorReadings,
                         output logic[31:0] ResultForCPU
);
    // registers communication implemented using AMBA protocol
    // Global CPU commands register
    logic[31:0] CPUCommandsReg;
    // Global Sensor Reading Register
    logic[31:0] SensorReadingReg;

    // Sensor Inputs
    logic ROSCReading;
    logic[11:0] DCAnalogReading;
    logic[7:0] EMReading;
    logic TDDBReading;
    logic[11:0] SILCADCReading;
    logic[11:0] TemperatureReading;
    logic[11:0] VoltageReading;

    // control signal for AMBA communication
    // pwrite is low during reading and high during writing
    // controlled by global FSM
    logic pwriteWr, pwriteRd;
    logic penableWr, penableRd;

    // control flags for FSM state changes
    // ready and error signals defined for each state
    logic[31:0] APBwritedata, APBreaddata;
    logic[31:0] APBwritedest, APBreadaddr;

    // 32-bit command from Global CPU Command Register, for the selected sensor
    logic[31:0] cpucommandforparsing;

    // control parameters for sensor
    // from cpucommandsforparsing
    logic[1:0] mode;
    logic[2:0] PSELx;
    logic[2:0] SensorIndex;
    logic[3:0] numclkcycles;
    logic[5:0] numdescendingslopes;
    logic[11:0] timeoutthreshold;
    logic cpureadcomplete;

    // global buffer measurement data of the selected sensor
    // this will be written into the Sensor Measurement Data Global Register
    // sensormeasdata [31:16] = result
    // sensormeasdata [15:13] = errorcode
    // sensormeasdata [0] = sensorvalready
    logic[31:0] sensormeasdata;
    
    // sensor output for CPU
    // these will be mapped to 'sensormeasdata above'
    logic[7:0][15:0] result;
    logic[7:0][2:0] errorcode;
    logic[7:0] sensorvalready;

    // hold AMBA read output, measurement result
    logic[31:0] resultforCPU;


    always_comb begin
        mode = cpucommandforparsing[31:30];
        PSELx = cpucommandforparsing[29:27];
        SensorIndex = cpucommandforparsing[26:24];
        numclkcycles = cpucommandforparsing[23:20];
        numdescendingslopes = cpucommandforparsing[19:14];
        timeoutthreshold = cpucommandforparsing[13:2];
        cpureadcomplete = cpucommandforparsing[0];
    end

    // parse sensor inputs
    always_comb begin
        ROSCReading = SensorReadings[1][0];
        DCAnalogReading = SensorReadings[2][11:0];
        EMReading = SensorReadings[3][7:0];
        TDDBReading = SensorReadings[4][0];
        SILCADCReading = SensorReadings[5][11:0];
        TemperatureReading = SensorReadings[6][11:0];
        VoltageReading = SensorReadings[7][11:0];
    end
    
    // FSM handling data acquisition cycle
    // 1. CPU writes commands into register
    // 2. Sensor reads commands from register and starts measurement
    // 3. Sensor writes measurement data into register 
    // 4. CPU reads measurement data

    // AMBA Write Module
    // Handling: CPU write command into register (1); Sensor processing unit write result into register (3)
    AMBA_write APBWrite(Clk, {32{1'b1}}, pwriteWr, penableWr, APBwritedata, APBwritedest, APBwriteready, APBwriteerr);

    // AMBA Read Module
    // Handling: Sensor processing unit read command from register (2); Sensor processing unit read measurement result from register (4)
    AMBA_read APBRead(Clk,{32{1'b1}}, pwriteRd, penableRd, APBreadaddr, APBreaddata, APBreadready, APBreaderr);

    // ROSC sensor
    ROSC ROSC_S1(mode, (PSELx == 1) & (SensorIndex == 0), ROSCReading, Clk, numclkcycles, cpureadcomplete, result[1], errorcode[1], sensorvalready[1]);

    // DC Analog sensor
    AnalogSensor DCAnalog_S1(mode, (PSELx == 2) & (SensorIndex == 0), DCAnalogReading, Clk, cpureadcomplete, result[2], errorcode[2], sensorvalready[2]);

    // EM Sensor
    EM EMSensor_S1(mode, (PSELx == 3) & (SensorIndex == 0), EMReading, Clk, cpureadcomplete, result[3], errorcode[3], sensorvalready[3]);

    // TDDB sensor
    ROSC TDDBSensor_S1(mode, (PSELx == 4) & (SensorIndex == 0), TDDBReading, Clk, numclkcycles, cpureadcomplete, result[4], errorcode[4], sensorvalready[4]);

    // SILC slope sensor
    SILC SILC_S1(mode, (PSELx == 5) & (SensorIndex == 0), SILCADCReading, Clk, numdescendingslopes, timeoutthreshold, cpureadcomplete, result[5], errorcode[5], sensorvalready[5]); 
    
    // Temperature sensor
    AnalogSensor Temperature_S1(mode, (PSELx == 6) & (SensorIndex == 0), TemperatureReading, Clk, cpureadcomplete, result[6], errorcode[6], sensorvalready[6]);

    // Voltage sensor
    AnalogSensor Voltage_S1(mode, (PSELx == 7) & (SensorIndex == 0), VoltageReading, Clk, cpureadcomplete, result[7], errorcode[7], sensorvalready[7]);

    /********************************************************************************************
    FSM Controlling workflow of a measurement cycle
    States: 
    Idle: No active measurement under way;
    CW: CPU writes commands into register;
    SR: Sensor reads commands from register;
    GETDATA: sensor undertaking measurement;
    SW: Sensor writes obtained measurement into register;
    CR: CPU reads measurement from register;
    COMPLETE: measurement cycle is fulfilled without error;
    ERROR: AMBA communication failed
    *********************************************************************************************/
    typedef enum logic[4:0] {IDLE, CW, SR, GETDATA, SW, CR, COMPLETE, ERROR} statetype;
    statetype state, nextstate;

    // state transitions
    always_ff @ (posedge Clk) begin
        // if system is not enabled, OR sensor type 0 selected, OR Commands are empty
        if (!En || CPUCommand == {32{1'b0}}) begin
            state <= IDLE;
        end
        else state <= nextstate;
    end

    // next state logic
    always_comb
        case (state)
            IDLE: nextstate = CW;

            CW: begin
                if (!APBwriteready) nextstate = CW;
                else begin
                    if (!APBwriteerr) nextstate = SR;
                    else nextstate = ERROR;
                end
            end

            SR: begin
                if (!APBreadready) nextstate = SR;
                else begin
                    if (!APBreaderr) nextstate = GETDATA;
                    else nextstate = ERROR;
                end
            end

            GETDATA: begin
                if (!sensorvalready[PSELx]) nextstate = GETDATA;
                else nextstate = SW;

            end

            SW: begin
                if (!APBwriteready) nextstate = SW;
                else begin
                    if (!APBwriteerr) nextstate = CR;
                    else nextstate = ERROR;
                end
            end

            CR: begin
                if (!APBreadready) nextstate = CR;
                else begin
                    if (!APBreaderr) nextstate = COMPLETE;
                    else nextstate = ERROR;
                end
            end

            // hold data, wait for reset
            COMPLETE: nextstate = COMPLETE;
            // remain in error state, wait for reset
            ERROR: nextstate = ERROR;
        endcase

    // output logic
    // control signals for AMBA_Read and AMBA_write
    always_comb
        case (state)
            IDLE: begin
                pwriteWr = 0;
                penableWr = 0;
                pwriteRd = 1;
                penableRd = 0;
                APBwritedata = 0;
                APBreadaddr = CPUCommandsReg;
                ResultForCPU = 0;
            end

            CW: begin
                pwriteWr = 1;
                penableWr = 1;
                pwriteRd = 1;
                penableRd = 0;
                APBwritedata = CPUCommand;
                APBreadaddr = CPUCommandsReg;
                ResultForCPU = 0;
            end

            SR: begin
                pwriteWr = 0;
                penableWr = 0;
                pwriteRd = 0;
                penableRd = 1;
                APBwritedata = 0;
                APBreadaddr = CPUCommandsReg;
                ResultForCPU = 0;
            end

            GETDATA: begin
                pwriteWr = 0;
                penableWr = 0;
                pwriteRd = 1;
                penableRd = 0;
                APBwritedata = 0;
                APBreadaddr = CPUCommandsReg;
                ResultForCPU = 0;
            end

            SW: begin
                pwriteWr = 1;
                penableWr = 1;
                pwriteRd = 1;
                penableRd = 0;
                APBwritedata = sensormeasdata;
                APBreadaddr = SensorReadingReg;
                ResultForCPU = 0;
            end

            CR: begin
                pwriteWr = 0;
                penableWr = 0;
                pwriteRd = 0;
                penableRd = 1;
                APBwritedata = 0;
                APBreadaddr = SensorReadingReg;
                ResultForCPU = 0;
            end

            COMPLETE: begin
                pwriteWr = 0;
                penableWr = 0;
                pwriteRd = 1;
                penableRd = 0;
                APBwritedata = 0;
                APBreadaddr = CPUCommandsReg;
                ResultForCPU = resultforCPU;
            end
            
            // AMBA Error state: output Result all 1's
            ERROR: begin
                pwriteWr = 0;
                penableWr = 0;
                pwriteRd = 1;
                penableRd = 0;
                APBwritedata = 0;
                APBreadaddr = CPUCommandsReg;
                ResultForCPU = {32{1'b1}};
            end
        endcase


always_ff @ (posedge Clk) begin
    // reset registers in idle mode, when a measurement cycle completes
    if (state == IDLE) begin
        // reset global registers
        CPUCommandsReg <= 0;
        SensorReadingReg <= 0;
        // reset local registers
        cpucommandforparsing <= 0;
        sensormeasdata <= 0;
        resultforCPU <= 0;
    end

    // update global cpu command register when CW ready
    if (state == CW) begin
        if (APBwriteready && !APBwriteerr) CPUCommandsReg <= APBwritedest;
    end

    // update cpu control commands for sensor when SR completes
    if (state == SR) begin
        if (APBreadready && !APBreaderr) begin
            cpucommandforparsing <= APBreaddata;
        end
    end

    if (state == GETDATA) begin
        // port results into Sensormeasdata if sensor indicates value is ready
        if (sensorvalready[PSELx]) begin
            sensormeasdata[31:16] <= result[PSELx];
            sensormeasdata[15:13] <= errorcode[PSELx];
            sensormeasdata[0] <= sensorvalready[PSELx];
        end
    end

    // update Global Sensor Data Register when SW complete
    if (state == SW) begin
        if (APBwriteready && !APBwriteerr) SensorReadingReg <= APBwritedest;
    end

    // update register holding results read by CPU when CPU read complete
    if (state == CR) begin
        if (APBreadready && !APBreaderr) begin
            resultforCPU <= APBreaddata;
        end
    end
end

endmodule




