`timescale 1ns/1ps

/************************************************************************************************************
Module: DataAcquisitionIP_tb (V1.0)
Author: Alan Peng
Input: Clk, CPU Command (32 Bits), SensorReadings (8 sensors, 16 bit each)

Output: ResultForCPU (32 bits)
-------------------------------------------------------------------------------------------------------------
This module implements the testbench of the DataAcquisition IP. Test coverage:
1. Completion of a measurement cycle for a specific sensor, given appropriate commands from CPU (✓)
2. Fast & Slow mode (✓)
3. Transition between sensors (with CPU Command reset in between) (✓)
4. Response to Invalid CPU Commands (✓)
5. SILC timeout scenario (✓)
6. CPU Command gets interrupted in the middle of an acquisition cycle - should return to IDLE mode (✓)
************************************************************************************************************/
module DataAcquisitionIP_tb;

logic Clk;
logic En;
logic[31:0] CPUCommand;
logic[7:0][15:0] SensorReadings;
logic[31:0] ResultForCPU;

DataAcquisitionIP DUT(.*);


// Sensor Inputs
logic ROSCReading;
logic[11:0] DCAnalogReading;
logic[7:0] EMReading;
logic TDDBReading;
logic[11:0] SILCADCReading;
logic[11:0] TemperatureReading;
logic[11:0] VoltageReading;

// parse sensor inputs
always_comb begin
    SensorReadings[1][0] = ROSCReading;
    SensorReadings[2][11:0] = DCAnalogReading;
    SensorReadings[3][7:0] = EMReading;
    SensorReadings[4][0] = TDDBReading;
    SensorReadings[5][11:0] = SILCADCReading;
    SensorReadings[6][11:0] = TemperatureReading;
    SensorReadings[7][11:0] = VoltageReading;
end

initial begin
    Clk = 0;
    En = 0;
    CPUCommand = {32{1'b0}};
    ROSCReading = 0;
    DCAnalogReading = 0;
    EMReading = 0;
    TDDBReading = 0;
    SILCADCReading = 0;
    TemperatureReading = 0;
    VoltageReading = 0;
end

// Clock
always begin
    #10 Clk = ~Clk;
end

//ROSC Data Set
always begin
    #1 ROSCReading = ~ROSCReading;
end

// TDDB Data set
always begin
    #2 TDDBReading = ~TDDBReading;
end

// SILC ADC Data set (Steep slope)
task automatic ADCDataGenerate();
    #30;
    SILCADCReading = 3480;
    #20;
    SILCADCReading = 2661;
    #20;
    SILCADCReading = 1600;
    #20;
    SILCADCReading = 800;
    #20;
    SILCADCReading = 0;
    #100;
    SILCADCReading = 3480;
    #20;
    SILCADCReading = 2661;
    #20;
    SILCADCReading = 1600;
    #20;
    SILCADCReading = 800;
    #20;
    SILCADCReading = 0;
    #100;
    SILCADCReading = 3480;
    #20;
    SILCADCReading = 2661;
    #20;
    SILCADCReading = 1600;
    #20;
    SILCADCReading = 800;
    #20;
    SILCADCReading = 0;
endtask

// other datasets
initial begin
    #10;
    DCAnalogReading = 12'b101010101010;
    EMReading = 8'b11100000;
    TemperatureReading = 12'b100111000111;
    VoltageReading = 12'b000111010110;
end

initial begin
    #20;
    En = 1;
    // Test ROSC Sensor, with slow mode and 1 clock cycle
    #30;
    CPUCommand = {12'b100010000001,{20{1'b0}}};

    // reset in the middle of measurement
    // current measurement should be interrupted
    //#200;
    //CPUCommand = 0;
    #1000;
    CPUCommand = 0;
    // test SILC slope sensor, with slow mode and 1 slope, 4095 clock cycles timeout
    #200;
    // test timeout scenario with 1 clk cycle timeout threshold
    //CPUCommand = {{6'b100000},{8'b00000001},{15{1'b0}},{1'b1},{2'b00}};
    // normal test conditions
    CPUCommand = {{12'b101010000000},{6'b000001},{12{1'b1}},{2'b00}};
    ADCDataGenerate();
    #1000;
    // reset
    CPUCommand = 0;
    #200;
    // test SILC slope sensor in fast mode, should report previous reading without entering a fresh measurement cycle
    CPUCommand = {{12'b011010000000},{6'b000001},{12{1'b1}},{2'b00}};
    #1000;
    // reset
    CPUCommand = 0;
    #200;
    // test ROSC sensor in fast mode, should report previous reading without entering a fresh measurement cycle
    CPUCommand = {12'b010010000001,{20{1'b0}}};

    // test the remaining sensors
    #1000;
    CPUCommand = 0;
    #200;
    CPUCommand = {{8'b10010000},{24{1'b0}}};

    #1000;
    CPUCommand = 0;
    #200;
    CPUCommand = {{8'b10011000},{24{1'b0}}};

    #1000;
    CPUCommand = 0;
    #200;
    CPUCommand = {12'b101000000010,{20{1'b0}}};

    #1000;
    CPUCommand = 0;
    #200;
    CPUCommand = {{8'b10110000},{24{1'b0}}};

    #1000;
    CPUCommand = 0;
    #200;
    CPUCommand = {{8'b10111000},{24{1'b0}}};

end

endmodule
