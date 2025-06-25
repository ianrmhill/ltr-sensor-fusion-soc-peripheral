/************************************************************************************************************
Module: SILC_tb (V2.0)
Author: Alan Peng
Input: Mode (Stop = 0X00, Fast = 0X01, Slow = 0X02); Enable; Clk; ADCReading (12-bits); 
       NumDescendingSlopes (Number of slopes accounted for within each measurement); 
       TimeoutThreshold (measurement times out beyond the threshold); CPUReadComplete (CPU reading completion flag)
Output: SILCReading (16-bit counter value), ErrorCode, SILCValReady (Value is ready for CPU)
-------------------------------------------------------------------------------------------------------------
This module is the testbench for the SILC Module
-------------------------------------------------------------------------------------------------------------
Items to be tested
1. Mode Transition between stop / slow / fast (✓)
2. Behaviour of each individual mode (✓)
3. NumDescendingSlope change (✓)
4. Time out scenario (✓)
************************************************************************************************************/

`timescale 1ns/1ps

module SILC_tb;

logic[1:0] Mode;
logic Enable, Clk;
logic[11:0] ADCReading;
logic[5:0] NumDescendingSlopes;
logic[11:0] TimeoutThreshold;
logic CPUReadComplete;

logic[15:0] SILCReading;
logic[2:0] ErrorCode;
logic SILCValReady;

SILC DUT(.*);

initial begin
    // Start with slow mode, where a measurement cycle will be carried out
    Mode = 2;
    Enable = 0;
    Clk = 0;
    // Measure 2 slopes
    NumDescendingSlopes = 2;
    TimeoutThreshold = 30;
    // Initial ADC Reading at 12'b111111111111
    ADCReading = {12{1'b1}};
end

initial begin
    #15;
    Enable = 1;
    // Leave CPU Reading complete on
    // In this case the CPU should receive and process the measurement instantaneously
    // The module would proceed to the next measurement cycle right after
    CPUReadComplete = 1;
    // Generate 3 descending slope patterns
    ADCDataGenerate();
    // Test stop mode, no output is expected
    #100;
    Mode = 0;
    // Test fast mode
    // Last successful reading would be reported
    #100;
    Mode = 1;

    // Change number of descending slopes
    NumDescendingSlopes = 1;
    // Back To Slow Mode
    #100;
    Mode = 2;
    // Test timeout and resetting
    #1000;
    Mode = 0;
    #100;
    Mode = 2;
    // Generate another set of descending slopes
    // Slower drop than previous set
    ADCDataGenerate2();
    // Disable CPU reading complete flag
    // In this case the IP would stay idle with outputs available for CPU
    CPUReadComplete = 0;


end

always begin
    // System (CPU) clock that altenates every 10 ns (40MHz clock)
    #10;
    Clk = ~Clk;
end

// ADC Data set 1 (Steep slope)
task automatic ADCDataGenerate();
    #30;
    ADCReading = 12'b111110111111;
    #20;
    ADCReading = 12'b100001111111;
    #20;
    ADCReading = 12'b011111111111;
    #20;
    ADCReading = 12'b000001111111;
    #20;
    ADCReading = {12{1'b0}};
    #100;
    ADCReading = 12'b111110111111;
    #20;
    ADCReading = 12'b100001111111;
    #20;
    ADCReading = 12'b011111111111;
    #20;
    ADCReading = 12'b000001111111;
    #20;
    ADCReading = {12{1'b0}};
    #100;
    ADCReading = 12'b111110111111;
    #20;
    ADCReading = 12'b100001111111;
    #20;
    ADCReading = 12'b011111111111;
    #20;
    ADCReading = 12'b000001111111;
    #20;
    ADCReading = {12{1'b0}};

endtask

// ADC Data set 2
// Mild slope
task automatic ADCDataGenerate2();
    #30;
    ADCReading = 12'b111110111111;
    #20;
    ADCReading = 12'b100001111111;
    #100;
    ADCReading = 12'b011111111111;
    #60;
    ADCReading = 12'b000001111111;
    #20;
    ADCReading = {12{1'b0}};
    #100;
    ADCReading = 12'b111110111111;
    #20;
    ADCReading = 12'b100001111111;
    #100;
    ADCReading = 12'b011111111111;
    #20;
    ADCReading = 12'b000001111111;
    #20;
    ADCReading = {12{1'b0}};
    #100;
    ADCReading = 12'b111110111111;
    #20;
    ADCReading = 12'b100001111111;
    #100;
    ADCReading = 12'b011111111111;
    #20;
    ADCReading = 12'b000001111111;
    #20;
    ADCReading = {12{1'b0}};

endtask

endmodule