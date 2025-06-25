/************************************************************************************************************
Module: ROSC_tb (V2.0)
Author: Alan Peng
Input: Mode (Stop = 0X00, Fast = 0X01, Slow = 0X02); EN (Enable); SensorReading (Frequency); Clk; NumClkCycles (Number of clock cycles
       for measurement time window); CPUReadComplete (CPU flag indicating reading complete)
Output: ROSCReading (16-bit counter value), ErrorCode, ROSCValReady (Value is ready for CPU)
-------------------------------------------------------------------------------------------------------------
This module is the testbench for the ROSC Module
-------------------------------------------------------------------------------------------------------------
Items to be tested
1. Mode Transition between stop / slow / fast (Y)
2. Behaviour of each individual mode (Y)
3. NumClkCycles change (Y)
************************************************************************************************************/

`timescale 1ns/1ps

module ROSC_tb;

logic[1:0] Mode;
logic Enable, SensorReading, Clk;
logic[3:0] NumClkCycles;
logic CPUReadComplete;

logic[15:0] ROSCReading;
logic[2:0] ErrorCode;
logic ROSCValReady;

ROSC DUT(.*);


initial begin
    // Test Slow Mode in which a measurement cycle will be fulfilled
    Mode = 2'b10;
    Enable = 0;
    {SensorReading, Clk} = 2'b00;
    // Collect measurement for 3 clock cycles
    NumClkCycles = 3;
    CPUReadComplete = 0;

    #20;
    Enable = 1;
    #160;
    // Raise CPU reading complete flag
    // The module should enter another measurement cycle in slow mode
    CPUReadComplete = 1;
    #50;
    // Switch to Fast Mode in which the previous reading is reported
    // The previous slow mode measurement under way should still finish
    Mode = 2'b01;
    // Switch back to Slow Mode to test Mode transition
    #200;
    Mode = 2'b10;
    // Change measurement time window to 4 clock cycles
    #200;
    NumClkCycles = 4;
    // Disable CPU Reading complete flag
    // The module should stay idle with Value Ready flag = 1
    #100;
    CPUReadComplete = 0;
    
end

always begin
    // ROSC Sensor reading that alternates every 1 ns (500MHz sensor reading)
    #1;
    SensorReading = ~SensorReading;
end

always begin
    // System (CPU) clock that altenates every 10 ns (40MHz clock)
    #10;
    Clk = ~Clk;
end

endmodule
