/************************************************************************************************************
Module: EM_tb (V1.0)
Author: Alan Peng
Input: Mode (Stop = 0X00, Fast = 0X01, Slow = 0X02); Enable (Enable); EMReading (8-bit binary reading from 8 EM sensors); Clk; CPUReadComplete (CPU completes reading measurement data)

Output: EMResult (last 8-bit EM measurement), ErrorCode (0 = Stop; 1 = Idle; 2 = Fast Mode; 3 = Slow Mode), AnalogValReady (Value is ready for CPU)
-------------------------------------------------------------------------------------------------------------
This module is a testbench for AnalogSensor module. Test coverage:
1. Switching between fast mode and slow mode (✓)
2. Fast / Slow mode measurement (✓)
3. Response to CPUComplete flag (✓)
************************************************************************************************************/
`timescale 1ns/1ps

module EM_tb;

logic[1:0] Mode;
logic Enable;
logic[7:0] EMReading;
logic Clk;
logic CPUReadComplete;

logic[15:0] EMResult;
logic[2:0] ErrorCode;
logic EMValReady;

EM DUT(.*);

// reset all inputs at startup
initial begin
    Mode = 0;
    Enable = 0;
    EMReading = {8{1'b0}};
    Clk = 0;
    CPUReadComplete = 0;

end

// clock
always begin    
    #10;
    Clk = ~Clk;
end

initial begin
    // generate EM reading
    // attempt to acquire it in slow mode
    // CPUReadComplete = 0, should remain in DataReady state
    #100;
    EMReading = 8'b01101011;
    Enable = 1;
    Mode = 2;

    // switch to fast mode
    #100;
    Mode = 1;

    // CPU Ready
    #30;
    CPUReadComplete = 1;

    // switch back to slow mode
    // measurement cycles should iterate since CPUReadComplete = 1
    #100;
    Mode = 2;

    // CPU Ready signal off
    // should remain in DataReady signal waiting for CPU
    #100;
    CPUReadComplete = 0;

end

endmodule

