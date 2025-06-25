/************************************************************************************************************
Module: AnalogSensor_tb (V1.0)
Author: Alan Peng
Input: Mode (Stop = 0X00, Fast = 0X01, Slow = 0X02); Enable (Enable); ADCReading (12-bit analog reading); Clk; CPUReadComplete (CPU completes reading measurement data)

Output: AnalogReading (last 12-bit analog measurement), ErrorCode (0 = Stop; 1 = Idle; 2 = Fast Mode; 3 = Slow Mode), AnalogValReady (Value is ready for CPU)
-------------------------------------------------------------------------------------------------------------
This module is a testbench for AnalogSensor module. Test coverage:
1. Switching between fast mode and slow mode (✓)
2. Fast / Slow mode measurement (✓)
3. Response to CPUComplete flag (✓)
************************************************************************************************************/
`timescale 1ns/1ps

module AnalogSensor_tb;

logic[1:0] Mode;
logic Enable;
logic[11:0] ADCReading;
logic Clk;
logic CPUReadComplete;

logic[15:0] AnalogReading;
logic[2:0] ErrorCode;
logic AnalogValReady;

AnalogSensor DUT(.*);

// reset all inputs at startup
initial begin
    Mode = 0;
    Enable = 0;
    ADCReading = {12{1'b0}};
    Clk = 0;
    CPUReadComplete = 0;

end

// clock
always begin    
    #10;
    Clk = ~Clk;
end

initial begin
    // generate ADC reading
    // attempt to acquire it in slow mode
    // CPUReadComplete = 0, should remain in DataReady state
    #100;
    ADCReading = 1234;
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
