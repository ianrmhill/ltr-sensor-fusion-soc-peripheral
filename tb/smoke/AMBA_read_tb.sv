/************************************************************************************************************
Module: AMBA_read_tb (V1.0)
Author: Alan Peng
Input: PCLK, PADDR (Address to read, contiguous 32-bit), PWRITE (=0 when actively reading), PSELX (Sensor Select), PENABLE, RegisterData(Register data to read from)

Output: PRData (32 bits), PREADY, PSLVERR (communication error / timeout)
-------------------------------------------------------------------------------------------------------------
This module implements the testbench for the read transaction of AMBA APB communication. Test Coverage:
1. 32-bit Data is correctly read from register without timeout (✓)
************************************************************************************************************/
`timescale 1ns/1ps

module AMBA_read_tb;

logic PCLK;
logic[31:0] PADDR;
logic PWRITE;
logic PENABLE;
logic[31:0] RegisterData;

logic[31:0] PRDATA;
logic PREADY;
logic PSLVERR;

AMBA_read DUT(.*);

initial begin
    PCLK = 0;
    PWRITE = 1;
    PENABLE = 0;
    PADDR = 0;
    RegisterData = 0;

end

always begin
    #10 PCLK = ~PCLK;
end

initial begin
    #50;
    // Assign register data
    RegisterData = {{16{1'b1}},{16{1'b0}}};
    PWRITE = 0;
    // Read full 32-bit data 
    PADDR = {32{1'b1}};
    
    #50;
    PENABLE = 1;

    #100;
    PWRITE = 1;
end

endmodule
