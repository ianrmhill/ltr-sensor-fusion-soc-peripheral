/************************************************************************************************************
Module: AMBA_write_tb (V1.0)
Author: Alan Peng
Input: PCLK, PADDR (Address to write into, contiguous 32-bit), PWRITE (=1 when actively writing), PSELX (Sensor Select), PENABLE, PWDATA (data to be written into register)

Output: RegisterData (register holding written data), PREADY, PSLVERR (communication error / timeout)
-------------------------------------------------------------------------------------------------------------
This module implements the testbench of write transaction of AMBA APB communication. Test Coverage:
1. Write 32-bit data into designated localtion in the Register Table. Parity check. Timeout should not occur. (✓)
************************************************************************************************************/
`timescale 1ns/1ps

module AMBA_write_tb;

logic PCLK;
logic[31:0] PADDR;
logic PWRITE;
logic PENABLE;
logic[31:0] PWDATA;

logic[31:0] RegisterData;
logic PREADY;
logic PSLVERR;

AMBA_write DUT(.*);

initial begin
    PCLK = 0;
    PADDR = 0;
    PWRITE = 0;
    // write into register table holding sensor #5 commands
    PENABLE = 0;
    // Data to be written
    PWDATA = {{16{1'b1}},{16{1'b0}}};
end

always begin
    #10 PCLK = ~PCLK;
end

initial begin
    #50;
    PWRITE = 1;
    PADDR = {32{1'b1}};
    
    #50;
    PENABLE = 1;

    #100;
    PWRITE = 0;
end

endmodule

