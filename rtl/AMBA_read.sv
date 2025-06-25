/************************************************************************************************************
Module: AMBA_read (V1.0)
Author: Alan Peng
Input: PCLK, PADDR (Address to read, contiguous 32-bit), PWRITE (=0 when actively reading), PSELX (Sensor Select), PENABLE, RegisterData(Register data to read from)

Output: PRData (32 bits), PREADY, PSLVERR (communication error / timeout)
-------------------------------------------------------------------------------------------------------------
This module implements the read transaction of AMBA APB communication.
************************************************************************************************************/
module AMBA_read(input logic PCLK,
                  input logic[31:0] PADDR,
                  input logic PWRITE,
                  input logic PENABLE,
                  input logic[31:0] RegisterData,
                  output logic[31:0] PRDATA,
                  output logic PREADY,
                  output logic PSLVERR
);

// local register holding data read from register
logic[31:0] readdata;
// timeout counter
logic[4:0] timeoutcounter;
// timeout threshold in terms of clock cycles
static logic[4:0] timeoutthreshold = 10;

// 4 states
// IDLE, Setup, Access, Complete
typedef enum logic[1:0] {IDLE, SETUP, ACCESS, COMPLETE} statetype;
statetype state, nextstate;

// state transitions
always_ff @ (posedge PCLK) begin
    if (PADDR == {32{1'b0}} || PWRITE == 1) begin
        state <= IDLE;
    end
    else begin
        state <= nextstate;
    end  
end

// next state logic
always_comb
    case (state)
        IDLE: nextstate = SETUP;
        // Enter ACCESS when PENABLE is true
        SETUP: begin
            if (PENABLE) nextstate = ACCESS;
            else nextstate = SETUP;
        end
        // Enter Complete Either data is successfully retrieved, OR timeout limit reached
        ACCESS: begin
            if (timeoutcounter < timeoutthreshold) begin
                if (readdata != RegisterData) begin
                    nextstate = ACCESS;
                end
                else nextstate = COMPLETE;
            end
            else begin
                nextstate = COMPLETE;
            end

        end
        // Remain in complete state where output is ready for CPU. Wait for reset
        COMPLETE: begin
            nextstate = COMPLETE;
        end

    endcase

// output logic
always_comb 
    case (state)
        IDLE: begin
            PREADY = 0;
            PRDATA = 0;
            PSLVERR = 0;
        end

        SETUP: begin
            PREADY = 0;
            PRDATA = 0;
            PSLVERR = 0;
        end

        ACCESS: begin
            PREADY = 0;
            PRDATA = 0;
            PSLVERR = 0;
        end

        COMPLETE: begin
            PREADY = 1;
            if (timeoutcounter < timeoutthreshold) begin
                PRDATA = readdata;
                PSLVERR = 0;
            end
            // if time out
            // do not update global register table
            // raise flag for CPU
            else begin
                PRDATA = 0;
                PSLVERR = 1;
            end
        end
    endcase

// reset local registers during IDLE state
always_ff @(posedge PCLK) begin
    if (state == IDLE) begin
        readdata <= 0;
        timeoutcounter <= 0;
    end

    // access register data during ACCESS state
    // update timeout tracker during ACCESS state
    if (state == ACCESS) begin
        readdata <= RegisterData;
        timeoutcounter <=  timeoutcounter + 1;
    end
end

endmodule