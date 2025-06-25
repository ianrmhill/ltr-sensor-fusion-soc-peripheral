/************************************************************************************************************
Module: AMBA_write (V1.0)
Author: Alan Peng
Input: PCLK, PADDR (Address to write into, contiguous 32-bit), PWRITE (=1 when actively writing), PSELX (Sensor Select), PENABLE, PWDATA (data to be written into register)

Output: RegisterData (register holding written data), PREADY, PSLVERR (communication error / timeout)
-------------------------------------------------------------------------------------------------------------
This module implements the write transaction of AMBA APB communication.
************************************************************************************************************/
module AMBA_write(input logic PCLK,
                  input logic[31:0] PADDR,
                  input logic PWRITE,
                  input logic PENABLE,
                  input logic[31:0] PWDATA,
                  output logic[31:0] RegisterData,
                  output logic PREADY,
                  output logic PSLVERR
);

// local register feeding output
logic[31:0] localreg;
// timeout counter
logic[4:0] timeoutcounter;
// timeout threshold
static logic[4:0] timeoutthreshold = 10;

// 4 states
// IDLE, Setup, Access, Complete
typedef enum logic[1:0] {IDLE, SETUP, ACCESS, COMPLETE} statetype;
statetype state, nextstate;

// state transitions
always_ff @(posedge PCLK) begin
    // remain in idle state if signals are not ready
    if (PADDR == {32{1'b0}} || PWRITE == 0) begin
        state <= IDLE;
    end
    else begin
        state <= nextstate;
    end
end

//next state logic
always_comb
    case (state)
        IDLE: nextstate = SETUP;
        // Enter ACCESS when PENABLE is true
        SETUP: begin
            if (PENABLE) nextstate = ACCESS;
            else nextstate = SETUP;
        end

        // check to make sure correct data is written
        // timeout is triggered eventually
        ACCESS: begin
            if (timeoutcounter < timeoutthreshold) begin
                if (localreg != PWDATA)  nextstate = ACCESS;
                else nextstate = COMPLETE;
            end

            else begin
                nextstate = COMPLETE;
            end

        end

        // wait for control signal reset
        COMPLETE: begin
            nextstate = COMPLETE;
        end
    endcase

// output logic
always_comb begin
    case (state)
        IDLE: begin
            PREADY = 0;
            RegisterData = 0;
            PSLVERR = 0;
        end

        SETUP: begin
            PREADY = 0;
            RegisterData = 0;
            PSLVERR = 0;
        end

        ACCESS: begin
            PREADY = 0;
            RegisterData = 0;
            PSLVERR = 0;
        end

        COMPLETE: begin
            PREADY = 1;
            // report to global register table if not timeout
            if (timeoutcounter < timeoutthreshold) begin
                RegisterData = localreg;
                PSLVERR = 0;
            end
            // if time out
            // do not update global register table
            // raise flag for CPU
            else begin
                RegisterData = 0;
                PSLVERR = 1;
            end
        end

    endcase
end

always_ff @(posedge PCLK) begin
    // reset local reg when idle
    if (state == IDLE) begin
        localreg <= 0;
        timeoutcounter <= 0;
    end

    // write to register during access phase
    // update local register with PWDATA during access phase
    // update timeout tracker to record time spent in access phase
    if (state == ACCESS) begin
        localreg <= PWDATA;
        timeoutcounter <= timeoutcounter + 1;
    end

end

endmodule
