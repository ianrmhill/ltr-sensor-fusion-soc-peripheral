/************************************************************************************************************
Module: EM (V1.0)
Author: Alan Peng
Input: Mode (Stop = 0X00, Fast = 0X01, Slow = 0X02); Enable (Enable); EMReading (8-bit reading represent an array of 8 EM sensors); Clk; CPUReadComplete (CPU completes reading measurement data)

Output: EM Result (16 bits, last 8-bit is EM sensor reading), ErrorCode (0 = Stop; 1 = Idle; 2 = Fast Mode; 3 = Slow Mode), EMValReady (Value is ready for CPU)
-------------------------------------------------------------------------------------------------------------
This module mimics an array of 8 EM sensors, each would output a binary reading of 0/1
There are 2 measurement modes:
Fast - continuous measurement, the module reports last counter value in memory without additional measurement
Slow - measurement upon request, the module fulfills a measurement cycle and updates the memory with the obtained measurement
************************************************************************************************************/
module EM(input logic[1:0] Mode,
          input logic Enable,
          input logic[7:0] EMReading,
          input logic Clk,
          input logic CPUReadComplete,

          output logic[15:0] EMResult,
          output logic[2:0] ErrorCode,
          output logic EMValReady
);

// register holding previous successful measurement
logic[7:0] prevEMreading;
// start measurement flag for EM data acquisition module
logic startmeasure;
// data acquisition completion flag for FSM
logic acquisitionready;

GetEMReadingFSM GetEMReading(Mode, Enable, Clk, prevEMreading, acquisitionready, CPUReadComplete, startmeasure, EMResult, ErrorCode, EMValReady);

AcquireEMData EMSampling(Clk, startmeasure, EMReading, prevEMreading, acquisitionready);

endmodule

/************************************************************************************************************
Module: GetEMReadingFSM (V1.0)
Author: Alan Peng
Input: mode (Stop = 0X00, Fast = 0X01, Slow = 0X02); enable; clk; prevEMreading (Previous reading for fast mode); 
       acquisitionready (Data acquisition completed); cpureadcomplete (CPU completes reading measurement data)

Output: startmeas (start measurement flag for acquisition module); emresult (measurement result from data acquisition module); 
        errorcode (0 = Stop; 1 = Idle; 2 = Fast Mode; 3 = Slow Mode), emvalready (Value is ready for CPU)
-------------------------------------------------------------------------------------------------------------
This module implements an FSM that controls the data acquisition workflow for an array of 8 EM sensors.

There are a total of 4 modes in this FSM.
Reset: Stop mode, the FSM is disabled, when mode input is 0 indicating stop, or !Enable
DataReady: Idle mode, the FSM is not actively acquiring measurement data. In this state, the measurement data is ready for CPU
Fast: Fast mode, the FSM gets data from the previous recorded value
Slow: Slow mode, the FSM triggers the acquisition module to carry out a measurement cycle and reports the value
************************************************************************************************************/
module GetEMReadingFSM(input logic[1:0] mode,
                       input logic enable,
                       input logic clk,
                       input logic[7:0] prevEMreading,
                       input logic acquisitionready,
                       input logic cpureadcomplete,

                       output logic startmeas,
                       output logic[15:0] emresult,
                       output logic[2:0] errorcode,
                       output logic emvalready);

typedef enum logic[1:0] {Reset, DataReady, Fast, Slow} statetype;
statetype state, nextstate;

// state transitions
always_ff @ (posedge clk) begin
    // remain in reset state if no start command received
    if (!enable || mode == 2'b00) state <= Reset;
    else state <= nextstate;

end

// next state logics
always_comb
case (state)
    Reset: begin
        // fast mode, proceed to State DataReady right away
        if (mode == 2'b01) nextstate = DataReady;
        // slow mode, proceed to State Slow
        else if (mode == 2'b11) nextstate = Reset;
        else nextstate = Slow;
    end

    // stay in ready state unless reset
    DataReady: begin
        // wait for CPU read data completion
        if (!cpureadcomplete) nextstate = DataReady;
        // completion, move to Dataready / Slow based on mode selected 
        else begin
            if (mode == 2'b01) nextstate = DataReady;
            else if (mode == 2'b11) nextstate = Reset;
            else nextstate = Slow;
        end
    end

    Fast: begin
        // In the fast mode, no data acquisition is required, go back to idle state to report previous reading
        nextstate = DataReady;
    end

    Slow: begin
        // if acquisition is not yet completed, remain in this state
        if(!acquisitionready) nextstate = Slow;
        // once acquisition completed, return to DataReady state to report reading
        else nextstate = DataReady;
    end

endcase

// output logic
always_comb

case (state)
    Reset: begin
        startmeas = 0;
        emresult = 0;
        errorcode = 0;
        emvalready = 0;
    end
    // DataReady mode, the reading in memory would be reported
    DataReady: begin
        startmeas = 0;
        emresult = {8'b00000000, prevEMreading};
        errorcode = 1;
        emvalready = 1;
    end

    // fast mode, no output would be reported
    Fast: begin
        startmeas = 0;
        emresult = 0;
        errorcode = 2;
        emvalready = 0;
    end

    // slow mode, no output would be reported
    // the startmeas flag would be enabled for data acquisition
    Slow: begin
        startmeas = 1;
        emresult = 0;
        errorcode = 3;
        emvalready = 0;
    end

endcase

endmodule

/************************************************************************************************************
Module: AcquireEMData (V1.0)
Author: Alan Peng
Input:  clk; startmeas (start measurement flag for acquisition module); sensorreading (binary reading from 8 EM sensors); 
Output: recordedvalue (obtained analog reading); acquisitionready (acquisition complete flag for FSM)

-------------------------------------------------------------------------------------------------------------
This module implements an FSM that implements data acquisition and transfer during the slow mode
************************************************************************************************************/
module AcquireEMData(input logic clk,
                     input logic startmeasure,
                     input logic[7:0] sensorreading,
                     output logic[7:0] recordedvalue,
                     output logic acquisitionready);

    always_ff @(posedge clk) begin
        // acquire reading and transfer to register when startmeasure flag raised
        // raise acuqisition complete flag
        if (startmeasure) begin
            recordedvalue <= sensorreading;
            acquisitionready <= 1;
        end

        else begin
            acquisitionready <= 0;
        end
    end
endmodule