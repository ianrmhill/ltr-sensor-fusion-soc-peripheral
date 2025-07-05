/************************************************************************************************************
Module: ROSC (V2.0)
Author: Alan Peng
Input: Mode (Stop = 0X00, Fast = 0X01, Slow = 0X02); EN (Enable); SensorReading (Frequency); Clk; NumClkCycles (Number of clock cycles for taking measurement); 
Threshold (threshold counter value below which sensor is deemed degraded); CPUReadComplete (CPU completes reading measurement data)

Output: ROSCReading (16-bit counter value), ErrorCode (0 = Stop; 1 = Idle; 2 = Fast Mode; 3 = Slow Mode), ROSCValReady (Value is ready for CPU)
-------------------------------------------------------------------------------------------------------------
This module employs an embedded counter mechanism to report the frequency of a ring oscillator sensor that degrades over time.
There are 2 measurement modes:
Fast - continuous measurement, the module reports last counter value in memory without additional measurement
Slow - measurement upon request, the module fulfills a measurement cycle and updates the memory with the obtained measurement

It is assumed that the frequency of the sensor is much faster than the clock. 
************************************************************************************************************/
module ROSC(input logic[1:0] Mode, 
            input logic Enable,
            input logic SensorReading,
            input logic Clk,
            input logic[3:0] NumClkCycles,
            input logic CPUReadComplete,

            output logic[15:0] ROSCReading,
            output logic[2:0] ErrorCode,
            output logic ROSCValReady
    );

    // Control signal for triggering the counter block
    logic startmeasure;
    // counter ready flag
    logic countervalready;
    // local register for number of clock cycles
    logic[3:0] numclkcycles;
    // register for storing the measurement
    bit[15:0] recordedvalue;

    always_ff @(posedge Clk)
    begin
        // reset all internal registers that are not controlled by downstream FSM
        if (!Enable) begin
            numclkcycles <= 0;

        end

        else begin
            // only update period of measurement when measurement is not under way
            if (!startmeasure) numclkcycles <= NumClkCycles;
        end
    end

    // FSM for acquiring ROSC data in different modes
    ROSCgetdataFSM ROSCGetData(Clk, Mode, Enable, recordedvalue, countervalready, CPUReadComplete, startmeasure, ROSCReading, ErrorCode, ROSCValReady);

    // Sequential logic resembling the counter 
    counter freqcounter(SensorReading, Clk, numclkcycles, startmeasure, recordedvalue, countervalready);


endmodule


/************************************************************************************************************
Module: getdataFSM (V2.0)
Author: Alan Peng
Input: clk, mode (Stop = 0X00, Fast = 0X01, Slow = 0X02), enable, prevroscreading (ROSC reading in memory from counter), 
       countervalready (counter value ready signal for FSM), cpureadcomplete (CPU completes reading measurement data)
Output: startmeas (start signal for counter), ROSCValue (output), ErrorCode, ROSCValReady (value ready for CPU)
-------------------------------------------------------------------------------------------------------------
This module implements a mealy machine, which interacts with the counter and reports ROSC measurement data depending on the mode.

There are a total of 4 states in the FSM:
S0: Stop mode, the FSM is disabled, when mode input is 0 indicating stop, or !Enable
S1: Idle mode, the FSM is not actively acquiring measurement data. In this state, the ROSC data is ready for CPU
S2: Fast mode, the FSM gets data from the previous recorded value
S3: Slow mode, the FSM triggers the counter to carry out a measurement cycle and reports the value from the counter
************************************************************************************************************/

module ROSCgetdataFSM(input logic clk,
                  input logic[1:0] mode,
                  input logic enable,
                  input logic[15:0] prevroscreading,
                  input logic countervalready,
                  input logic cpureadcomplete,

                  output logic startmeas,
                  output logic[15:0] ROSCValue,
                  output logic[2:0] ErrorCode,
                  output logic ROSCValReady
);

// S0: reset state; S1: idle; S2: fast mode; S3: slow mode, data acquisition in progress
typedef enum logic[1:0] {Reset, DataReady, Fast, Slow} statetype;
statetype state, nextstate;

// State transitions
always_ff @(posedge clk) begin
    if ((mode == 2'b00) || (enable == 0)) begin
        state <= Reset;
    end
    else begin
        state <= nextstate;
    end
end

// Next state logics
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
            // if counter is not yet completed, remain in this state
            if(!countervalready) nextstate = Slow;
            // once counter completed, return to idle state to report reading
            else nextstate = DataReady;
        end
    endcase
    
//output logic
always_comb
    case (state)
        // stop mode, no output would be reported
        Reset: begin
            startmeas = 0;
            ROSCValue = 0;
            ErrorCode = 0;
            ROSCValReady = 0;
        end
        // idle mode, the reading in memory would be reported
        DataReady: begin
            startmeas = 0;
            ROSCValue = prevroscreading;
            ErrorCode = 1;
            ROSCValReady = 1;
        end

        // fast mode, no output would be reported
        Fast: begin
            startmeas = 0;
            ROSCValue = 0;
            ErrorCode = 2;    //check later if this is correct; on datasheet for ROSC, error code is 3'b100
            ROSCValReady = 0;
        end

        // slow mode, no output would be reported
        // the counter would be enabled for data acquisition
        Slow: begin
            startmeas = 1;
            ROSCValue = 0;
            ErrorCode = 3;
            ROSCValReady = 0;
        end
    endcase
    
endmodule


/************************************************************************************************************
Module: counter (V2.0)
Author: Alan Peng
Input: FastClk (ROSC signal), SlowClk (Clock), Period (User defined measurement window in terms of # clock cycles), Enable
Output: CounterValue (Counter Value upon measurement completion), CounterComplete (Flag)
-------------------------------------------------------------------------------------------------------------
This module implements a counter that reports the frequency of the a ROSC sensor by counting the number of rising 
edges of the ROSC signal within a user-defined number of clock cycles. 

It is assumed that the frequency of the ROSC sensor is much faster than the clock. 
************************************************************************************************************/
module counter( input logic FastClk, 
                input logic SlowClk, 
                input logic[3:0] Period, 
                input logic Enable, 
                output logic[15:0] CounterValue,
                output logic CounterComplete);

    // State variable tracking the number of clock cycles incurred
    logic[8:0] NumCycles;
    logic[15:0] countervalue;
    // State transition
    // 0 -- Initialization; 1 - Period -- measurement; Period + 1 -- Completion 
    always_ff @(posedge SlowClk) 
        // State variable needs to reset to 0 after current measurement cycle done, even if !Enable
        if (Enable || (NumCycles == Period + 1)) begin
            if (NumCycles < Period + 1) NumCycles <= NumCycles + 1;
            else NumCycles <= 0;
        end
        else NumCycles <= 0;

    // Counter logic
    always_ff @(posedge FastClk) begin
        if (Enable) begin
            // Within measurement window; counter increments
            if ((NumCycles < Period + 1) && (NumCycles > 0)) begin
                countervalue <= countervalue + 1;
                CounterComplete <= 0;
            end
            // NumCycles == 0; Before measurement starts; counter resets
            else if (NumCycles == 0) begin
                countervalue <= 0;
                CounterComplete <= 0;

            end
            // NumCycles == Period + 1; Measurement cycle completed; reset state variable and update completion flag
            else if (NumCycles == Period + 1) begin
                CounterComplete <= 1;
                CounterValue <= countervalue;
            end
        end

        // If Enable Off, reset all values
        else begin
                countervalue <= 0;
        end
    end

endmodule