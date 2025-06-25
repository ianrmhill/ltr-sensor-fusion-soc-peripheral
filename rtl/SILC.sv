/************************************************************************************************************
Module: SILC (V2.0)
Author: Alan Peng
Input: mode (Stop = 0X00, Fast = 0X01, Slow = 0X02), enable, ADCReading, Clk, 
       NumDescendingSlopes (Number of slopes to be collected), TimeoutThreshold (Time out condition in terms of # clk cycles),
       CPUReadComplete (CPU completes reading measurement data)
Output: SILCReading, ErrorCode (Reset = 0, DataReady = 1, Acquisition fast mode = 2, Acquisition slow mode = 3, Timeout = 7)
        ROSCValReady (value ready for CPU)
-------------------------------------------------------------------------------------------------------------
The module is a data collector and parser for SILC slope based sensor. In the slow mode (mode = 2), it assumes that no recent 
data is found and starts a measurement cycle through the FSMs SILCGetData (which is a bridge between the CPU and the counter), and
SILCGetSlope (which is a counter which counts the elapsed time over N descending slopes). As the sensor degrades, the slope becomes
steeper, so expect the reading from this module to decrease. In the fast mode (mode = 1), it assumes that the previous measurement
is fresh, thus it would not initiate a new measurement cycle. 

To reduce margin of error, the CPU has the option to specify the number of descending slopes to measure, and the module would report the 
aggregrate clk cycle count over the N slopes. The CPU would also specify a timeout threshold in terms of clock cycles - if measurement
is not completed at timeout threshold, the module would report the most recent reading with an errorcode = 7 to signal current cycle time
out.

Once data acquitision is complete, the SILCValReady would become high. CPU should check the error code in this scenario. Errorcode = 1
indicates the current measurement cycle is successful, while Errorcode = 7 indicates timeout for the current cycle. 
************************************************************************************************************/
module SILC(input logic[1:0] Mode, 
            input logic Enable,
            input logic[11:0] ADCReading,
            input logic Clk,
            input logic[5:0] NumDescendingSlopes,
            input logic[11:0] TimeoutThreshold,
            input logic CPUReadComplete,

            output logic[15:0] SILCReading,
            output logic[2:0] ErrorCode,
            output logic SILCValReady
);

// local registers for input data
logic[5:0] numdescendingslopes;
logic[11:0] timeoutthreshold;

// local register for measurement data
bit[15:0] recordedvalue;
logic[15:0] countervalue;

// flags
// start measurement flag
logic startmeasure;
// slope measurement ready flag
logic slopevalready;
// measurement timeout flag
logic timeout;

always_ff @(posedge Clk)
begin
    // reset all registers that are not controlled by downstream FSM if module not enabled
    if (!Enable) begin
        numdescendingslopes <= 0;
        timeoutthreshold <= 0;
    end

    else begin
        // only update period of measurement / timeout threshold when measurement is not under way
        if (!startmeasure) begin
            numdescendingslopes <= NumDescendingSlopes;
            timeoutthreshold <= TimeoutThreshold;
        end
        // only update recorded value when the counter completes a measurement cycle and no timeout reported
        // otherwise, recordedvalue would retain the previous successful measurement
        if (slopevalready && !timeout) begin
            recordedvalue <= countervalue;
        end
    end
end

SILCgetdataFSM SILCGetData(Clk, Mode, Enable, recordedvalue, slopevalready, timeout, CPUReadComplete, startmeasure, SILCReading, ErrorCode, SILCValReady);

GetSlopeFSM SILCMeasureSlope(startmeasure, ADCReading, Clk, numdescendingslopes, timeoutthreshold, countervalue, slopevalready, timeout);

endmodule

/************************************************************************************************************
Module: SILCgetdataFSM (V2.0)
Author: Alan Peng
Input: clk, mode (Stop = 0X00, Fast = 0X01, Slow = 0X02), enable, prevSILCreading (SILC reading in memory), 
       slopevalready (counter value ready signal from counter), timeout (time out signal from counter),
       cpureadcomplete (CPU completes reading measurement data for current cycle)
Output: startmeas (start signal for counter), SILCValue (output), ErrorCode (Same as module SILC), ROSCValReady (value ready for CPU)
-------------------------------------------------------------------------------------------------------------
This module implements a mealy machine, which interacts with the counter and reports SILC measurement data depending on the mode.

There are a total of 5 states in the FSM:
S0: Stop mode, the FSM is disabled.
S1: Idle mode, the FSM is not actively acquiring measurement data. In this state, the ROSC data is ready for CPU
S2: Timeout mode: the FSM indicates recent measurement timeout. It reports the previous successful reading with a timeout error code = 7.
S3: Fast mode, the FSM gets data from the previous recorded value
S4: Slow mode, the FSM triggers the counter to carry out a measurement cycle and reports the value from the counter
************************************************************************************************************/
module SILCgetdataFSM(input logic clk,
                  input logic[1:0] mode,
                  input logic enable,
                  input logic[15:0] prevSILCreading,
                  input logic slopevalready,
                  input logic timeout,
                  input logic cpureadcomplete,

                  output logic startmeas,
                  output logic[15:0] SILCvalue,
                  output logic[2:0] errorcode,
                  output logic SILCvalready
);

typedef enum logic[2:0] {Reset, DataReady, Timeout, Fast, Slow} statetype;
statetype state, nextstate;

// State transitions
always_ff @(posedge clk) begin
    // reset conditions
    // Stop mode or disabled
    if ((mode == 2'b00) || (!enable)) begin
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
            // fast mode, proceed to State Dataready
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

        // stay in timeout mode unless reset
        Timeout: begin
            nextstate = Timeout;
        end

        Fast: begin
            // In the fast mode, no data acquisition is required, go back to idle state to report previous reading
            nextstate = DataReady;
        end

        Slow: begin
            // if counter is not yet completed, remain in this state
            if(!slopevalready) nextstate = Slow;
            // counter completed
            else begin
                // if time out flag raised, move to Timeout state
                if (!timeout) nextstate = DataReady;
                else nextstate = Timeout;
            end
        end
    endcase
    
//output logic
always_comb
    case (state)
        // stop mode, no output would be reported
        Reset: begin
            startmeas = 0;
            SILCvalue = 0;
            errorcode = 0;
            SILCvalready = 0;
        end
        // dataready mode, the reading in memory would be reported
        // prevSILCreading would be updated with the latest counter value and reported
        // value_ready flag would be raised
        DataReady: begin
            startmeas = 0;
            SILCvalue = prevSILCreading;
            SILCvalready = 1;
            errorcode = 1;
        end
        // time out mode
        // the counter times out and would not update prevSILCreading 
        // the most recent successful reading in memory would be reported instead
        // error code = 7 indicating time out
        Timeout: begin
            startmeas = 0;
            SILCvalue = prevSILCreading;
            SILCvalready = 1;
            errorcode = 7;
        end

        // fast mode, no output would be reported
        Fast: begin
            startmeas = 0;
            SILCvalue = 0;
            errorcode = 2;
            SILCvalready = 0;
        end

        // slow mode, no output would be reported
        // the counter would be enabled for data acquisition
        Slow: begin
            startmeas = 1;
            SILCvalue = 0;
            SILCvalready = 0;
            errorcode = 3;
        end

        default: begin
            startmeas = 0;
            SILCvalue = 0;
            errorcode = 0;
            SILCvalready = 0;
        end
    endcase
    
endmodule


/************************************************************************************************************
Module: GetSlopeFSM (V2.0)
Author: Alan Peng
Input: Enable, ADCVal (ADC reading), Clock, NumDescendingSlopes (from CPU), TimeoutThreshold (from CPU)
Output: Count (counter value), ReadingComplete, Timeout (current measurement times out)
-------------------------------------------------------------------------------------------------------------
This module is a sub-module of GetDataFSM and implements a FSM that conducts a measurement cycle upon Slow Mode request from upperstream.
It reports a counter value, in terms of the number of clock cycles over N NumDescendingSLopes, if completed within TimeoutThreshold.
If measurement times out, it would not report a measurement data. Rather, it raises a time out flag for GetDataFSM to handle.

There are a total of 4 states in the FSM:
S1: Idle mode, the FSM is disbaled. No measurement is undertaken.
S2: Acquisition mode, the current measurement cycle is active.
S3: Completion mode, the current measurement cycle is completed. Data is reported to GetDataFSM.
S4: Timeouterror mode, the current measurement cycle times out. No data is reported. Timeout flag raised.
************************************************************************************************************/
module GetSlopeFSM( input logic Enable,
                    input logic[11:0] ADCVal,
                    input logic Clock,
                    input logic[5:0] NumDescendingSlopes,
                    input logic[11:0] TimeoutThreshold,

                    output logic[15:0] Count,
                    output logic ReadingComplete,
                    output logic TimeOut
);


typedef enum logic[1:0] {Idle, Acquisition, Completion, Timeouterror} statetype;
statetype state, nextstate;

// local registers

// enable flag for MeasureSlope counter
logic countactive;
// flag for holding measurement data in register
// previous measurement would be erased if flag turns low
logic holddata;

// clock cycle counter 
logic[5:0] elapsednumdescendingslopes;
// slope counter (against NumDescendingSlopes)
logic[15:0] elapsedclkcycles;
// elapsed time tracker (against TimeoutThreshold)
logic[11:0] elapsedtime;

// state transitions
always_ff @(posedge Clock) begin
    if (!Enable) state <= Idle;
    else state <= nextstate;
end

// next state logic
always_comb 
    case (state)
        Idle: begin
            nextstate = Acquisition;
        end

        Acquisition: begin
            if (elapsedtime < TimeoutThreshold) begin
                if (elapsednumdescendingslopes < NumDescendingSlopes) begin
                    nextstate = Acquisition;
                end
                // if the required number of descending slopes is covered, go to completion
                else begin
                    nextstate = Completion;
                end
            end
            // if time reaches timeout threshold, go to Timeouterror state
            else begin
                nextstate = Timeouterror;
            end
        end

        // Remain in completion before reset to Idle
        Completion: begin
            nextstate = Completion;
        end

        // Remain in Timeouterror before reset to Idle
        Timeouterror: begin
            nextstate = Timeouterror;
        end
    endcase

// output logic
always_comb
    case (state)
        // Idle mode, all output is reset
        Idle: begin
            Count = 0;
            ReadingComplete = 0;
            TimeOut = 0;
            countactive = 0;
            holddata = 0;
        end
        // Acquisition mode, enable the counter, hold the measurement data while the counter increments
        Acquisition: begin
            Count = 0;
            ReadingComplete = 0;
            TimeOut = 0;
            countactive = 1;
            holddata = 1;
        end
        // Completion mode, report counter reading, raise reading completion flag for CPU
        Completion: begin
            Count = elapsedclkcycles;
            ReadingComplete = 1;
            TimeOut = 0;
            countactive = 0;
            holddata = 1;
        end
        // Timeouterror mode, raise timeout flag, do not report a counter reading as it is not valid
        Timeouterror: begin
            Count = 0;
            ReadingComplete = 1;
            TimeOut = 1;
            countactive = 0;
            holddata = 0;
        end

        default: begin
            Count = 0;
            ReadingComplete = 0;
            TimeOut = 0;
            countactive = 0;
            holddata = 0;
        end
    endcase

SlopeCounter MeasureSlope(countactive, holddata, ADCVal, Clock, elapsedclkcycles, elapsednumdescendingslopes, elapsedtime);

endmodule

/************************************************************************************************************
Module: SlopeCounter (V2.0)
Author: Alan Peng
Input: Enable, HoldData (hold counter value), ADCVal (ADC reading), Clock
Output: ClkCycleCount (counter measurement), SlopeCount (elapsed number of descending slopes), ElapsedTime (for current measurement)
-------------------------------------------------------------------------------------------------------------
This module is a counter which, when enabled, counts the number of clock cycles during N descending slopes. It also keeps track of
the number of slopes already measured, and the elapsed time in current measurement. 
************************************************************************************************************/
module SlopeCounter(input logic Enable,
                    input logic HoldData,
                    input logic[11:0] ADCVal,
                    input logic Clock,

                    output logic[15:0] ClkCycleCount,
                    output logic[5:0] SlopeCount,
                    output logic[11:0] ElapsedTime
);

// Upper and Lower Threshold at 75% and 25% of full ADC Range
static logic[11:0] upperthreshold = ({12{1'b1}} >> 2) + ({12{1'b1}} >> 1);
static logic[11:0] lowerthreshold = ({12{1'b1}} >> 2);

// store the previous ADC value
// this will be used to capture start / stop condition
logic[11:0] prevadcval;
// local adc value register
logic[11:0] adcval;

// local registers for the output
logic[15:0] clkcyclecount;
logic[5:0] slopecount;
logic[11:0] elapsedtime;

// flag indicating descending slope detected with ADC measurement within thresholds
// when raised, counter would increment at every rising edge of clock
logic datainrange;

// port register to output wires
always_comb begin
    ClkCycleCount = clkcyclecount;
    SlopeCount = slopecount;
    ElapsedTime = elapsedtime;
end

always_ff @(posedge Clock) begin
    if (Enable) begin
        // time tracker
        elapsedtime <= elapsedtime + 1;
        // update current and previous ADC readings
        prevadcval <= adcval;
        adcval <= ADCVal;
        // start conditions: current ADC reading below upperthreshold && prev ADC reading above upperthreshold
        if ((upperthreshold > adcval) && (prevadcval >= upperthreshold)) begin
            datainrange <= 1;
        end
        // end condition (for current slope): current ADC reading below lowerthreshold && prev ADC reading above lowerthreshold
        else if ((lowerthreshold > adcval) && (prevadcval >= lowerthreshold)) begin
            if (datainrange) begin
                // reset flag
                datainrange <= 0;
                // increment slope counter, only when datainrange flag on
                // otherwise, there might be a scenario where measurement starts halfway of a descending slope
                // slope count should not increment in this case       
                slopecount <= slopecount + 1;
            end
        end

        // increment clock cycle counter if datainrange flag raised
        if (datainrange) clkcyclecount <= clkcyclecount + 1;
    end

    else begin
        // if GetSlopeFSM no longer requires data to be held, erase all
        if (!HoldData) begin
        adcval <= 0;
        prevadcval <= 0;
        clkcyclecount <= 0;
        slopecount <= 0;
        elapsedtime <= 0;
        datainrange <= 0;
        end
    end
end

endmodule




