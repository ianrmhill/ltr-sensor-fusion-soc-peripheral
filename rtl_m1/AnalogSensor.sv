/************************************************************************************************************
Module: AnalogSensor (V1.0)
Author: Alan Peng
Input: Mode (Stop = 0X00, Fast = 0X01, Slow = 0X02); Enable (Enable); ADCReading (12-bit analog reading); Clk; CPUReadComplete (CPU completes reading measurement data)

Output: AnalogReading (16 bits, last 12-bit is analog reading), ErrorCode (0 = Stop; 1 = Idle; 2 = Fast Mode; 3 = Slow Mode), AnalogValReady (Value is ready for CPU)
-------------------------------------------------------------------------------------------------------------
This module mimics analog wearout sensors including temperature, voltage sensors etc.
There are 2 measurement modes:
Fast - continuous measurement, the module reports last counter value in memory without additional measurement
Slow - measurement upon request, the module fulfills a measurement cycle and updates the memory with the obtained measurement
************************************************************************************************************/
module AnalogSensor(input logic[1:0] Mode, 
                    input logic Enable,
                    input logic[11:0] ADCReading,
                    input logic Clk,
                    input logic CPUReadComplete,

                    output logic[15:0] AnalogReading,
                    output logic[2:0] ErrorCode,
                    output logic AnalogValReady);

// register holding previous measurement
bit[11:0] recordedvalue;
// acquisition start flag from FSM, for acquisition module
logic startmeasure;
// acquisition complete flag from acquisition module, for FSM
logic acquisitionready;

// FSM
AnalogGetDataFSM AnalogGetData(Mode, Enable, Clk, recordedvalue, acquisitionready, CPUReadComplete, startmeasure, AnalogReading, ErrorCode, AnalogValReady);
// Data acquisition module
AcquireAnalogData AnalogSampling(Clk, startmeasure, ADCReading, recordedvalue, acquisitionready);

endmodule

/************************************************************************************************************
Module: AnalogGetDataFSM (V1.0)
Author: Alan Peng
Input: mode (Stop = 0X00, Fast = 0X01, Slow = 0X02); enable; clk; prevanalogreading (Previous reading for fast mode); 
       acquisitionready (Data acquisition completed); cpureadcomplete (CPU completes reading measurement data)

Output: startmeas (start measurement flag for acquisition module); analogreading (measurement result from data acquisition module); 
        errorcode (0 = Stop; 1 = Idle; 2 = Fast Mode; 3 = Slow Mode), analogvalready (Value is ready for CPU)
-------------------------------------------------------------------------------------------------------------
This module implements an FSM that controls the data acquisition workflow for an analog sensor.

There are a total of 4 modes in this FSM.
Reset: Stop mode, the FSM is disabled, when mode input is 0 indicating stop, or !Enable
DataReady: Idle mode, the FSM is not actively acquiring measurement data. In this state, the measurement data is ready for CPU
Fast: Fast mode, the FSM gets data from the previous recorded value
Slow: Slow mode, the FSM triggers the acquisition module to carry out a measurement cycle and reports the value
************************************************************************************************************/
module AnalogGetDataFSM(input logic[1:0] mode,
                        input logic enable,
                        input logic clk,
                        input logic[11:0] prevanalogreading,
                        input logic acquisitionready,
                        input logic cpureadcomplete,

                        output logic startmeas,
                        output logic[15:0] analogreading,
                        output logic[2:0] errorcode,
                        output logic analogvalready
                        );

typedef enum logic[2:0] {Reset, DataReady, Fast, Slow} statetype;
statetype state, nextstate;

// state transitions
always_ff @(posedge clk) begin
    if (!enable || mode == 2'b00) begin
        state <= Reset;
    end
    else begin
        state <= nextstate;
    end
end

// next state logic
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
            analogreading = 0;
            errorcode = 0;
            analogvalready = 0;
        end
        // DataReady mode, the reading in memory would be reported
        DataReady: begin
            startmeas = 0;
            analogreading = {4'b0000, prevanalogreading};
            errorcode = 1;
            analogvalready = 1;
        end

        // fast mode, no output would be reported
        Fast: begin
            startmeas = 0;
            analogreading = 0;
            errorcode = 2;
            analogvalready = 0;
        end

        // slow mode, no output would be reported
        // the startmeas flag would be enabled for data acquisition
        Slow: begin
            startmeas = 1;
            analogreading = 0;
            errorcode = 3;
            analogvalready = 0;
        end
    endcase

endmodule


/************************************************************************************************************
Module: AcquireAnalogData (V1.0)
Author: Alan Peng
Input:  clk; startmeas (start measurement flag for acquisition module); sensorreading (ADC reading); 
Output: recordedvalue (obtained analog reading); acquisitionready (acquisition complete flag for FSM)

-------------------------------------------------------------------------------------------------------------
This module implements an FSM that implements data acquisition and transfer during the slow mode
************************************************************************************************************/
module AcquireAnalogData(input logic clk,
                         input logic startmeasure,
                         input logic[11:0] sensorreading,
                         output logic[11:0] recordedvalue,
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
