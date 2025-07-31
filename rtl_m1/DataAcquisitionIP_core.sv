/*************************************************UPDATE 2025******************************************/
/************************************************************************************************************

/**
 * Module: DataAcquisitionIP_core (V2.0)
 * Author: Raul Vazquez Guerrero (2025), based on Alan Peng's work (2024) (see first commit on this repo)
 * 
 * This module implements the HDL logics of a data acquisition IP for on-chip wear-out sensors. It supports all sensor types developed
 * at the Ivanov SoC Lab and up to 8 sensors of each type. Communication between the CPU and the sensor is implemented using the
 * AMBA APB protocol. 
 *
 * Inputs:
 *   Clk              – global clock for measurement FSM
 *   En               – global enable (active-high)
 *   CPUCommand[31:0] – 32-bit command word from APB wrapper
 *   STATUS_CLEAR     – write-1-to-clear latched status bits
 *   SensorReadings   – 8 × 16-bit raw sensor inputs
 *
 * Outputs:
 *   ResultForCPU[31:0] – {value[31:16], ERR[15:13], REF_TAG[12:10], reserved[9:0]}
 *   StatusBits[2:0]    – {busy, err_sticky, done_latched}
 *
 * Register map (APB offsets):
 *   0x00 = CPUCommandsReg
 *   0x04 = STATUS         // {busy, err_sticky, done_latched}
 *   0x08 = RESULT         // measurement result
 *   0x0C = STATUS_CLEAR   // write-1-to-clear
 *
 * Conditional parsing of CPUCommand fields based on PSELx:
 *   if PSELx == 3'b001 (ROSC) or 3'b100 (TDDB):
 *     numclkcycles       = CPUCommand[23:20]
 *     numdescendingslopes= 0
 *     timeoutthreshold   = 0
 *
 *   else if PSELx == 3'b101 (SILC):
 *     numclkcycles       = 0
 *     numdescendingslopes= CPUCommand[23:18]
 *     timeoutthreshold   = { CPUCommand[15:8], CPUCommand[7:4] }
 *
 *   else (DC, EM, Temperature, Voltage):
 *     numclkcycles       = 0
 *     numdescendingslopes= 0
 *     timeoutthreshold   = 0
 *
 *
 NOTES:
 *   1. The V1.0 DataAcquisitionIP was refactored into a pure DataAcquisitionIP_core by pulling all AMBA/APB bus logic
 *   into a separate wrapper leaving the core to focus solely on sensor FSM control and data flow.
 *   2. We added a 3-bit read-only STATUS register (busy/err_sticky/done) so the CPU can efficiently poll just the
 *   measurement state instead of reading the full 32 bit result word.
 *   3. done_latched and err_sticky are cleared by STATUS_CLEAR.
 *   4. busy is combinational.
 *
 *
 *
 * New in V2.1
 * -----
 *   1. Mode[31:30] repurposed as {REF_SEL, speed}:
 *       00 Slow-RAW        01 Slow-RAW-REF1
 *       10 Slow-RAW-REF2   11 FAST (return last slow result)
 *   2. CMD3[3:0] “Reference Configuration” nibble:
 *       0000 default {reserved}
 *       0001 Print REF1       0010 Overwrite REF1
 *       0011 Print REF2       0100 Overwrite REF2
 *       0101 Overwrite both   others reserved
 *   3. Per-sensor memories added: REF1 (non-volatile), REF2 (volatile),
 *     valid bits, and last_ref_tag.
 *   4. Result[12:10] carries REF_TAG so firmware knows source of FAST data.
 *   5. Expanded ERR codes. (see new_commands_definitions.txt)
 */

module DataAcquisitionIP_core(
  input  logic              Clk,
  input  logic              En,
  input  logic   [31:0]     CPUCommand,
  input  logic              STATUS_CLEAR,
  input  logic  [7:0][15:0] SensorReadings,
  output logic   [31:0]     ResultForCPU,
  output logic    [2:0]     StatusBits
);

  //–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
  // Internal status signals
  logic busy;           // high while measurement in progress
      //TO BE REMOVEDD
      //logic done;  {now im driving this dignal on ff}
  logic done_latched;   // pulses high on completion until STATUS_CLEAR
  logic err_sticky;     // latches any non-001 error until STATUS_CLEAR
  //–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

  // NEW
  //–––––––––––––––––––––––––––––––––––––––––
  // Refernece memories
  //array of 8 types of sensors (in reality 7 but 000 doesnt count on PSEL) * 8 instances of 16-bit each
    logic [15:0] ref1_mem [7:0][7:0];  //how im modeling non-volatile
    logic [15:0] ref2_mem [7:0][7:0];

    logic        ref1_v   [7:0][7:0]; //modeled non-volatile
    logic        ref2_v   [7:0][7:0];
    logic [1:0]  last_ref_tag [7:0][7:0];  // 00/01/10

    // POR (power up) detector
    logic por_seen;
    initial por_seen = 1'b0;
    always_ff @(posedge Clk or negedge En)
      if (!En) por_seen <= 1'b0;
      else     por_seen <= 1'b1;

  //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

  // Capture the command word when entering BUSY
  logic [31:0] cpucommandforparsing;

  // Decoded fields from the captured command
  logic [1:0]  mode; // now differen in V2.1
  logic [2:0]  PSELx;
  logic [2:0]  SensorIndex;
  logic [3:0]  numclkcycles;
  logic [5:0]  numdescendingslopes;
  logic [11:0] timeoutthreshold;
  logic [3:0]  ref_cfg; //NEW addition ; // references configuration

            //TO BE REMOVEDD
            // Direct “CPU read complete” bit from the live input
            //logic cpureadinput;

  // Sensor inputs, extracted from SensorReadings
  logic         ROSCReading;
  logic [11:0]  DCAnalogReading;
  logic [7:0]   EMReading;
  logic         TDDBReading;
  logic [11:0]  SILCADCReading;
  logic [11:0]  TemperatureReading;
  logic [11:0]  VoltageReading;

  // Per-sensor outputs
  logic [7:0][15:0] result;
  logic [7:0][2:0]  errorcode;
  logic [7:0]       sensorvalready;

  // Latched 32-bit measurement word
  logic [31:0] sensormeasdata;

  //–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
  // 1) Parse the command fields (conditional because of dynamic 8 / 16 / 32 bit words)
  //–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
  always_comb begin
    mode        = cpucommandforparsing[31:30];
    PSELx       = cpucommandforparsing[29:27];
    SensorIndex = cpucommandforparsing[26:24];
    ref_cfg     = cpucommandforparsing[3:0];   //NEW in V2.1
    unique case (PSELx)
      3'b001, 3'b100: begin        // ROSC or TDDB
        numclkcycles        = cpucommandforparsing[23:20];
        numdescendingslopes = 6'd0;
        timeoutthreshold    = 12'd0;
      end
      3'b101: begin               // SILC
        numclkcycles        = 4'd0;
        numdescendingslopes = cpucommandforparsing[23:18];
        timeoutthreshold    = {cpucommandforparsing[15:8], cpucommandforparsing[7:4]};
      end
      default: begin              // DC, EM, Temperature, Voltage
        numclkcycles        = 4'd0;
        numdescendingslopes = 6'd0;
        timeoutthreshold    = 12'd0;
      end
    endcase

    //TO BE REMOVEDD {from my diagram}
        //cpureadinput       = CPUCommand[0];

  end

  //–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
  // 2) Break out the raw sensor‐bus into named signals
  //–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
  always_comb begin
    ROSCReading        = SensorReadings[1][0];
    DCAnalogReading    = SensorReadings[2][11:0];   //only taking 12/16 bits from the data in SensorReadings[2][15:0]
    EMReading          = SensorReadings[3][7:0];
    TDDBReading        = SensorReadings[4][0];
    SILCADCReading     = SensorReadings[5][11:0];
    TemperatureReading = SensorReadings[6][11:0];
    VoltageReading     = SensorReadings[7][11:0];
  end

  //–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
  // 3) Instantiate each sensor wrapper
  //–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

        // NOTE FOR LATER: do notice that each wrapper is explicitly asking for ONLY sensoeIndex0; this is how V1.0 of this module
        //                 had it; so for the sake of simplicity i'll leave it like this for now. in the future i'll have to 
        //                 instantiate an array instead. 

  ROSC ROSC_S1(
    mode,
    (PSELx == 1) && (SensorIndex == 0),
    ROSCReading, Clk,
    numclkcycles,
        //TO BE REMOVEDD
        //cpureadinput,
    STATUS_CLEAR,
    result[1], errorcode[1], sensorvalready[1]
  );

  AnalogSensor DCAnalog_S1(
    mode,
    (PSELx == 2) && (SensorIndex == 0),
    DCAnalogReading, Clk,
        //TO BE REMOVEDD
        //cpureadinput,
    STATUS_CLEAR,
    result[2], errorcode[2], sensorvalready[2]
  );

  EM EMSensor_S1(
    mode,
    (PSELx == 3) && (SensorIndex == 0),
    EMReading, Clk,
        //TO BE REMOVEDD
        //cpureadinput,
    STATUS_CLEAR,
    result[3], errorcode[3], sensorvalready[3]
  );

  ROSC TDDBSensor_S1(
    mode,
    (PSELx == 4) && (SensorIndex == 0),
    TDDBReading, Clk,
    numclkcycles,
        //TO BE REMOVEDD
        //cpureadinput,
    STATUS_CLEAR,
    result[4], errorcode[4], sensorvalready[4]
  );

  SILC SILC_S1(
    mode,
    (PSELx == 5) && (SensorIndex == 0),
    SILCADCReading, Clk,
    numdescendingslopes, timeoutthreshold,
        //TO BE REMOVEDD
        //cpureadinput,
    STATUS_CLEAR,
    result[5], errorcode[5], sensorvalready[5]
  );

  AnalogSensor Temperature_S1(
    mode,
    (PSELx == 6) && (SensorIndex == 0),
    TemperatureReading, Clk,
        //TO BE REMOVEDD
        //cpureadinput,
    STATUS_CLEAR,
    result[6], errorcode[6], sensorvalready[6]
  );

  AnalogSensor Voltage_S1(
    mode,
    (PSELx == 7) && (SensorIndex == 0),
    VoltageReading, Clk,
        //TO BE REMOVEDD
        //cpureadinput,
    STATUS_CLEAR,
    result[7], errorcode[7], sensorvalready[7]
  );

  //–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
  // 4) Measurement FSM: IDLE → BUSY → COMPLETE
  //–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
  typedef enum logic [1:0] { IDLE, BUSY, COMPLETE } state_t;
  state_t state, nextstate;

  // State register with asynchronous disable
  always_ff @(posedge Clk or negedge En) begin
    if (!En)       state <= IDLE;
    else           state <= nextstate;
  end

  // Next‐state logic
  always_comb begin
    unique case (state)
      
          //TO BE REMOVEDD
          //IDLE:     nextstate = (CPUCommand != 32'd0 && CPUCommand[0]==0) ? BUSY : IDLE;
      IDLE:     nextstate = (CPUCommand != 32'd0)   ? BUSY : IDLE;
      BUSY:     nextstate = sensorvalready[PSELx]   ? COMPLETE : BUSY;
          //TO BE REMOVEDD
          //COMPLETE: nextstate = cpureadinput            ? IDLE : COMPLETE;
      COMPLETE: nextstate = STATUS_CLEAR            ? IDLE : COMPLETE;
      default:  nextstate = IDLE;
    endcase
  end

  // Latch command, then measurement data, then push result out
  always_ff @(posedge Clk or negedge En) begin
    if (!En) begin
      cpucommandforparsing <= 32'd0;
      sensormeasdata       <= 32'd0;
      ResultForCPU         <= 32'd0;
    end else begin
      case (state)
        IDLE: begin
              //TO BE REMOVEDD
              //if (CPUCommand != 32'd0 && CPUCommand[0]==0)
          if (CPUCommand != 32'd0)
            cpucommandforparsing <= CPUCommand;
        end
        BUSY: begin
          if (sensorvalready[PSELx]) begin
            sensormeasdata[31:16] <= result[PSELx];
            sensormeasdata[15:13] <= errorcode[PSELx];
            sensormeasdata[12:0]  <= 12'd0; // cleare although core repacks [12:10] (REF_TAG) later
          end
        end
        COMPLETE: begin
          
          /***NEW this section completely refactored In v2.1***/
          // 1. we post process raw
          // 2. we apply REF subtraction if needed
          // 3. we build ERR & REF_TAG


          // Notes --> every scenario starts with:
          //  val_raw = fresh reading from sensor wrapper
          //  err_code is preset to 001 [success)
          //  ref_tag is preset to 000 (ra)w
          logic [15:0] val_raw   = sensormeasdata[31:16]; //raw result 16-bit word
          logic [15:0] val_final;                         //final value we'll return
          logic [2:0]  err_code  = 3'b001;                //assume valid
          logic [2:0]  ref_tag   = 3'b000;                //default RAW

          //-----Auto capture logic (only after a successful RAW (mode00))------
          //---we check if REF1 is empty (like @start-up)---
          if (mode==2'b00) begin
            if (!ref1_v[PSELx][SensorIndex]) begin
              ref1_mem[PSELx][SensorIndex] <= val_raw; //if REF1 is empty we fill corresponding sensor non-volatile memory
              ref1_v  [PSELx][SensorIndex] <= 1'b1;    //we also set its ref1_v flag (also non-volatile) to 1 to indicate data is valid on mem
              err_code = 3'b110; // Valid measurement and valid write/overwrite to REF (either REF1 or REF2)
            end
            else if (!ref2_v[PSELx][SensorIndex] && por_seen) begin //important to note that never gets triggered @start-up; only when a power reset is done (and REF1 is valid)
              ref2_mem[PSELx][SensorIndex] <= val_raw;
              ref2_v  [PSELx][SensorIndex] <= 1'b1;
              err_code = 3'b110;
            end
          end

          // Forced overwrite via ref_cfg bits (ONLY valid when mode==00)
          // notes:
          //    - when ref_cfc is 0010, only REF1 is overwritten
          //    - when ref_cfc is 0100, only REF2 is overwritten
          //    - when ref_cfc is 0101, btoh REF1 and REF2 are overwritten
          if (mode==2'b00 && ref_cfg!=4'b0000) begin
            if (ref_cfg==4'b0010 || ref_cfg==4'b0101) begin //overwrite REF1 alone with 0010
              ref1_mem[PSELx][SensorIndex] <= val_raw;
              ref1_v  [PSELx][SensorIndex] <= 1'b1;
              err_code = 3'b110;
            end
            if (ref_cfg==4'b0100 || ref_cfg==4'b0101) begin //overwrite REF2 alone with 0100
              ref2_mem[PSELx][SensorIndex] <= val_raw;
              ref2_v  [PSELx][SensorIndex] <= 1'b1;
              err_code = 3'b110;
            end            
          end
    
          // Subtraction / REF selection
          unique case (mode)
            2'b00: begin           //SLOW raw
              val_final = val_raw;
              ref_tag   = 3'b000;
              last_ref_tag[PSELx][SensorIndex] <= 2'b00;
            end
            2'b01: begin           //SLOW raw - REF1
              if (ref1_v[PSELx][SensorIndex]) begin
                val_final = val_raw - ref1_mem[PSELx][SensorIndex];
                ref_tag   = 3'b001; //raw-ref1
                last_ref_tag[PSELx][SensorIndex] <= 2'b01;
              end else begin
                val_final = 16'd0;
                ref_tag   = 3'b111; //error
                err_code  = 3'b010;  // REF1 missing
              end
            end
            2'b10: begin           //SLOW raw - REF2
              if (ref2_v[PSELx][SensorIndex]) begin
                val_final = val_raw - ref2_mem[PSELx][SensorIndex];
                ref_tag   = 3'b010; //raw-ef2
                last_ref_tag[PSELx][SensorIndex] <= 2'b10;
              end else begin
                val_final = 16'd0;
                ref_tag   = 3'b111; //error
                err_code  = 3'b011;  // REF2 missing
              end
            end
            2'b11: begin  // FAST
              val_final = val_raw;          // sensor wrapper gave last value stoed in 'recordedvalue'
              //and here we label the fast mesaurement reusult with the last slow measurement type
              ref_tag   = (last_ref_tag[PSELx][SensorIndex]==2'b01)?3'b001:
                           (last_ref_tag[PSELx][SensorIndex]==2'b10)?3'b010:
                           3'b000;
            end
          endcase

          // Pack final result
          ResultForCPU[31:16] <= val_final;
          ResultForCPU[15:13] <= err_code;
          ResultForCPU[12:10] <= ref_tag;
          ResultForCPU[9:0]   <= 10'd0;      // reserved

        end
      endcase
    end
  end

  //–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
  // 5) Sticky‐error latch
  //–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

      //to add this latch to my diagram {later}

  always_ff @(posedge Clk or negedge En) begin
    if (!En)
      err_sticky <= 1'b0;
        //TO BE REMOVEDD
        //else if (cpureadinput)
    else if (STATUS_CLEAR)
      err_sticky <= 1'b0;
    else if (state == COMPLETE)
      err_sticky <= (sensormeasdata[15:13] != 3'b001);
    
  end

  //–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
  // 5B) Done‐latch
  //–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
  always_ff @(posedge Clk or negedge En) begin
    if (!En)
      done_latched <= 1'b0;
    else if (STATUS_CLEAR)
      done_latched <= 1'b0;
    else if (state == COMPLETE)
      done_latched <= 1'b1;
  end


  //–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
  // 6) Combinational status signals
  //–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
  always_comb begin
    busy = (state == BUSY);   //{now only combinational status signal cuz err and done are latched now}
        //TO BE REMOVEDD
        //done = (state == COMPLETE);
  end

  //–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
  // 7) Pack into the 3-bit StatusBits output
  //–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
  always_comb begin
        //TO BE REMOVEDD
        //StatusBits = { busy, err_sticky, done };   {now i defined done to be in ff so that i can clear it from the CPU}
    StatusBits = { busy, err_sticky, done_latched };
  end

endmodule
