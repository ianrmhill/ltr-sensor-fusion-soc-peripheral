/**

 * Module: DataAcquisitionIP_apb (V1.0)
 * Author: Raul Vazquez Guerrero

 * DataAcquisitionIP APB Wrapper with Dynamic Byte-/Halfword-/Word-Write Support
 *
 * Register map (APB offsets):
 *   0x00 = CPUCommandsReg   // 32-bit command word; accepts 1/2/4-byte writes via PSTRB
 *   0x04 = STATUS           // read-only: {busy, err_sticky, done}
 *   0x08 = RESULT           // read-only: 32-bit measurement result
 *   0x0C = STATUS_CLEAR     // write-1-to-clear err_sticky & done
 *
 * CPUCommandsReg byte breakdown and field definitions:
 *   CMD0 [7:0]:
 *     [7:6] = Mode (00 = Stop, 01 = Fast, 10 = Slow)
 *     [5:3] = PSELx (000=None, 001=ROSC, 010=DC Analog,
 *                   011=EM, 100=TDDB, 101=SILC, 110=Temperature, 111=Voltage)
 *     [2:0] = Sensor Index (0–7)
 *

//////////////FIXXXX

 *   CMD1 [15:8]:
 *     - For ROSC/TDDB (PSELx=001 or 100): bits [3:0] = Measurement Duration (# clock cycles)
 *     - For SILC (PSELx=101): bits [5:0] = SILC slope count
 *     - Reserved otherwise
 *
 *   CMD2 [23:16]:
 *     - For SILC: MSB bits [15:8] of timeout threshold (upper 8 bits)
 *     - Reserved otherwise
 *
 *   CMD3 [31:24]:
 *     - For SILC: bits [7:4] = LSB bits [11:8] of timeout threshold, [3:0] reserved
 *     - Reserved otherwise
 *
 * Usage examples (firmware):
 *   // 8-bit DC Analog read (Mode=Fast, PSELx=010, Index=0):
 *   //   CMD0 = 01_010_000 = 0x50
 *   write_APB(0x00, 32'h00000050, PSTRB=4'b0001);
 *
 *   // 16-bit ROSC slow read with 5-cycle duration (Mode=Slow, PSELx=001, Index=0, cycles=5):
 *   //   CMD0 = 10_001_000 = 0x88; CMD1[3:0] = 5 -> 0x05 -> full halfword = 0x0588
 *   write_APB(0x00, 32'h00000588, PSTRB=4'b0011);
 *
 *   // 32-bit SILC read (Mode=Slow, PSELx=101, Index=0, slopes=12, timeout=0x123):
 *   //   CMD0 = 10_101_000 = 0xA8
 *   //   CMD1[5:0] = 12 -> 0x0C
 *   //   CMD2 = 0x01, CMD3[7:4] = 0x2 -> raw word = 0x02_01_0C_A8
 *   write_APB(0x00, 32'h02010CA8, PSTRB=4'b1111);
 *
 *  Very importntly, the core always reads a full 32-bit CPUCommandsReg and decodes fields based on PSELx.
    Note that the function ptovided in these examples is just dummy, but is juts to show how the cpu would write into into the core
 */

module DataAcquisitionIP_apb(
  input  logic            PCLK,       // APB clock
  input  logic            PRESETn,    // async active low reset or sync reset?
  input  logic            PSEL,       // slave select
  input  logic            PENABLE,    // access phase
  input  logic            PWRITE,     // 1 = write, 0 = read
  input  logic  [7:0]     PADDR,      // byte address (we only use 0x00,0x04,0x08, 0x0C)  {we'll modify this later to have the dynamic bus 8 and 16 words as well}
  input  logic [31:0]     PWDATA,     // write data
  input  logic   [3:0]    PSTRB,      // this is the field that allows us to hav 8, 16 and 32 bit writes
  input  logic [7:0][15:0] SensorReadings, // raw sensor inputs
  output logic [31:0]     PRDATA,     // read data
  output logic            PREADY,     // always ready in one cycle
  output logic            PSLVERR     // no errors
);


  // 1) CPUCommand register (32-bit)
  logic [31:0] CPUCommandsReg;

  // 2) STATUS_CLEAR pulse (write-1-to-clear both err_sticky & done)
  logic STATUS_CLEAR;

  // 3) Wires from the core
  logic [31:0] coreResult;
  logic  [2:0] coreStatusBits;

  // 4) We instantiate the core (always enabled FOR NOW only cuz im testing)
  DataAcquisitionIP_core core (
    .Clk           (PCLK),
    .En            (1'b1),
    .CPUCommand    (CPUCommandsReg),
    .STATUS_CLEAR  (STATUS_CLEAR),
    .SensorReadings(SensorReadings),
    .ResultForCPU  (coreResult),
    .StatusBits    (coreStatusBits)
  );

  // 5) APB write path: write CPUCommandsReg at offset 0x00
  always_ff @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn)
      CPUCommandsReg <= 32'd0;
    else if (PSEL && PWRITE && PENABLE && (PADDR == 8'h00)) begin
      if (PSTRB[0]) CPUCommandsReg[ 7: 0] <= PWDATA[ 7: 0];  // CMD0
      if (PSTRB[1]) CPUCommandsReg[15: 8] <= PWDATA[15: 8];  // CMD1
      if (PSTRB[2]) CPUCommandsReg[23:16] <= PWDATA[23:16];  // CMD2
      if (PSTRB[3]) CPUCommandsReg[31:24] <= PWDATA[31:24];  // CMD3
    end
  end

  // 6) APB write path: STATUS_CLEAR at offset 0x0C (W1C)
  always_ff @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn)
      STATUS_CLEAR <= 1'b0;
    else if (PSEL && PWRITE && PENABLE && (PADDR == 8'h0C))
      STATUS_CLEAR <= PWDATA[0];  // <-- only clear when bit 0 is a 1 {could also write a zero and do nothing; i think this is safer than having STATUS_CLEAR <= 1'b1;}
    else
      STATUS_CLEAR <= 1'b0;
  end

  // 7) APB read path: multiplex PRDATA by PADDR
  always_comb begin
    unique case (PADDR)
      8'h00: PRDATA = CPUCommandsReg;          // this is rlly optional but apparently is also good practice to be able to read what you sent for debugging
      8'h04: PRDATA = {29'd0, coreStatusBits}; // {busy, err_sticky, done}
      8'h08: PRDATA = coreResult;              // full 32-bit result
      8'h0C: PRDATA = 32'd0;                   // write-only clear register {alsp for debugging}
      default: PRDATA = 32'd0;                 // illegal addr --> zero
    endcase
  end

  // 8) APB handshake signals
  assign PREADY  = PSEL && PENABLE;  // always ready in one cycle
  assign PSLVERR = 1'b0;             // no error reporting

endmodule
