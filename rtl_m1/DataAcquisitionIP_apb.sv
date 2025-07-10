//add documentation note here {TO DO later}

// CPUCommandsReg at 0x00
// StatusBits     at 0x04  [offsets defined artbitrarly; can ask Ian if there's a better way]
// ResultForCPU   at 0x08

module DataAcquisitionIP_apb(
  input  logic            PCLK,       // APB clock
  input  logic            PRESETn,    // async active low reset or sync reset?
  input  logic            PSEL,       // slave select
  input  logic            PENABLE,    // access phase
  input  logic            PWRITE,     // 1 = write, 0 = read
  input  logic  [7:0]     PADDR,      // byte address (we only use 0x00,0x04,0x08)  {we'll modify this later to have the dynamic bus 8 and 16 words as well}
  input  logic [31:0]     PWDATA,     // write data
  input  logic [7:0][15:0] SensorReadings, // raw sensor inputs
  output logic [31:0]     PRDATA,     // read data
  output logic            PREADY,     // always ready in one cycle
  output logic            PSLVERR     // no errors
);

//check PSTRB later when doing the dynamic bus for 8 and 16 bit wors

  // 1) CPUCommand register (32-bit)
  logic [31:0] CPUCommandsReg;

  // 2) Wires from the core
  logic [31:0] coreResult;
  logic  [2:0] coreStatusBits;

  // 3) We instantiate the core (always enabled FOR NOW only cuz im testing)
  DataAcquisitionIP_core core (
    .Clk           (PCLK),
    .En            (1'b1),
    .CPUCommand    (CPUCommandsReg),
    .SensorReadings(SensorReadings),
    .ResultForCPU  (coreResult),
    .StatusBits    (coreStatusBits)
  );

  // 4) APB write path: write CPUCommandsReg at offset 0x00
  always_ff @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn)
      CPUCommandsReg <= 32'd0;
    else if (PSEL && PWRITE && PENABLE && (PADDR == 8'h00))
      CPUCommandsReg <= PWDATA;
  end

  // 5) APB read path: multiplex PRDATA by PADDR
  always_comb begin
    unique case (PADDR)
      8'h00: PRDATA = CPUCommandsReg; // this is rlly optional but apparently is also good practice to be able to read what you sent for debugging
      8'h04: PRDATA = {29'd0, coreStatusBits};                         // {busy, err_sticky, done}
      8'h08: PRDATA = coreResult;                                      // full 32-bit result
      default: PRDATA = 32'd0;                                         // illegal addr --> zero
    endcase
  end

  // 6) APB handshake signals
  assign PREADY  = PSEL && PENABLE;  // always ready in one cycle
  assign PSLVERR = 1'b0;             // no error reporting

endmodule
