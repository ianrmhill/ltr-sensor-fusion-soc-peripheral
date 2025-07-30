`timescale 1ns/1ps

module tb_APB_IPCore;
  // APB master signals
  logic         PCLK;
  logic         PRESETn;
  logic         PSEL, PENABLE, PWRITE;
  logic [7:0]   PADDR;
  logic [31:0]  PWDATA;
  logic [3:0]   PSTRB;
  // Sensor inputs (8 slots of 16 bits)
  logic [7:0][15:0] SensorReadings;
  // APB slave outputs
  logic [31:0]  PRDATA;
  logic         PREADY, PSLVERR;

  // DUT instantiation (APB wrapper)
  DataAcquisitionIP_apb dut (
    .PCLK          (PCLK),
    .PRESETn       (PRESETn),
    .PSEL          (PSEL),
    .PENABLE       (PENABLE),
    .PWRITE        (PWRITE),
    .PADDR         (PADDR),
    .PWDATA        (PWDATA),
    .PSTRB         (PSTRB),
    .SensorReadings(SensorReadings),
    .PRDATA        (PRDATA),
    .PREADY        (PREADY),
    .PSLVERR       (PSLVERR)
  );

    // ----------------------------------------------------------------
    // Localparams for APB addresses and byte strobes so readibilty is better
    localparam [7:0] ADDR_CPU_CMD   = 8'h00;
    localparam [7:0] ADDR_STATUS    = 8'h04;
    localparam [7:0] ADDR_RESULT    = 8'h08;
    localparam [7:0] ADDR_CLEAR     = 8'h0C;

    localparam [3:0] STROBE_BYTE0   = 4'b0001;
    localparam [3:0] STROBE_BYTE1   = 4'b0010;
    localparam [3:0] STROBE_HALF    = 4'b0011;
    localparam [3:0] STROBE_WORD    = 4'b1111;
  // ----------------------------------------------------------------

  // Clock gen: 50 MHz
  initial begin
    PCLK = 0;
    forever #10 PCLK = ~PCLK;
  end

  // Reset pulse
  initial begin
    PRESETn = 0;
    PSEL    = 0; PENABLE = 0; PWRITE = 0;
    PADDR   = 8'h00; PWDATA = 32'd0; PSTRB = 4'd0;
    SensorReadings = '{default:16'd0};
    #50;
    PRESETn = 1;
  end

  // APB write task
  task write_apb(
    input [7:0]   addr,
    input [31:0]  data,
    input [3:0]   strb_mask
  );
    begin
      // Address phase
      @(posedge PCLK);
      PSEL    <= 1;
      PWRITE  <= 1;
      PENABLE <= 0;
      PADDR   <= addr;
      PWDATA  <= data;
      PSTRB   <= strb_mask;
      // Enable phase
      @(posedge PCLK);
      PENABLE <= 1;
      // Wait one cycle, then deassert
      @(posedge PCLK);
      PSEL    <= 0;
      PENABLE <= 0;
      PWRITE  <= 0;
      PSTRB   <= 4'd0;
    end
  endtask

  // APB read task
  task read_apb(
    input  [7:0]  addr,
    output [31:0] data_out
  );
    begin
      // Address phase
      @(posedge PCLK);
      PSEL    <= 1;
      PWRITE  <= 0;
      PENABLE <= 0;
      PADDR   <= addr;
      PSTRB   <= 4'd0;
      // Enable phase
      @(posedge PCLK);
      PENABLE <= 1;
      // Capture data
      @(posedge PCLK);
      data_out <= PRDATA;
      // Tear down
      PSEL    <= 0;
      PENABLE <= 0;
    end
  endtask

  // Main test sequence
  initial begin
    logic [31:0] status, result;
    // Wait for reset release
    @(posedge PRESETn);

    //--- 1) ROSC Slow measurement at Index=0, 20 clock cycles ---
    // CMD0 = {Mode=10, PSELx=001, Index=000} = 2'b10_3'b001_3'b000 = 8'h88
    write_apb(8'h00, 32'h00000088, 4'b0001);  // byte-write CMD0

    // CMD1: cycles=20 → place in bits [23:20], full half-word=0x(0014)<<4|0x88 = 0x14_88
    write_apb(8'h00, (20<<20),           4'b0010);  // halfword-write CMD1

    // Simulate sensor readiness after 20 cycles
    fork
      begin
        repeat (20) @(posedge PCLK);
        // ROSCReading = SensorReadings[1][0]
        SensorReadings[1] <= 16'h0001;  // trigger ready
      end
    join_none

    //--- 2) Poll STATUS until done_latched (bit0) goes high ---
    do begin
      read_apb(8'h04, status);
      @(posedge PCLK);
    end while (status[0] == 1'b0);
    $display("[%0t] STATUS signaled DONE (0x04) = 0x%0h", $time, status);

    //--- 3) Read RESULT and display ---
    read_apb(8'h08, result);
    $display("[%0t] RESULT (0x08) = 0x%0h → Value=0x%0h  Err=0x%0h",
             $time,
             result,
             result[31:16],
             result[15:13]);

    //--- 4) Clear status via STATUS_CLEAR (0x0C) ---
    $display("[%0t] STATUS before clear = 0x%0h", $time, status);
    write_apb(8'h0C, 32'h1, 4'b0001);
    read_apb(8'h04, status);
    $display("[%0t] STATUS  after clear = 0x%0h", $time, status);

    //--- 5) Reset the command register so FSM returns to IDLE ---
    write_apb(8'h00, 32'd0, 4'b1111);
    @(posedge PCLK);

    $display("[%0t] ROSC slow‐mode test complete.", $time);
    $finish;
  end

endmodule
