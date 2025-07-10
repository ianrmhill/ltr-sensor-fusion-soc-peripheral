//TO DO: add documentation here later

`timescale 1ns/1ps

module tb_apb;
  // APB signals
  logic        PCLK;
  logic        PRESETn;
  logic        PSEL, PENABLE, PWRITE;
  logic [7:0]  PADDR;
  logic [31:0] PWDATA;
  logic [31:0] PRDATA;
  logic        PREADY, PSLVERR;

  // Dummy sensor inputs: only type-2 (DCAnalog) matters here
  logic [7:0][15:0] SensorReadings;

  // Device Under Test
  DataAcquisitionIP_apb dut (
    .PCLK          (PCLK),
    .PRESETn       (PRESETn),
    .PSEL          (PSEL),
    .PENABLE       (PENABLE),
    .PWRITE        (PWRITE),
    .PADDR         (PADDR),
    .PWDATA        (PWDATA),
    .PRDATA        (PRDATA),
    .PREADY        (PREADY),
    .PSLVERR       (PSLVERR),
    .SensorReadings(SensorReadings)
  );

  // 1) Clock gen: 10 ns period
  initial PCLK = 0;
  always #5 PCLK = ~PCLK;

  // 2) Reset
  initial begin
    PRESETn = 0;
    #20;
    PRESETn = 1;
  end

  // 3) We drive dummy SensorReadings: make slot 2 = 16'h0FAB
  initial begin
    integer i;
    for (i = 0; i < 8; i++) SensorReadings[i] = 16'd0;
    SensorReadings[2] = 16'h0FAB;
  end

  // 4) Test sequence
  initial begin
    // wait for de-assert reset
    @(negedge PRESETn);
    @(posedge PRESETn);
    @(posedge PCLK);

    // Write a Slow-mode DC-Analog command:
    // mode=2'b10 (Slow), PSELx=3'b010 (type 2), idx=0, other fields=0, clear-bit=0
    logic [31:0] cmd = {2'b10,3'b010,3'b000, 4'd0,6'd0,12'd0, 1'b0};
    apb_write(8'h00, cmd);

    // Poll STATUS @0x04 until done==1
    do begin
      apb_read(8'h04);
      // StatusBits = PRDATA[2:0] == {busy,err,done}
      #10;
    end while (PRDATA[2] == 1'b1);  // busy == PRDATA[2]

    // Now done==PRDATA[0] should be 1
    if (PRDATA[0] !== 1'b1) $fatal(1, "Done never asserted");

    // Read back RESULT @0x08
    apb_read(8'h08);
    if (PRDATA !== 32'h00000FAB)
      $fatal(1, "Result mismatch: got 0x%08h, expected 0x00000FAB", PRDATA);

    $display("PASS: APB wrapper + core behaved as expected.");
    $finish;
  end

  //----------------------------------------------------------------------  
  // APB bus helper tasks
  //----------------------------------------------------------------------
  task apb_write(input logic [7:0] addr, input logic [31:0] data);
    begin
      @(posedge PCLK);
      PSEL    = 1;
      PWRITE  = 1;
      PENABLE = 0;
      PADDR   = addr;
      PWDATA  = data;
      @(posedge PCLK);
      PENABLE = 1;
      @(posedge PCLK);
      // complete on this cycle
      PSEL    = 0;
      PENABLE = 0;
    end
  endtask

  task apb_read(input logic [7:0] addr);
    begin
      @(posedge PCLK);
      PSEL    = 1;
      PWRITE  = 0;
      PENABLE = 0;
      PADDR   = addr;
      @(posedge PCLK);
      PENABLE = 1;
      @(posedge PCLK);
      // capture PRDATA
      $display("APB Read @%02h => %08h", addr, PRDATA);
      PSEL    = 0;
      PENABLE = 0;
    end
  endtask

endmodule
