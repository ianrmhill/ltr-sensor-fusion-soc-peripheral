`include "uvm_macros.svh"
import uvm_pkg::*;
`define FAST_MODE 1
`define SLOW_MODE 2

interface MyInterface(input clk, input enable);

    logic [1:0] Mode;
    logic [11:0] ADCReading;
    logic [5:0] NumDescendingSlopes;
    logic [11:0] TimeoutThreshold;
    logic CPUReadComplete;
    logic [15:0] SILCReading;
    logic [2:0] ErrorCode;
    logic SILCValReady;

endinterface

class Transaction extends uvm_sequence_item;

    `uvm_object_utils(Transaction)

    // This is a key variable, the expected time for one slope cycle.
    // We use it to generate the ADCReading cycles in the driver. The driver directly send this data to the model, 
    // who then simply passes on to the scoreboard in the case of slow mode, it will be compared with SILCReading from the DUT in the scoreboard.
    randc int expected_slope_time;  
    randc bit [1:0] Mode;
    randc bit [5:0] NumDescendingSlopes;
    randc bit [11:0] TimeoutThreshold;
    bit [15:0] SILCReading;
    bit [2:0] ErrorCode;

    function new(string name = "Transaction");
        super.new(name);
    endfunction

    constraint slow_test_constraint {
        Mode == `SLOW_MODE;
        NumDescendingSlopes > 0;
        // >= 10 and < 33. The reason for < 33 is that max TimeoutThreshold = 4095, max NumDescendingSlopes = 63. 4095/63/2 = 33
        // Division by 2 is because only half of the slope time is within valid range, from 75% of ADC to 25% ADC. 
        expected_slope_time inside {[10:32]};  
        TimeoutThreshold == {12{1'b1}};
    };

    constraint fast_test_constraint {
        Mode == `FAST_MODE;
    };

    constraint random_test_constraint {
        Mode inside {`FAST_MODE, `SLOW_MODE};
        NumDescendingSlopes > 0;
        expected_slope_time inside {[10:32]};
        TimeoutThreshold == {12{1'b1}};
    };

    constraint corner_test_constraint {
        Mode == `SLOW_MODE;
        NumDescendingSlopes > 0;
        expected_slope_time inside {[10:32]};
        TimeoutThreshold == {12{1'b1}};
    };

    virtual function void do_print(uvm_printer printer);
        super.do_print(printer);
        printer.print_field_int("expected_slope_time", expected_slope_time, $bits(expected_slope_time), UVM_DEC);
        printer.print_field_int("Mode", Mode, $bits(Mode) , UVM_DEC);
        printer.print_field_int("NumDescendingSlopes", NumDescendingSlopes, $bits(NumDescendingSlopes), UVM_DEC);
        printer.print_field_int("TimeoutThreshold", TimeoutThreshold, $bits(TimeoutThreshold), UVM_DEC);
        printer.print_field_int("SILCReading", SILCReading, $bits(SILCReading), UVM_DEC);
        printer.print_field_int("ErrorCode", ErrorCode, $bits(ErrorCode), UVM_DEC);
    endfunction

endclass

class SlowTestSequence extends uvm_sequence #(Transaction);

    `uvm_object_utils(SlowTestSequence)
    Transaction trans;

    function new(string name = "SlowTestSequence");
        super.new(name);
        trans = Transaction::type_id::create("trans");
    endfunction

    virtual task body();
        trans.slow_test_constraint.constraint_mode(1);
        trans.fast_test_constraint.constraint_mode(0);
        trans.random_test_constraint.constraint_mode(0);
        trans.corner_test_constraint.constraint_mode(0);
        repeat(100) begin
            start_item(trans);
            trans.randomize();
            finish_item(trans);
        end
    endtask

endclass

class FastTestSequence extends uvm_sequence #(Transaction);

    `uvm_object_utils(FastTestSequence)
    Transaction trans;

    function new(string name = "FastTestSequence");
        super.new(name);
        trans = Transaction::type_id::create("trans");
    endfunction

    virtual task body();
        trans.slow_test_constraint.constraint_mode(0);
        trans.fast_test_constraint.constraint_mode(1);
        trans.random_test_constraint.constraint_mode(0);
        trans.corner_test_constraint.constraint_mode(0);
        repeat(5) begin
            start_item(trans);
            trans.randomize();
            finish_item(trans);
        end
    endtask

endclass

class RandomTestSequence extends uvm_sequence #(Transaction);
    `uvm_object_utils(RandomTestSequence)
    Transaction trans;

    function new(string name = "RandomTestSequence");
        super.new(name);
        trans = Transaction::type_id::create("trans");
    endfunction

    virtual task body();
        trans.slow_test_constraint.constraint_mode(0);
        trans.fast_test_constraint.constraint_mode(0);
        trans.random_test_constraint.constraint_mode(1);
        trans.corner_test_constraint.constraint_mode(0);
        repeat(100) begin
            start_item(trans);
            trans.randomize();
            finish_item(trans);
        end
    endtask
endclass

class CornerTestSequence extends uvm_sequence #(Transaction);
    `uvm_object_utils(CornerTestSequence)
    Transaction trans;

    function new(string name = "CornerTestSequence");
        super.new(name);
        trans = Transaction::type_id::create("trans");
    endfunction

    virtual task body();
        trans.slow_test_constraint.constraint_mode(0);
        trans.fast_test_constraint.constraint_mode(0);
        trans.random_test_constraint.constraint_mode(0);
        trans.corner_test_constraint.constraint_mode(1);
        test_zero_cycle();
        test_fast_adc();
    endtask
    
    task test_zero_cycle();
        start_item(trans);
        trans.randomize();
        trans.NumDescendingSlopes = 0;
        finish_item(trans);
    endtask

    task test_fast_adc();
        start_item(trans);
        trans.randomize();
        trans.expected_slope_time = 1;
        finish_item(trans);
    endtask
endclass

class DataReg extends uvm_reg;
    `uvm_object_utils(DataReg)
    uvm_reg_field data;
    
    function new(string name="DataReg");
        super.new(name, 16, UVM_NO_COVERAGE);
    endfunction

    virtual function void build();
        data = uvm_reg_field::type_id::create("data");
        data.configure(
            .parent(this),
            .size(16),
            .lsb_pos(0),
            .access("RW"),
            .volatile(0),
            .reset(0),
            .has_reset(1),
            .is_rand(1),
            .individually_accessible(1)
        );
    endfunction
endclass

class RegModel extends uvm_reg_block;
    `uvm_object_utils(RegModel)
    DataReg data_reg;

    function new(string name="RegModel");
        super.new(name, UVM_NO_COVERAGE);
    endfunction

    virtual function void build();
        data_reg = DataReg::type_id::create("data_reg");
        data_reg.build();
        data_reg.configure(this, null);

        default_map = create_map("default_map", 0, 1, UVM_LITTLE_ENDIAN);
        default_map.add_reg(data_reg, 0, "RW");

        lock_model();
    endfunction
endclass

class Driver extends uvm_driver #(Transaction);

    `uvm_component_utils(Driver)
    uvm_analysis_port #(Transaction) ap;
    Transaction trans;
    virtual MyInterface vif;
    const int delta_adc = ({12{1'b1}} >> 1);  // (upper limit + 1) - (lower limit -1)
    int rand_delay;
    event sampling_event;

    function new(string name = "Driver", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if(!uvm_config_db #(virtual MyInterface)::get(this, "", "vif", vif)) begin
            `uvm_error("Driver", "Cannot get access to vif!")
        end
        trans = Transaction::type_id::create("trans");
        ap = new("ap", this);
    endfunction

    virtual task main_phase(uvm_phase phase);
        test_time_out(); 
        forever begin
            drive();
        end
    endtask

    task test_time_out();  // Once the DUT entered the time-out state, it cannot exit without a reset. Hence, its test case is not generated from the sequence but by the driver
        wait(vif.enable == 1); // Wait until the first reset to be deasserted.
        seq_item_port.get_next_item(trans);
        trans.TimeoutThreshold = 2; // A small number to guarantee time-out
        fork
            drive_CPUReadComplete();
            drive_trans();
            drive_ADCReading();
        join_none
        wait(vif.enable == 0);  // Wait for the second reset;
        disable fork;
        seq_item_port.item_done(trans);
    endtask
    
    task drive();
        if (vif.enable == 0) begin
            @(posedge vif.clk);
            vif.Mode <= 0;
            vif.ADCReading <= 12'b111111111111;
            vif.NumDescendingSlopes <= 0;
            vif.TimeoutThreshold <= 0;
            vif.CPUReadComplete <= 0;
        end
        else begin
            seq_item_port.get_next_item(trans);
            ap.write(trans);
            `uvm_info("Driver", "Transaction received from the Sequencer and sent to the Model", UVM_LOW)
            trans.print();
            if (trans.Mode == `FAST_MODE) begin
                fast_mode();
            end
            else if (trans.Mode == `SLOW_MODE) begin
                slow_mode();
            end
            else begin
                `uvm_fatal("Driver", "Incorrect Mode!")
            end
            seq_item_port.item_done();
        end
    endtask

    task slow_mode();
        fork
            drive_CPUReadComplete();
            drive_trans();
            drive_ADCReading();
        join_none
        // After we have driven the signals, we wait for SILCValReady to be raised, after which we wait for a random cycle < 10 to send a new transaction
        @(posedge vif.SILCValReady);
        -> sampling_event;
        disable fork;
        rand_delay = $urandom_range(10, 0);
        repeat(rand_delay) @(posedge vif.clk);
    endtask

    task fast_mode();
        fork
            drive_trans();
            drive_CPUReadComplete();
        join
        wait(vif.SILCValReady == 1);
        -> sampling_event;
        rand_delay = $urandom_range(10, 0);
        repeat(rand_delay) @(posedge vif.clk);
    endtask

    task drive_CPUReadComplete();
        @(posedge vif.clk);
        vif.CPUReadComplete <= 1;
        @(posedge vif.clk);
        vif.CPUReadComplete <= 0;
    endtask

    task drive_trans();
        @(posedge vif.clk);
        vif.Mode <= trans.Mode;
        vif.NumDescendingSlopes <= trans.NumDescendingSlopes;
        vif.TimeoutThreshold <= trans.TimeoutThreshold;
    endtask

    task drive_ADCReading();
        int adc;
        int dcrmt_adc;
        dcrmt_adc = delta_adc / trans.expected_slope_time; // The decrement of ADC for each cycle, has rounding error
        for (int i=0; i<=trans.NumDescendingSlopes; i++) begin
            adc = 12'b111111111111;
            @(posedge vif.clk);
            vif.ADCReading <= adc;
            while (adc >= 0) begin
                adc = adc - dcrmt_adc;
                @(posedge vif.clk);
                vif.ADCReading <= adc;  // Drive the decremented ADC
            end
        end
    endtask

endclass

class Monitor extends uvm_monitor;

    `uvm_component_utils(Monitor)
    Transaction trans;
    uvm_analysis_port #(Transaction) ap;
    virtual MyInterface vif;
    event sampling_event;

    function new(string name = "Monitor", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        ap = new("ap", this);
        if (!uvm_config_db #(virtual MyInterface)::get(this, "", "vif", vif)) begin
            `uvm_error("Monitor", "Cannot get access to vif!")
        end
        trans = Transaction::type_id::create("trans");
    endfunction

    virtual task main_phase(uvm_phase phase);
        forever begin
            mon();
        end
    endtask

    task mon();
        if (vif.enable == 0) begin
            @(posedge vif.clk);  // If reset, do nothing
        end
        else begin
            @(sampling_event);  // Controlled by the driver
            trans.SILCReading = vif.SILCReading;
            trans.ErrorCode = vif.ErrorCode;
            `uvm_info("Monitor", "Actual Transaction sent to the Scoreboard", UVM_LOW)
            `uvm_info("Monitor", $sformatf(trans.sprint()), UVM_LOW)
            ap.write(trans);
        end
    endtask

endclass

class Agent extends uvm_agent;

    `uvm_component_utils(Agent)
    Driver driver;
    Monitor monitor;
    uvm_sequencer #(Transaction) sequencer;
    uvm_analysis_port #(Transaction) ap_mon, ap_dri;

    function new(string name = "Agent", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (is_active == UVM_ACTIVE) begin
            driver = Driver::type_id::create("driver", this);
            sequencer = uvm_sequencer #(Transaction)::type_id::create("sequencer", this);
        end
        monitor = Monitor::type_id::create("monitor", this);
    endfunction

    virtual function void connect_phase(uvm_phase phase);
        if (is_active == UVM_ACTIVE) begin
            driver.seq_item_port.connect(sequencer.seq_item_export);
            ap_dri = driver.ap;
        end
        ap_mon = monitor.ap;
    endfunction

endclass

class Model extends uvm_component;

    `uvm_component_utils(Model)
    RegModel reg_model;
    uvm_analysis_port #(Transaction) ap;
    uvm_blocking_get_port #(Transaction) port;
    Transaction trans, trans_exp;

    function new(string name = "Model", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        port = new("port", this);
        ap = new("ap", this);
        trans_exp = Transaction::type_id::create("trans_exp");
    endfunction

    virtual task main_phase(uvm_phase phase);
        forever begin
            port.get(trans);
            generate_exp();
            `uvm_info("Model", "Expected Transaction sent to the Scoreboard", UVM_LOW)
            trans_exp.print();
            ap.write(trans_exp);
        end
    endtask

    task generate_exp();
        trans_exp.Mode = trans.Mode;
        if (trans.Mode == `FAST_MODE) begin  // Fast mode
            trans_exp.SILCReading = reg_model.data_reg.get();
            trans_exp.ErrorCode = 1;
        end
        else if (trans.Mode == `SLOW_MODE) begin
            if (trans.expected_slope_time * trans.NumDescendingSlopes < trans.TimeoutThreshold) begin  // Slow mode
                trans_exp.NumDescendingSlopes = trans.NumDescendingSlopes;
                trans_exp.expected_slope_time = trans.expected_slope_time;
                trans_exp.SILCReading = trans.expected_slope_time * trans.NumDescendingSlopes;
                reg_model.data_reg.predict(trans_exp.SILCReading);
                trans_exp.ErrorCode = 1;
            end
            else begin  // Time-out
                trans_exp.ErrorCode = 7;
            end
        end
    endtask

endclass

class Scoreboard extends uvm_scoreboard;

    `uvm_component_utils(Scoreboard)
    uvm_blocking_get_port #(Transaction) exp_port, act_port;
    Transaction trans_exp, trans_act, trans_temp;
    Transaction exp_queue[$];
    real max_reading;
    const int delta_adc = ({12{1'b1}} >> 1);  // (upper limit + 1) - (lower limit -1)

    function new(string name = "Scoreboard", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        exp_port = new("exp_port", this);
        act_port = new("act_port", this);
    endfunction

    virtual task main_phase(uvm_phase phase);
        fork
            get_exp();
            get_act_compare();
        join
    endtask

    task get_exp();
        forever begin
            exp_port.get(trans_exp);
            exp_queue.push_back(trans_exp);
        end
    endtask

    task get_act_compare();
        forever begin
            act_port.get(trans_act);
            wait(exp_queue.size()>0);
            trans_temp = exp_queue.pop_front();
            if (trans_temp.Mode == `SLOW_MODE) begin
                max_reading = calculate_max_reading(trans_temp.expected_slope_time, trans_temp.NumDescendingSlopes);
                assert(trans_act.SILCReading >= trans_temp.SILCReading && trans_act.SILCReading <= max_reading && trans_act.ErrorCode == trans_temp.ErrorCode) else throw_error();
            end
            else begin
                assert (trans_act.SILCReading == trans_temp.SILCReading && trans_act.ErrorCode == trans_temp.ErrorCode) else throw_error();
            end
            $display("--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
            $display("--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
            $display("--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
        end
    endtask

    // Calculate the max reading due to rounding errors when generating the ADCReading cycles.
    function real calculate_max_reading(int expected_slope_time, int NumDescendingSlopes);
        real max_slope_time = delta_adc/((delta_adc/expected_slope_time)-1);
        return max_slope_time * NumDescendingSlopes;
    endfunction

    task throw_error();
        `uvm_error("Scoreboard", "Compare FAILED")
        `uvm_info("Scoreboard", "Expected transaction:", UVM_LOW)
        trans_temp.print();
        `uvm_info("Scoreboard", "Actual transaction:", UVM_LOW)
        trans_act.print();
    endtask

endclass

class CovCollector extends uvm_component;

    `uvm_component_utils(CovCollector)
    `uvm_analysis_imp_decl(_port_in)
    `uvm_analysis_imp_decl(_port_out)

    uvm_analysis_imp_port_in #(Transaction, CovCollector) imp_in;
    uvm_analysis_imp_port_out #(Transaction, CovCollector) imp_out;

    Transaction trans_in, trans_out;

    covergroup cg_in;
        coverpoint trans_in.Mode {
            ignore_bins igvalues = {0, 3};
        }
        coverpoint trans_in.NumDescendingSlopes {
            bins min = {0};
            bins max = {32};
            bins normal = {[1:31]};
        }
        coverpoint trans_in.expected_slope_time {
            bins fast = {1};
            bins noraml = {[2:64]};
        }
        coverpoint trans_in.TimeoutThreshold {
            bins normal = {[0:4095]};
        }
    endgroup

    covergroup cg_out;
        coverpoint trans_out.SILCReading {
            bins bin = {[0:4095]};
        }
        coverpoint trans_out.ErrorCode {
            bins normal = {1};
            ignore_bins igvalues = {0, [2:7]};
        }
    endgroup

    function new(string name="CovCollector", uvm_component parent=null);
        super.new(name, parent);
        cg_in = new();
        cg_out = new();
    endfunction

    virtual function void build_phase(uvm_phase phase);
        imp_in = new("imp_in", this);
        imp_out = new("imp_out", this);
        trans_in = Transaction::type_id::create("trans_in");
        trans_out = Transaction::type_id::create("trans_out");
    endfunction

    virtual function void write_port_in(Transaction trans_in_arg);
        trans_in = trans_in_arg;
        cg_in.sample();
    endfunction

    virtual function void write_port_out(Transaction trans_out_arg);
        trans_out = trans_out_arg;
        cg_out.sample();
    endfunction

endclass

class Environment extends uvm_env;

    `uvm_component_utils(Environment)
    Agent agent_input, agent_output;
    Model model;
    RegModel reg_model;
    Scoreboard scoreboard;
    CovCollector cov_collector;
    uvm_tlm_analysis_fifo #(Transaction) agti_mdl_fifo;
    uvm_tlm_analysis_fifo #(Transaction) agto_scb_fifo;
    uvm_tlm_analysis_fifo #(Transaction) mdl_scb_fifo;
    event sampling_event;

    function new(string name = "Environment", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        agent_input = Agent::type_id::create("agent_input", this);
        agent_output = Agent::type_id::create("agent_output", this);
        agent_input.is_active = UVM_ACTIVE;
        agent_output.is_active = UVM_PASSIVE;

        model = Model::type_id::create("model", this);
        scoreboard = Scoreboard::type_id::create("scoreboard", this);
        cov_collector = CovCollector::type_id::create("cov_collector", this);
        reg_model = RegModel::type_id::create("reg_model");
        reg_model.build();

        agti_mdl_fifo = new("agti_mdl_fifo", this);
        agto_scb_fifo = new("agto_scb_fifo", this);
        mdl_scb_fifo = new("mdl_scb_fifo", this);
    endfunction

    virtual function void connect_phase(uvm_phase phase);
        agent_input.ap_dri.connect(agti_mdl_fifo.analysis_export);
        agent_input.ap_dri.connect(cov_collector.imp_in);
        model.port.connect(agti_mdl_fifo.blocking_get_export);
        agent_output.ap_mon.connect(agto_scb_fifo.analysis_export);
        agent_output.ap_mon.connect(cov_collector.imp_out);
        scoreboard.act_port.connect(agto_scb_fifo.blocking_get_export);
        model.ap.connect(mdl_scb_fifo.analysis_export);
        scoreboard.exp_port.connect(mdl_scb_fifo.blocking_get_export);
        agent_input.driver.sampling_event = sampling_event;
        agent_output.monitor.sampling_event = sampling_event;
        model.reg_model = this.reg_model;
    endfunction

    // Suppress the logging of the input monitor 
    virtual function void end_of_elaboration_phase(uvm_phase phase);
        agent_input.monitor.set_report_verbosity_level(UVM_NONE);
    endfunction

endclass

class Test extends uvm_test;

    `uvm_component_utils(Test)
    Environment env;
    SlowTestSequence slow_test_sequence;
    FastTestSequence fast_test_sequence;
    RandomTestSequence random_test_sequence;
    CornerTestSequence corner_test_sequence;

    function new(string name = "Test", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        env = Environment::type_id::create("env", this);
        slow_test_sequence = SlowTestSequence::type_id::create("slow_test_sequence");
        fast_test_sequence = FastTestSequence::type_id::create("fast_test_sequence");
        random_test_sequence = RandomTestSequence::type_id::create("random_test_sequence");
        corner_test_sequence = CornerTestSequence::type_id::create("corner_test_sequence");
    endfunction

    virtual task main_phase(uvm_phase phase);
        phase.raise_objection(this);
        slow_test_sequence.start(env.agent_input.sequencer);
        fast_test_sequence.start(env.agent_input.sequencer);
        random_test_sequence.start(env.agent_input.sequencer);
        corner_test_sequence.start(env.agent_input.sequencer);
        phase.drop_objection(this);
    endtask
endclass

module TB_SILC;

    logic clk, enable;
    MyInterface itfc(clk, enable);

    SILC DUT (
        .Mode(itfc.Mode),
        .Enable(enable),
        .ADCReading(itfc.ADCReading),
        .Clk(clk),
        .NumDescendingSlopes(itfc.NumDescendingSlopes),
        .TimeoutThreshold(itfc.TimeoutThreshold),
        .CPUReadComplete(itfc.CPUReadComplete),
        .SILCReading(itfc.SILCReading),
        .ErrorCode(itfc.ErrorCode),
        .SILCValReady(itfc.SILCValReady)
    );

    initial begin
        clk <= 0;
        enable <= 0;
        #100;
        enable <= 1;
        #100;
        enable <= 0;  // Second reset to make the DUT exit its time-out state
        #100;
        enable <= 1;
        #10000000;
        $stop();
    end

    always #5 clk <= ~clk;

    initial begin
        $dumpfile("dump.vcd");
        $dumpvars;
    end

    initial begin
        uvm_config_db #(virtual MyInterface)::set(null, "uvm_test_top.env.agent_input.*", "vif", itfc);
        uvm_config_db #(virtual MyInterface)::set(null, "uvm_test_top.env.agent_output.*", "vif", itfc);
        run_test("Test");
    end

endmodule