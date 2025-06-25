`include "uvm_macros.svh"
import uvm_pkg::*;
`define FAST_MODE 1
`define SLOW_MODE 2

interface MyInterface (input clk);
    logic [1:0] Mode;
    logic Enable;
    logic SensorReading;
    logic [3:0] NumClkCycles;
    logic CPUReadComplete;
    logic [15:0] ROSCReading;
    logic [2:0] ErrorCode;
    logic ROSCValReady;
endinterface

class Transaction extends uvm_sequence_item;
    `uvm_object_utils(Transaction)
    randc bit [1:0] mode;
    randc bit [3:0] rosc_cycle;
    randc int rosc_freq_ratio;
    bit [15:0] rosc_reading;
    bit [3:0] error_code;
    

    function new(string name="Transaction");
        super.new(name);
    endfunction

    constraint random_test_constraint {
        mode inside {`FAST_MODE, `SLOW_MODE};
        rosc_cycle > 0;
        rosc_freq_ratio inside {1, 2, 5, 10}; // Multiple of 10, because the clock's half period is 10
    };

    constraint slow_test_constraint {
        mode == `SLOW_MODE;
        rosc_cycle > 0;
        rosc_freq_ratio inside {1, 2, 5, 10};
    };

    constraint fast_test_constraint {
        mode == `FAST_MODE;
    }

    constraint corner_test_constraint {
        mode == `SLOW_MODE;
        rosc_cycle inside {4'b1111, 0};
        rosc_freq_ratio inside {1, 2, 5, 10};
    }

    virtual function void do_print(uvm_printer printer);
        super.do_print(printer);
        printer.print_field_int("mode", mode, $bits(mode), UVM_DEC);
        printer.print_field_int("rosc_cycle", rosc_cycle, $bits(rosc_cycle), UVM_DEC);
        printer.print_field_int("rosc_reading", rosc_reading, $bits(rosc_reading), UVM_DEC);
        printer.print_field_int("error_code", error_code, $bits(error_code), UVM_DEC);
        printer.print_field_int("rosc_freq_ratio", rosc_freq_ratio, $bits(rosc_freq_ratio), UVM_DEC);    
    endfunction

endclass

class SlowTestSequence extends uvm_sequence #(Transaction);
    `uvm_object_utils(SlowTestSequence)
    Transaction trans;

    function new(string name="SlowTestSequence");
        super.new(name);
        trans = Transaction::type_id::create("trans");
    endfunction

    virtual task body();
        trans.slow_test_constraint.constraint_mode(1);
        trans.fast_test_constraint.constraint_mode(0);
        trans.random_test_constraint.constraint_mode(0);
        trans.corner_test_constraint.constraint_mode(0);
        repeat(20) begin
            start_item(trans);
            trans.randomize();
            finish_item(trans);
        end
    endtask
endclass

class FastTestSequence extends uvm_sequence #(Transaction);
    `uvm_object_utils(FastTestSequence)
    Transaction trans;
    function new(string name="FastTestSequence");
        super.new(name);
        trans = Transaction::type_id::create("trans");
    endfunction

    virtual task body();
        trans.slow_test_constraint.constraint_mode(0);
        trans.fast_test_constraint.constraint_mode(1);
        trans.random_test_constraint.constraint_mode(0);
        trans.corner_test_constraint.constraint_mode(0);
        repeat(20) begin
            start_item(trans);
            trans.randomize();
            finish_item(trans);
        end
    endtask
endclass

class RandomTestSequence extends uvm_sequence #(Transaction);
    `uvm_object_utils(RandomTestSequence)
    Transaction trans;
    function new(string name="RandomTestSequence");
        super.new(name);
        trans = Transaction::type_id::create("trans");
    endfunction

    virtual task body();
        trans.slow_test_constraint.constraint_mode(0);
        trans.fast_test_constraint.constraint_mode(0);
        trans.random_test_constraint.constraint_mode(1);
        trans.corner_test_constraint.constraint_mode(0);
        repeat(20) begin
            start_item(trans);
            trans.randomize();
            finish_item(trans);
        end
    endtask
endclass

class CornerTestSequence extends uvm_sequence #(Transaction);
    `uvm_object_utils(CornerTestSequence)
    Transaction trans;
    function new(string name="CornerTestSequence");
        super.new(name);
        trans = Transaction::type_id::create("trans");
    endfunction

    virtual task body();
        trans.slow_test_constraint.constraint_mode(0);
        trans.fast_test_constraint.constraint_mode(0);
        trans.random_test_constraint.constraint_mode(0);
        trans.corner_test_constraint.constraint_mode(1);
        repeat(3) begin
            start_item(trans);
            trans.randomize();
            finish_item(trans);
        end
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
    Transaction trans;
    virtual MyInterface vif;
    uvm_analysis_port #(Transaction) ap;
    event sampling_event;
    bit cpu_read_complete = 0;
    int rand_delay;

    function new(string name="Driver", uvm_component parent=null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        if(!uvm_config_db #(virtual MyInterface)::get(this, "", "vif", vif)) begin
            `uvm_fatal("Driver", "Cannot access the virtual interface!")
        end
        ap = new("ap", this);
    endfunction

    virtual task reset_phase(uvm_phase phase);
        @(posedge vif.clk);
        vif.Enable <= 0;
        vif.Mode <= 0;
        vif.NumClkCycles <= 0;
        vif.CPUReadComplete <= 0;
        vif.ROSCReading <= 0;
        #100;
        vif.Enable <= 1;
    endtask

    virtual task main_phase(uvm_phase phase);
        forever begin
            drive();
        end
    endtask

    task drive();
        seq_item_port.get_next_item(trans);
        ap.write(trans);
        `uvm_info("Driver", "Transaction received from the Sequencer and sent to the Model", UVM_LOW)
        trans.print();
        if (trans.mode == `FAST_MODE) begin
            fast_mode();
        end
        else if (trans.mode == `SLOW_MODE) begin
            slow_mode();
        end
        else begin
            `uvm_fatal("Driver", "Incorrect mode!");
        end
        seq_item_port.item_done();
    endtask

    task slow_mode();
        fork
            drive_trans();
            drive_sensor();
            drive_CPUReadComplete();
        join_none
        @(posedge vif.ROSCValReady);
        disable fork;
        -> sampling_event;
        rand_delay = $urandom_range(10, 0);
        repeat(rand_delay) @(posedge vif.clk);
    endtask

    task fast_mode();
        fork
            drive_trans();
            drive_CPUReadComplete();
        join
        wait(vif.ROSCValReady == 1);
        -> sampling_event;
        rand_delay = $urandom_range(10, 0);
        repeat(rand_delay) @(posedge vif.clk);
    endtask

    task drive_trans();
        vif.Mode <= trans.mode;
        vif.NumClkCycles <= trans.rosc_cycle;
    endtask

    task drive_CPUReadComplete();
        @(posedge vif.clk);
        vif.CPUReadComplete <= 1;
        @(posedge vif.clk);
        vif.CPUReadComplete <= 0;
    endtask

    task drive_sensor();
        bit sensor_value = 0;
        @(posedge vif.clk);
        vif.SensorReading <= sensor_value;
        forever begin
            sensor_value = ~sensor_value;
            #(10/trans.rosc_freq_ratio);
            vif.SensorReading <= sensor_value;
        end
    endtask

endclass

class Monitor extends uvm_monitor;
    `uvm_component_utils(Monitor)
    uvm_analysis_port #(Transaction) ap;
    virtual MyInterface vif;
    Transaction trans;
    event sampling_event;

    function new(string name="Monitor", uvm_component parent=null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        if (!uvm_config_db #(virtual MyInterface)::get(this, "", "vif", vif)) begin
            `uvm_fatal("Monitor", "Cannot access the virtual interface!")
        end
        ap = new("ap", this);
        trans = Transaction::type_id::create("trans");
    endfunction

    virtual task main_phase(uvm_phase phase);
        forever begin
            mon();
        end
    endtask

    task mon();
        @sampling_event;
        trans.rosc_reading = vif.ROSCReading;
        trans.error_code = vif.ErrorCode;
        `uvm_info("Monitor", "Actual Transaction sent to the Scoreboard", UVM_LOW)
        trans.print();
        ap.write(trans);
    endtask
endclass

class Agent extends uvm_agent;
    `uvm_component_utils(Agent)
    uvm_analysis_port #(Transaction) ap_dri, ap_mon;
    Driver driver;
    Monitor monitor;
    uvm_sequencer #(Transaction) sequencer;

    function new(string name="Agent", uvm_component parent=null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        monitor = Monitor::type_id::create("monitor", this);
        if(is_active == UVM_ACTIVE) begin
            driver = Driver::type_id::create("driver", this);
            sequencer = uvm_sequencer #(Transaction)::type_id::create("sequencer", this);
        end
    endfunction

    virtual function void connect_phase(uvm_phase phase);
        ap_mon = monitor.ap;
        if(is_active == UVM_ACTIVE) begin
            ap_dri = driver.ap;
            driver.seq_item_port.connect(sequencer.seq_item_export);
        end
    endfunction
endclass

class Model extends uvm_component;
    `uvm_component_utils(Model)
    RegModel reg_model;
    uvm_blocking_get_port #(Transaction) port;
    uvm_analysis_port #(Transaction) ap;
    Transaction trans, trans_exp;

    function new(string name = "Model", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase (uvm_phase phase);
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
        trans_exp.mode = trans.mode;
        trans_exp.error_code = 1;
        if(trans.mode == `FAST_MODE) begin
            trans_exp.rosc_reading = reg_model.data_reg.get();
        end
        else if (trans.mode == `SLOW_MODE) begin
            trans_exp.rosc_reading = trans.rosc_cycle * trans.rosc_freq_ratio;
            reg_model.data_reg.predict(trans_exp.rosc_reading);
        end
    endtask

endclass

class Scoreboard extends uvm_scoreboard;
    `uvm_component_utils(Scoreboard)
    uvm_blocking_get_port #(Transaction) exp_port, act_port;
    Transaction trans_exp, trans_act, trans_temp;
    Transaction exp_queue[$];

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
            if (trans_act.rosc_reading == trans_temp.rosc_reading && trans_act.error_code == trans_temp.error_code) begin
                `uvm_info("Scoreboard", "DATA MATCHED", UVM_LOW)
            end
            else begin
                throw_error();
            end
            $display("--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
            $display("--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
            $display("--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
        end
    endtask

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
        coverpoint trans_in.mode {
            ignore_bins igvalues = {0, 3};
        }
        coverpoint trans_in.rosc_cycle {
            bins min = {0};
            bins max = {15};
            bins normal = {[1:14]};
        }
    endgroup

    covergroup cg_out;
        coverpoint trans_out.rosc_reading {
            bins bin = {[0:65535]};
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
        agent_input.driver.sampling_event = this.sampling_event;
        agent_output.monitor.sampling_event = this.sampling_event;
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

    virtual task reset_phase(uvm_phase phase);
        phase.raise_objection(this);
            #100;
        phase.drop_objection(this);
    endtask

    virtual task main_phase(uvm_phase phase);
        phase.raise_objection(this);
        slow_test_sequence.start(env.agent_input.sequencer);
        fast_test_sequence.start(env.agent_input.sequencer);
        random_test_sequence.start(env.agent_input.sequencer);
        corner_test_sequence.start(env.agent_input.sequencer);
        phase.drop_objection(this);
    endtask
endclass

module TB_ROSC;

    logic clk;
    MyInterface itfc(clk);

    // Clock generation
    initial begin
        clk <= 0;
        #100000;
        $stop();
    end
    always #10 clk <= ~clk;

    ROSC DUT (
        .Mode(itfc.Mode),
        .Enable(itfc.Enable),
        .SensorReading(itfc.SensorReading),
        .Clk(clk),
        .NumClkCycles(itfc.NumClkCycles),
        .CPUReadComplete(itfc.CPUReadComplete),
        .ROSCReading(itfc.ROSCReading),
        .ErrorCode(itfc.ErrorCode),
        .ROSCValReady(itfc.ROSCValReady)
    );

    initial begin
        uvm_config_db #(virtual MyInterface)::set(null, "uvm_test_top.env.agent_input.*", "vif", itfc);
        uvm_config_db #(virtual MyInterface)::set(null, "uvm_test_top.env.agent_output.*", "vif", itfc);
        run_test("Test");
    end

    initial begin
        $dumpfile("dump.vcd"); 
        $dumpvars;
    end

endmodule