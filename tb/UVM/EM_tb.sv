`include "uvm_macros.svh"
import uvm_pkg::*;

`define SLOW_MODE 2
`define FAST_MODE 1

interface MyInterface (input logic clk);
    logic [1:0] Mode;
    logic Enable;
    logic [7:0] EMReading;
    logic CPUReadComplete;
    logic [16:0] EMResult;
    logic [2:0] ErrorCode;
    logic EMValReady;
endinterface

class Transaction extends uvm_sequence_item;
    `uvm_object_utils(Transaction);
    randc bit [1:0] mode;
    randc bit [7:0] em_reading;
    bit [7:0] em_result;
    bit [2:0] error_code;
    
    function new(string name="Transaction");
        super.new(name);
    endfunction

    constraint slow_test_constraint {
        mode == `SLOW_MODE;
    };

    constraint fast_test_constraint {
        mode == `FAST_MODE;
    };

    constraint random_test_constraint {
        mode inside {`FAST_MODE, `SLOW_MODE};
    };

    virtual function void do_print(uvm_printer printer);
        super.do_print(printer);
        printer.print_field_int("mode", mode, $bits(mode), UVM_DEC);
        printer.print_field_int("em_reading", em_reading, $bits(em_reading), UVM_DEC);
        printer.print_field_int("em_result", em_result, $bits(em_result), UVM_DEC);
        printer.print_field_int("error_code", error_code, $bits(error_code), UVM_DEC);
    endfunction

endclass

class SlowTestSequence extends uvm_sequence #(Transaction);
    `uvm_object_utils(SlowTestSequence);
    Transaction trans;
    function new(string name="SlowTestSequence");
        super.new(name);
        trans = Transaction::type_id::create("trans");
    endfunction

    virtual task body();
        trans.slow_test_constraint.constraint_mode(1);
        trans.fast_test_constraint.constraint_mode(0);
        trans.random_test_constraint.constraint_mode(0);
        repeat(2000) begin
            start_item(trans);
            trans.randomize();
            finish_item(trans);
        end
    endtask
endclass

class FastTestSequence extends uvm_sequence #(Transaction);
    `uvm_object_utils(FastTestSequence);
    Transaction trans;
    function new(string name="FastTestSequence");
        super.new(name);
        trans = Transaction::type_id::create("trans");
    endfunction

    virtual task body();
        trans.slow_test_constraint.constraint_mode(0);
        trans.fast_test_constraint.constraint_mode(1);
        trans.random_test_constraint.constraint_mode(0);
        repeat(5) begin
            start_item(trans);
            trans.randomize();
            finish_item(trans);
        end
    endtask
endclass

class RandomTestSequence extends uvm_sequence #(Transaction);
    `uvm_object_utils(RandomTestSequence);
    Transaction trans;
    function new(string name="RandomTestSequence");
        super.new(name);
        trans = Transaction::type_id::create("trans");
    endfunction

    virtual task body();
        trans.slow_test_constraint.constraint_mode(0);
        trans.fast_test_constraint.constraint_mode(0);
        trans.random_test_constraint.constraint_mode(1);
        repeat(2000) begin
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
        super.new(name, 8, UVM_NO_COVERAGE);
    endfunction

    virtual function void build();
        data = uvm_reg_field::type_id::create("data");
        data.configure(
            .parent(this),
            .size(8),
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
    `uvm_component_utils(Driver);
    virtual MyInterface vif;
    Transaction trans;
    uvm_analysis_port #(Transaction) ap;
    event sampling_event;

    function new(string name="Driver", uvm_component parent=null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        if (!uvm_config_db #(virtual MyInterface)::get(this,"", "vif", vif)) begin
            `uvm_fatal("Driver", "Cannot access to the virtual interface!")
        end
        ap = new("ap", this);
    endfunction

    virtual task reset_phase(uvm_phase phase);
        vif.Enable <= 0;
        vif.Mode <= 0;
        vif.CPUReadComplete <= 0;
        vif.EMReading <= 0;
        #100;
        vif.Enable <= 1;
    endtask

    virtual task main_phase(uvm_phase phase);
        forever begin
            drive();
        end
    endtask

    task drive();
        int rand_delay;

        seq_item_port.get_next_item(trans);
        ap.write(trans);
        `uvm_info("Driver", "Transaction received from the Sequencer and sent to the Scoreboard", UVM_LOW)
        trans.print();
        fork
            drive_CPUReadComplete();
            drive_trans();
        join
        if (trans.mode == `SLOW_MODE) @(posedge vif.EMValReady);
        else if (trans.mode == `FAST_MODE) wait(vif.EMValReady==1);
        -> sampling_event;
        rand_delay = $urandom_range(10,0);
        repeat(rand_delay) @(posedge vif.clk);
        seq_item_port.item_done();
    endtask

    task drive_CPUReadComplete();
        @(posedge vif.clk);
        vif.CPUReadComplete <= 1;
        @(posedge vif.clk);
        vif.CPUReadComplete <= 0;
    endtask

    task drive_trans();
        @(posedge vif.clk);
        vif.Mode <= trans.mode;
        vif.EMReading <= trans.em_reading;
    endtask

endclass

class Monitor extends uvm_monitor;
    `uvm_component_utils(Monitor);
    uvm_analysis_port #(Transaction) ap;
    virtual MyInterface vif;
    Transaction trans;
    event sampling_event;

    function new(string name="Monitor", uvm_component parent=null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        ap = new("ap", this);
        if (!uvm_config_db #(virtual MyInterface)::get(this,"", "vif", vif)) begin
            `uvm_fatal("Driver", "Cannot access to the virtual interface!")
        end
        trans = Transaction::type_id::create("trans");
    endfunction

    virtual task main_phase(uvm_phase phase);
        forever begin
            mon();
        end
    endtask

    task mon();
        @sampling_event;
        trans.em_result = vif.EMResult;
        trans.error_code = vif.ErrorCode;
        `uvm_info("Monitor", "Actual Transaction sent to the Scoreboard", UVM_LOW);
        `uvm_info("Monitor", $sformatf(trans.sprint()), UVM_LOW);
        ap.write(trans);
    endtask
endclass

class Agent extends uvm_agent;
    `uvm_component_utils(Agent);
    Driver driver;
    Monitor monitor;
    uvm_sequencer #(Transaction) sequencer;
    uvm_analysis_port #(Transaction) ap_dri, ap_mon;

    function new(string name="Agent", uvm_component parent=null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        monitor = Monitor::type_id::create("monitor", this);
        if (is_active == UVM_ACTIVE) begin
            driver = Driver::type_id::create("driver", this);
            sequencer = uvm_sequencer #(Transaction)::type_id::create("sequencer", this);
        end
    endfunction

    virtual function void connect_phase (uvm_phase phase);
        if (is_active == UVM_ACTIVE) begin
            driver.seq_item_port.connect(sequencer.seq_item_export);
            ap_dri = driver.ap;
        end
        ap_mon = monitor.ap;
    endfunction
endclass

class Scoreboard extends uvm_scoreboard;
    `uvm_component_utils(Scoreboard);
    RegModel reg_model;
    uvm_status_e status;
    uvm_blocking_get_port #(Transaction) port_act, port_exp;
    Transaction exp_queue[$];
    Transaction trans_exp, trans_act, trans_temp;

    function new(string name="Scoreboard", uvm_component parent=null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        port_act = new("port_act", this);
        port_exp = new("port_exp", this);
    endfunction

    virtual task main_phase(uvm_phase phase);
        fork 
            get_exp();
            get_act_compare();
        join
    endtask

    task get_exp();
        forever begin
            port_exp.get(trans_exp);
            exp_queue.push_back(trans_exp);
        end
    endtask

    task get_act_compare();
        forever begin
            port_act.get(trans_act);
            wait(exp_queue.size()>0);
            trans_temp = exp_queue.pop_front();
            if (trans_temp.mode == `SLOW_MODE) begin
                if (trans_act.em_result == trans_temp.em_reading && trans_act.error_code == 1) begin
                    reg_model.data_reg.predict(trans_act.em_result);
                    `uvm_info("Scoreboard", "DATA MATCHED", UVM_LOW)
                end
                else begin
                    throw_error();
                end
            end
            else if (trans_temp.mode == `FAST_MODE) begin
                if (trans_act.em_result == reg_model.data_reg.get() && trans_act.error_code == 1) begin
                    `uvm_info("Scoreboard", "DATA MATCHED", UVM_LOW)
                end
                else begin
                    throw_error();
                end
            end
            $display("--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
            $display("--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
            $display("--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
        end
    endtask

    task throw_error();
        `uvm_info("Scoreboard", "Expected transaction:", UVM_LOW)
        trans_temp.print();
        `uvm_info("Scoreboard", "Actual transaction:", UVM_LOW)
        trans_act.print();
        `uvm_fatal("Scoreboard", "DATA MISMATCH!")
    endtask

endclass


class CovCollector extends uvm_component;
    // DUT input from agent_in and DUT output from agent_out
    `uvm_analysis_imp_decl(_port_in)
    `uvm_analysis_imp_decl(_port_out)
    `uvm_component_utils(CovCollector)

    Transaction trans_in, trans_out;
    
    uvm_analysis_imp_port_in # (Transaction, CovCollector) imp_in;
    uvm_analysis_imp_port_out # (Transaction, CovCollector) imp_out;
    
    covergroup cg_in;
        coverpoint trans_in.mode {
            ignore_bins igvalues = {0, 3};
        }
    endgroup

    covergroup cg_out;
        coverpoint trans_out.em_result {
            // The extreme cases are put in separated bins.
            bins min = {0};
            bins normal = {[1:254]};
            bins max = {255};
        }
        coverpoint trans_out.error_code {
            bins normal = {1};  // Only care about the nornal operation
            ignore_bins igvalues = {0, [2:7]};
        }
    endgroup
    
    function new(string name="CovCollector", uvm_component parent=null);
        super.new(name, parent);
        cg_in = new();
        cg_out = new();
    endfunction

    virtual function void build_phase(uvm_phase phase);
        trans_in = Transaction::type_id::create("trans_in");
        trans_out = Transaction::type_id::create("trans_out");
        imp_in = new("imp_in", this);
        imp_out = new("imp_out", this);
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
    `uvm_component_utils(Environment);
    Agent agent_in, agent_out;
    Scoreboard scoreboard;
    CovCollector cov_collector;
    RegModel reg_model;
    uvm_tlm_analysis_fifo #(Transaction) agti_scb_fifo, agto_scb_fifo;
    event sampling_event;

    function new(string name="Environment", uvm_component parent=null);
        super.new(name, parent);
    endfunction
    
    virtual function void build_phase(uvm_phase phase);
        agent_in = Agent::type_id::create("agent_in", this);
        agent_out = Agent::type_id::create("agent_out", this);
        scoreboard = Scoreboard::type_id::create("scoreboard", this);
        cov_collector = CovCollector::type_id::create("cov_collector", this);
        reg_model = RegModel::type_id::create("reg_model");
        reg_model.build();
        agent_in.is_active = UVM_ACTIVE;
        agent_out.is_active = UVM_PASSIVE;
        agti_scb_fifo = new("agti_scb_fifo", this);
        agto_scb_fifo = new("agto_scb_fifo", this);
    endfunction

    virtual function void connect_phase(uvm_phase phase);
        agent_in.ap_dri.connect(agti_scb_fifo.analysis_export);
        agent_in.ap_dri.connect(cov_collector.imp_in);
        scoreboard.port_exp.connect(agti_scb_fifo.blocking_get_export);

        agent_out.ap_mon.connect(agto_scb_fifo.analysis_export);
        agent_out.ap_mon.connect(cov_collector.imp_out);
        scoreboard.port_act.connect(agto_scb_fifo.blocking_get_export);

        agent_in.driver.sampling_event = sampling_event;
        agent_out.monitor.sampling_event = sampling_event;

        scoreboard.reg_model = this.reg_model;
    endfunction

    virtual function void end_of_elaboration_phase(uvm_phase phase);
        agent_in.monitor.set_report_verbosity_level(UVM_NONE);
    endfunction
endclass

class Test extends uvm_test;
    `uvm_component_utils(Test);
    Environment env;
    SlowTestSequence slow_test_sequence;
    FastTestSequence fast_test_sequence;
    RandomTestSequence random_test_sequence;

    function new(string name="Test", uvm_component parent=null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        env = Environment::type_id::create("env", this);
        slow_test_sequence = SlowTestSequence::type_id::create("slow_test_sequence");
        fast_test_sequence = FastTestSequence::type_id::create("fast_test_sequence");
        random_test_sequence = RandomTestSequence::type_id::create("random_test_sequence");
    endfunction

    virtual task reset_phase(uvm_phase phase);
        phase.raise_objection(this);
            #100;
        phase.drop_objection(this);
    endtask

    virtual task main_phase(uvm_phase phase);
        phase.raise_objection(this);
            slow_test_sequence.start(env.agent_in.sequencer);
            fast_test_sequence.start(env.agent_in.sequencer);
            random_test_sequence.start(env.agent_in.sequencer);
        phase.drop_objection(this);
    endtask
endclass

module TB_EM;
    logic clk;

    MyInterface itfc (clk);

    EM DUT (
        .Mode(itfc.Mode),
        .Enable(itfc.Enable),
        .EMReading(itfc.EMReading),
        .Clk(clk),
        .CPUReadComplete(itfc.CPUReadComplete),
        .EMResult(itfc.EMResult),
        .ErrorCode(itfc.ErrorCode),
        .EMValReady(itfc.EMValReady)
    );

    initial begin
        clk <= 0;
        #10000000000;
        $stop(); 
    end

    always #5 clk <= ~clk;

    initial begin
        uvm_config_db #(virtual MyInterface)::set(null, "uvm_test_top.env.agent_in.*", "vif", itfc);
        uvm_config_db #(virtual MyInterface)::set(null, "uvm_test_top.env.agent_out.*", "vif", itfc);
        run_test("Test");
    end

    initial begin
        $dumpfile("dump.vcd");
        $dumpvars;
    end
endmodule