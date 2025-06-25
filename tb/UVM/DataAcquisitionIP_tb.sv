`include "uvm_macros.svh"
import uvm_pkg::*;
`define ROSC 1
`define ANALOG 2
`define EM 3
`define TDDB 4
`define SILC 5
`define TEMP 6
`define VOLT 7
`define FAST_MODE 1
`define SLOW_MODE 2

interface MyInterface (input clk);
    logic enable;
    logic [31:0] CPUCommand, ResultForCPU;
    logic [7:0][15:0] SensorReadings;
endinterface

class Transaction extends uvm_sequence_item;
    `uvm_object_utils(Transaction)

    randc bit [1:0] mode;
    randc bit [2:0] module_select;
    randc bit [3:0] rosc_tddb_cycle;
    randc bit [11:0] analog_adc;
    randc bit [7:0] em_reading;
    randc bit [5:0] silc_cycle;
    randc bit [11:0] silc_threshold;
    randc bit [11:0] temp_adc;
    randc bit [11:0] volt_adc;
    randc int expected_silc_time;
    randc int rosc_tddb_freq_ratio;

    bit [15:0] result;
    bit [2:0] error_code;

    function new(string name = "Transaction");
        super.new(name);
    endfunction

    constraint slow_test_constraint {
        mode == `SLOW_MODE;
        module_select inside {`ROSC, `ANALOG, `EM, `TDDB, `SILC, `TEMP, `VOLT};
        rosc_tddb_cycle > 0;
        silc_cycle inside {[1:19]};
        silc_threshold == {12{1'b1}};
        expected_silc_time inside {[10:64]};
        rosc_tddb_freq_ratio inside {1, 2, 5, 10}; // Multiple of 10, because the clock's half period is 10
    };

    constraint fast_test_constraint {
        mode == `FAST_MODE;
        module_select inside {`ROSC, `ANALOG, `EM, `TDDB, `SILC, `TEMP, `VOLT};
    };

    constraint random_test_constraint {
        mode inside {`FAST_MODE, `SLOW_MODE};
        module_select inside {`ROSC, `ANALOG, `EM, `TDDB, `SILC, `TEMP, `VOLT};
        rosc_tddb_cycle > 0;
        silc_cycle inside {[1:19]};
        silc_threshold == {12{1'b1}};
        expected_silc_time inside {[10:64]};
        rosc_tddb_freq_ratio inside {1, 2, 5, 10};
    }

    constraint corner_test_constraint {
        mode == `SLOW_MODE;
        silc_cycle inside {[1:19]};
        silc_threshold == {12{1'b1}};
        expected_silc_time inside {[10:64]};
        rosc_tddb_freq_ratio inside {1, 2, 5, 10};
    }

    virtual function void do_print(uvm_printer printer);
        super.do_print(printer);
        printer.print_field_int("expected_silc_time", expected_silc_time, $bits(expected_silc_time), UVM_DEC);
        printer.print_field_int("mode", mode, $bits(mode), UVM_DEC);
        printer.print_field_int("module_select", module_select, $bits(module_select), UVM_DEC);
        printer.print_field_int("rosc_tddb_cycle", rosc_tddb_cycle, $bits(rosc_tddb_cycle), UVM_DEC);
        printer.print_field_int("silc_cycle", silc_cycle, $bits(silc_cycle), UVM_DEC);
        printer.print_field_int("silc_threshold", silc_threshold, $bits(silc_threshold), UVM_DEC);
        printer.print_field_int("analog_adc", analog_adc, $bits(analog_adc), UVM_DEC);
        printer.print_field_int("em_reading", em_reading, $bits(em_reading), UVM_DEC);
        printer.print_field_int("temp_adc", temp_adc, $bits(temp_adc), UVM_DEC);
        printer.print_field_int("volt_adc", volt_adc, $bits(volt_adc), UVM_DEC);
        printer.print_field_int("rosc_tddb_freq_ratio", rosc_tddb_freq_ratio, $bits(rosc_tddb_freq_ratio), UVM_DEC);
        printer.print_field_int("result", result, $bits(result), UVM_DEC);
        printer.print_field_int("error_code", error_code, $bits(error_code), UVM_DEC);
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
        repeat(2000) begin
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
        repeat(50) begin
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
        repeat(10000) begin
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
        silc_zero_cycle();
        silc_fast_adc();
        rosc_extreme_cycle(0);
        rosc_extreme_cycle(4'b1111);
    endtask

    task rosc_extreme_cycle(int num_cycle);
        start_item(trans);
        trans.randomize();
        trans.rosc_tddb_cycle = num_cycle;
        trans.module_select = `ROSC;
        finish_item(trans);
    endtask

    task silc_zero_cycle();
        start_item(trans);
        trans.randomize();
        trans.silc_cycle = 0;
        trans.module_select = `SILC;
        finish_item(trans);
    endtask

    task silc_fast_adc();
        start_item(trans);
        trans.randomize();
        trans.expected_silc_time = 1;
        trans.module_select = `SILC;
        finish_item(trans);
    endtask
endclass

class ErrorTestSequence extends uvm_sequence #(Transaction);  // Test the time-out feature of SILC
    `uvm_object_utils(ErrorTestSequence)
    Transaction trans;
    function new(string name = "ErrorTestSequence");
        super.new(name);
        trans = Transaction::type_id::create("trans");
    endfunction

    virtual task body();
        trans.slow_test_constraint.constraint_mode(1);
        trans.fast_test_constraint.constraint_mode(0);
        trans.random_test_constraint.constraint_mode(0);
        trans.corner_test_constraint.constraint_mode(0);
        start_item(trans);
        trans.randomize();
        trans.silc_threshold = 0;
        trans.module_select = `SILC;
        finish_item(trans);
    endtask
endclass

class Driver extends uvm_driver #(Transaction);
    `uvm_component_utils(Driver)
    Transaction trans;
    virtual MyInterface vif;
    //SILC_Driver silc_driver;
    uvm_analysis_port #(Transaction) ap;
    event sampling_event;
    int rand_delay;
    const int delta_adc = ({12{1'b1}} >> 1);  // (upper limit + 1) - (lower limit -1)

    function new(string name = "Driver", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        if (!uvm_config_db #(virtual MyInterface)::get(this, "", "vif", vif)) begin
            `uvm_fatal("Driver", "Cannot access to the virtual interface!")
        end
        trans = Transaction::type_id::create("trans");
        ap = new("ap", this);
    endfunction

    virtual task reset_phase(uvm_phase phase);
        vif.enable <= 0;
        vif.CPUCommand <= 0;
        for (int i=0; i<8; i++) begin
            vif.SensorReadings[i] <= 0;
        end
        #100;
        vif.enable <= 1;
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
            `uvm_fatal("Driver", "Incorrect mode!")
        end
        seq_item_port.item_done();
    endtask

    task fast_mode();
        drive_command();
        wait(vif.ResultForCPU[0] == 1);
        -> sampling_event;
        rand_delay = $urandom_range(10, 0);
        repeat(rand_delay) @(posedge vif.clk);
        drive_command_reset();
        wait(vif.ResultForCPU[0] == 0);
    endtask

    task slow_mode();
        fork
            drive_command();
            case (trans.module_select)
                `ROSC: drive_rosc_tddb_sensor();
                `ANALOG: drive_other_signals();
                `EM: drive_other_signals();
                `TDDB: drive_rosc_tddb_sensor();
                `SILC: drive_silc_adc();
                `TEMP: drive_other_signals();
                `VOLT: drive_other_signals();
                default: `uvm_fatal("Driver", "Incorrect module select!")
            endcase
        join_none
        @(posedge vif.ResultForCPU[0]);
        disable fork;
        -> sampling_event;
        rand_delay = $urandom_range(10, 0);
        repeat(rand_delay) @(posedge vif.clk);
        drive_command_reset();
        wait(vif.ResultForCPU[0] == 0);
    endtask

    task drive_command_reset();
        @(posedge vif.clk);
        vif.CPUCommand <= 0;
    endtask

    task drive_command();
        @(posedge vif.clk);
        vif.CPUCommand[31:30] <= trans.mode;
        vif.CPUCommand[29:27] <= trans.module_select;
        vif.CPUCommand[23:20] <= trans.rosc_tddb_cycle;
        vif.CPUCommand[19:14] <= trans.silc_cycle;
        vif.CPUCommand[13:2] <= trans.silc_threshold;
    endtask

    task drive_rosc_tddb_sensor();
        bit sensor_value = 0;
        @(posedge vif.clk);
        vif.SensorReadings[trans.module_select] <= sensor_value;
        forever begin
            sensor_value = ~sensor_value;
            #(10/trans.rosc_tddb_freq_ratio);
            vif.SensorReadings[trans.module_select] <= sensor_value;
        end
    endtask

    task drive_other_signals();  // General drive task for EM, Analog, Temperature, and Voltage
        @(posedge vif.clk);
        vif.SensorReadings[`ANALOG] <=  trans.analog_adc;
        vif.SensorReadings[`EM] <=  trans.em_reading;
        vif.SensorReadings[`TEMP] <= trans.temp_adc;
        vif.SensorReadings[`VOLT] <= trans.volt_adc;
    endtask

    task drive_silc_adc();  // For SILC
        int adc;
        int dcrmt_adc;
        dcrmt_adc = delta_adc / trans.expected_silc_time; // The decrement of ADC for each cycle, has rounding error
        forever begin
            adc = 12'b111111111111;
            @(posedge vif.clk);
            vif.SensorReadings[`SILC] <= adc;
            while (adc >= 0) begin
                adc = adc - dcrmt_adc;
                if (adc < 0) begin
                    break;
                end
                @(posedge vif.clk);
                vif.SensorReadings[`SILC] <= adc;  // Refer to the RTL code, SILCADCReading = SensorReadings[5][11:0];
            end
        end
    endtask

endclass

class Monitor extends uvm_monitor;
    `uvm_component_utils(Monitor)
    virtual MyInterface vif;
    Transaction trans;
    uvm_analysis_port #(Transaction) ap;
    event sampling_event;

    function new(string name = "Monitor", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase (uvm_phase phase);
        if(!uvm_config_db #(virtual MyInterface)::get(this, "", "vif", vif)) begin
            `uvm_fatal("Monitor", "Cannot access to the virtual interface!")
        end
        ap = new("ap", this);
        trans = Transaction::type_id::create("trans");
    endfunction

    virtual task reset_phase (uvm_phase phase);
        forever begin
            @(posedge vif.clk);  // Do nothing
        end
    endtask

    virtual task main_phase (uvm_phase phase);
        forever begin
            mon();
        end
    endtask

    task mon();
        @(sampling_event);
        trans.result = vif.ResultForCPU[31:16];
        trans.error_code = vif.ResultForCPU[15:13];
        `uvm_info("Monitor", "Actual Transaction sent to the Scoreboard", UVM_LOW)
        `uvm_info("Monitor", $sformatf(trans.sprint()), UVM_LOW)
        ap.write(trans);
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
        monitor = Monitor::type_id::create("monitor", this);
        if (is_active == UVM_ACTIVE) begin
            driver = Driver::type_id::create("driver", this);
            sequencer = uvm_sequencer #(Transaction)::type_id::create("sequencer", this);
        end
    endfunction

    virtual function void connect_phase(uvm_phase phase);
        ap_mon = monitor.ap;
        if (is_active == UVM_ACTIVE) begin
            driver.seq_item_port.connect(sequencer.seq_item_export);
            ap_dri = driver.ap;
        end
    endfunction

endclass

class Model extends uvm_component;
    `uvm_component_utils(Model)
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
        trans_exp.module_select = trans.module_select;
        if(trans.mode == `FAST_MODE) begin
            trans_exp.error_code = 1;
        end
        else if (trans.mode == `SLOW_MODE) begin
            case (trans.module_select)
                `ROSC: rosc_tddb_exp();
                `ANALOG: other_exp();
                `EM: other_exp();
                `TDDB: rosc_tddb_exp();
                `SILC: silc_exp();
                `TEMP: other_exp();
                `VOLT: other_exp();
                default: `uvm_fatal("Model", "Incorrect module!")
            endcase
        end
    endtask

    task rosc_tddb_exp();
        trans_exp.result = trans.rosc_tddb_cycle * trans.rosc_tddb_freq_ratio;
        trans_exp.error_code = 1;
    endtask

    task other_exp();
        case(trans.module_select)
            `ANALOG: trans_exp.result = trans.analog_adc;
            `EM: trans_exp.result = trans.em_reading;
            `TEMP: trans_exp.result = trans.temp_adc;
            `VOLT: trans_exp.result = trans.volt_adc;
        endcase
        trans_exp.error_code = 1;
    endtask

    task silc_exp();
        if (trans.expected_silc_time * trans.silc_cycle < trans.silc_threshold) begin  // No time-out
            trans_exp.silc_cycle = trans.silc_cycle;
            trans_exp.expected_silc_time = trans.expected_silc_time;
            trans_exp.silc_threshold = trans.silc_threshold;
            trans_exp.result = trans.silc_cycle * trans.expected_silc_time;
            trans_exp.error_code = 1;
        end
        else begin
            trans_exp.silc_cycle = trans.silc_cycle;
            trans_exp.expected_silc_time = trans.expected_silc_time;
            trans_exp.silc_threshold = trans.silc_threshold;
            trans_exp.error_code = 7;  // Time-out
        end
    endtask

endclass

class Scoreboard extends uvm_scoreboard;
    `uvm_component_utils(Scoreboard)
    uvm_blocking_get_port #(Transaction) port_exp, port_act;
    Transaction exp_queue[$];
    Transaction trans_exp, trans_act, trans_temp;
    bit [15:0] previous_reading [8];

    function new(string name = "Scoreboard", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase (uvm_phase phase);
        port_exp = new("port_exp", this);
        port_act = new("port_act", this);
    endfunction

    virtual function void end_of_elaboration_phase (uvm_phase phase);
        for (int i=0; i<8; i++) begin
            previous_reading[i] = 0;
        end
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
            if (trans_temp.module_select == `SILC) begin
                silc_compare();
            end
            else begin
                general_compare();
            end
            `uvm_info ("Scoreboard", "DATA MATCHED!", UVM_LOW)
            $display("--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
            $display("--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
            $display("--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
        end
    endtask

    task general_compare();
        if (trans_temp.mode == `SLOW_MODE) begin
            assert(trans_act.result == trans_temp.result && trans_act.error_code == trans_temp.error_code) else throw_error();
            previous_reading[trans_temp.module_select] = trans_act.result;
        end
        else if (trans_temp.mode == `FAST_MODE) begin
            assert(trans_act.result == previous_reading[trans_temp.module_select] && trans_act.error_code == trans_temp.error_code) else throw_error();
        end
        else begin
            `uvm_fatal("Scoreboard", "Incorrect mode!")
        end
    endtask

    task silc_compare();  // SILC is special as a range calculation is needed instead of precise values.
        int max_silc_reading;
        
        if (trans_temp.mode == `SLOW_MODE) begin
            if (trans_temp.expected_silc_time * trans_temp.silc_cycle < trans_temp.silc_threshold) begin  // No time-out
                max_silc_reading = calculate_max_silc_reading(trans_temp.expected_silc_time, trans_temp.silc_cycle);
                assert(trans_act.result >= trans_temp.result && trans_act.result <= max_silc_reading && trans_act.error_code == trans_temp.error_code) else throw_error();
                previous_reading[`SILC] = trans_act.result;
            end
            else begin  // Time-out
                assert(trans_act.error_code == trans_temp.error_code) else throw_error();
            end
        end
        else if (trans_temp.mode == `FAST_MODE)begin
            //$display("previous reading = %0d", previous_reading[`SILC]);
            assert(trans_act.result == previous_reading[`SILC] && trans_act.error_code == trans_temp.error_code) else throw_error();
        end
        else begin
            `uvm_fatal("Scoreboard", "Incorrect mode!")
        end
    endtask

    function real calculate_max_silc_reading(int expected_slope_time, int NumDescendingSlopes);
        const int delta_adc = ({12{1'b1}} >> 1); // ADC for SILC
        real max_slope_time = delta_adc/((delta_adc/expected_slope_time)-1);
        return max_slope_time * NumDescendingSlopes;
    endfunction

    task throw_error();
        `uvm_info("Scoreboard", "Expected transaction:", UVM_LOW)
        trans_temp.print();
        `uvm_info("Scoreboard", "Actual transaction:", UVM_LOW)
        trans_act.print();
        `uvm_fatal("Scoreboard", "DATA MISMATCH!")
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
        coverpoint trans_in.module_select {
            bins rosc = {1};
            bins analog = {2};
            bins em = {3};
            bins tddb = {4};
            bins silc = {5};
            bins temp = {6};
            bins volt = {7};
        }
        coverpoint trans_in.rosc_tddb_cycle {
            bins min = {0};
            bins max = {15};
            bins normal = {[1:14]};
        }
        coverpoint trans_in.analog_adc {
            bins min = {0};
            bins normal = {[1:4094]};
            bins max = {4095};
        }
        coverpoint trans_in.em_reading {
            bins min = {0};
            bins normal = {[1:254]};
            bins max = {255};
        }
        coverpoint trans_in.silc_cycle {
            bins min = {0};
            bins max = {32};
            bins normal = {[1:31]};
        }
        coverpoint trans_in.silc_threshold {
            bins normal = {[0:4095]};
        }
        coverpoint trans_in.temp_adc {
            bins min = {0};
            bins normal = {[1:4094]};
            bins max = {4095};
        }
        coverpoint trans_in.volt_adc {
            bins min = {0};
            bins normal = {[1:4094]};
            bins max = {4095};
        }
        coverpoint trans_in.expected_silc_time {
            bins fast = {1};
            bins noraml = {[2:64]};
        }
        coverpoint trans_in.rosc_tddb_freq_ratio {
            bins bin[] = {1, 2, 5, 10};
        }
    endgroup

    covergroup cg_out;
        coverpoint trans_out.error_code {
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
    Scoreboard scoreboard;
    CovCollector cov_collector;
    Agent agent_in, agent_out;
    Model model;
    uvm_tlm_analysis_fifo #(Transaction) mdl_scb_fifo, agto_scb_fifo, agti_mdl_fifo;
    event sampling_event;

    function new(string name = "Environment", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase (uvm_phase phase);
        scoreboard = Scoreboard::type_id::create("scoreboard", this);
        cov_collector = CovCollector::type_id::create("cov_collector", this);
        agent_in = Agent::type_id::create("agent_in", this);
        agent_out = Agent::type_id::create("agent_out", this);
        model = Model::type_id::create("model", this);
        agent_in.is_active = UVM_ACTIVE;
        agent_out.is_active = UVM_PASSIVE;
        mdl_scb_fifo = new("mdl_scb_fifo", this);
        agti_mdl_fifo = new("agti_mdl_fifo", this);
        agto_scb_fifo = new("agto_scb_fifo", this);
    endfunction

    virtual function void connect_phase (uvm_phase phase);
        agent_in.ap_dri.connect(agti_mdl_fifo.analysis_export);
        agent_in.ap_dri.connect(cov_collector.imp_in);
        model.port.connect(agti_mdl_fifo.blocking_get_export);
        agent_out.ap_mon.connect(agto_scb_fifo.analysis_export);
        agent_out.ap_mon.connect(cov_collector.imp_out);
        scoreboard.port_act.connect(agto_scb_fifo.blocking_get_export);
        model.ap.connect(mdl_scb_fifo.analysis_export);
        scoreboard.port_exp.connect(mdl_scb_fifo.blocking_get_export);
        agent_in.driver.sampling_event = sampling_event;
        agent_out.monitor.sampling_event = sampling_event;
    endfunction

    virtual function void end_of_elaboration_phase (uvm_phase phase);
        agent_in.monitor.set_report_verbosity_level(UVM_NONE);
    endfunction

endclass

class Test extends uvm_test;
    `uvm_component_utils(Test)
    Environment env;
    SlowTestSequence slow_test_sequence;
    FastTestSequence fast_test_sequence;
    RandomTestSequence random_test_sequence;
    CornerTestSequence corner_test_sequence;
    ErrorTestSequence error_test_sequence;

    function new(string name = "Test", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase (uvm_phase phase);
        env = Environment::type_id::create("env", this);
        slow_test_sequence = SlowTestSequence::type_id::create("slow_test_sequence");
        fast_test_sequence = FastTestSequence::type_id::create("fast_test_sequence");
        random_test_sequence = RandomTestSequence::type_id::create("random_test_sequence");
        corner_test_sequence = CornerTestSequence::type_id::create("corner_test_sequence");
        error_test_sequence = ErrorTestSequence::type_id::create("error_test_sequence");
    endfunction

    virtual task reset_phase (uvm_phase phase);  // Raise the objection of the reset phase here for driver and monitor
        phase.raise_objection(this);
        #100;
        phase.drop_objection(this);
    endtask

    virtual task main_phase (uvm_phase phase);
        phase.raise_objection(this);
        slow_test_sequence.start(env.agent_in.sequencer);
        fast_test_sequence.start(env.agent_in.sequencer);
        random_test_sequence.start(env.agent_in.sequencer);
        corner_test_sequence.start(env.agent_in.sequencer);
        error_test_sequence.start(env.agent_in.sequencer);
        phase.drop_objection(this);
    endtask
endclass

module TB_IP;

    logic clk;
    MyInterface itfc (clk);

    DataAcquisitionIP DUT(
        .Clk(clk),
        .En(itfc.enable),
        .CPUCommand(itfc.CPUCommand),
        .SensorReadings(itfc.SensorReadings),
        .ResultForCPU(itfc.ResultForCPU)
    );

    initial begin
        clk <= 0; 
        #1000000000;
        //#10000;
        $stop(); 
    end

    always #10 clk <= ~clk;

    initial begin
        uvm_config_db #(virtual MyInterface)::set(null, "uvm_test_top.env.agent_in.*", "vif", itfc);
        uvm_config_db #(virtual MyInterface)::set(null, "uvm_test_top.env.agent_out.*", "vif", itfc);
        run_test("Test");
    end

    initial begin
        $dumpfile("dump.vcd");
        $dumpvars;
    end

    /*
    APB assertions
    */

    // ALL signals must not be unknown
    assert property (@(posedge DUT.APBWrite.PCLK) disable iff (!DUT.En)
        (!$isunknown(DUT.APBWrite.PENABLE) and 
        !$isunknown(DUT.APBWrite.PWRITE) and 
        !$isunknown(DUT.APBWrite.PENABLE) and
        !$isunknown(DUT.APBWrite.PWDATA) and
        !$isunknown(DUT.APBWrite.PREADY) and
        !$isunknown(DUT.APBWrite.PSLVERR))) 
    else $error("ERROR: assertion no_wr_unknown failed at %0t", $time);

    assert property (@(posedge DUT.APBRead.PCLK) disable iff (!DUT.En)
        (!$isunknown(DUT.APBRead.PENABLE) and 
        !$isunknown(DUT.APBRead.PWRITE) and 
        !$isunknown(DUT.APBRead.PENABLE) and
        !$isunknown(DUT.APBRead.PRDATA) and
        !$isunknown(DUT.APBRead.PREADY) and
        !$isunknown(DUT.APBRead.PSLVERR)))
    else $error("ERROR: assertion no_wr_unknown failed at %0t", $time);

    // If PENABLE is asserted, it must be kept high, until PREADY is asserted
    assert property (@(posedge DUT.APBWrite.PCLK) disable iff (!DUT.En)
        $rose(DUT.APBWrite.PENABLE) |=> $stable(DUT.APBWrite.PENABLE) [*1:$] ##1 $rose(DUT.APBWrite.PREADY)) 
    else $error("ERROR: assertion wr_en_rdy failed at %0t", $time);

    assert property (@(posedge DUT.APBRead.PCLK) disable iff (!DUT.En)
        $rose(DUT.APBRead.PENABLE) |=> $stable(DUT.APBRead.PENABLE) [*1:$] ##1 $rose(DUT.APBRead.PREADY)) 
    else $error("ERROR: assertion rd_en_rdy failed at %0t", $time);
    
    // After PREADY is asserted, PENABLE must be deasserted
    assert property (@(posedge DUT.APBWrite.PCLK) disable iff (!DUT.En)
        $rose(DUT.APBWrite.PREADY) |=> $fell(DUT.APBWrite.PENABLE)) 
    else $error("ERROR: assertion wr_en_deassert failed at %0t", $time);

    assert property ( @(posedge DUT.APBRead.PCLK) disable iff (!DUT.En)
        $rose(DUT.APBRead.PREADY) |=> $fell(DUT.APBRead.PENABLE)) 
    else $error("ERROR: assertion rd_en_deassert failed at %0t", $time);

    // PADDR, PWRITE, and PWDATA must not change while PENABLE is asserted in a write transaction
    assert property (@(posedge DUT.APBWrite.PCLK) disable iff (!DUT.En)
        $rose(DUT.APBWrite.PENABLE) |=> ($stable(DUT.APBWrite.PADDR) and $stable(DUT.APBWrite.PWDATA) and $stable(DUT.APBWrite.PWRITE)) [*1:$] ##1 $rose(DUT.APBWrite.PREADY)) 
    else $error("ERROR: assertion wr_stbl_sig failed at %0t", $time);

    // PADDR and PWRITE must not change while PENABLE is asserted in a read transaction
    assert property (@(posedge DUT.APBRead.PCLK) disable iff (!DUT.En)
        $rose(DUT.APBRead.PENABLE) |=> ($stable(DUT.APBRead.PADDR) and $stable(DUT.APBRead.PWRITE)) [*1:$] ##1 $rose(DUT.APBRead.PREADY))
    else $error("ERROR: assertion rd_stbl_sig failed at %0t", $time);

endmodule