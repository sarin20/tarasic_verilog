module ONCHIPRAM(address,
    byteenable,
    chipselect,
    clk,
    clken,
    reset,
    reset_req,
    write,
    writedata,
    readdata                                            
);
    parameter
        D = 256
        , W = 64
        , AW = 8;

    input [AW - 1:0] address;
    input [7:0] byteenable;
    input chipselect;
    input clk;
    input clken;
    input reset;
    input reset_req;
    input write;
    input [W - 1:0] writedata;

    output [W - 1:0] readdata;


    integer c;
    reg [W - 1:0] ramblock [D - 1:0];

    assign readdata = ramblock[address];

    always @(posedge clk) begin
        if (reset) begin
            for (c = 0; c < D; c = c + 1) begin
                ramblock[c] <= 0;
            end
        end else if (write) begin
            if (address || address == 0) begin
                ramblock[address] <= writedata;
            end
        end
    end
endmodule

module LTC2308DRV(clk, rst, conf, start, res, ready, convst, sck, sdi, sdo);
    parameter w = 12;
    
    localparam
        STATE_START = 5'b1000x,
        STATE_STARTED = 5'bx0100,
        STATE_TRANSMITION_START = 5'bx110x,
        STATE_TRANSMITION = 5'bx0010;
        
    input clk;
    input [5:0] conf;
    reg [5:0] cr;
    input start;
    input rst;
    output reg [w - 1:0] res;
    output reg ready;
    reg hold;
    output reg convst;
    output reg sck;
    output reg sdi;
    input sdo;
    reg [3:0] bc;
    wire [4:0] state;
    reg going;
    reg pol;

    assign state = {start, hold, convst, going, ready};

    always @(posedge clk) begin
        if (rst) begin
            hold <= 0;
            convst <= 0;
            going <= 0;
            sck <= 0;
            res <= 0;
            sdi <= 0;
            ready <= 0;
            pol <= 0;
        end else if (rst == 1'b0) begin
            pol <= ~pol;
            if (pol) begin
                casex (state)
                    STATE_TRANSMITION: begin
                        sck <= 1;
                        res <= {res[w - 2:0], sdo};
                    end
                endcase
            end else if (pol == 1'b0) begin
                casex (state)
                    STATE_START: begin
                        convst <= 1;
                        bc <= 0;
                        sck <= 0;
                        going <= 0;
                        cr <= conf;
                        res <= 0;
                        ready <= 0;
                    end
                    STATE_STARTED: begin
                        hold <= 1;
                    end
                    STATE_TRANSMITION_START: begin
                        hold <= 0;
                        convst <= 0;
                        going <= 1;
                        bc <= bc + 1;
                        sdi <= cr[5];
                        cr <= {cr[4:0], cr[5]};
                    end
                    STATE_TRANSMITION: begin
                        sck <= 0;
                        bc <= bc + 1;
                        if (bc == 12) begin
                            ready <= 1;
                        end else if (bc < 6) begin
                            sdi <= cr[5];
                            cr <= {cr[4:0], cr[5]};
                        end else begin
                            sdi <= 0;
                        end
                    end
                    default: begin
                        going <= 0;
                    end
                endcase
            end
        end
    end
endmodule

module counter(clk, rst, out);
    parameter
        W = 8;

    input clk;
    input rst;
    output reg [W - 1:0] out;

    always @(posedge clk) begin
        if (rst) begin
            out <= 1'b0;
        end else begin
            out <= out + 8'b1;
        end
    end
endmodule

module acounter(clk, rst, out);
    parameter
        W = 8;

    input clk;
    input rst;
    output reg [W - 1:0] out;

    always @(negedge clk) begin
        if (rst) begin
            out <= 0;
        end else begin
            out <= out + 1'b1;
        end
    end
endmodule

module databufer(clk, adcin, cntin, hold, out);
    parameter
        W = 32;

    input clk;
    input [(W / 2) - 1:0] adcin;
    input [(W / 2) - 1:0] cntin;
    input hold;
    output reg [W - 1:0] out;
    
    always @(posedge clk) begin
        if (hold == 1'b0) begin
            out <= {cntin, adcin};
        end
    end
endmodule

module handle(clk, rst, next, opmode, resmux, convst, sck, sdi, sdo, led, address, dataout, write, datain);
    parameter
        WORD_LEN = 32,
        ADDR_W = 8,
        ADC_CONF_BITS = 6'b000000;

    localparam
        ADC_MODE = 2'b00,
        RAM_MODE = 2'b10;

    input clk;
    input rst;
    input next;
    input [1:0] opmode;
    input [1:0] resmux;
    output convst;
    output sck;
    output sdi;
    input sdo;
    output [7:0] led;
    output [7:0] ledq;
    output [7:0] ledw;

    wire [WORD_LEN - 1:0] word;
    wire [WORD_LEN - 1:0] q;
    output [ADDR_W - 1:0] address;
    output [63:0] dataout;
    output write;
    input [63:0] datain;

    wire ready;
    wire [11:0] data;
    reg start;
    wire [5:0] cnt;    
    wire [63:0] cnt_out;
    wire hold;

    assign ledq = resmux[1] ? (resmux[0] ? datain[31:24] : datain[23:16]) : (resmux[0] ? datain[15:8] : datain[7:0]);
    assign ledw = resmux[1] ? (resmux[0] ? word[31:24] : word[23:16]) : (resmux[0] ? word[15:8] : word[7:0]);
    assign led = opmode[1] ? ledq : ledw;
    assign cnt = cnt_out[5:0];
    assign dataout = {16'b0, word};
    assign write = ready;

    always @(posedge clk) begin
        if (rst == 1'b0) begin
            if (cnt) begin
                if (cnt == 8'd1 || cnt == 8'd2) begin
                    start <= 1;                    
                end else begin
                    start <= 0;
                end
            end else begin
                start <= 0;                
            end
            
        end else if (rst) begin

        end
    end

    LTC2308DRV drv(
        .clk(clk)
        , .rst(rst)
        , .conf(ADC_CONF_BITS)
        , .start(start)
        , .res(data)
        , .ready(ready)
        , .convst(convst)
        , .sck(sck)
        , .sdi(sdi)
        , .sdo(sdo)
    );

    databufer dbf(
        .clk(clk)
        , .adcin({4'b0000, data})
        , .cntin(cnt_out[15:0])
        , .hold(next)
        , .out(word)
    );

    defparam cnt_inst.W = 64;

    counter cnt_inst(
        .clk(clk)
        , .rst(rst)
        , .out(cnt_out)
    );

    acounter addr_cnt(
        .clk(next)
        , .rst(rst)
        , .out(address)
    );

/*    ONCHIPRAM ram (
        .address(address),
        .rst(rst),
        .clock(clk),
        .data(word),
        .wren((opmode[1] == 0)),
        .q(q)
    ); */
    
endmodule

module test();
    reg clk;
    reg gres;
    reg next;
    reg [1:0] opmode;
    reg [1:0] resmux;
    
    output convst;
    output sck;
    output sdi;
    reg sdo;
    output [7:0] led;

    initial begin        
        $dumpfile("mem.vcd");
        $dumpvars(0, clk, gres, next, opmode, resmux, convst, led, h.start, h.address, h.q, h.ledq, h.ledw, h.drv.ready, h.drv.res, h.addr_cnt.rst);
        #0 opmode <= 2'b00;
        #0 resmux <= 2'b00;
        #0 next <= 0;
        #0 sdo <= 1;
        #0 clk <= 0;
        #00 gres <= 1;
        #80 gres <= 0;
        
        #250 next <= 1;
        #40 next <= 0;

        #250 next <= 1;
        #40 next <= 0;

        #250 next <= 1;
        #40 next <= 0;

        #250 next <= 1;
        #40 next <= 0;
        #0 opmode <= 2'b10;
        #100 gres <= 1;
        #20 next <= 1;
        #20 next <= 0;
        #40 gres <= 0;

        #40 resmux <= 2'b00;
        #40 resmux <= 2'b01;
        #40 resmux <= 2'b10;
        #40 resmux <= 2'b11;
        #40 next <= 1;
        #40 next <= 0;

        #40 resmux <= 2'b00;
        #40 resmux <= 2'b01;
        #40 resmux <= 2'b10;
        #40 resmux <= 2'b11;
        #40 next <= 1;
        #40 next <= 0;

        #40 resmux <= 2'b00;
        #40 resmux <= 2'b01;
        #40 resmux <= 2'b10;
        #40 resmux <= 2'b11;
        #40 next <= 1;
        #40 next <= 0;

        #40 resmux <= 2'b00;
        #40 resmux <= 2'b01;
        #40 resmux <= 2'b10;
        #40 resmux <= 2'b11;
        #40 next <= 1;
        #40 next <= 0;

        #200 $finish;
    end

    always begin
        #10 clk <= ~clk;
    end

    wire [7:0] address;
    wire [63:0] dataout;
    wire [63:0] datain;
    wire write;

    handle h(
        .clk(clk)
        , .rst(gres)
        , .next(next)
        , .opmode(opmode)
        , .resmux(resmux)
        , .convst(convst)
        , .sck(sck)
        , .sdi(sdi)
        , .sdo(sdo)
        , .led(led)
        , .address(address)
        , .dataout(dataout)
        , .write(write)
        , .datain(datain)
    );

    ONCHIPRAM m(
        .address(address)
        , .byteenable(8'b1111_1111)
        , .chipselect(1'b1)
        , .clk(clk)
        , .clken(1'b1)
        , .reset(gres)
        , .reset_req(1'b0)
        , .write(write)
        , .writedata(dataout)
        , .readdata(datain)
    );

endmodule
