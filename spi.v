module shiftIn(clk, latch, bin, out);
    parameter w = 8;

    input clk;
    input latch;
    input bin;
    output reg [w - 1:0] out;
    reg [w - 1:0] bf;

    always @(negedge clk) begin
        if (latch == 1'b0) begin
            bf <= {bf[w - 2:0], bin};
        end else if (latch == 1'b1) begin
            out <= bf;
        end
    end

endmodule

module shiftOut(clk, latch, in, out);
    parameter w = 8;

    input clk;
    input latch;
    input [w - 1:0] in;    
    output reg out;
    reg [w - 1:0] bf;
  
    always @(posedge clk) begin
        if (latch == 1'b0) begin
            out <= bf[w - 1];
            bf <= {bf[w - 2:0], 1'b0};
        end else if (latch == 1'b1) begin
            bf <= in;
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
        end else begin
            casex (state)  
                
                STATE_TRANSMITION: begin
                    sck <= 1;
                    res <= {res[w - 2:0], sdo};
                end
                default: begin
                    
                end
            endcase
        end
    end

    always @(negedge clk) begin
        if (rst == 1'b0) begin
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
    

endmodule

module test();

    reg clk;
    reg latch;
    reg bin;
    wire [7:0] out;
    wire ob;
    reg [7:0] d;    

    reg [5:0] conf;
    reg start;
    wire [11:0] res;
    wire ready;
    wire convst;
    wire sck;
    reg rst;
    wire sdi;
    reg sdo;

    initial begin
        $dumpfile("spi2.vcd");
        // $dumpvars(0, clk, latch, d, out, ob, sin.bf, sin.bin, sout.bf);
        $dumpvars(0, clk, rst, start, ready, conf, convst, sck, sdi, sdo, res, drv.hold, drv.going, drv.state);
        
        #0 clk <= 0;
        #0 start <= 0;
        #0 rst <= 1;
        #0 conf <= 6'b100001;
        #0 sdo <= 1;

        #5 rst <= 1;
        #15 rst <= 0;

        #25 start <= 1;
        #20 start <= 0;

        
        /*#0 latch <= 0;
        #0 bin <= 0;
        #10 d <= 8'b11110000;

        #11 latch <= 1;
        #21 latch <= 0;
        #150 d <= 8'b10101010;
        #11 latch <= 1;

        

        #21 latch <= 0;
        #162 latch <= 1;
        #15 latch <= 0;
        #200 latch <= 1;
        #15 latch <= 0;
        #174 latch <= 1;
        #15 latch <= 0;*/
        #400 start <= 1;
        #100 start <= 0;
        #300 $finish;
    end

    always begin
        #10 clk <= ~clk;
    end

    shiftIn sin(.clk(clk), .latch(latch), .bin(ob), .out(out));
    shiftOut sout(.clk(clk), .latch(latch), .in(d), .out(ob));

    LTC2308DRV drv(.clk(clk), .rst(rst), .conf(conf), .start(start), .res(res), .ready(ready), .convst(convst), .sck(sck), .sdi(sdi), .sdo(sdo));

endmodule