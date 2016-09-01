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

module handle(clk, rst, convst, sck, sdi, sdo, led);

    localparam
        ADC_CONF_BITS = 6'b000000;

    input clk;
    input rst;
    output convst;
    output sck;
    output sdi;
    input sdo;
    output [7:0] led;

    wire ready;
    wire [11:0] data;

    reg start;
    reg [4:0] cnt;

    assign led = data[11:5];

    always @(posedge clk) begin        
        if (cnt) begin
            if (cnt == 8'd1 || cnt == 8'd2) begin
                start <= 1;
            end else begin
                start <= 0;
            end
            cnt <= cnt + 1;
        end else begin
            cnt <= 1;
            start <= 0;
        end
    end

    LTC2308DRV drv(.clk(clk)
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
    
endmodule

module hardinstance(clk, rst, convst, sck, sdi, sdo, led);

    input clk; /* synthesis chip_pin = "E20" */;
    input rst; /* synthesis chip_pin = "L10" */;
    output convst; /* synthesis chip_pin = "U9" */;
    output sck; /* synthesis chip_pin = "V10" */;
    output sdi; /* synthesis chip_pin = "AC4" */;
    input sdo; /* synthesis chip_pin = "AD4" */;
    output [7:0] led; /* synthesis chip_pin = "AA23, Y16, AE26, AF26, V15, V16, AA24, W15" */;

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

    wire [7:0] led;

    initial begin
        $dumpfile("spi2.vcd");
        // $dumpvars(0, clk, latch, d, out, ob, sin.bf, sin.bin, sout.bf);
        //$dumpvars(0, clk, rst, start, ready, conf, convst, sck, sdi, sdo, res, drv.hold, drv.going, drv.state);
        $dumpvars(0, clk, rst, convst, sck, sdi, sdo, led, hnd.start, hnd.cnt);
        
        #0 clk <= 0;
        #0 sdo <= 1;
        #0 rst <= 0;

        #5 rst <= 1;
        #15 rst <= 0;
        
        #2500 $finish;
    end

    always begin
        #20 clk <= ~clk;
    end

    handle hnd(.clk(clk)
        , .rst(rst)
        , .convst(convst)
        , .sck(sck)
        , .sdi(sdi)
        , .sdo(sdo)
        , .led(led)
    );

    
endmodule
