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
