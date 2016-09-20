module counter(clk, rst, out);
    parameter
        w = 64;

    input clk;
    input rst;
    output reg [w - 1: 0] out;

    always @(posedge clk) begin
        if (rst) begin
            out <= 64'b0;
        end else if (rst == 1'b0) begin
            out <= out + 1'b1;
        end
    end

endmodule

module handle(clk, rst, convst, sck, sdi, sdo, led);
    parameter
        WORD_LEN = 32,
        ADDR_W = 6,
        ADC_CONF_BITS = 6'b000000;

    input clk;
    input rst;
    output convst;
    output sck;
    output sdi;
    input sdo;
    output [7:0] led;

    reg [WORD_LEN - 1:0] word;
    reg [ADDR_W - 1:0] address;
    reg wren;
    wire ready;
    wire [11:0] data;
    reg start;
    reg [4:0] cnt;    
    wire [63:0] cnt_out;

    assign led = data[11:4];

    always @(posedge clk) begin
        if (rst == 1'b0) begin
            if (cnt) begin
                if (cnt == 8'd1 || cnt == 8'd2) begin
                    start <= 1;
                    wren <= 0;
                end else begin
                    start <= 0;
                end
                cnt <= cnt + 1;
            end else begin
                cnt <= 1;
                start <= 0;                
            end
            if (ready == 1'b1 && (wren == 1'b0)) begin
                word <= {cnt_out[15:0], 4'b0000, data};
                address <= address + 1'b1;
                wren <= 1;
            end
        end else if (rst) begin
            address <= 0;
            wren <= 0;
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

    counter cnt_inst(.clk(clk)
        , .rst(rst)
        , .out(cnt_out)
    );
    
endmodule
