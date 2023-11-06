`timescale 1ns / 100ps
module axis_ddr3_ctrl (
    clock,
    reset,

    s_valid_i,
    s_ready_o,
    s_last_i,
    s_data_i,

    m_valid_o,
    m_ready_i,
    m_last_o,
    m_data_o,

    awvalid_o,
    awready_i,
    awburst_o,
    awlen_o,
    awid_o,
    awaddr_o,

    wvalid_o,
    wready_i,
    wlast_o,
    wstrb_o,
    wdata_o,

    bvalid_i,
    bready_o,
    bid_i,
    bresp_i,

    arvalid_o,
    arready_i,
    arburst_o,
    arlen_o,
    arid_o,
    araddr_o,

    rvalid_i,
    rready_o,
    rlast_i,
    rid_i,
    rresp_i,
    rdata_i
);

  localparam WIDTH = 32;
  localparam MSB = WIDTH - 1;
  localparam MASKS = WIDTH / 8;
  localparam SSB = MASKS - 1;

  localparam ADDRS = 27;
  localparam ASB = ADDRS - 1;

  localparam REQID = 4;
  localparam ISB = REQID - 1;


  input clock;
  input reset;

  // From USB AXI-S interface
  input s_valid_i;
  output s_ready_o;
  input s_last_i;
  input [7:0] s_data_i;

  // To USB AXI-S interface
  output m_valid_o;
  input m_ready_i;
  output m_last_o;
  output [7:0] m_data_o;

  // AXI4 write -address, -data, and -response ports
  output awvalid_o;
  input awready_i;
  output [1:0] awburst_o;
  output [7:0] awlen_o;
  output [ISB:0] awid_o;
  output [ASB:0] awaddr_o;

  output wvalid_o;
  input wready_i;
  output wlast_o;
  output [SSB:0] wstrb_o;
  output [MSB:0] wdata_o;

  input bvalid_i;
  output bready_o;
  input [ISB:0] bid_i;
  input [1:0] bresp_i;

  // AXI4 read -address, and -data ports
  output arvalid_o;
  input arready_i;
  output [1:0] arburst_o;
  output [7:0] arlen_o;
  output [ISB:0] arid_o;
  output [ASB:0] araddr_o;

  input rvalid_i;
  output rready_o;
  input rlast_i;
  input [ISB:0] rid_i;
  input [1:0] rresp_i;
  input [MSB:0] rdata_i;


  // -- Constants -- //

  localparam [3:0] ST_IDLE = 4'b0000;
  localparam [3:0] ST_READ = 4'b0100;
  localparam [3:0] ST_WAIT = 4'b0101;
  localparam [3:0] ST_SEND = 4'b0110;
  localparam [3:0] ST_RECV = 4'b1001;
  localparam [3:0] ST_WRIT = 4'b1000;
  localparam [3:0] ST_DAMN = 4'b0011;


  reg fetch, store, awvld, arvld;
  reg mvalid, mlast;
  reg [7:0] mdata;
  reg dir_q;  // 1 = WRITE
  reg [7:0] len_q;
  reg [ISB:0] tid_q;
  reg [ASB:0] addr_q;
  reg [MSB:0] wdata;
  reg [1:0] count;
  wire [1:0] cnext;
  wire fetch_w, store_w;
  wire xvalid, xready, xlast;
  wire [MSB:0] xdata;
  reg [3:0] state;


  // -- I/O Assignments -- //

  assign m_valid_o = mvalid;
  assign m_last_o  = mlast;
  assign m_data_o  = mdata;

  // Write address port
  assign awvalid_o = awvld;
  assign awburst_o = 2'b01; // INCR
  assign awaddr_o  = addr_q;
  // assign awlen_o = {3'b0, len_q, 2'b11};
  assign awlen_o = len_q;
  assign awid_o  = tid_q;

  // Write response port
  assign bready_o = store;

  // Read address port
  assign arvalid_o = arvld;
  assign arburst_o = 2'b01; // INCR
  // assign arlen_o   = {3'b0, len_q, 2'b11};
  assign arlen_o   = len_q;
  assign arid_o    = tid_q;
  assign araddr_o  = addr_q;


  // -- Internal Signal Assignments -- //

  assign fetch_w = ~addr_q[23];
  assign store_w =  addr_q[23];

  assign cnext = count + 1;

  assign xready = state == ST_SEND && count == 3;


  always @(posedge clock) begin
    if (reset) begin
      state <= ST_IDLE;
      mvalid <= 1'b0;
      mlast  <= 1'b0;
      fetch  <= 1'b0;
      store  <= 1'b0;
      awvld  <= 1'b0;
      arvld  <= 1'b0;
    end else begin
      case (state)
        ST_IDLE: begin
          if (s_valid_i && s_ready_o) begin
            // Byte-shift in the command:
            //   {1b, 3b, 4b, 24b]
            {len_q, dir_q, tid_q, addr_q} <= {dir_q, tid_q, addr_q, s_data_i};

            if (s_last_i) begin
              fetch <= fetch_w;
              arvld <= fetch_w;
              store <= store_w;
              state <= fetch_w ? ST_READ : ST_RECV;
            end

          end
        end

        // -- READ States -- //

        ST_READ: begin
          if (arready_i) begin
            arvld <= 1'b0;
            fetch <= 1'b1;
            state <= ST_WAIT;
          end
        end

        ST_WAIT: begin
          if (rvalid_i && rready_o && rlast_i) begin
            fetch  <= 1'b0;
            mvalid <= 1'b1;

            if (rid_i != tid_q || rresp_i != 2'b00) begin
              state <= ST_DAMN;
              mlast <= 1'b1;
              mdata <= {2'b10, rresp_i, rid_i};
              count <= 0;
            end else begin
              state <= ST_SEND;
              mlast <= 1'b0;
              mdata <= xdata[7:0];
              count <= cnext;
            end
          end
        end

        ST_SEND: begin
          mdata <= count == 0 ? xdata[7:0] :
                   count == 1 ? xdata[15:8] :
                   count == 2 ? xdata[23:16] :
                   xdata[31:24] ;

          mvalid <= xvalid;
          mlast <= xlast;

          if (mvalid && m_ready_i) begin
            count <= cnext;

            if (mlast) begin
              state <= ST_IDLE;
            end
          end
        end

        // -- WRITE States -- //

        ST_RECV: begin
          if (s_valid_i && s_ready_o) begin
            wdata <= {s_data_i, wdata[MSB:8]};
            count <= cnext;

            if (s_last_i) begin
              state <= ST_WRIT;
              awvld <= 1'b1;
            end
          end
        end

        ST_WRIT: begin
          if (awready_i) begin
            awvld <= 1'b0;
          end

          if (bvalid_i) begin
            store <= 1'b0;
            if (bid_i != tid_q || bresp_i != 2'b00) begin
              state  <= ST_DAMN;
              mvalid <= 1'b1;
              mlast  <= 1'b1;
              mdata  <= {2'b01, bresp_i, bid_i};
            end else begin
              state <= ST_IDLE;
            end
          end
        end

        // -- ERROR States -- //

        default: begin
          $error("On noes!");
          if (m_ready_i) begin
            mvalid <= 1'b0;
            mlast  <= 1'b0;
            state  <= ST_IDLE;
          end
        end
      endcase
    end
  end


  // -- Synchronous, 2 kB, Write-Data FIFO -- //

  packet_fifo #(
      .WIDTH (MASKS + WIDTH),
      .ABITS (9),
      .OUTREG(1)
  ) wrdata_fifo_inst (
      .clock(clock),
      .reset(reset),

      .valid_i(s_valid_i && store && count == 3),
      .ready_o(s_ready_o),
      .last_i (s_last_i & store),
      .drop_i (1'b0),
      .data_i ({{MASKS{1'b1}}, wdata}),

      .valid_o(wvalid_o),
      .ready_i(wready_i & store),
      .last_o (wlast_o),
      .data_o ({wstrb_o, wdata_o})
  );


  // -- Synchronous, 2 kB, Read-Data FIFO -- //

  sync_fifo #(
      .WIDTH (WIDTH + 1),
      .ABITS (9),
      .OUTREG(1)
  ) rddata_fifo_inst (
      .clock(clock),
      .reset(reset),

      .valid_i(rvalid_i),
      .ready_o(rready_o),
      .data_i ({rlast_i, rdata_i}),

      .valid_o(xvalid),
      .ready_i(xready),
      .data_o ({xlast, xdata})
  );

/*
  packet_fifo #(
      .WIDTH (WIDTH),
      .ABITS (9),
      .OUTREG(1)
  ) rddata_fifo_inst (
      .clock(clock),
      .reset(reset),

      .valid_i(rvalid_i),
      .ready_o(rready_o),
      .last_i (rlast_i),
      .drop_i (1'b0),
      .data_i (rdata_i),

      .valid_o(xvalid),
      .ready_i(xready),
      .last_o (xlast),
      .data_o (xdata)
  );
*/


endmodule  // axis_ddr3_ctrl
