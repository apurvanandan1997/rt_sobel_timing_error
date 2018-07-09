//----------------------------------------------------------------------------
// readcop - module
//----------------------------------------------------------------------------
// IMPORTANT:
// DO NOT MODIFY THIS FILE EXCEPT IN THE DESIGNATED SECTIONS.
//
// SEARCH FOR --USER TO DETERMINE WHERE CHANGES ARE ALLOWED.
//
// TYPICALLY, THE ONLY ACCEPTABLE CHANGES INVOLVE ADDING NEW
// PORTS AND GENERICS THAT GET PASSED THROUGH TO THE INSTANTIATION
// OF THE USER_LOGIC ENTITY.
//----------------------------------------------------------------------------
//
// ***************************************************************************
// ** Copyright (c) 1995-2012 Xilinx, Inc.  All rights reserved.            **
// **                                                                       **
// ** Xilinx, Inc.                                                          **
// ** XILINX IS PROVIDING THIS DESIGN, CODE, OR INFORMATION "AS IS"         **
// ** AS A COURTESY TO YOU, SOLELY FOR USE IN DEVELOPING PROGRAMS AND       **
// ** SOLUTIONS FOR XILINX DEVICES.  BY PROVIDING THIS DESIGN, CODE,        **
// ** OR INFORMATION AS ONE POSSIBLE IMPLEMENTATION OF THIS FEATURE,        **
// ** APPLICATION OR STANDARD, XILINX IS MAKING NO REPRESENTATION           **
// ** THAT THIS IMPLEMENTATION IS FREE FROM ANY CLAIMS OF INFRINGEMENT,     **
// ** AND YOU ARE RESPONSIBLE FOR OBTAINING ANY RIGHTS YOU MAY REQUIRE      **
// ** FOR YOUR IMPLEMENTATION.  XILINX EXPRESSLY DISCLAIMS ANY              **
// ** WARRANTY WHATSOEVER WITH RESPECT TO THE ADEQUACY OF THE               **
// ** IMPLEMENTATION, INCLUDING BUT NOT LIMITED TO ANY WARRANTIES OR        **
// ** REPRESENTATIONS THAT THIS IMPLEMENTATION IS FREE FROM CLAIMS OF       **
// ** INFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS       **
// ** FOR A PARTICULAR PURPOSE.                                             **
// **                                                                       **
// ***************************************************************************
//
//----------------------------------------------------------------------------
// Filename:          sobelcop
// Version:           1.00.a
// Description:       Example FSL core (Verilog).
// Date:              Mon Mar  4 12:51:06 2013 (by Create and Import Peripheral Wizard)
// Verilog Standard:  Verilog-2001
//----------------------------------------------------------------------------
// Naming Conventions:
//   active low signals:                    "*_n"
//   clock signals:                         "clk", "clk_div#", "clk_#x"
//   reset signals:                         "rst", "rst_n"
//   generics:                              "C_*"
//   user defined types:                    "*_TYPE"
//   state machine next state:              "*_ns"
//   state machine current state:           "*_cs"
//   combinatorial signals:                 "*_com"
//   pipelined or register delay signals:   "*_d#"
//   counter signals:                       "*cnt*"
//   clock enable signals:                  "*_ce"
//   internal version of output port:       "*_i"
//   device pins:                           "*_pin"
//   ports:                                 "- Names begin with Uppercase"
//   processes:                             "*_PROCESS"
//   component instantiations:              "<ENTITY_>I_<#|FUNC>"
//----------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
//
//
// Definition of Ports
// FSL_Clk             : Synchronous clock
// FSL_Rst           : System reset, should always come from FSL bus
// FSL_S_Clk       : Slave asynchronous clock
// FSL_S_Read      : Read signal, requiring next available input to be read
// FSL_S_Data      : Input data
// FSL_S_Control   : Control Bit, indicating the input data are control word
// FSL_S_Exists    : Data Exist Bit, indicating data exist in the input FSL bus
// FSL_M_Clk       : Master asynchronous clock
// FSL_M_Write     : Write signal, enabling writing to output FSL bus
// FSL_M_Data      : Output data
// FSL_M_Control   : Control Bit, indicating the output data are contol word
// FSL_M_Full      : Full Bit, indicating output FSL bus is full
//
////////////////////////////////////////////////////////////////////////////////

//----------------------------------------
// Module Section
//----------------------------------------
module sobelcop
	(
		// ADD USER PORTS BELOW THIS LINE
		/*********************************/
		/**  NPI Interface Ports below ***/
		/**  CS150 Tutorial 1			 ***/
		/*********************************/
		system_dcm_locked,
		XIL_NPI_Addr,
		XIL_NPI_AddrReq,
		XIL_NPI_AddrAck,
		XIL_NPI_RNW,
		XIL_NPI_Size,
		XIL_NPI_WrFIFO_Data,
		XIL_NPI_WrFIFO_BE,
		XIL_NPI_WrFIFO_Push,
		XIL_NPI_RdFIFO_Data,
		XIL_NPI_RdFIFO_Pop,
		XIL_NPI_RdFIFO_RdWdAddr,
		XIL_NPI_WrFIFO_Empty,
		XIL_NPI_WrFIFO_AlmostFull,
		XIL_NPI_WrFIFO_Flush,
		// NPI_RdFIFO_DataAvailable, // I Only used for MPMC2 connections
		XIL_NPI_RdFIFO_Empty,
		XIL_NPI_RdFIFO_Flush,
		XIL_NPI_RdFIFO_Latency,
		XIL_NPI_RdModWr,
		XIL_NPI_InitDone,
		/**End of NPI Port addition **/
            XIL_NPI_Addr2,
		XIL_NPI_AddrReq2,
		XIL_NPI_AddrAck2,
		XIL_NPI_RNW2,
		XIL_NPI_Size2,
		XIL_NPI_WrFIFO_Data2,
		XIL_NPI_WrFIFO_BE2,
		XIL_NPI_WrFIFO_Push2,
		XIL_NPI_RdFIFO_Data2,
		XIL_NPI_RdFIFO_Pop2,
		XIL_NPI_RdFIFO_RdWdAddr2,
		XIL_NPI_WrFIFO_Empty2,
		XIL_NPI_WrFIFO_AlmostFull2,
		XIL_NPI_WrFIFO_Flush2,
		// NPI_RdFIFO_DataAvailable, // I Only used for MPMC2 connections
		XIL_NPI_RdFIFO_Empty2,
		XIL_NPI_RdFIFO_Flush2,
		XIL_NPI_RdFIFO_Latency2,
		XIL_NPI_RdModWr2,
		XIL_NPI_InitDone2,

		GPIO_COMPSW_3,
		GPIO_LED,
		// ADD USER PORTS ABOVE THIS LINE

		// DO NOT EDIT BELOW THIS LINE ////////////////////
		// Bus protocol ports, do not add or delete.
		FSL_Clk,
		FSL_Rst,
		FSL_S_Clk,
		FSL_S_Read,
		FSL_S_Data,
		FSL_S_Control,
		FSL_S_Exists,
		FSL_M_Clk,
		FSL_M_Write,
		FSL_M_Data,
		FSL_M_Control,
		FSL_M_Full
		// DO NOT EDIT ABOVE THIS LINE ////////////////////
	);

// ADD USER PORTS BELOW THIS LINE
/*********************************/
/**  NPI Interface Ports below ***/
/**  CS150 Tutorial 1			 ***/
/*********************************/

/** NPI interface parameters 			**/
parameter C_PI_ADDR_WIDTH = 32;
parameter C_PI_DATA_WIDTH = 64;
parameter C_PI_BE_WIDTH = 8;
parameter C_PI_RDWDADDR_WIDTH = 4;
/** End of NPI parameter definition **/
output [7:0] GPIO_LED;
input		system_dcm_locked;
output 	[C_PI_ADDR_WIDTH-1:0] XIL_NPI_Addr;
output 	XIL_NPI_AddrReq;
input 	XIL_NPI_AddrAck;
output 	XIL_NPI_RNW;
output 	[3:0] XIL_NPI_Size;
output 	[C_PI_DATA_WIDTH-1:0] XIL_NPI_WrFIFO_Data;
output 	[C_PI_BE_WIDTH-1:0] XIL_NPI_WrFIFO_BE;
output 	XIL_NPI_WrFIFO_Push;
input 	[C_PI_DATA_WIDTH-1:0] XIL_NPI_RdFIFO_Data;
output 	XIL_NPI_RdFIFO_Pop;
input 	[C_PI_RDWDADDR_WIDTH-1:0] XIL_NPI_RdFIFO_RdWdAddr;
input 	XIL_NPI_WrFIFO_Empty;
input 	XIL_NPI_WrFIFO_AlmostFull;
output 	XIL_NPI_WrFIFO_Flush;
// input NPI_RdFIFO_DataAvailable; //Only used for MPMC2 designs
input 	XIL_NPI_RdFIFO_Empty;
output 	XIL_NPI_RdFIFO_Flush;
input 	[1:0] XIL_NPI_RdFIFO_Latency;
output 	XIL_NPI_RdModWr;
input 	XIL_NPI_InitDone;
/**End of NPI Port addition **/
output 	[C_PI_ADDR_WIDTH-1:0] XIL_NPI_Addr2;
output 	XIL_NPI_AddrReq2;
input 	XIL_NPI_AddrAck2;
output 	XIL_NPI_RNW2;
output 	[3:0] XIL_NPI_Size2;
output 	[C_PI_DATA_WIDTH-1:0] XIL_NPI_WrFIFO_Data2;
output 	[C_PI_BE_WIDTH-1:0] XIL_NPI_WrFIFO_BE2;
output 	XIL_NPI_WrFIFO_Push2;
input 	[C_PI_DATA_WIDTH-1:0] XIL_NPI_RdFIFO_Data2;
output 	XIL_NPI_RdFIFO_Pop2;
input 	[C_PI_RDWDADDR_WIDTH-1:0] XIL_NPI_RdFIFO_RdWdAddr2;
input 	XIL_NPI_WrFIFO_Empty2;
input 	XIL_NPI_WrFIFO_AlmostFull2;
output 	XIL_NPI_WrFIFO_Flush2;
// input NPI_RdFIFO_DataAvailable; //Only used for MPMC2 designs
input 	XIL_NPI_RdFIFO_Empty2;
output 	XIL_NPI_RdFIFO_Flush2;
input 	[1:0] XIL_NPI_RdFIFO_Latency2;
output 	XIL_NPI_RdModWr2;
input 	XIL_NPI_InitDone2;
/**End of NPI Port addition **/


input GPIO_COMPSW_3;
// ADD USER PORTS ABOVE THIS LINE

input                                     FSL_Clk;
input                                     FSL_Rst;
input                                     FSL_S_Clk;
output                                    FSL_S_Read;
input      [0 : 31]                       FSL_S_Data;
input                                     FSL_S_Control;
input                                     FSL_S_Exists;
input                                     FSL_M_Clk;
output                                    FSL_M_Write;
output     [0 : 31]                       FSL_M_Data;
output                                    FSL_M_Control;
input                                     FSL_M_Full;
// ADD USER PARAMETERS BELOW THIS LINE
//----------------------------------------
// Implementation Section
//----------------------------------------
// we will build a simple state machine
// it will read one word from the FSL,
// use it as address, and then fetch
// the data from DRAM, finally write to
// FSL
localparam INIT 	= 6'b000001;
localparam RDADDR	= 6'b000010;
localparam FTDATA	= 6'b000100;
localparam WAITEMP	= 6'b001000;
localparam WAITLAT	= 6'b010000;
localparam INIT2        = 6'b100000;
reg [5:0] state;

wire[31:0] ADDR_MB;
reg[31:0] DATA_MB;
// ADD USER PARAMETERS ABOVE THIS LINE
//wire video_ready;
//wire video_valid;
reg [23:0] video;
//reg [2:0] counter;
reg [12:0] fifo_size;
wire [12:0] data_count;
assign XIL_NPI_Addr = ADDR_MB;
assign XIL_NPI_RdModWr = 1'b0; // does not have to be asserted
assign XIL_NPI_RNW = XIL_NPI_AddrReq;  // we will assert this to read when AddrReq is high


assign XIL_NPI_Size = 4'h4; //32-Word Cacheline read
assign XIL_NPI_WrFIFO_Flush = 1'b0;
assign XIL_NPI_WrFIFO_BE = 8'h00; // not writing

assign XIL_NPI_AddrReq = (state == FTDATA);

assign XIL_NPI_RdFIFO_Pop = (~XIL_NPI_RdFIFO_Empty) & XIL_NPI_InitDone; // the FIFO_Empty flag should be 0 after pop is one

reg [1:0] latency_counter; // this counter is used to keep track of
			   // when to latch in the returned data
			   // from DRAM

// In this module we will make both FSL_S_Clk and
// and FSL_M_Clk equivalent to the MPMC clock
wire pixel_af_wr_en;
wire fifo_empty, fifo_full;
reg [63:0] rdf_dout;
wire [19:0] pixel_ind_wr;
reg pixel_rdf_valid;
reg [2:0] dram_data_counter;
wire rst;
wire [7:0] out_videor;
wire [7:0] out_videog;
wire [7:0] out_videob;
reg [7:0] videor ;
reg [7:0] videog;
reg [7:0] videob;
wire rstr;
wire rstg;
wire rstb;
wire rdy_wirer;
wire rdy_wireg;
wire rdy_wireb;
wire [7:0] video_wire_r;
wire [7:0] video_wire_g;
wire [7:0] video_wire_b;
assign video_wire_r = videor;
assign video_wire_g = videog;
assign video_wire_b = videob;
reg rst_wirer;
reg rst_wireg;
reg rst_wireb;
assign rstr = rst_wirer;
assign rstg = rst_wireg;
assign rstb = rst_wireb;
top topr(
.clock(FSL_Clk),
.reset(rst || ~XIL_NPI_InitDone|| rstr),
.datain(video_wire_r),
.dataout(out_videor),
.dat_rdy(rdy_wirer)
);
top topg(
.clock(FSL_Clk),
.reset(rst || ~XIL_NPI_InitDone || rstg),
.datain(video_wire_g),
.dataout(out_videog),
.dat_rdy(rdy_wireg)
);
top topb(
.clock(FSL_Clk),
.reset(rst || ~XIL_NPI_InitDone||rstb),
.datain(video_wire_b),
.dataout(out_videob),
.dat_rdy(rdy_wireb)
);
/*Debouncer rst_parse(
      .clk(FSL_S_Clk),
      .in(GPIO_COMPSW_1),
      .out(rst));*/

      ButtonParse #(
			.Width(				1),
			.DebWidth(			20),
			.EdgeOutWidth(		1)
		) toggle_parse_1(
			.Clock(				FSL_S_Clk),
			.Reset(				1'b0),
			.Enable(			1'b1),
			.In(				GPIO_COMPSW_3),
			.Out(				rst));
reg [19:0] pixel_ind;
assign ADDR_MB = 32'h90000000 + pixel_ind*4;

//reg [19:0] dvi_rst_counter; // don't reset dvi until this reaches zero

//reg dvi_rst;
//reg dvi_rst_pending;

always @(posedge FSL_S_Clk) begin
	if (FSL_Rst | rst)
		begin
			state <= INIT;
			latency_counter <=0;
			pixel_rdf_valid <= 0;
			pixel_ind <= 0;
			//dvi_rst <= 0;
			//dvi_rst_pending <= 1;
		end
	else
		case (state)
		INIT:
			begin
			if (data_count < 13'd8176) state <= RDADDR;
			pixel_rdf_valid <= 0;
			end
		RDADDR:
			begin
				if (data_count < 13'd8176) begin  //& fifo_size < 13'd8189
					latency_counter <= XIL_NPI_RdFIFO_Latency;
					state <= FTDATA;
					//fifo_size <= fifo_size + 4;
				end else begin
				//	dvi_rst <= dvi_rst_pending; // if dvi reset is pending, then reset it now
				//	dvi_rst_pending <= 0;
				end
				pixel_rdf_valid <= 0;
			end
		FTDATA:
			begin
				if(XIL_NPI_AddrAck) begin
					state <= WAITEMP;
				end
				pixel_rdf_valid <= 0;
			end
		WAITEMP:// we wait for the FIFO to be non-empty
			begin
				if((~XIL_NPI_RdFIFO_Empty) & XIL_NPI_InitDone) begin
					if(latency_counter == 0)
						begin
							rdf_dout[31:0] <= XIL_NPI_RdFIFO_Data[63:32];
							rdf_dout[63:32] <= XIL_NPI_RdFIFO_Data[31:0];
							//pixel_ind_wr <= XIL_NPI_RdFIFO_Data[83:64];
							pixel_rdf_valid <= 1;
							if (pixel_ind == 20'd786430) //1024*768-2
								pixel_ind <= 0;
							else
								pixel_ind <= pixel_ind + 2;
							if (pixel_ind[4:0] == 5'b11110)
								state <= RDADDR;
						end
					else
						begin
							pixel_rdf_valid <= 0;
							latency_counter <= latency_counter-1;
							state <= WAITLAT;
						end
					end
				else
					pixel_rdf_valid <= 0;
			end
		WAITLAT:
			begin
				if(latency_counter == 0)
					begin
						rdf_dout[31:0] <= XIL_NPI_RdFIFO_Data[63:32];
						rdf_dout[63:32] <= XIL_NPI_RdFIFO_Data[31:0];
						pixel_rdf_valid <= 1;
						if (pixel_ind == 20'd786430) //1024*768-2
							pixel_ind <= 0;
						else
							pixel_ind <= pixel_ind + 2;
						if (pixel_ind[4:0] == 5'b11110)
							state <= RDADDR;
					end
				else
					begin
						pixel_rdf_valid <= 0;
						latency_counter <= latency_counter-1;
						state <= WAITLAT;
					end
				//if (counter != 0) counter <= counter - 1;
			end
		default:
			state <= INIT;
		endcase
end
	reg[19:0] pix_count=20'b0;
	wire [31:0] video_32;
	always@(*) begin
       if(rst_wirer == 0)
	videor	 <= video_32[7:0];
    if(rst_wireg == 0)
	videog	<= video_32[15:8];
    if(rst_wireb == 0)
	videob	<= video_32[23:16];
	if(pix_count==20'd786431)
	begin
	rst_wirer<=1;
	rst_wireg<=1;
	rst_wireb<=1;
	pix_count=0;
	end
	else
	begin
	rst_wirer<=0;
	rst_wireg<=0;
	rst_wireb<=0;
	pix_count=pix_count+1;
        end
end
//	assign video_valid = ~fifo_empty & video_ready;

fifo_generator_v9_1 fiforam1(
    	.rst(rst || ~XIL_NPI_InitDone),
    	.wr_clk(FSL_S_Clk),
    	.rd_clk(FSL_Clk),
    	.din(rdf_dout),
    	.wr_en(pixel_rdf_valid),
    	.rd_en(~fifo_empty),
    	.dout(video_32),
    	.full(fifo_full),
    	.wr_data_count(data_count),
    	.empty(fifo_empty));

reg [83:0] data_84;
reg [10:0] j, i;
	reg [63:0] data_mask;
	wire [31:0] data_32;
  assign pixel_ind_wr = i*1024 + {{9{0}},j};
	assign data_32 = {8'b0, out_videor, out_videog, out_videob};
wire rdy_state;
assign rdy_state =rdy_wirer & rdy_wireg & rdy_wireb;
	always@(posedge rdy_state) // grouping the vga signals as a packet of 84 bits with 2 pixels and their index
	begin
		if(rst || ~XIL_NPI_InitDone|| rstr || rstg || rstb)
			begin
				j <= 0;
				i <= 0;
				data_mask <= 64'h0;
				//pixel_ind_wr <=0;
			end
		else
			begin
				if ((j == 1023) && (i == 767)) begin
		        j <= 0;
		        i <= 0;
		      end else if (j == 1023) begin
		        j <= 0;
		        i <= i + 1;
		      end else begin
		        j <= j + 1;
		        i <= i;
		      end
					if (pixel_ind_wr[0]) begin
						data_84[63:32] <= data_32;
						data_mask <= 64'hffffffffffffffff;
					end else begin
						data_84[31:0] <= data_32;
						data_mask <= 64'h0;
						data_84[83:64] <= pixel_ind_wr;
					end
			end
	end
dram_write_frame write_conv_signal(
	.rst(rst | FSL_Rst),
	.dram_clk(FSL_S_Clk),
	.data_clk(rdy_wirer && rdy_wireg && rdy_wireb),
	.data(data_84),
	.data_mask(data_mask),
	.star_addr(32'h98000000),
	.XIL_NPI_Addr(XIL_NPI_Addr2),
	.XIL_NPI_AddrReq(XIL_NPI_AddrReq2),
	.XIL_NPI_AddrAck(XIL_NPI_AddrAck2),
	.XIL_NPI_RNW(XIL_NPI_RNW2),
	.XIL_NPI_Size(XIL_NPI_Size2),
	.XIL_NPI_WrFIFO_Data(XIL_NPI_WrFIFO_Data2),
	.XIL_NPI_WrFIFO_BE(XIL_NPI_WrFIFO_BE2),
	.XIL_NPI_WrFIFO_Push(XIL_NPI_WrFIFO_Push2),
	.XIL_NPI_RdFIFO_Data(XIL_NPI_RdFIFO_Data2),
	.XIL_NPI_RdFIFO_Pop(XIL_NPI_RdFIFO_Pop2),
	.XIL_NPI_RdFIFO_RdWdAddr(XIL_NPI_RdFIFO_RdWdAddr2),
	.XIL_NPI_WrFIFO_Empty(XIL_NPI_WrFIFO_Empty2),
	.XIL_NPI_WrFIFO_AlmostFull(XIL_NPI_WrFIFO_AlmostFull2),
	.XIL_NPI_WrFIFO_Flush(XIL_NPI_WrFIFO_Flush2),
	.XIL_NPI_RdFIFO_Empty(XIL_NPI_RdFIFO_Empty2),
	.XIL_NPI_RdFIFO_Flush(XIL_NPI_RdFIFO_Flush2),
	.XIL_NPI_RdFIFO_Latency(XIL_NPI_RdFIFO_Latency2),
	.XIL_NPI_RdModWr(XIL_NPI_RdModWr2),
	.XIL_NPI_InitDone(XIL_NPI_InitDone2));

    //	wire [35:0] chipscope_control;
assign GPIO_LED = { rdy_wirer,rdy_wireg ,1'b1,1'b0,rdy_wireb ,rstr,rstg ,rstb  };

// chipscope_icon icon(
// 	.CONTROL0(chipscope_control)
// 	);
//
// 	chipscope_ila ila(
// 	.CONTROL(chipscope_control),
// 	.CLK(FSL_S_Clk),
// 	.DATA({video_wire_r,video_wire_g ,video_wire_b ,out_videor,out_videog,out_videob,rdy_wirer,rdy_wireg,rdy_wireb,rdy_state,rstr,rstg,rstb,data_32, video_32, rdf_dout,pixel_rdf_valid,fifo_empty, fifo_full,data_count ,state	,50'b0 }),
// 	.TRIG0(rst)
// 	) ;

endmodule

module fifo_generator_v9_4(
  rst,
  wr_clk,
  rd_clk,
  din,
  wr_en,
  rd_en,
  dout,
  full,
  empty,
  rd_data_count
);

input rst;
input wr_clk;
input rd_clk;
input [83 : 0] din;
input wr_en;
input rd_en;
output [83 : 0] dout;
output full;
output empty;
output [12 : 0] rd_data_count;
endmodule
module fifo_generator_v9_1(
  rst,
  wr_clk,
  rd_clk,
  din,
  wr_en,
  rd_en,
  dout,
  full,
  empty,
  wr_data_count
);

input rst;
input wr_clk;
input rd_clk;
input [63 : 0] din;
input wr_en;
input rd_en;
output [31 : 0] dout;
output full;
output empty;
output [12 : 0] wr_data_count;
endmodule

// module chipscope_ila(
//     CONTROL,
//     CLK,
//     DATA,
//     TRIG0);
//
//
// inout [35 : 0] CONTROL;
// input CLK;
// input [255 : 0] DATA;
// input [0 : 0] TRIG0;
//
// endmodule
//
// module chipscope_icon(
//     CONTROL0);
//
//
// inout [35 : 0] CONTROL0;
//
// endmodule
