module dut #(
  parameter int DRAM_ADDRESS_WIDTH = 32,
  parameter int SRAM_ADDRESS_WIDTH = 10,
  parameter int DRAM_DQ_WIDTH = 8,
  parameter int SRAM_DATA_WIDTH = 32
)(
  // System Signals
  input  wire                             clk           ,
  input  wire                             reset_n       , 
 
  // Control signals
  input  wire                             start         ,
  output reg                              ready         ,

  // DRAM Input memory interface
  output reg   [1:0]                     input_CMD     ,  // 2'b00=IDLE, 2'b01=READ, 2'b10=WRITE
  output wire   [DRAM_ADDRESS_WIDTH-1:0]  input_addr    ,
  input  wire   [DRAM_DQ_WIDTH-1:0]       input_dout    ,
  output wire   [DRAM_DQ_WIDTH-1:0]       input_din     ,
  output reg                             input_oe      ,

  // DRAM Output memory interface
  output reg   [1:0]                     output_CMD    ,  // 2'b00=IDLE, 2'b01=READ, 2'b10=WRITE
  output wire   [DRAM_ADDRESS_WIDTH-1:0]  output_addr   ,
  input  wire   [DRAM_DQ_WIDTH-1:0]       output_dout   ,
  output wire   [DRAM_DQ_WIDTH-1:0]       output_din    ,
  output reg                             output_oe     ,

  // Port A: Read Port 
  output reg  [SRAM_ADDRESS_WIDTH-1 :0] read_address  ,
  input  wire  [SRAM_DATA_WIDTH-1 :0]    read_data     ,
  output reg                            read_enable   , 
  
  //---------------------------------------------------------------
  // Port B: Write Port 
  output    reg  [SRAM_ADDRESS_WIDTH-1 :0] write_address ,
  output   reg  [SRAM_DATA_WIDTH-1 :0]    write_data    ,
  output   reg                            write_enable          


);
 // Sram Tie off. Do not change this it you are not using the SRAM
   //always@(posedge clk) begin
   always@(posedge clk) begin
    read_address  <= 'b0;
    read_enable   <= 'b0;
    write_address <= 'b0;
    write_enable  <= 'b0;
    write_data  <= 'b0;
  end
  
  

//*********************DRAM<------------>DUT Handshake**********************
wire [DRAM_ADDRESS_WIDTH-1:0] kernel_Address_0, kernel_Address_1, dram_addr_input;
reg  [DRAM_ADDRESS_WIDTH-1:0] in_addr_q, in_addr_d;
reg [1:0] in_addr_sel; 
wire signed [DRAM_DQ_WIDTH-1:0] dq_in_d;
reg  signed [DRAM_DQ_WIDTH-1:0] dq_in_q;
reg sel_ker,sel_input;
reg [3:0] beat_count;
reg signed [DRAM_DQ_WIDTH-1:0] ker_data, input_data;
reg signed [DRAM_DQ_WIDTH-1:0] ker_0, ker_1, ker_2, ker_3, ker_4, ker_5, ker_6, ker_7, ker_8, ker_9, ker_10, ker_11, ker_12, ker_13, ker_14, ker_15;
reg signed [DRAM_DQ_WIDTH-1:0] ker_0_d, ker_1_d, ker_2_d, ker_3_d, ker_4_d, ker_5_d, ker_6_d, ker_7_d, ker_8_d, ker_9_d, ker_10_d, ker_11_d, ker_12_d, ker_13_d, ker_14_d, ker_15_d;
reg signed [DRAM_DQ_WIDTH-1:0] ker_0_q, ker_1_q, ker_2_q, ker_3_q, ker_4_q, ker_5_q, ker_6_q, ker_7_q, ker_8_q, ker_9_q, ker_10_q, ker_11_q, ker_12_q, ker_13_q, ker_14_q, ker_15_q;
reg save_ker_0, save_ker_1, save_ker_2, save_ker_3, save_ker_4, save_ker_5, save_ker_6, save_ker_7, save_ker_8, save_ker_9, save_ker_10, save_ker_11, save_ker_12, save_ker_13, save_ker_14, save_ker_15;
reg [3:0] ker_sel, current_ker_sel;
reg signed [DRAM_DQ_WIDTH-1:0] input_a0, input_a1, input_a2, input_a3, input_a4, input_a5, input_a6, input_a7;
reg signed [DRAM_DQ_WIDTH-1:0] input_b0, input_b1, input_b2, input_b3, input_b4, input_b5, input_b6, input_b7;
reg signed [DRAM_DQ_WIDTH-1:0] input_a0_d, input_a1_d, input_a2_d, input_a3_d, input_a4_d, input_a5_d, input_a6_d, input_a7_d;
reg signed [DRAM_DQ_WIDTH-1:0] input_b0_d, input_b1_d, input_b2_d, input_b3_d, input_b4_d, input_b5_d, input_b6_d, input_b7_d;
reg signed [DRAM_DQ_WIDTH-1:0] input_a0_q, input_a1_q, input_a2_q, input_a3_q, input_a4_q, input_a5_q, input_a6_q, input_a7_q;
reg signed [DRAM_DQ_WIDTH-1:0] input_b0_q, input_b1_q, input_b2_q, input_b3_q, input_b4_q, input_b5_q, input_b6_q, input_b7_q;
reg save_input_a0, save_input_a1, save_input_a2, save_input_a3, save_input_a4, save_input_a5, save_input_a6, save_input_a7;
reg save_input_b0, save_input_b1, save_input_b2, save_input_b3, save_input_b4, save_input_b5, save_input_b6, save_input_b7;
reg [3:0] input_sel, current_input_sel;
reg ker_wait, input_wait;
reg signed [DRAM_DQ_WIDTH-1:0]   ker_out, input_out;
wire signed [(2*DRAM_DQ_WIDTH)-1:0]   raw_mul; 
reg signed [(2*DRAM_DQ_WIDTH)+4:0]   raw_sum0_q, raw_sum1_q, raw_sum2_q, raw_sum3_q, raw_sum4_q, raw_sum5_q, raw_sum6_q, raw_sum7_q;
reg signed [(2*DRAM_DQ_WIDTH)+4:0]   raw_sum_d, raw_sum_q;
reg [3:0] sel_acc;
reg [2:0] get_raw_sum, track_raw_sum;
reg signed [(2*DRAM_DQ_WIDTH)+4:0] rs_0, rs_1, rs_2, rs_3, rs_4, rs_5, rs_6, rs_7;
reg signed [(2*DRAM_DQ_WIDTH)+4:0]   raw_sum0_d, raw_sum1_d, raw_sum2_d, raw_sum3_d, raw_sum4_d, raw_sum5_d,raw_sum6_d, raw_sum7_d;
reg save_raw_sum0, save_raw_sum1, save_raw_sum2, save_raw_sum3, save_raw_sum4, save_raw_sum5, save_raw_sum6, save_raw_sum7;
reg [2:0] sel_raw_sum;
reg signed [(2*DRAM_DQ_WIDTH)+4:0] final_raw_sum;
reg signed [DRAM_DQ_WIDTH-1:0] sum;
reg signed [DRAM_DQ_WIDTH-1:0] s_0, s_1, s_2, s_3, s_4, s_5, s_6, s_7;
reg [2:0] get_sum, track_sum;
reg signed [DRAM_DQ_WIDTH-1:0] sum_0_d, sum_1_d, sum_2_d, sum_3_d, sum_4_d, sum_5_d, sum_6_d, sum_7_d;
reg signed [DRAM_DQ_WIDTH-1:0] sum_0_q, sum_1_q, sum_2_q, sum_3_q, sum_4_q, sum_5_q, sum_6_q, sum_7_q;
reg save_sum_0, save_sum_1, save_sum_2, save_sum_3, save_sum_4, save_sum_5, save_sum_6, save_sum_7;
reg [7:0] output_count;
reg [2:0] sel_sum;
reg signed [DRAM_DQ_WIDTH-1:0]  dq_out_q;
reg signed [DRAM_DQ_WIDTH-1:0] dq_out_d;
reg raw_sum_wait, sum_wait;
wire [DRAM_ADDRESS_WIDTH-1:0] dram_addr_output;
reg [DRAM_ADDRESS_WIDTH-1:0] out_addr_d, out_addr_q;
reg [1:0] out_addr_sel;
reg [6:0] col_window, col;  
reg [16:0] col_out;
reg [9:0] row, row_index; 
reg [1:0] col_next, row_next;
 //*********************Controller***************************
  parameter [31:0]
        S0  = 32'h00000001,
        S1  = 32'h00000002,
        S2  = 32'h00000004,
        S3  = 32'h00000008,
        S4  = 32'h00000010,
        S5  = 32'h00000020,
        S6  = 32'h00000040,
        S7  = 32'h00000080,
        S8  = 32'h00000100,
        S9  = 32'h00000200,
        S10 = 32'h00000400,
        S11 = 32'h00000800,
        S12 = 32'h00001000,
        S13 = 32'h00002000,
        S14 = 32'h00004000,
        S15 = 32'h00008000,
        S16 = 32'h00010000,
        S17 = 32'h00020000,
        S18 = 32'h00040000,
        S19 = 32'h00080000,
        S20 = 32'h00100000,
        S21 = 32'h00200000,
        S22 = 32'h00400000,
        S23 = 32'h00800000;
  reg [31:0] current_state, next_state;
  reg [2:0] count;   // 3 bits enough for 0–7 range
  reg [2:0] count_1;   // 3 bits enough for 0–7 range
  reg [2:0] count_2;   // 3 bits enough for 0–7 range
  reg [2:0] count_3;   // 3 bits enough for 0–7 range
  reg [2:0] count_4;   // 3 bits enough for 0–7 range
  reg [2:0] input_add;   // 3 bits enough for 0-7 range
  reg [1:0] ker_add;   // 2 bits enough for 0-4 range
//****************************************************************************DATAPATH******************************************************************************************************
  //***********************DRAM<------------>DUT Handshake*************************
 
 

//flipflop
always_ff @(posedge clk or negedge reset_n)begin
    if (!reset_n) begin
      in_addr_q  <= '0;
      end else begin
      in_addr_q  <= in_addr_d;
      end
  end 


always_ff @(posedge clk or negedge reset_n)begin
    if (!reset_n) begin
      dq_in_q  <= '0;
      end else begin
      dq_in_q  <= dq_in_d;
      end
  end 

always_ff @(posedge clk or negedge reset_n)begin
    if (!reset_n) begin
      ker_0_q  <= '0;
      ker_1_q  <= '0;
      ker_2_q  <= '0;
      ker_3_q  <= '0;
      ker_4_q  <= '0;
      ker_5_q  <= '0;
      ker_6_q  <= '0;
      ker_7_q  <= '0;
      ker_8_q  <= '0;
      ker_9_q  <= '0;
      ker_10_q  <= '0;
      ker_11_q  <= '0;
      ker_12_q  <= '0;
      ker_13_q  <= '0;
      ker_14_q  <= '0;
      ker_15_q  <= '0;
      end else begin
      ker_0_q  <= ker_0_d;
      ker_1_q  <= ker_1_d;
      ker_2_q  <= ker_2_d;
      ker_3_q  <= ker_3_d;
      ker_4_q  <= ker_4_d;
      ker_5_q  <= ker_5_d;
      ker_6_q  <= ker_6_d;
      ker_7_q  <= ker_7_d;
      ker_8_q  <= ker_8_d;
      ker_9_q  <= ker_9_d;
      ker_10_q  <= ker_10_d;
      ker_11_q  <= ker_11_d;
      ker_12_q  <= ker_12_d;
      ker_13_q  <= ker_13_d;
      ker_14_q  <= ker_14_d;
      ker_15_q  <= ker_15_d;
      end
  end 

  always_ff @(posedge clk or negedge reset_n)begin
    if (!reset_n) begin
      input_a0_q  <= '0;
      input_a1_q  <= '0;
      input_a2_q  <= '0;
      input_a3_q  <= '0;
      input_a4_q  <= '0;
      input_a5_q  <= '0;
      input_a6_q  <= '0;
      input_a7_q  <= '0;
      input_b0_q  <= '0;
      input_b1_q  <= '0;
      input_b2_q  <= '0;
      input_b3_q  <= '0;
      input_b4_q  <= '0;
      input_b5_q  <= '0;
      input_b6_q  <= '0;
      input_b7_q  <= '0;
      end else begin
      input_a0_q  <= input_a0_d;
      input_a1_q  <= input_a1_d;
      input_a2_q  <= input_a2_d;
      input_a3_q  <= input_a3_d;
      input_a4_q  <= input_a4_d;
      input_a5_q  <= input_a5_d;
      input_a6_q  <= input_a6_d;
      input_a7_q  <= input_a7_d;
      input_b0_q  <= input_b0_d;
      input_b1_q  <= input_b1_d;
      input_b2_q  <= input_b2_d;
      input_b3_q  <= input_b3_d;
      input_b4_q  <= input_b4_d;
      input_b5_q  <= input_b5_d;
      input_b6_q  <= input_b6_d;
      input_b7_q  <= input_b7_d;
      end
  end 

 always_ff @(posedge clk or negedge reset_n)begin
    if (!reset_n) begin
      raw_sum_q  <= '0;
      end else begin
      raw_sum_q  <= raw_sum_d;
      end
  end 

 always_ff @(posedge clk or negedge reset_n)begin
    if (!reset_n) begin
      raw_sum0_q  <= '0;
      raw_sum1_q  <= '0;
      raw_sum2_q  <= '0;
      raw_sum3_q  <= '0;
      raw_sum4_q  <= '0;
      raw_sum5_q  <= '0;
      raw_sum6_q  <= '0;
      raw_sum7_q  <= '0;
      end else begin
      raw_sum0_q  <= raw_sum0_d;
      raw_sum1_q  <= raw_sum1_d;
      raw_sum2_q  <= raw_sum2_d;
      raw_sum3_q  <= raw_sum3_d;
      raw_sum4_q  <= raw_sum4_d;
      raw_sum5_q  <= raw_sum5_d;
      raw_sum6_q  <= raw_sum6_d;
      raw_sum7_q  <= raw_sum7_d;
      end
  end 

  always_ff @(posedge clk or negedge reset_n)begin
    if (!reset_n) begin
      sum_0_q  <= '0;
      sum_1_q  <= '0;
      sum_2_q  <= '0;
      sum_3_q  <= '0;
      sum_4_q  <= '0;
      sum_5_q  <= '0;
      sum_6_q  <= '0;
      sum_7_q  <= '0;
      end else begin
      sum_0_q  <= sum_0_d;
      sum_1_q  <= sum_1_d;
      sum_2_q  <= sum_2_d;
      sum_3_q  <= sum_3_d;
      sum_4_q  <= sum_4_d;
      sum_5_q  <= sum_5_d;
      sum_6_q  <= sum_6_d;
      sum_7_q  <= sum_7_d;
      end
  end 


  always_ff @(posedge clk or negedge reset_n)begin
    if (!reset_n) begin
      dq_out_q  <= '0;
      end else begin
      dq_out_q  <= dq_out_d;
      end
  end 

  
  always_ff @(posedge clk or negedge reset_n)begin
    if (!reset_n) begin
      out_addr_q  <= '0;
      end else begin
      out_addr_q  <= out_addr_d;
      end
  end 


  assign input_addr = in_addr_q;
  assign dq_in_d = input_dout;
  assign raw_mul = $signed({{8{ker_out[7]}}, ker_out}) * $signed({{8{input_out[7]}}, input_out});
  assign output_din = dq_out_q;
  assign output_addr = out_addr_q;

  assign input_din = '0;   // never writing to the input DRAM


 // Declare internal address
  assign kernel_Address_0 = 32'h0000_0000;
  assign kernel_Address_1 = 32'h0000_0008;
  assign dram_addr_input  = 32'h0000_0010 + (row * 1024) + col*8;
  assign dram_addr_output = 32'h0000_0000 + col_out*8;

  assign row = row_next + row_index;
  assign col = col_next + col_window;




 // in_addr_sel determines which  address gets sent to DRAM
  // 00 -> kernal address 0
  // 01 -> kernal address 1
  // 10 -> input address
  // 11 -> feedback
  always_comb begin
      case (in_addr_sel)
      2'b00:   in_addr_d = kernel_Address_0;
      2'b01:   in_addr_d = kernel_Address_1;
      2'b10:   in_addr_d = dram_addr_input;
      2'b11:   in_addr_d = in_addr_q;
      default: in_addr_d = '0;
      endcase
  end


always_comb begin
      if (sel_ker)
      ker_data = dq_in_q;
      else 
      ker_data = '0;

      if(sel_input) 
      input_data = dq_in_q;
      else 
      input_data = '0;
  end


always_comb begin
// drive ALL outputs every time (dmux)
  ker_0=ker_0_q ; ker_1=ker_1_q ; ker_2=ker_2_q ; ker_3=ker_3_q ;
  ker_4=ker_4_q ; ker_5=ker_5_q ; ker_6=ker_6_q ; ker_7=ker_7_q ;
  ker_8=ker_8_q ; ker_9=ker_9_q ; ker_10=ker_10_q ; ker_11=ker_11_q ;
  ker_12=ker_12_q ; ker_13=ker_13_q ; ker_14=ker_14_q ; ker_15=ker_15_q ;

    case (ker_sel) 
    4'd0: if((beat_count==7)&&(sel_ker)) 
             ker_0 = ker_data;
    4'd1: if((beat_count==6)&&(sel_ker)) 
             ker_1 = ker_data; 
    4'd2: if((beat_count==5)&&(sel_ker)) 
             ker_2 = ker_data; 
    4'd3: if((beat_count==4)&&(sel_ker)) 
             ker_3 = ker_data; 
    4'd4: if((beat_count==3)&&(sel_ker)) 
             ker_4 = ker_data;
    4'd5: if((beat_count==2)&&(sel_ker)) 
             ker_5 = ker_data;  
    4'd6: if((beat_count==1)&&(sel_ker)) 
             ker_6 = ker_data; 
    4'd7: if((beat_count==0)&&(sel_ker)) 
             ker_7 = ker_data;
    4'd8: if((beat_count==15)&&(sel_ker)) 
             ker_8 = ker_data; 
    4'd9: if((beat_count==14)&&(sel_ker)) 
             ker_9 = ker_data; 
    4'd10: if((beat_count==13)&&(sel_ker)) 
             ker_10 = ker_data; 
    4'd11: if((beat_count==12)&&(sel_ker)) 
             ker_11 = ker_data; 
    4'd12: if((beat_count==11)&&(sel_ker))
             ker_12 = ker_data; 
    4'd13: if((beat_count==10)&&(sel_ker)) 
             ker_13 = ker_data; 
    4'd14: if((beat_count==9)&&(sel_ker)) 
             ker_14 = ker_data; 
    4'd15: if((beat_count==8)&&(sel_ker)) 
             ker_15 = ker_data; 
    endcase 
end
  





always_comb begin
      if (!save_ker_0)
      ker_0_d = ker_0;
      else 
      ker_0_d = ker_0_q;

      if (!save_ker_1)
      ker_1_d = ker_1;
      else 
      ker_1_d = ker_1_q;

      if (!save_ker_2)
      ker_2_d = ker_2;
      else 
      ker_2_d = ker_2_q;

      if (!save_ker_3)
      ker_3_d = ker_3;
      else 
      ker_3_d = ker_3_q;

      if (!save_ker_4)
      ker_4_d = ker_4;
      else 
      ker_4_d = ker_4_q;

      if (!save_ker_5)
      ker_5_d = ker_5;
      else 
      ker_5_d = ker_5_q;

      if (!save_ker_6)
      ker_6_d = ker_6;
      else 
      ker_6_d = ker_6_q;

      if (!save_ker_7)
      ker_7_d = ker_7;
      else 
      ker_7_d = ker_7_q;

      if (!save_ker_8)
      ker_8_d = ker_8;
      else 
      ker_8_d = ker_8_q;

      if (!save_ker_9)
      ker_9_d = ker_9;
      else 
      ker_9_d = ker_9_q;

      if (!save_ker_10)
      ker_10_d = ker_10;
      else 
      ker_10_d = ker_10_q;
      
      if (!save_ker_11)
      ker_11_d = ker_11;
      else 
      ker_11_d = ker_11_q;
      
      if (!save_ker_12)
      ker_12_d = ker_12;
      else 
      ker_12_d = ker_12_q;

      if (!save_ker_13)
      ker_13_d = ker_13;
      else 
      ker_13_d = ker_13_q;

      if (!save_ker_14)
      ker_14_d = ker_14;
      else 
      ker_14_d = ker_14_q;

      if (!save_ker_15)
      ker_15_d = ker_15;
      else 
      ker_15_d = ker_15_q;
end

always_comb begin
  if(ker_wait)
      case (current_ker_sel)
      4'd0:   ker_out =  ker_0_q;
      4'd1:   ker_out =  ker_1_q;
      4'd2:   ker_out =  ker_2_q;
      4'd3:   ker_out =  ker_3_q;
      4'd4:   ker_out =  ker_4_q;
      4'd5:   ker_out =  ker_5_q;
      4'd6:   ker_out =  ker_6_q;
      4'd7:   ker_out =  ker_7_q;
      4'd8:   ker_out =  ker_8_q;
      4'd9:   ker_out =  ker_9_q;
      4'd10:  ker_out =  ker_10_q;
      4'd11:  ker_out =  ker_11_q;
      4'd12:  ker_out =  ker_12_q;
      4'd13:  ker_out =  ker_13_q;
      4'd14:  ker_out =  ker_14_q;
      4'd15:  ker_out =  ker_15_q;
      default:  ker_out =  '0;
      endcase
  else
   ker_out = '0;
  end


always_comb begin
// drive ALL outputs every time (dmux)
  input_a0=input_a0_q; input_a1=input_a1_q; input_a2=input_a2_q; input_a3=input_a3_q; 
  input_a4=input_a4_q; input_a5=input_a5_q; input_a6=input_a6_q; input_a7=input_a7_q; 
  input_b0=input_b0_q; input_b1=input_b1_q; input_b2=input_b2_q; input_b3=input_b3_q;
  input_b4=input_b4_q; input_b5=input_b5_q; input_b6=input_b6_q; input_b7=input_b7_q;  


      case (input_sel)
      4'd0:   if((beat_count==7)&&(sel_input))
                    input_a0 =  input_data;

      4'd1:   if((beat_count==6)&&(sel_input))
                    input_a1 =  input_data;
                 
      4'd2:   if((beat_count==5)&&(sel_input))
                    input_a2 =  input_data;
                 
      4'd3:   if((beat_count==4)&&(sel_input))
                    input_a3 =  input_data;
                 
      4'd4:   if((beat_count==3)&&(sel_input))
                    input_a4 =  input_data;
                 
      4'd5:   if((beat_count==2)&&(sel_input))
                    input_a5 =  input_data;
                
      4'd6:   if((beat_count==1)&&(sel_input))
                    input_a6 =  input_data;
                 
      4'd7:   if((beat_count==0)&&(sel_input))
                    input_a7 =  input_data;
                 
      4'd8:   if((beat_count==15)&&(sel_input))
                    input_b0 =   input_data; //zero padding
                
      4'd9:   if((beat_count==14)&&(sel_input))
                    input_b1 =   input_data; //zero padding
                 
      4'd10:  if((beat_count==13)&&(sel_input))
                    input_b2 =   input_data; // zero padding
                 
      4'd11:  if((beat_count==12)&&(sel_input))
                    input_b3 =  input_data;
                 
      4'd12:  if((beat_count==11)&&(sel_input))
                    input_b4 =  input_data;
                 
      4'd13:   if((beat_count==10)&&(sel_input))
                    input_b5 =  input_data;
                 
      4'd14:   if((beat_count==9)&&(sel_input))
                    input_b6 =  input_data;
                 
      4'd15:   if((beat_count==8)&&(sel_input))
                    input_b7 =  input_data;
                 
      endcase
  end


always_comb begin
      if (!save_input_a0)
      input_a0_d = input_a0;
      else 
      input_a0_d = input_a0_q;

      if (!save_input_a1)
      input_a1_d = input_a1;
      else 
      input_a1_d = input_a1_q;

      if (!save_input_a2)
      input_a2_d = input_a2;
      else 
      input_a2_d = input_a2_q;

      if (!save_input_a3)
      input_a3_d = input_a3;
      else 
      input_a3_d = input_a3_q;

      if (!save_input_a4)
      input_a4_d = input_a4;
      else 
      input_a4_d = input_a4_q;

      if (!save_input_a5)
      input_a5_d = input_a5;
      else 
      input_a5_d = input_a5_q;

      if (!save_input_a6)
      input_a6_d = input_a6;
      else 
      input_a6_d = input_a6_q;

      if (!save_input_a7)
      input_a7_d = input_a7;
      else 
      input_a7_d = input_a7_q;

      if (!save_input_b0)
      input_b0_d = input_b0;
      else 
      input_b0_d = input_b0_q;

      if (!save_input_b1)
      input_b1_d = input_b1;
      else 
      input_b1_d = input_b1_q;

      if (!save_input_b2)
      input_b2_d = input_b2;
      else 
      input_b2_d = input_b2_q;

      if (!save_input_b3)
      input_b3_d = input_b3;
      else 
      input_b3_d = input_b3_q;

      if (!save_input_b4)
      input_b4_d = input_b4;
      else 
      input_b4_d = input_b4_q;

      if (!save_input_b5)
      input_b5_d = input_b5;
      else 
      input_b5_d = input_b5_q;

      if (!save_input_b6)
      input_b6_d = input_b6;
      else 
      input_b6_d = input_b6_q;

      if (!save_input_b7)
      input_b7_d = input_b7;
      else 
      input_b7_d = input_b7_q;

end

always_comb begin
  if(input_wait)
      case (current_input_sel)
      4'd0:   input_out =  input_a0_q;
      4'd1:   input_out =  input_a1_q;
      4'd2:   input_out =  input_a2_q;
      4'd3:   input_out =  input_a3_q;
      4'd4:   input_out =  input_a4_q;
      4'd5:   input_out =  input_a5_q;
      4'd6:   input_out =  input_a6_q;
      4'd7:   input_out =  input_a7_q;
      4'd8:   input_out =  input_b0_q;
      4'd9:   input_out =  input_b1_q;
      4'd10:  input_out =  input_b2_q;
      4'd11:  input_out =  input_b3_q;
      4'd12:  input_out =  input_b4_q;
      4'd13:  input_out =  input_b5_q;
      4'd14:  input_out =  input_b6_q;
      4'd15:  input_out =  input_b7_q;
      default:  input_out =  '0;
      endcase
  else
      input_out = '0;
  end


always_comb begin
      case (sel_acc)
      4'd0:   raw_sum_d =  '0;
      4'd1:   raw_sum_d =  raw_sum0_q;
      4'd2:   raw_sum_d =  raw_sum1_q;
      4'd3:   raw_sum_d =  raw_sum2_q;
      4'd4:   raw_sum_d =  raw_sum3_q;
      4'd5:   raw_sum_d =  raw_sum4_q;
      4'd6:   raw_sum_d =  raw_sum5_q;
      4'd7:   raw_sum_d =  raw_sum6_q;
      4'd8:   raw_sum_d =  raw_sum7_q;
      4'd9:   raw_sum_d =  raw_mul;
      4'd10:   raw_sum_d =  $signed(raw_mul)+ raw_sum_q;
      default:  raw_sum_d =  '0;
      endcase
  end


always_comb begin
// drive ALL outputs every time (dmux)
  rs_0 = raw_sum0_q; rs_1 = raw_sum1_q; rs_2 =raw_sum2_q; rs_3 = raw_sum3_q;
  rs_4 = raw_sum4_q; rs_5 = raw_sum5_q; rs_6 = raw_sum6_q; rs_7 =raw_sum7_q;

      case (get_raw_sum)
      3'd0:   if(track_raw_sum==0)
                    rs_0 =  raw_sum_q;

      3'd1:   if(track_raw_sum==1)
                    rs_1 =  raw_sum_q;
                 
      3'd2:   if(track_raw_sum==2)
                    rs_2 =  raw_sum_q;
                 
      3'd3:   if(track_raw_sum==3)
                    rs_3 =  raw_sum_q;
                 
      3'd4:   if(track_raw_sum==4)
                    rs_4 =  raw_sum_q;
                 
      3'd5:   if(track_raw_sum==5)
                    rs_5 =  raw_sum_q;
                
      3'd6:  if(track_raw_sum==6)
                    rs_6 =  raw_sum_q;
                 
      3'd7:   if(track_raw_sum==7)
                    rs_7 =  raw_sum_q;
      
      endcase
  end

 always_comb begin
      if (!save_raw_sum0)
      raw_sum0_d = rs_0;
      else 
      raw_sum0_d = raw_sum0_q;

       if (!save_raw_sum1)
      raw_sum1_d = rs_1;
      else 
      raw_sum1_d = raw_sum1_q;

       if (!save_raw_sum2)
      raw_sum2_d = rs_2;
      else 
      raw_sum2_d = raw_sum2_q;

       if (!save_raw_sum3)
      raw_sum3_d = rs_3;
      else 
      raw_sum3_d = raw_sum3_q;

       if (!save_raw_sum4)
      raw_sum4_d = rs_4;
      else 
      raw_sum4_d = raw_sum4_q;

       if (!save_raw_sum5)
      raw_sum5_d = rs_5;
      else 
      raw_sum5_d = raw_sum5_q;

       if (!save_raw_sum6)
      raw_sum6_d = rs_6;
      else 
      raw_sum6_d = raw_sum6_q;

       if (!save_raw_sum7)
      raw_sum7_d = rs_7;
      else 
      raw_sum7_d = raw_sum7_q;
 
end


 always_comb begin
  if(raw_sum_wait)
      case (sel_raw_sum)
      3'd0:   final_raw_sum =  raw_sum0_q;
      3'd1:   final_raw_sum =  raw_sum1_q;
      3'd2:   final_raw_sum =  raw_sum2_q;
      3'd3:   final_raw_sum =  raw_sum3_q;
      3'd4:   final_raw_sum =  raw_sum4_q;
      3'd5:   final_raw_sum =  raw_sum5_q;
      3'd6:   final_raw_sum =  raw_sum6_q;
      3'd7:   final_raw_sum =  raw_sum7_q;
      default:  final_raw_sum =  '0;
      endcase
  else
      final_raw_sum = '0;
  end


always_comb begin
  
  if (final_raw_sum>= 9'sd127)
      sum = 8'sd127;
  else if (final_raw_sum <= -9'sd128)
      sum = -8'sd128;
  else
      sum = $signed(final_raw_sum[DRAM_DQ_WIDTH-1:0]);

end

always_comb begin
// drive ALL outputs every time (dmux)
  s_0 = sum_0_q; s_1 = sum_1_q; s_2 = sum_2_q; s_3 =sum_3_q;
  s_4 =sum_4_q; s_5 =sum_5_q; s_6 = sum_6_q; s_7 = sum_7_q;

      case (get_sum)
      3'd0:   if(track_sum==0)
                    s_0 = sum;

      3'd1:   if(track_sum==1)
                    s_1 = sum;
                 
      3'd2:   if(track_sum==2)
                    s_2 = sum;
                 
      3'd3:   if(track_sum==3)
                    s_3 = sum;
                 
      3'd4:   if(track_sum==4)
                    s_4 = sum;
                 
      3'd5:   if(track_sum==5)
                    s_5 = sum;
                
      3'd6:  if(track_sum==6)
                    s_6 = sum;
                 
      3'd7:   if(track_sum==7)
                    s_7 = sum;
      endcase
  end

  always_comb begin
      if (!save_sum_0)
      sum_0_d = s_0;
      else 
      sum_0_d = sum_0_q;

      if (!save_sum_1)
      sum_1_d = s_1;
      else 
      sum_1_d = sum_1_q;

      if (!save_sum_2)
      sum_2_d = s_2;
      else 
      sum_2_d = sum_2_q;

      if (!save_sum_3)
      sum_3_d = s_3;
      else 
      sum_3_d = sum_3_q;

      if (!save_sum_4)
      sum_4_d = s_4;
      else 
      sum_4_d = sum_4_q;

      if (!save_sum_5)
      sum_5_d = s_5;
      else 
      sum_5_d = sum_5_q;
      
      if (!save_sum_6)
      sum_6_d = s_6;
      else 
      sum_6_d = sum_6_q;

      if (!save_sum_7)
      sum_7_d = s_7;
      else 
      sum_7_d = sum_7_q;

end

always_comb begin
  if(sum_wait)
      case (sel_sum)
      3'd0:   dq_out_d =  sum_0_q;
      3'd1:   dq_out_d =  sum_1_q;
      3'd2:   dq_out_d =  sum_2_q;
      3'd3:   dq_out_d =  sum_3_q;
      3'd4:   dq_out_d =  sum_4_q;
      3'd5:   dq_out_d =  sum_5_q;
      3'd6:   dq_out_d =  sum_6_q;
      3'd7:   dq_out_d =  sum_7_q;
      default:  dq_out_d =  '0;
      endcase
  else
      dq_out_d = '0;
  end

 always_comb begin
      case (out_addr_sel)
      2'b00:   out_addr_d = dram_addr_output;
      2'b01:   out_addr_d = out_addr_q;
      default: out_addr_d = '0;
      endcase
  end


//*********************************************************************************************************************************************************************************************





//***************************************************************Controller********************************************************************************************************************


 // State register update
 always_ff @(posedge clk or negedge reset_n) begin
  if (!reset_n)
    current_state <= S0;
  else
    current_state <= next_state;
 end


// Counter logic (for latency/turnaround tracking)
always_ff @(posedge clk or negedge reset_n) begin
  if (!reset_n)
    count <= 3'd0;
  else if ((current_state == S3)||(current_state == S8))
    count <= count + 3'd1;
  else
    count <= 3'd0;
end


// Counter logic (for latency/turnaround tracking)
always_ff @(posedge clk or negedge reset_n) begin
  if (!reset_n)
    count_1 <= 3'd0;
  else if ((current_state == S4)||(current_state == S9))
    count_1 <= count_1 + 3'd1;
  else
    count_1 <= 3'd0;
end

// Counter logic (for latency/turnaround tracking)
always_ff @(posedge clk or negedge reset_n) begin
  if (!reset_n)
    count_2 <= 3'd0;
  else if ((current_state == S5)||(current_state == S10))
    count_2 <= count_2 + 3'd1;
  else
    count_2 <= 3'd0;
end

always_ff @(posedge clk or negedge reset_n) begin
  if (!reset_n)
    count_3 <= 3'd0;
  else if (current_state == S18)
    count_3 <= count_3 + 3'd1;
  else
    count_3 <= 3'd0;
end

// counter logic for row sel
always_ff @(posedge clk or negedge reset_n) begin
  if (!reset_n)
    input_add <= 3'd0;
  else if (current_state == S16) begin
    if (input_add == 3'd7)
    input_add <= 3'd0;
  else
    input_add <= input_add + 3'd1;
  end
end

// counter logic for row sel
always_ff @(posedge clk or negedge reset_n) begin
  if (!reset_n)
    ker_add <= 3'd0;
  else if (current_state == S17) begin
    if (ker_add == 3'd3)
    ker_add <= 3'd0;
  else
    ker_add <= ker_add + 3'd1;
  end
end

always_ff @(posedge clk or negedge reset_n) begin
  if (!reset_n)
    count_4 <= 3'd0;
  else if (current_state == S19) begin
    if (count_4 == 3'd7)
    count_4 <= 3'd0;
  else
    count_4 <= count_4 + 3'd1;
  end
end

//column window increment until 0-127
always_ff @(posedge clk or negedge reset_n) begin
  if (!reset_n)
    col_window <= 7'd0;
  else if (current_state == S21) begin
    if (col_window == 7'd127)
    col_window <= 7'd0;
  else
    col_window <= col_window + 7'd1;
  end
end


//column out increment until 0-131071
always_ff @(posedge clk or negedge reset_n) begin
  if (!reset_n)
    col_out <= 17'd0;
  else if (current_state == S21) begin
    if (col_out == 17'd131071)
    col_out <= 17'd0;
  else
    col_out <= col_out + 17'd1;
  end
end

always_ff @(posedge clk or negedge reset_n) begin
  if (!reset_n)
    row_index <= 10'd0;
  else if (current_state == S22) begin
    if (row_index == 10'd1020)
      row_index <= row_index;       // hold at max
    else
      row_index <= row_index + 10'd1;
  end
end




always_comb
    begin
         // default: hold state
        next_state = current_state;
         //Default to prevent latches
        input_CMD=2'b00;  output_CMD=2'b00; input_oe=0; output_oe=0; ready=0; in_addr_sel=2'b00; sel_ker=0; sel_input=0; ker_sel=4'b0; beat_count=4'd0;
        save_ker_0=1; save_ker_1=1; save_ker_2=1; save_ker_3=1; save_ker_4=1; save_ker_5=1; save_ker_6=1; save_ker_7=1; save_ker_8=1; save_ker_9=1;
        save_ker_10=1; save_ker_11=1; save_ker_12=1; save_ker_13=1; save_ker_14=1; save_ker_15=1; current_ker_sel=4'b0; ker_wait=0; input_sel=4'b0;
        save_input_a0=1; save_input_a1=1; save_input_a2=1; save_input_a3=1; save_input_a4=1; save_input_a5=1; save_input_a6=1; save_input_a7=1;    
        save_input_b0=1; save_input_b1=1; save_input_b2=1; save_input_b3=1; save_input_b4=1; save_input_b5=1; save_input_b6=1; save_input_b7=1; 
        current_input_sel=4'b0; input_wait=0; sel_acc=4'b0; get_raw_sum=3'b0; track_raw_sum=3'b0; save_raw_sum0=1; save_raw_sum1=1; save_raw_sum2=1;
        save_raw_sum3=1; save_raw_sum4=1; save_raw_sum5=1; save_raw_sum6=1; save_raw_sum7=1; sel_raw_sum=3'b0; raw_sum_wait=0; get_sum=3'b0; track_sum=3'b0;
        save_sum_0=1; save_sum_1=1; save_sum_2=1; save_sum_3=1; save_sum_4=1; save_sum_5=1; save_sum_6=1; save_sum_7=1; sel_sum=3'b0; output_count=8'b0; sum_wait=0;
        out_addr_sel=2'b0; row_next=1'd0; col_next=1'b0;
        
        
       
        
        case (current_state)
          S0: begin   
           // On reset, transition to idle state
           
           next_state = S1;
        end

        S1: begin
        //select first kernal
        ready = 1'b1;
        if (start)begin
          in_addr_sel=2'b00;
          next_state = S2;
        end else begin
          next_state = S1;
        end
        end

         S2: begin
        //Dram receives first kernal address & select second kernal
        input_CMD = 2'b01;
        in_addr_sel=2'b01;
        next_state = S3;
        end

         S3: begin
        // Wait 5 cycles for READ latency & hold second kernal address
        in_addr_sel=2'b11; //hold 2nd kernal address
        if (count == 3'd5)
          next_state = S4;
        else
          next_state = S3;
        end


      S4: begin
        sel_ker    = 1'b1;

        // 1) Fire READ for kernel 1 exactly once at S4 one clock cycle after entry, then hold address
        if (count_1 == 3'd1) begin
          input_CMD  = 2'b01;        // READ
        end else begin
          in_addr_sel=2'b11; //hold 2nd kernal address
        end

        // 2) Stream first 8 beats into ker_7..ker_0
        ker_sel    = 4'd7 - count_1;           // 7..0
        beat_count = {1'b0, count_1};          // 0..7

        // default holds are already 1 at top of always_comb
        unique case (4'd7 - count_1)
          4'd7: save_ker_7 = 1'b0;
          4'd6: save_ker_6 = 1'b0;
          4'd5: save_ker_5 = 1'b0;
          4'd4: save_ker_4 = 1'b0;
          4'd3: save_ker_3 = 1'b0;
          4'd2: save_ker_2 = 1'b0;
          4'd1: save_ker_1 = 1'b0;
          4'd0: save_ker_0 = 1'b0;
        endcase

        // 3) When done with 8 beats, move to S5 (which loads ker_15..ker_8)
        if (count_1 == 3'd7)
          next_state = S5;
        else
          next_state = S4;
       end


      S5: begin
        sel_ker     = 1'b1;
        in_addr_sel = 2'b11;                   // hold kernel_1 address during this burst

        ker_sel     = 4'd15 - count_2;        // 15..8
        beat_count  =  4'd8 + count_2;         // 8..15

        unique case (4'd15 - count_2)
          4'd15: save_ker_15 = 1'b0;
          4'd14: save_ker_14 = 1'b0;
          4'd13: save_ker_13 = 1'b0;
          4'd12: save_ker_12 = 1'b0;
          4'd11: save_ker_11 = 1'b0;
          4'd10: save_ker_10 = 1'b0;
          4'd9 : save_ker_9  = 1'b0;
          4'd8 : save_ker_8  = 1'b0;
        endcase

        if (count_2 == 3'd7)
          next_state = S6;    
        else
          next_state = S5;
      end

      
      S6: begin 
       //select input address 
          row_next=ker_add;  //incerment for row address 0-3

          col_next=2'b0; //adds to col_window whe generating another address
          in_addr_sel=2'b10; //selects dram address
          next_state = S7;

        end

      S7: begin
        //Dram receives first input address & select second input address
        row_next=ker_add; //incerment for row address 0-3

        input_CMD = 2'b01;
        col_next = 2'b1; //adds to col_window whe generating another address
        in_addr_sel=2'b10;
        next_state = S8;
        end
      
      S8: begin
        // Wait 5 cycles for READ latency & hold second input address
        in_addr_sel=2'b11; //hold 2nd input address
        if (count == 3'd5)
          next_state = S9;
        else
          next_state = S8;
        end

       
      S9: begin
        sel_input    = 1'b1;
        // 1) Fire READ for input0 exactly once at S9 one clock cycle after entry, then hold address
        if (count_1 == 3'd1) begin
          input_CMD  = 2'b01;        // Dram reads input address 1
        end else begin
          in_addr_sel=2'b11; //hold 2nd address
        end

        // 2) Stream first 8 beats into ker_7..ker_0
        input_sel    = 4'd7 - count_1;           // 7..0
        beat_count = {1'b0, count_1};          // 0..7

        // default holds are already 1 at top of always_comb
        unique case (4'd7 - count_1)
          4'd7: save_input_a7 = 1'b0;
          4'd6: save_input_a6 = 1'b0;
          4'd5: save_input_a5 = 1'b0;
          4'd4: save_input_a4 = 1'b0;
          4'd3: save_input_a3 = 1'b0;
          4'd2: save_input_a2 = 1'b0;
          4'd1: save_input_a1 = 1'b0;
          4'd0: save_input_a0 = 1'b0;
        endcase

        // 3) When done with 8 beats, move to S5 (which loads input_b15..input_b8)
        if (count_1 == 3'd7)
          next_state = S10;
        else
          next_state = S9;
       end


      S10: begin
        sel_input    = 1'b1;
        in_addr_sel = 2'b11;                   // hold address1 address during this burst
        row_next=ker_add;                     //incerment for row address 0-3
        input_sel     = 4'd15 - count_2;        // 15..8
        beat_count  =  4'd8 + count_2;         // 8..15

        unique case (4'd15 - count_2)
          4'd15: save_input_b7 = 1'b0;
          4'd14: save_input_b6 = 1'b0;
          4'd13: save_input_b5 = 1'b0;
          4'd12: save_input_b4 = 1'b0;
          4'd11: save_input_b3 = 1'b0;
          4'd10: save_input_b2 = 1'b0;
          4'd9 : save_input_b1  = 1'b0;
          4'd8 : save_input_b0  = 1'b0;
        endcase

        if (count_2 == 3'd7) begin
        if (row_next == 3'd0)
          next_state = S12;  // Begin calculation for first row
        else
          next_state = S11;  // For rows beyond first, use S11 pre-step
        end else begin
        next_state = S10;    // Keep loading
        end
            
      end
      
      S11: begin
        sel_acc = 4'd1 + input_add;
        next_state = S12;
      end



      S12: begin//calculation starts
        row_next=ker_add;                     //incerment for row address 0-3

        input_wait= 1;
        ker_wait= 1;

        if (ker_add==3'd0) begin
        current_ker_sel=4'd0;
        end 
        else if (ker_add == 3'd1) begin
        current_ker_sel=4'd4; 
        end
        else if (ker_add == 3'd2) begin
        current_ker_sel=4'd8; 
        end
        else begin
        current_ker_sel=4'd12; 
        end

        current_input_sel=4'd0 + input_add;
        
        if (row_next==0)begin
        sel_acc=4'd9;// multiplier
        end
        else begin
        sel_acc=4'd10;
        end

        next_state = S13;
      end
      
      S13: begin
        input_wait= 1;
        ker_wait= 1;

        if (ker_add==3'd0) begin
        current_ker_sel=4'd1;
        end 
        else if (ker_add == 3'd1) begin
        current_ker_sel=4'd5; 
        end
        else if (ker_add == 3'd2) begin
        current_ker_sel=4'd9; 
        end
        else begin
        current_ker_sel=4'd13; 
        end


        current_input_sel=4'd1 + input_add;
        sel_acc=4'd10;// multiplier and adder
        next_state = S14;
      end

      S14: begin
        input_wait= 1;
        ker_wait= 1;

        if (ker_add==3'd0) begin
        current_ker_sel=4'd2;
        end 
        else if (ker_add == 3'd1) begin
        current_ker_sel=4'd6; 
        end
        else if (ker_add == 3'd2) begin
        current_ker_sel=4'd10; 
        end
        else begin
        current_ker_sel=4'd14; 
        end

        current_input_sel=4'd2 + input_add;
        sel_acc=4'd10;// multiplier and adder
        next_state = S15;
      end

       S15: begin
        input_wait= 1;
        ker_wait= 1;

         if (ker_add==3'd0) begin
        current_ker_sel=4'd3;
        end 
        else if (ker_add == 3'd1) begin
        current_ker_sel=4'd7; 
        end
        else if (ker_add == 3'd2) begin
        current_ker_sel=4'd11; 
        end
        else begin
        current_ker_sel=4'd15; 
        end

        current_input_sel=4'd3 + input_add;
        sel_acc=4'd10;// multiplier and adder
        next_state = S16;
      end


      S16: begin
      //moves accross the row and saves row 0
      get_raw_sum = input_add;
      track_raw_sum = input_add;

      case (input_add)
      3'd0: save_raw_sum0 = 0;
      3'd1: save_raw_sum1 = 0;
      3'd2: save_raw_sum2 = 0;
      3'd3: save_raw_sum3 = 0;
      3'd4: save_raw_sum4 = 0;
      3'd5: save_raw_sum5 = 0;
      3'd6: save_raw_sum6 = 0;
      3'd7: save_raw_sum7 = 0;
      endcase

    
      if ((input_add < 3'd7)&&(ker_add == 3'd0))
      next_state = S12;
      else if ((input_add < 3'd7)&&(ker_add > 3'd0))
      next_state = S11;
      else
      next_state = S17;
      end
    


      S17: begin
      //increment rows 4 times  after completing 7 col shifts in each individual row 
      if (ker_add < 3'd3)
      next_state = S6;
      else
      next_state = S18;

      end



     S18: begin
     raw_sum_wait = 1;
     sel_raw_sum = count_3;
     get_sum = count_3;
     track_sum = count_3;

      case (count_3)
      3'd0: save_sum_0 = 0;
      3'd1: save_sum_1 = 0;
      3'd2: save_sum_2 = 0;
      3'd3: save_sum_3 = 0;
      3'd4: save_sum_4 = 0;
      3'd5: save_sum_5 = 0;
      3'd6: save_sum_6 = 0;
      3'd7: save_sum_7 = 0;
      endcase

     if (count_3==3'd4)begin //send output write command
      output_CMD = 2'b10;
     end 
     else begin
      output_CMD = 2'b00;
     end

     if (count_3 < 3'd7)begin
      next_state = S18;
     end
     else begin
      next_state = S19;
     end

     end 


     S19: begin // Dut writes first address to dram
     
      if ((col_window==7'd127) && (count_4<3'd3))begin
      sum_wait=0;
      end
      else begin
      sum_wait = 1;
      end

      output_oe  = 1'b1;       // enable driving the bus
      sel_sum =3'd7 - count_4; 

     if (count_4 < 3'd7)begin
      next_state = S19;
     end
     else begin
      next_state = S20;
     end

     end

     S20:begin // Dram accepts last data from dut
     output_oe  = 1'b1;
     next_state =S21;
     end


    S21: begin //complete one whole row calculation
    
    
    if (col_window < 7'd127)

     next_state = S6;
    else 
    next_state = S22;

    end


   /* S22:begin

       next_state = S1;
     end
*/
  
S22: begin
  if (row_index == 10'd1020)
    next_state = S23;
  else
    next_state = S6;
end


   S23: begin
 
    next_state = S1; 
end



     endcase
    end
endmodule                                           

