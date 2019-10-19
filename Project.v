// Part 2 skeleton

module Project
	(
		CLOCK_50,						//	On Board 50 MHz
		// Your inputs and outputs here
        KEY,
        SW,
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,
		HEX5,
		HEX4,
		HEX3,
		HEX2,
		HEX1,
		HEX0   //	VGA Blue[9:0]
	);

	input			CLOCK_50;				//	50 MHz
	input   [9:0]   SW;
	input   [3:0]   KEY;
	output [6:0] HEX5,HEX4,HEX3,HEX2,HEX1,HEX0;

	// Declare your inputs and outputs here
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
	
	wire resetn;
	assign resetn = KEY[0];
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.
	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn/plot
	// for the VGA controller, in addition to any other functionality your design may require.
    

	 wire w0,w1,w2,w3,w4,w5,w6,w7,w8,w9,w10,w11,w12,w13;
	 wire w14,w15,w16,w17,w18;
	 wire w19,w20,w21;
	 wire w22, w23, w24,w25,w26,w27;


	 
	 phase3_score_keeper count(CLOCK_50, w27,HEX5, HEX4, HEX3, HEX2, HEX1, HEX0, w26);
	 
	 

	 	 
     datapath d(
	 .clk(CLOCK_50),
	 .r_set(KEY[0]),
	 .in_colour_enable(w0),
	 .in_draw_start(w18),
	 .in_counter_start(w2),
	 .in_draw_reset(w3),
	 .out_draw_end(w4), 
	 .out_count_end(w5), 
	 .in_counter_reset(w6), 
	 .in_jump_start(w7),
	 .in_jump_reset(w8),
	 .out_jump_end(w9),
	 .out_x(x),
	 .in_right_mode(w13),
	 .out_y(y),
	 .in_left_mode(w11),
	 .in_jump_mode(w12),
	 .out_c(colour),
	 .in_rec(w14),
	 .in_draw_start_rec(w16),
	 .in_draw_reset_rec(w17),
	 .out_draw_end_rec(w15),
	 .out_finish_rec(w19),
	 .in_x_rec_reset(w20),
	 .in_x_rec_increase(w21),
	 .color_control(SW[9:7]),
	 .out_rec_indicator(w22),
	 .in_jump_finish_reset(w23),
	 .out_pause(w24),
	 .in_stop_pause(w25),
	 .speed(HEX1),
	 .in_double(~KEY[3])
	 );
	 
	  control cont(
	  .clk(CLOCK_50), 
	  .r_set(KEY[0]), 
	  .in_jump_press(~KEY[2]),
	  .in_jump_press_1(~KEY[1]),
	  .out_colour_enable(w0), 
	  .out_draw_start(w18), 
	  .out_draw_reset(w3), 
	  .in_draw_end(w4),
	  .out_counter_start(w2), 
	  .in_counter_end(w5),
	  .out_counter_reset(w6),
	  .out_jump_start(w7),
	  .out_right_mode(w13),
	  .out_jump_reset(w8),
	  .out_left_mode(w11),
	  .out_jump_mode(w12),
	  .in_jump_end(w9),
	  .out_draw_reset_rec(w17),
	  .out_draw_start_rec(w16),
	  .out_rec(w14),
	  .in_draw_end_rec(w15),
	  .in_finish_rec(w19),
	 .out_x_rec_reset(w20),
	 .out_x_rec_increase(w21),
	 .in_rec_indicator(w22),
	  .out_jump_finish_reset(w23),
	  .in_pause(w24),
	  .stop_pause(w25),
	  .out_hex_enable(w26),
	  .out_hex_reset(w27)
	  );
	  
	  assign writeEn = w18 || w16;
endmodule

// in_colour_enable : whether has colour
// in_left_mode : return back from control to see whether there is only left leg shows in the register
// in_jump_mode : control whether show both legs
// in_draw_start: inform begin to draw the diagram
// in_counter_start: inform begin to start the counter
// in_draw_reset: the draw's reset
// out_draw_end: send out the message about finising drawing
// in_jump_start: enable for y location to go up
// in_jump_reset: reset for y location
// out_jump_end: send out the message about already finishing jumping
// out_count_end: when finish counting
// in_right_mode: show right leg
module datapath(clk,speed,in_double,color_control,in_stop_pause,out_pause,in_jump_finish_reset,r_set,out_rec_indicator,in_colour_enable,in_x_rec_reset,in_x_rec_increase,in_left_mode,in_jump_mode, in_rec, in_draw_start_rec, out_finish_rec,in_draw_reset_rec, out_draw_end_rec, in_draw_start, in_counter_start,in_draw_reset,out_draw_end, in_jump_start, in_jump_reset, out_jump_end, out_count_end, in_right_mode,in_counter_reset, out_x,out_y,out_c);
   
	input clk,r_set,in_colour_enable,in_draw_start,in_counter_start,in_draw_reset,in_counter_reset,in_jump_start, in_jump_reset;
	input in_left_mode;
	input in_jump_mode;
	input in_draw_start_rec, in_draw_reset_rec;
	input [2:0]color_control;
	input [6:0] speed;
	reg in_double_jump;
	input in_double;
	

	output reg [7:0]out_x;
	output reg [6:0]out_y;
	output [2:0]out_c;
	output out_draw_end;
	output out_count_end;
	output out_jump_end;
	input in_right_mode;
	input in_rec;

	
	wire [13:0] colour_out;	
	wire [5:0] increase_x;
	wire[5:0] increase_y;
	
	wire [5:0] increase_x_di;
	wire[5:0] increase_y_di;
	
	wire [5:0] increase_x_rec;
	wire[5:0] increase_y_rec;

	wire register_enable;
	wire[7:0] x;
	wire[6:0] y;
	wire[7:0] x_di;
	wire[6:0] y_di;
   reg [2:0] c;
	
	wire[7:0] x_rec;
	wire[6:0] y_rec;
	
	output out_draw_end_rec;
	output out_finish_rec;
	output reg out_pause;
	input in_x_rec_reset;
	input in_x_rec_increase;
	output reg out_rec_indicator;
	input in_jump_finish_reset;
	input in_stop_pause;

	wire [3:0]q1;
	wire [3:0] q2;
	reg [3:0] q3;
	wire [3:0] q4;
	
	reg count1;
	reg count2;
	//slow speed
   framcounter f(in_counter_start,clk,in_counter_reset,q1,speed);

	// jump speed
   framcounter1 f1 (in_counter_start,clk,in_counter_reset,q2,speed);
	framcounter2 f2(in_counter_start,clk,in_counter_reset,q4);

	
	always @(*)
		begin
		if(in_jump_mode)
			begin
			count1 = (q2 == 4'b0100) ? 1:0;
//			count2 = (q2 == 4'b0001) ? 1:0;
			end
//		else if (!in_stop_pause)
//			count1 = (q4 == 5'b11111) ? 1:0;
		else
			count1 = (q1 == 4'b1001) ? 1:0;
		end
   
	assign out_count_end = count1;
	
	
	

		
	always @(posedge clk)
		begin
			if(in_jump_finish_reset == 1'b0)
				begin
				q3 <= 4'b0000;
				out_rec_indicator <= 1'b0;
				end
			else
				begin
				if(in_jump_start)
					begin 
						if(q3 == 4'b0001)
							begin
							q3 <= 4'b0000;
							out_rec_indicator <= 1'b1;
							end
						else
							begin
							q3 <= q3 + 1'b1;
							out_rec_indicator <= 1'b0;
							end
					end
				end
		end	




		
	always @ (*)
		begin
		if(!r_set)
			in_double_jump <= 1'b0;
		else
			begin
			if(in_jump_mode)
				begin
				if(in_double)
					in_double_jump <= 1'b1;
				else if (out_jump_end)
					in_double_jump <= 1'b0;
				else
					in_double_jump <= in_double_jump;
				end
			else
				in_double_jump <= 1'b0;
			end
		end
 
				
			
   // Draw dina
	xlocation x1(clk, 1'b1, 1'b1, x_di);
	ylocation y1(clk, in_jump_reset, in_jump_start, y_di, out_jump_end, in_double_jump);
	pixelincrement incre(clk, in_draw_reset, in_draw_start, increase_x_di, increase_y_di, register_enable, out_draw_end);
	registers r(clk,in_draw_reset, in_left_mode, in_right_mode,in_jump_mode, register_enable, colour_out);
	
	// Draw rec
	xlocation_rec x2 (clk, in_x_rec_increase, in_x_rec_reset, x_rec, out_finish_rec);
	ylocation_rec y2 (clk, 1'b1, 1'b1, y_rec);
	pixelincrement_rec incre1(clk, in_draw_reset_rec, in_draw_start_rec, increase_x_rec, increase_y_rec, out_draw_end_rec);
	
	
	always @(*)
		begin
			if(in_colour_enable && in_rec)
				c = color_control;
			else
				begin		
				if(in_colour_enable && colour_out[13 - increase_y_di] == 1'b1)
					c = color_control;
				else
					c = 3'b000;
				end
		 end
		 

	always @(*)
	   begin
		if(in_rec)
			begin
			out_x = x_rec + increase_x_rec;
			out_y = y_rec + increase_y_rec;
			end
		else
			begin 
			out_x = x_di + increase_x_di;
			out_y = y_di + increase_y_di;
			end
		end

//	assign out_x =	increase_x + x;
//	assign out_y = increase_y + y;	
   assign out_c = c;
	
	always @ (*)
		begin		
//		if(!r_set)
//			out_pause <= 1'b0;
//		else
//			begin
		if(!in_stop_pause)
			out_pause <= 1'b0;
		else

				if((x_rec <= out_x) & (out_x <= x_rec + 3'd4) & (out_y <= 7'd94) & (out_y >= 7'd74) & (c != 3'b000) & (!in_rec))
							begin
							out_pause <= 1'b1;
							end
				else
							begin
							out_pause <= 1'b0;
							end

		end

endmodule



module control(clk, r_set,out_hex_enable, out_hex_reset, out_counter_reset, stop_pause,in_pause,in_finish_rec, out_jump_finish_reset,in_rec_indicator, out_x_rec_reset, out_x_rec_increase, in_jump_press, in_jump_press_1, out_draw_reset_rec,out_draw_start_rec,out_rec,in_draw_end_rec,out_colour_enable, out_left_mode,out_right_mode,out_jump_mode,out_draw_start, out_draw_reset, in_draw_end, out_jump_start, in_jump_end, out_jump_reset, out_counter_start, in_counter_end,out_counter_reset);
	 input clk, r_set, in_draw_end, in_counter_end, in_jump_end, in_jump_press;
	 input in_jump_press_1;
	 output reg out_colour_enable, out_draw_start, out_draw_reset,out_counter_start,out_counter_reset,out_jump_start,out_jump_reset;
	 output reg out_right_mode;
	 output reg out_draw_reset_rec;
	 output reg out_draw_start_rec;
	 output reg out_rec;
	 input in_draw_end_rec;
	 input in_finish_rec;
	 output reg out_x_rec_reset, out_x_rec_increase;
	 input in_rec_indicator;
	 input in_pause;
	 output reg out_jump_finish_reset;



	 
	 output reg stop_pause;

	 output reg out_left_mode,out_jump_mode;
	 reg [6:0] current, next;
	 output reg out_hex_enable;
	 output reg out_hex_reset;

	
	 localparam Draw = 7'd0,
					Counter_state_left = 7'd1,
					Erase_Jump_1 = 7'd2,
					Counter_state_left_1 = 7'd3,
					Jump = 7'd4,
					Counter_state_1 = 7'd5,
					Erase_Jump = 7'd6,
					Update = 7'd7,
					Update1 = 7'd8,
					Erase_left= 7'd9,
					Reset = 7'd10,
					Reset_wait = 7'd11,
					Left = 7'd12,
					Right = 7'd13,
					Counter_state_right = 7'd14,
					Erase_right = 7'd15,
					Counter_state_right_1 = 7'd16,
					Left_and_Right = 7'd17,
					Left_and_Right_1 = 7'd18,
					Counter_state_both = 7'd19, 
					Counter_state_both_1 = 7'd20,
					Erase_both_1 = 7'd21,
					Erase_both = 7'd22,
					Wait = 7'd23,
					Wait_1 = 7'd24,
					Draw_rec = 7'd25,
					Draw_rec_1 = 7'd26,
					Draw_rec_2 = 7'd27,
					Draw_rec_3 = 7'd28,
					Erase_rec = 7'd29,
					Erase_rec_1 = 7'd30,
					Erase_rec_2 = 7'd31,
					Erase_rec_3 = 7'd32,
					Erase_rec_4 = 7'd33,
					Update_rec  = 7'd34,
					Update_rec1 = 7'd35,
					Update_rec2 = 7'd36,
					Update_rec3 = 7'd37,
					Pause = 7'd38,
					Pause1 = 7'd39,
					Pause2 = 7'd40;


	//state table
	always @(*)
		begin
			case(current)
			
			   //Run	
				Reset: next = in_jump_press ? Left : Reset;
				Left: next = in_draw_end ? Draw_rec: Left;
				Draw_rec: next = in_draw_end_rec ? Counter_state_left : Draw_rec;
            Counter_state_left: next = in_counter_end ? Erase_left : Counter_state_left;
            Erase_left:	next = in_draw_end ? Erase_rec : Erase_left;
				Erase_rec: next = in_draw_end_rec ? Counter_state_left_1: Erase_rec;
				//Update_rec: next = Counter_state_left_1;
				Counter_state_left_1: next = Left_and_Right;

				
				Left_and_Right: next = in_draw_end ? Draw_rec_1 : Left_and_Right;
				Draw_rec_1: next = in_draw_end_rec ? Counter_state_both : Draw_rec_1;
				Counter_state_both: next = in_counter_end ? Erase_both : Counter_state_both;
				Erase_both: next = in_draw_end ? Erase_rec_1 : Erase_both;
				Erase_rec_1: next = in_draw_end_rec ? Wait : Erase_rec_1;
				//Update_rec: next = W;
				Wait: next = Right;
				
				Right: next =  in_draw_end ? Draw_rec_2 : Right;
				Draw_rec_2: next = in_draw_end_rec ? Counter_state_right : Draw_rec_2;
            Counter_state_right: next = in_counter_end ? Erase_right : Counter_state_right;
				Erase_right: next = in_draw_end ?  Erase_rec_2 : Erase_right;
				Erase_rec_2: next = in_draw_end_rec ? Counter_state_right_1 : Erase_rec_2;
				Counter_state_right_1: next = Left_and_Right_1;
				
				
				
				Left_and_Right_1: next = in_draw_end ? Draw_rec_3 : Left_and_Right_1;
				Draw_rec_3: next = in_draw_end_rec ? Counter_state_both_1 : Draw_rec_3;
				Counter_state_both_1:next = in_counter_end ? Erase_both_1 : Counter_state_both_1;
				Erase_both_1: next = in_draw_end ? Erase_rec_3 : Erase_both_1;
				Erase_rec_3: next = in_draw_end_rec ? Wait_1 : Erase_rec_3;
				Wait_1: next = Left;
				Erase_Jump_1: next = in_draw_end ? Erase_rec_4 : Erase_Jump_1;
				Erase_rec_4: next = in_draw_end_rec ? Reset : Erase_rec_4;
				
				Pause: next = in_draw_end ? Pause1: Pause;
				Pause1: next = in_draw_end_rec ? Pause2 : Pause1;
				Pause2: next = Pause2;
		
	
				//Jump
            Draw: next = in_draw_end ? Update_rec : Draw;
				//Draw rec
				Update_rec: next = in_draw_end_rec ? Counter_state_1 : Update_rec;
				Counter_state_1: next = in_counter_end ? Erase_Jump : Counter_state_1;
				//Erase rec
				Update_rec1: next = in_draw_end_rec ? Update_rec2 : Update_rec1;
				Erase_Jump: next = in_draw_end ? Update : Erase_Jump;
				Update_rec2: next = Update;
				Jump: next = Draw;
				Update:
					begin
					if(in_jump_end)
						next = Erase_both_1;
					else
						begin
						if(in_rec_indicator)
							next = Update_rec1;
  					   else
							next = Jump;
						end
					end			
				default: next = Reset;
			endcase
		end

	
	//flags
	always @(*) 
		begin
			out_draw_start = 1'b0;
			out_counter_start = 1'b0;
			out_colour_enable = 1'b0;
			out_counter_reset = 1'b0;
			out_draw_reset = 1'b0;
			out_jump_start = 1'b0;
			out_jump_reset = 1'b1;
			out_left_mode = 1'b0;
			out_right_mode = 1'b0;
			out_jump_mode = 1'b0;
			out_draw_reset_rec = 1'b0;
			out_draw_start_rec = 1'b0;
			out_rec = 1'b0;
			out_x_rec_reset = 1'b1;
			out_x_rec_increase = 1'b0;
			out_jump_finish_reset = 1'b1;
			stop_pause = 1'b1;
			out_hex_reset = 1'b1;
			out_hex_enable = 1'b1;

		
			case(current)
				Pause2:
					begin
				   out_hex_enable = 1'b0;
				   stop_pause = 1'b0;
					end
				Left_and_Right:
					begin
					out_colour_enable = 1'b1;
					out_draw_start = 1'b1;
					out_draw_reset = 1'b1;
					end
				Counter_state_left_1:
					begin
					out_x_rec_increase = 1'b1;
					end
				Wait:
					begin
					out_x_rec_increase = 1'b1;
					end			
				Counter_state_right_1:
					begin
					out_x_rec_increase = 1'b1;
					end
				Update:
					begin
					out_jump_mode = 1'b1;
					end
				Wait_1:
					begin
					out_x_rec_increase = 1'b1;
					end	
				Update_rec:
					begin
					out_colour_enable= 1'b1;
					out_draw_start_rec = 1'b1;
					out_draw_reset_rec = 1'b1;
					out_rec = 1'b1;
					out_jump_mode = 1'b1;
					end
				Update_rec1:
					begin
					out_colour_enable= 1'b0;
					out_draw_start_rec = 1'b1;
					out_draw_reset_rec = 1'b1;
					out_rec = 1'b1;
					out_jump_mode = 1'b1;
					end
				Draw_rec:
					begin
					out_colour_enable= 1'b1;
					out_draw_start_rec = 1'b1;
					out_draw_reset_rec = 1'b1;
					out_rec = 1'b1;
					end
				Draw_rec_1:
					begin
					out_colour_enable= 1'b1;
					out_draw_start_rec = 1'b1;
					out_draw_reset_rec = 1'b1;
					out_rec = 1'b1;
					end				
				Draw_rec_2:
					begin
					out_colour_enable= 1'b1;
					out_draw_start_rec = 1'b1;
					out_draw_reset_rec = 1'b1;
					out_rec = 1'b1;
					end	
				Draw_rec_3:
					begin
					out_colour_enable= 1'b1;
					out_draw_start_rec = 1'b1;
					out_draw_reset_rec = 1'b1;
					out_rec = 1'b1;
					end	
				Left_and_Right_1:
					begin
					out_colour_enable = 1'b1;
					out_draw_start = 1'b1;
					out_draw_reset = 1'b1;
					end
				Counter_state_both:
					begin
					out_counter_start = 1'b1;
					out_counter_reset = 1'b1;
					end
				Counter_state_both_1:
					begin
					out_counter_start = 1'b1;
					out_counter_reset = 1'b1;
					end
				Erase_both_1:
					begin
					out_colour_enable = 1'b0;
					out_draw_start = 1'b1;
					out_draw_reset = 1'b1;	
					out_jump_reset = 1'b0;	
					end
				Erase_both:
					begin
					out_colour_enable = 1'b0;
					out_draw_start = 1'b1;
					out_draw_reset = 1'b1;	
					end
				Erase_rec:
					begin
					out_colour_enable= 1'b0;
					out_draw_start_rec = 1'b1;
					out_draw_reset_rec = 1'b1;
					out_rec = 1'b1;
					end
				Erase_rec_1:
					begin
					out_colour_enable= 1'b0;
					out_draw_start_rec = 1'b1;
					out_draw_reset_rec = 1'b1;
					out_rec = 1'b1;
					end
				Erase_rec_2:
					begin
					out_colour_enable= 1'b0;
					out_draw_start_rec = 1'b1;
					out_draw_reset_rec = 1'b1;
					out_rec = 1'b1;
					end
				Erase_rec_3:
					begin
					out_colour_enable= 1'b0;
					out_draw_start_rec = 1'b1;
					out_draw_reset_rec = 1'b1;
					out_rec = 1'b1;
					end
				Erase_rec_4:
					begin
					out_colour_enable= 1'b0;
					out_draw_start_rec = 1'b1;
					out_draw_reset_rec = 1'b1;
					out_rec = 1'b1;
					end
				//Left
				Left:
					begin
					out_colour_enable = 1'b1;
					out_draw_start = 1'b1;
					out_draw_reset = 1'b1;
					out_left_mode = 1'b1;
					out_right_mode = 1'b0;
				 	end
				Right:
					begin
					out_colour_enable = 1'b1;
					out_draw_start = 1'b1;
					out_draw_reset = 1'b1;
					out_left_mode = 1'b0;
					out_right_mode = 1'b1;
					end
				Counter_state_left:
					begin
					out_counter_start = 1'b1;
					out_counter_reset = 1'b1;
					end

				Erase_left:
					begin
					out_colour_enable = 1'b0;
					out_draw_start = 1'b1;
					out_draw_reset = 1'b1;
					end		
				Counter_state_right:
					begin
					out_counter_start = 1'b1;
					out_counter_reset = 1'b1;
					end
				Erase_right:
					begin
					out_colour_enable = 1'b0;
					out_draw_start = 1'b1;
					out_draw_reset = 1'b1;		
					end	
				Reset:
					begin
					out_jump_reset = 1'b0;
					out_colour_enable = 1'b1;
					out_draw_start = 1'b1;
					out_draw_reset = 1'b1;
					out_x_rec_reset = 1'b0;
					out_jump_finish_reset = 1'b0;
					out_hex_reset = 1'b0;
					end
				Draw:
					begin
					out_colour_enable = 1'b1;
					out_draw_start = 1'b1;
					out_draw_reset = 1'b1;
					out_jump_mode = 1'b1;
					end
				Update_rec2:
					begin
					out_x_rec_increase = 1'b1;
					out_jump_finish_reset = 1'b0;
					out_jump_mode = 1'b1;
					end
				Jump:
					begin
					out_jump_start = 1'b1;
				   out_jump_mode = 1'b1;
					end					
				Counter_state_1:
					begin
					out_counter_start = 1'b1;
					out_counter_reset = 1'b1;
					out_jump_mode = 1'b1;
					end
				Erase_Jump:
					begin
					out_colour_enable = 1'b0;
					out_draw_start = 1'b1;
					out_draw_reset = 1'b1;
			      out_jump_mode = 1'b1;	
					end
				Erase_Jump_1:
					begin
					out_colour_enable = 1'b0;
					out_draw_start = 1'b1;
					out_draw_reset = 1'b1;
			      out_jump_reset = 1'b1;		
					end
				Pause:
					begin
					out_colour_enable = 1'b1;
					out_draw_start = 1'b1;
					out_draw_reset = 1'b1;
					stop_pause = 1'b0;
					out_hex_enable = 1'b0;
					end
				Pause1:
					begin
					out_colour_enable= 1'b1;
					out_draw_start_rec = 1'b1;
					out_draw_reset_rec = 1'b1;
					out_rec = 1'b1;
					stop_pause = 1'b0;
					out_hex_enable = 1'b0;
					end
			endcase
		
			
			
	  end
		
		
	//state transition
//	always @(posedge clk)
//		begin
//			if(!r_set)
//			   current <= Erase_static;
//			else if(in_jump_press)
//			  current <= Jump_wait;
//			else if(in_jump_end)
//				current <= Erase_Jump_1;
//			else
//				current <= next;
//
//		end

		always @(posedge clk)
		begin
			if(!r_set)
			   current <= Erase_Jump_1;
			else if((!out_jump_mode) & in_jump_press_1)
				current <= Draw;
		   else if (stop_pause & in_pause)
				current <= Pause;
			else
				begin
					current <= next;
				end
		end
		
endmodule



module pixelincrement_rec(clk, r_set, enable,ix,iy, finish);
	input clk,r_set;
	output reg [5:0] ix,iy;
	input enable;
	output reg finish;
	reg en_iy;
	
	always @(posedge clk, negedge r_set)
		begin
			if(!r_set)
				begin
					ix <= 6'd0;
					iy <= 6'd0;	
					finish <= 1'b0;
				end
			else
			begin
				if(enable)
					begin
						if(ix == 6'd3)
							begin
								ix <= 6'd0;
								en_iy <= 1'b1;
							end
						else
							begin
								ix <= ix + 6'd1;
								en_iy <= 1'b0;
							end
						if(en_iy)
							begin
							if(iy == 6'd19)
								iy <= iy;
							else
								iy <= iy + 6'd1;
							end
						if(ix == 6'd3 && iy == 6'd19)
							begin
							finish <= 1'b1;
							end
					end
			end
		end
endmodule
	




module pixelincrement(clk, r_set, enable,ix,iy, enable_register, finish);
	input clk,r_set;
	output reg [5:0] ix,iy;
	input enable;
	output reg finish;
	output reg enable_register;
	reg en_ix;
	
	always @(posedge clk, negedge r_set)
		begin
			if(!r_set)
				begin
					ix <= 6'd0;
					iy <= 6'd0;
					enable_register <= 1'b0;
					finish <= 1'b0;
				end
			else
			begin
				 if(enable)
					begin
						if(iy == 6'd0)
							enable_register <= 1'b1;
						else
							enable_register <= 1'b0;
						if(iy == 6'd13)
							begin
								iy <= 6'd0;
								en_ix <= 1'b1;
							end
						else
							begin
								iy <= iy + 6'd1;
								en_ix <= 1'b0;
							end
						if(en_ix)
							begin
							if(ix == 6'd12)
								ix <= ix;
							else
								ix <= ix + 6'd1;
							end
						if(ix == 6'd12 && iy == 6'd13)
							begin
							finish <= 1'b1;
							enable_register <= 1'b0;
							end
					end
			end
		end
endmodule
	

module registers (clock,r_set,left_mode,right_mode,jump_mode,enable,data_out);
	input clock,r_set,enable;
	input left_mode,jump_mode;
	input right_mode;
	wire data_out0,data_out1,data_out2,data_out3,data_out4,data_out5,data_out6,data_out7,data_out8,data_out9,data_out10,
	data_out11,data_out12,data_out13;
	output [13:0] data_out;

	register r0(clock,r_set,left_mode,right_mode,jump_mode,13'b0000000011110,13'b0000000011110,13'b0000000011110,enable,data_out0);
	register r1(clock,r_set,left_mode,right_mode,jump_mode,13'b0000001101110,13'b0000001101110,13'b0000001101110,enable,data_out1);
	register r2(clock,r_set,left_mode,right_mode,jump_mode,13'b1000000111110,13'b1000000111110,13'b1000000111110,enable,data_out2);
	register r3(clock,r_set,left_mode,right_mode,jump_mode,13'b1000000111000,13'b1000000111000,13'b1000000111000,enable,data_out3);
	register r4(clock,r_set,left_mode,right_mode,jump_mode,13'b1000000111100,13'b1000000111100,13'b1000000111100,enable,data_out4);
	register r5(clock,r_set,left_mode,right_mode,jump_mode,13'b1000001110000,13'b1000001110000,13'b1000001110000,enable,data_out5);
	register r6(clock,r_set,left_mode,right_mode,jump_mode,13'b1100011110000,13'b1100011110000,13'b1100011110000,enable,data_out6);
	register r7(clock,r_set,left_mode,right_mode,jump_mode,13'b1111111111100,13'b1111111111100,13'b1111111111100,enable,data_out7);
	register r8(clock,r_set,left_mode,right_mode,jump_mode,13'b1111111110100,13'b1111111110100,13'b1111111110100,enable,data_out8);
	register r9(clock,r_set,left_mode,right_mode,jump_mode,13'b0111111110000,13'b0111111110000,13'b0111111110000,enable,data_out9);
	register r10(clock,r_set,left_mode,right_mode,jump_mode,13'b0011111100000,13'b0011111100000,13'b0011111100000,enable,data_out10);
	register r11(clock,r_set,left_mode,right_mode,jump_mode,13'b0001111000000,13'b0001111000000,13'b0001111000000,enable,data_out11);
	register r12(clock,r_set,left_mode,right_mode,jump_mode,13'b0001001000000,13'b0001000000000,13'b0000001000000,enable,data_out12);
	register r13(clock,r_set,left_mode,right_mode,jump_mode,13'b0001101100000,13'b0001100000000,13'b0000001100000,enable,data_out13);


	assign data_out = {data_out0,data_out1,data_out2,data_out3,data_out4,data_out5,data_out6,data_out7,data_out8,data_out9,data_out10,
	data_out11,data_out12,data_out13};
	
endmodule

module register (clk,r_set,left_mode,right_mode,jump_mode,data_in,data_in_left,data_in_right,enable,data_out);
    input [12:0] data_in_left;
	 input [12:0] data_in_right;
	 input clk,enable,r_set;
	 input right_mode;
	 input left_mode,jump_mode;
	 input [12:0]data_in;
	 output reg data_out;
	 reg [5:0]increase;
	 
	 always @(posedge clk, negedge r_set)
	 begin
		if(!r_set)
			begin
				increase <= 1'b0;
			end
		else
			begin
			if(enable)
				begin
					if(increase == 6'd12)
						begin
						end
					else
						begin
							if((left_mode) && (!right_mode))
								begin
								data_out <= data_in_left[12-increase];
								end
							if((!left_mode) && (!right_mode))			
								begin
								data_out <= data_in[12-increase];
								end
							if((right_mode) && (!left_mode))
							   begin
								data_out <= data_in_right[12-increase];
								end
							increase <= increase + 1'b1;
						end
				end
			end
	 end
endmodule


module ylocation_rec (clk, enable, r_set, out_y);
	input clk,r_set;
	input enable;
	output reg [6:0]out_y;

	always @(posedge clk, negedge r_set)
		begin
			if(!r_set)
				begin
				out_y <= 7'd74;
				end
			else
			  begin
			  if(enable)
					begin 
						out_y <= 7'd74;
					end
				end
		end
endmodule


module xlocation_rec (clk, enable, r_set, out_x, finish_reg);
	input clk,r_set;
	input enable;
	//reg left_x;
	output reg [7:0]out_x;
   output reg finish_reg;
	
	always @(posedge clk, negedge r_set)
		begin
			if(!r_set)
				begin
				out_x <= 8'd160;
				//left_x <= 1'd0;
				finish_reg <= 1'd0;
				end
			else
			  begin
			  if(enable)
					begin 
						out_x <= out_x - 1'b1;
						if(out_x == 8'd0)
							begin
								out_x <= 8'd160;
								finish_reg <= 1'b1;
							end	
					end
			  end
		end
endmodule



//
//module xlocation_rec (clk, enable, r_set, out_x, finish_reg);
//	input clk,r_set;
//	input enable;
//	//reg left_x;
//	output reg [7:0]out_x;
//   output reg finish_reg;
//	
//	always @(posedge clk, negedge r_set)
//		begin
//			if(!r_set)
//				begin
//				out_x <= 8'd120;
//				//left_x <= 1'd0;
//				finish_reg <= 1'd0;
//				end
//			else
//			   out_x <= 8'd120;
//		end
//endmodule

module xlocation (clk, enable, r_set, out_x);
	input clk,r_set;
	input enable;
	reg right_x;
	output reg [7:0]out_x;

	always @(posedge clk, negedge r_set)
		begin
			if(!r_set)
				begin
				out_x <= 8'd20;
				end
			else
			  begin
			  if(enable)
					begin 
						out_x <= 8'd20;
					end
				end
		end
endmodule

module ylocation (clk,r_set,in,out_y,out_jump_end,in_double_jump);
	input clk,r_set,in;
	output reg [6:0] out_y;
	output reg out_jump_end;
	input in_double_jump;
	reg up_y;
	reg up_y1;


always @(posedge clk, negedge r_set)
		begin
			if (!r_set)
				begin
				 out_y <= 7'd80;
				 up_y <= 1'b1;
				 up_y1 <= 1'b1;
				 out_jump_end <= 1'b0;
				end
			else 
				begin
					if(in)
						begin
						if(!in_double_jump)
							begin
							if(out_y <= 7'd35)
								up_y <= 1'b0;
							if(up_y)
								out_y <= out_y - 1'b1;
							if(!up_y)
								begin
								out_y <= out_y + 1'b1;
								if(out_y == 7'd80)
									begin
									out_y <= 7'd80;
									out_jump_end <= 1'b1;
									end
								end	
							end
						else	
							begin
								if(out_y <= 7'd50)
									up_y1 <= 1'b0;
								if(up_y1)
									out_y <= out_y - 1'b1;
								if(!up_y1)
									begin
									out_y <= out_y + 1'b1;
									if(out_y == 7'd80)
										begin
										out_y <= 7'd80;
										out_jump_end <= 1'b1;
										end
									end
							end
						end
				end
		end
endmodule

module framcounter(start,clock,r_set,q1,speed);
		input start;
		input clock,r_set;
		output reg [3:0]q1;
		wire [19:0] w1;
		wire enable_1;
		input [6:0] speed;
		
		delaycounter1 d1(
			.en(start),
			.clk(clock),
			.rset(r_set),
			.speed(speed),
			.q(w1)
		);
		
	assign enable_1 = (w1 == 20'd0) ? 1:0;
		
	always @(posedge clock)
		begin
			if(r_set == 1'b0)
				q1 <= 4'b0000;
			if(enable_1)
				begin 
					if(q1 == 4'b1001)
						q1 <= q1;
					else
						q1 <= q1 + 1'b1;
				end
		end	
endmodule

module framcounter1(start,clock,r_set,q1,speed);
		input start;
		input clock,r_set;
		output reg [4:0]q1;
		wire [19:0] w1;
		wire enable_1;
		input [6:0] speed;
		
		delaycounter1 d1(
			.en(start),
			.clk(clock),
			.rset(r_set),
			.speed(speed),
			.q(w1)
		);
		
		assign enable_1 = (w1 == 20'd0) ? 1:0;
		
	always @(posedge clock)
		begin
			if(r_set == 1'b0)
				q1 <= 4'b0000;
			if(enable_1)
				begin 
					if(q1 == 4'b0100)
						q1 <= q1;
					else
						q1 <= q1 + 1'b1;
				end
		end	
endmodule


module framcounter2(start,clock,r_set,q1);
		input start;
		input clock,r_set;
		output reg [4:0]q1;
		wire [19:0] w1;
		wire enable_1;
		
		
		delaycounter1 d1(
			.en(start),
			.clk(clock),
			.rset(r_set),
			.q(w1)
		);
		
		assign enable_1 = (w1 == 20'd0) ? 1:0;
		
	always @(posedge clock)
		begin
			if(r_set == 1'b0)
				q1 <= 5'b0000;
			if(enable_1)
				begin 
					if(q1 == 5'b11111)
						q1 <= q1;
					else
						q1 <= q1 + 1'b1;
				end
		end	
endmodule

module delaycounter(en,clk,rset,q);
		input en;
		input clk,rset;
		output reg [19:0]q;
		always @(posedge clk)
		begin
			if(rset == 1'b0)
				q <= 20'b00100101110011010000;
			if (en == 1'b1)
				begin 
					if(q == 20'd0)
						q <= 20'b00100101110011010000;
					else
						q <= q - 1'b1;
				end
		end	

endmodule

module delaycounter1(en,clk,rset,speed,q);
		input en;
		input clk,rset;
		input [6:0] speed;
		output reg [19:0]q;
		always @(posedge clk)
		begin
			if(rset == 1'b0)
				q <= 20'b00100101110011010000;
			if (en == 1'b1)
				begin 
					if(q == 20'd0)
						begin
						if((speed == 7'b100_0000) || (speed == 7'b111_1001) || (speed == 7'b010_0100))
							q <= 20'b00110001110011010000;
						else if ((speed == 7'b001_1001) || (speed == 7'b001_0010) || (speed == 7'b011_0000))
							q <= 20'b00011100011100110100;
						else 
							q <= 20'b00001111100000110100;
						end
					else
						q <= q - 1'b1;
				end
		end	

endmodule

module phase3_score_keeper(CLOCK_50, reset_n, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0, en);
	input CLOCK_50;
	input reset_n;
	input en;
	
	output [6:0] HEX5;
	output [6:0] HEX4;
	output [6:0] HEX3;
	output [6:0] HEX2;
	output [6:0] HEX1;
	output [6:0] HEX0;
	
	wire [3:0] digit_5;
	wire [3:0] digit_4;
	wire [3:0] digit_3;
	wire [3:0] digit_2;
	wire [3:0] digit_1;
	wire [3:0] digit_0;
//	
//	wire [26:0] q1;
//	
	wire digit_pulse;
	incr_counter ic0(
					  .clk(CLOCK_50), 
					  .r_set(reset_n), 
					  .out_pulse(digit_pulse)
					  );
					  
	digit_collection dc (
						  .pulse(digit_pulse), 
						  .r_set(reset_n), 
						  .value5(digit_5), 
						  .value4(digit_4), 
						  .value3(digit_3), 
						  .value2(digit_2), 
						  .value1(digit_1), 
						  .value0(digit_0),
						  .en(en)
						  );
					  
	//display score	
	hex_decoder H5(
        .hex_digit(digit_5[3:0]), 
        .segments(HEX5)
        );
		
	hex_decoder H4(
        .hex_digit(digit_4[3:0]), 
        .segments(HEX4)
        );
		
	hex_decoder H3(
        .hex_digit(digit_3[3:0]), 
        .segments(HEX3)
        );
		
	hex_decoder H2(
        .hex_digit(digit_2[3:0]), 
        .segments(HEX2)
        );
	
	hex_decoder H1(
		.hex_digit(digit_1[3:0]), 
		.segments(HEX1)
	);
	
	hex_decoder H0(
		.hex_digit(digit_0[3:0]), 
		.segments(HEX0)
	);
		
endmodule

module digit_value(pulse, r_set, en, value);
	input pulse;
	input r_set;
	input en;
	output reg [3:0] value;

	always @(posedge pulse, negedge r_set)
	begin
		if (r_set == 1'b0)
			value <= 1'b0;
		else
		begin
			if (en == 1'b1)
				begin
				if (value < 4'b1001)	
					value <= value + 1'b1;	
				else
					value <= 1'b0;
				end
			else
				value <= value;
		end
	end
endmodule

module digit_collection(pulse, r_set, value5, value4, value3, value2, value1, value0, en);
	input pulse;
	input r_set;
	input en;
	output [3:0] value5, value4, value3, value2, value1, value0;
	
	digit_value d0(.pulse(pulse), .r_set(r_set), .en(en), .value(value0));
	wire digit_1_en;
	assign digit_1_en = (value0 == 4'b1001) ? 1:0 ;
	digit_value d1(.pulse(pulse), .r_set(r_set), .en(digit_1_en), .value(value1));
	
	assign digit_2_en = (value1 == 4'b1001 && value0 == 4'b1001) ? 1:0 ;
	digit_value d2(.pulse(pulse), .r_set(r_set), .en(digit_2_en), .value(value2));
	
	assign digit_3_en = (value2 == 4'b1001 && value1 == 4'b1001 && value0 == 4'b1001) ? 1:0 ;
	digit_value d3(.pulse(pulse), .r_set(r_set), .en(digit_3_en), .value(value3));
	
	assign digit_4_en = (value3 == 4'b1001 && value2 == 4'b1001 && value1 == 4'b1001 && value0 == 4'b1001) ? 1:0 ;
	digit_value d4(.pulse(pulse), .r_set(r_set), .en(digit_4_en), .value(value4));
	
	assign digit_5_en = (value4 == 4'b1001 && value3 == 4'b1001 && value2 == 4'b1001 && value1 == 4'b1001 && value0 == 4'b1001) ? 1:0 ;
	digit_value d5(.pulse(pulse), .r_set(r_set), .en(digit_5_en), .value(value5));
	
endmodule

module incr_counter(clk, r_set, out_pulse);
	input clk;
	input r_set;
	reg [26:0] time_count;
	output out_pulse;
	
	always @(posedge clk, negedge r_set)
	begin
		if (r_set == 1'b0)
			time_count <= 1'b0;
		else
		begin
			if (time_count == 27'b10111110101111000010000000)
				time_count <= 1'b0;
			else
				time_count <= time_count + 1'b1;
		end
	end
	
	assign out_pulse = (time_count == 27'b10111110101111000001111111) ? 1:0;
	
endmodule

module hex_decoder(hex_digit, segments);
    input [3:0] hex_digit;
    output reg [6:0] segments;
   
    always @(*)
        case (hex_digit)
            4'h0: segments = 7'b100_0000;
            4'h1: segments = 7'b111_1001;
            4'h2: segments = 7'b010_0100;
            4'h3: segments = 7'b011_0000;
            4'h4: segments = 7'b001_1001;
            4'h5: segments = 7'b001_0010;
            4'h6: segments = 7'b000_0010;
            4'h7: segments = 7'b111_1000;
            4'h8: segments = 7'b000_0000;
            4'h9: segments = 7'b001_1000;
            4'hA: segments = 7'b000_1000;
            4'hB: segments = 7'b000_0011;
            4'hC: segments = 7'b100_0110;
            4'hD: segments = 7'b010_0001;
            4'hE: segments = 7'b000_0110;
            4'hF: segments = 7'b000_1110;   
            default: segments = 7'h7f;
        endcase
endmodule



