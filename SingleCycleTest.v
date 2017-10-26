`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    21:21:12 11/23/2016 
// Design Name: 
// Module Name:    SingleCycleTest 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module SingleCycleTest;
	wire [31:0] s0, s1, s2, s3, s4, s5, s6, s7, t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, CurrentPC;
	//wire [31:0] I;
	reg SwitchClk_10, reset;
	//reg [4:0] ShowRegIndex;
	reg [9:0] Time;
	
	SingleCycle UUT(s0, s1, s2, s3, s4, s5, s6, s7, t0, t1, t2, t3, t4, t5, t6, t7, t8, t9,CurrentPC, SwitchClk_10, reset);
	
	always @(posedge SwitchClk_10) begin
		Time=Time+1;
	end
	
	initial begin 
		#0 SwitchClk_10=1; reset=1;Time=-1;
		#10 reset=0;
	end
	
	initial begin
		$display("**********************************************");
		$display("The textual simulation results:");
		$display("**********************************************");
		$monitor(" Time: %d, CLK = %d, PC	= %h \n [$s0] = %h, [$s1] = %h, [$s2] = %h \n [$s3] = %h, [$s4] = %h, [$s5] = %h \n [$s6] = %h, [$s7] = %h, [$t0] = %h \n [$t1] = %h, [$t2] = %h, [$t3] = %h \n [$t4] = %h, [$t5] = %h, [$t6] = %h \n [$t7] = %h, [$t8] = %h, [$t9] = %h \n ", Time, SwitchClk_10, CurrentPC, s0, s1, s2, s3, s4, s5, s6, s7, t0, t1, t2, t3, t4, t5, t6, t7, t8, t9);
	end
	
	always #50 SwitchClk_10=~SwitchClk_10;
	
	initial #3000 $stop;

endmodule