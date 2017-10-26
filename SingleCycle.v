`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    21:20:27 11/23/2016 
// Design Name: 
// Module Name:    SingleCycle 
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
module SingleCycle(s0, s1, s2, s3, s4, s5, s6, s7, t0, t1, t2, t3, t4, t5, t6, t7, t8, t9,PCOut, SwitchClk_10, reset);
	input SwitchClk_10, reset;
	//input [4:0] ShowRegIndex;
	output [31:0] s0, s1, s2, s3, s4, s5, s6, s7, t0, t1, t2, t3, t4, t5, t6, t7, t8, t9;
	//output [3:0] ALUControlSignal;
	output [31:0] PCOut;
	
	reg [31:0] PCOut=0; 
	wire [31:0] PCIn, Instruction, ReadData1, ReadData2, s0, s1, s2, s3, s4, s5, s6, s7, t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, WriteData_RegFile, Immediate_Sign_32, Immediate_Zero_32, Immediate_32;
	wire [31:0] ALUInput2, ALUResult, ReadData, PCAddFour, BranchOffset, BranchAddress, PCNoJump, ShowNum;
	wire RegDst, Jump, Branch, BranchN, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite, Andi, Zero, BranchMuxSignal;
	wire [1:0] ALUOp;
	wire [4:0] WriteRegister;
	wire [3:0] ALUControlSignal;
	wire [27:0] JumpImmediate;
	
	always @(posedge SwitchClk_10, posedge reset) begin
		if (reset==1) PCOut=0;
		else PCOut=PCIn;
	end
	InstructionMemory M_InstructionMemory(Instruction, PCOut);
	Control M_Control(RegDst, Jump, Branch, BranchN, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite, Andi, Instruction[31:26]);
	Mux_5bit_2to1 M_Mux_WriteReg(WriteRegister, Instruction[20:16], Instruction[15:11], RegDst);
	RegisterFile M_RegisterFile(ReadData1,ReadData2,s0, s1, s2, s3, s4, s5, s6, s7, t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, Instruction[25:21], Instruction[20:16],WriteRegister,WriteData_RegFile,RegWrite,SwitchClk_10, reset);
	SignExtend M_SignExtend(Immediate_Sign_32, Instruction[15:0]);
	ZeroExtend M_ZeroExtend(Immediate_Zero_32, Instruction[15:0]);
	Mux_32bit_2to1 M_Mux_Immediate(Immediate_32, Immediate_Sign_32, Immediate_Zero_32, Andi);
	Alucontrol M_Alucontrol(ALUControlSignal,ALUOp,Instruction[5:0]);
	Mux_32bit_2to1 M_Mux_ALUSrc(ALUInput2, Immediate_32, ReadData2, ALUSrc);
	ALU M_ALU(ALUResult,Zero,ALUControlSignal,ReadData1,ALUInput2);
	DataMemory M_DataMemory(ReadData, ALUResult, ReadData2, MemWrite, MemRead, SwitchClk_10, reset);
	Mux_32bit_2to1 M_Mux_MemtoReg(WriteData_RegFile, ALUResult, ReadData, MemtoReg);
	AddFour M_AddFour(PCAddFour, PCOut);
	ShiftLeftTwo_26to28 M_Shifter_Jump(JumpImmediate, Instruction[25:0]);
	ShiftLeftTwo_32to32 M_Shifter_Branch(BranchOffset, Immediate_32);
	Adder_32bit M_AdderBranch(BranchAddress, PCAddFour, BranchOffset);
	BrenchDetection M_BranchDetection(BranchMuxSignal, Branch, BranchN, Zero);
	Mux_32bit_2to1 M_Mux_Branch(PCNoJump, PCAddFour, BranchAddress, BranchMuxSignal);
	Mux_32bit_2to1 M_Mux_Jump(PCIn, PCNoJump, {PCAddFour[31:28],JumpImmediate}, Jump);
	//ChooseOutput M_ChooseOutput(ShowNum, PCOut, ShowRegOut, ShowPC);
	
endmodule

/////////////////////////////////////////////////////////////////////////////
//*************************************************************************//

module RegisterFile(rddata1,rddata2,s0, s1, s2, s3, s4, s5, s6, s7, t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, rdreg1,rdreg2,wrtreg,wrtdata,RegWrite,clock,reset);
	input [4:0] rdreg1,rdreg2,wrtreg;
	input [31:0] wrtdata;
	input RegWrite, clock, reset;
	//input [4:0] ShowRegIndex;
	output [31:0] rddata1,rddata2;
	//output [31:0] ShowRegOut;
	output [31:0] s0, s1, s2, s3, s4, s5, s6, s7, t0, t1, t2, t3, t4, t5, t6, t7, t8, t9;

	reg [31:0] rddata1,rddata2;
	reg [31:0] RegFile [31:0];
	//reg [31:0] ShowRegOut;
	//reg [5:0] i; 
	reg [31:0] s0, s1, s2, s3, s4, s5, s6, s7, t0, t1, t2, t3, t4, t5, t6, t7, t8, t9;
	integer i;

	always @(negedge clock, posedge reset) begin
		if (reset==1) begin
			for(i=0; i<32; i=i+1) 
				RegFile[i]=0;
		end
		else if (RegWrite) RegFile[wrtreg] = wrtdata;
	end
	always @(rdreg1, rdreg2, RegFile[rdreg1], RegFile[rdreg2]) begin
		//if (reset==0) begin
			rddata1 = RegFile[rdreg1]; 
			rddata2 = RegFile[rdreg2];
		//end
	end
	always @(RegFile[8], RegFile[9], RegFile[10], RegFile[11], RegFile[12], RegFile[13], RegFile[14], RegFile[15], RegFile[16], RegFile[17], RegFile[18], RegFile[19], RegFile[20], RegFile[21], RegFile[22], RegFile[23], RegFile[24], RegFile[25]) begin
		s0=RegFile[16];
		s1=RegFile[17];
		s2=RegFile[18];
		s3=RegFile[19];
		s4=RegFile[20];
		s5=RegFile[21];
		s6=RegFile[22];
		s7=RegFile[23];
		t0=RegFile[8];
		t1=RegFile[9];
		t2=RegFile[10];
		t3=RegFile[11];
		t4=RegFile[12];
		t5=RegFile[13];
		t6=RegFile[14];
		t7=RegFile[15];
		t8=RegFile[24];
		t9=RegFile[25];
	end
endmodule

///////////////////////////////////////////////////////////////////////////////////

module ShiftLeftTwo_26to28(Q, I);
	input [25:0] I;
	output [27:0] Q;

	reg [27:0] Q;

	always @ (I)
	begin
		Q[27:2] = I[25:0];
		Q[1:0] = 2'b00;
	end
endmodule

////////////////////////////////////////////////////////////////////////////////////

module ShiftLeftTwo_32to32(Q, I);
	input [31:0] I;
	output [31:0] Q;

	reg [31:0] Q;

	always @ (I)
	begin
		Q[31:2] = I[29:0];
		Q[1:0] = 'b0;
	end
endmodule


////////////////////////////////////////////////////////////////////////////////////

module ALU(Alurlt,zero,Aluctrl,I1,I2);
	input [31:0] I1,I2;
	input [3:0] Aluctrl;
	output [31:0] Alurlt;
	output zero;

	reg [31:0] Alurlt;
	reg zero;

	always @ (I1 or I2 or Aluctrl)
	begin
		if(Aluctrl==4'b0010) Alurlt = I1 + I2;
		else if(Aluctrl==4'b0110) Alurlt=I1-I2;
		else if(Aluctrl==4'b0000) Alurlt = I1 & I2; 
		else if(Aluctrl==4'b0001) Alurlt = I1 | I2; 
		else begin
			if(I1<I2) Alurlt=1;
			else Alurlt=0;
		end
		if (I1==I2) zero=1;
		else zero=0;
	end
endmodule

////////////////////////////////////////////////////////////////////////////////////

module Alucontrol(Aluctrl,Aluop,funct);
	input [5:0] funct;
	input [1:0] Aluop;
	output [3:0] Aluctrl;

	reg [3:0] Aluctrl;

	always @ (Aluop or funct)
	begin
		if(Aluop==2'b00) Aluctrl=4'b0010;
		else if(Aluop==2'b01) Aluctrl=4'b0110;
		else if(Aluop==2'b11) Aluctrl=4'b0000;
		else if(Aluop==2'b10 && funct==6'b100000) Aluctrl=4'b0010;
		else if(Aluop==2'b10 && funct==6'b100010) Aluctrl=4'b0110;
		else if(Aluop==2'b10 && funct==6'b100100) Aluctrl=4'b0000;
		else if(Aluop==2'b10 && funct==6'b100101) Aluctrl=4'b0001;
		else if(Aluop==2'b10 && funct==6'b101010) Aluctrl=4'b0111;
	end
endmodule

////////////////////////////////////////////////////////////////////////////////////

module ZeroExtend(Q, I);
	input [15:0] I;
	output [31:0] Q;

	reg [31:0] Q;

	always @ (I)
	begin
		Q[15:0] = I[15:0];
		Q[31:16] = 16'b0;
	end
endmodule

////////////////////////////////////////////////////////////////////////////////////

module SignExtend(Q, I);
	input [15:0] I;
	output [31:0] Q;

	reg [31:0] Q;

	always @ (I)
	begin
		Q[15:0] = I[15:0];
		Q[31:16] = {16{I[15]}};
	end
endmodule

///////////////////////////////////////////////////////////////////////////////////

module AddFour(Q, I);
	input [31:0] I;
	output [31:0] Q;

	reg [31:0] Q;

	always @ (I) begin
		Q=I+4;
	end
endmodule

///////////////////////////////////////////////////////////////////////////////////

module Adder_32bit(Q, I1, I2);
	input [31:0] I1;
	input [31:0] I2;
	output [31:0] Q;

	reg [31:0] Q;

	always @(I1, I2) begin
		Q=I1+I2;
	end
endmodule

////////////////////////////////////////////////////////////////////////////////////

module Mux_32bit_2to1(Q, I1, I2, s);
// output is I1 if s==0, otherwise I2. 
	input [31:0] I1;
	input [31:0] I2;
	input s;
	output [31:0] Q;

	reg [31:0] Q;

	always @(I1, I2, s) begin
		if (s==0) Q=I1;
		else Q=I2;
	end
endmodule

////////////////////////////////////////////////////////////////////////////////////

module Mux_5bit_2to1(Q, I1, I2, s);
// output is I1 if s==0, otherwise I2. 
	input [4:0] I1;
	input [4:0] I2;
	input s;
	output [4:0] Q;

	reg [4:0] Q;

	always @(I1, I2, s) begin
		if (s==0) Q=I1;
		else Q=I2;
	end
endmodule

////////////////////////////////////////////////////////////////////////////////////

module And_1bit(Q, I1, I2);
// Q=I1&I2;
	input I1, I2;
	output Q;

	reg Q;

	always @(I1, I2) Q=I1&I2;
endmodule

/////////////////////////////////////////////////////////////////////////////////////

module Control(RegDst, Jump, Branch, BranchN, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite, Andi, I);
	input [5:0] I;
	output RegDst, Jump, Branch, BranchN, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite, Andi;
	output [1:0] ALUOp;

	reg RegDst, Jump, Branch, BranchN, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite, Andi;
	reg [1:0] ALUOp;

	always @(I) begin
		if (I==0) RegDst=1;
		else RegDst=0;
		if (I==2) Jump=1;
		else Jump=0;
		if (I==4) Branch=1;
		else Branch=0;
		if (I==5) BranchN=1;
		else BranchN=0;
		if (I==6'h23) MemRead=1;
		else MemRead=0;
		if (I==6'h23) MemtoReg=1;
		else MemtoReg=0;
		if (I==6'h23 || I==6'h2b || I==8) ALUOp=0;
		else if (I==4 || I==5) ALUOp=1;
		else if (I==6'h0c) ALUOp=3;
		else ALUOp=2;
		if (I==6'h2b) MemWrite=1;
		else MemWrite=0;
		if (I==0 || I==4 || I==5) ALUSrc=1;
		else ALUSrc=0;
		if (I==4 || I==5 || I==2 || I==6'h2b) RegWrite=0;
		else RegWrite=1;
		if (I==6'h0c) Andi=1;
		else Andi=0;
	end
endmodule

///////////////////////////////////////////////////////////////////////////////////

module InstructionMemory(Instruction, ReadAddress);
	input [31:0] ReadAddress;
	output [31:0] Instruction;

	reg [31:0] Instruction;
	reg [31:0] Imemory [31:0];

	initial begin
		Imemory[0] = 32'b00100000000010000000000000100000; //addi $t0, $zero, 32
		Imemory[1] = 32'b00100000000010010000000000110111; //addi $t1, $zero, 55
		Imemory[2] = 32'b00000001000010011000000000100100; //and $s0, $t0, $t1
		Imemory[3] = 32'b00000001000010011000000000100101; //or $s0, $t0, $t1
		Imemory[4] = 32'b10101100000100000000000000000100; //sw $s0, 4($zero)
		Imemory[5] = 32'b10101100000010000000000000001000; //sw $t0, 8($zero)
		Imemory[6] = 32'b00000001000010011000100000100000; //add $s1, $t0, $t1
		Imemory[7] = 32'b00000001000010011001000000100010; //sub $s2, $t0, $t1
		Imemory[8] = 32'b00010010001100100000000000001001; //beq $s1, $s2, error0
		Imemory[9] = 32'b10001100000100010000000000000100; //lw $s1, 4($zero)
		Imemory[10]= 32'b00110010001100100000000001001000; //andi $s2, $s1, 72
		Imemory[11] =32'b00010010001100100000000000001001; //beq $s1, $s2, error1
		Imemory[12] =32'b10001100000100110000000000001000; //lw $s3, 8($zero)
		Imemory[13] =32'b00010010000100110000000000001010; //beq $s0, $s3, error2
		Imemory[14] =32'b00000010010100011010000000101010; //slt $s4, $s2, $s1 (Last)
		Imemory[15] =32'b00010010100000000000000000001111; //beq $s4, $0, EXIT
		Imemory[16] =32'b00000010001000001001000000100000; //add $s2, $s1, $0
		Imemory[17] =32'b00001000000000000000000000001110; //j Last
		Imemory[18] =32'b00100000000010000000000000000000; //addi $t0, $0, 0(error0)
		Imemory[19] =32'b00100000000010010000000000000000; //addi $t1, $0, 0
		Imemory[20] =32'b00001000000000000000000000011111; //j EXIT
		Imemory[21] =32'b00100000000010000000000000000001; //addi $t0, $0, 1(error1)
		Imemory[22] =32'b00100000000010010000000000000001; //addi $t1, $0, 1
		Imemory[23] =32'b00001000000000000000000000011111; //j EXIT
		Imemory[24] =32'b00100000000010000000000000000010; //addi $t0, $0, 2(error2)
		Imemory[25] =32'b00100000000010010000000000000010; //addi $t1, $0, 2
		Imemory[26] =32'b00001000000000000000000000011111; //j EXIT
		Imemory[27] =32'b00100000000010000000000000000011; //addi $t0, $0, 3(error3)
		Imemory[28] =32'b00100000000010010000000000000011; //addi $t1, $0, 3
		Imemory[29] =32'b00001000000000000000000000011111; //j EXIT
		Imemory[30] =32'b00000000000000000000000000000000;
		Imemory[31] =32'b00000000000000000000000000000000;
		/*Imemory[0]= 32'b00100010010100100000000000001011;    
		Imemory[1]= 32'b00100001010010100000000000001010;   
		Imemory[2]= 32'b00100001100011000000000000001010;   
		Imemory[3]= 32'b00100001101011010000000000001010;    
		Imemory[4]= 32'b00000001101011000101000000100010;	
		Imemory[5]= 32'b00000001010100010110000000100100;   
		Imemory[6]= 32'b00000010010010100111000000100101;   
		Imemory[7]= 32'b00000001010010100111100000100000;  
		Imemory[8]= 32'b10101101010011010000000000000100;   
		Imemory[9]= 32'b00000001101011100110100000100000;    
		Imemory[10]=32'b00000001101011100110100000100000;    
		Imemory[11]=32'b00000001101011100110100000100000;  
		Imemory[12]=32'b10001101010011000000000000000100;           
		Imemory[13]=32'b00000001100011100111100000100100;       
		Imemory[14]=32'b00010001100011110000000000000001;         
		Imemory[15]=32'b00000001101011100110100000100000;         
		Imemory[16]=32'b00000001101010100110100000100000;
		Imemory[18] =32'b00000000000000000000000000000000; //
		Imemory[19] =32'b00000000000000000000000000000000; //
		Imemory[20] =32'b00000000000000000000000000000000; //
		Imemory[21] =32'b00000000000000000000000000000000; //
		Imemory[22] =32'b00000000000000000000000000000000; //
		Imemory[23] =32'b00000000000000000000000000000000; //
		Imemory[24] =32'b00000000000000000000000000000000; //
		Imemory[25] =32'b00000000000000000000000000000000; //
		Imemory[26] =32'b00000000000000000000000000000000; //
		Imemory[27] =32'b00000000000000000000000000000000; //
		Imemory[28] =32'b00000000000000000000000000000000; //
		Imemory[29] =32'b00000000000000000000000000000000; //
		Imemory[30] =32'b00000000000000000000000000000000; //
		Imemory[31] =32'b00000000000000000000000000000000; //*/
	end

	always @(ReadAddress or Imemory[ReadAddress[31:2]]) Instruction=Imemory[ReadAddress[31:2]];

endmodule

/////////////////////////////////////////////////////////////////////////////////////

module DataMemory(ReadData, Address, WriteData, MemWrite, MemRead, clock, reset);
	input [31:0] Address, WriteData;
	input MemWrite, MemRead, clock, reset;
	output [31:0] ReadData;

	reg [31:0] ReadData;
	reg [31:0] Dmemory [31:0];
	integer i;

	always @(Address[31:2], MemRead, Dmemory[Address[31:2]]) begin
		if (MemRead) ReadData=Dmemory[Address[31:2]];
	end
	always @(negedge clock, posedge reset) begin
		if (reset) begin
			for(i=0; i<32; i=i+1) 
				Dmemory[i]=0;
		end
		else if (MemWrite) Dmemory[Address[31:2]]=WriteData;
	end

endmodule

//////////////////////////////////////////////////////////////////////////////////////

module BrenchDetection(BranchMuxSignal, Branch, BranchN, Zero);
	input Branch, BranchN, Zero;
	output BranchMuxSignal;

	reg BranchMuxSignal;

	always @(Branch, BranchN, Zero) begin
		if (Branch && Zero || BranchN && ~Zero) BranchMuxSignal=1;
		else BranchMuxSignal=0;
	end
endmodule

/////////////////////////////////////////////////////////////////////////////////////

module ChooseOutput(ShowNum, PCOut, ShowRegOut, ShowPC);
	input [31:0] PCOut, ShowRegOut;
	input ShowPC;
	output [31:0] ShowNum;
	
	reg [31:0] ShowNum;
	
	always @(PCOut, ShowRegOut, ShowPC) begin
		if (ShowPC) ShowNum=PCOut;
		else ShowNum=ShowRegOut;
	end
endmodule

//////////////////////////////////////////////////////////////////////////////////////

module SSD_Driver (Num_0, Num_1, Num_2, Num_3, Cathod, Anode, clk_500);
	input [3:0] Num_0, Num_1, Num_2, Num_3;
	input clk_500;
	output [6:0] Cathod;
	output [3:0] Anode;
	reg [6:0] Cathod;
	reg [3:0] Anode;
	reg [1:0] digit=0;
	
	always @ (posedge clk_500)
	begin
		case(digit)
			2'b00: digit<=2'b01;
			2'b01: digit<=2'b10;
			2'b10: digit<=2'b11;
			2'b11: digit<=2'b00;
		endcase
	end
	always @(digit, Num_0, Num_1, Num_2, Num_3)
	begin 
		case (digit)
			2'b00: begin 
				Anode <= 4'b1110; 
				//dot <= 1;
				case (Num_0)
					4'b0000: Cathod <= 7'b0000001; 
					4'b0001: Cathod <= 7'b1001111;
					4'b0010: Cathod <= 7'b0010010;
					4'b0011: Cathod <= 7'b0000110;
					4'b0100: Cathod <= 7'b1001100;
					4'b0101: Cathod <= 7'b0100100;
					4'b0110: Cathod <= 7'b0100000;
					4'b0111: Cathod <= 7'b0001111;
					4'b1000: Cathod <= 7'b0000000;
					4'b1001: Cathod <= 7'b0000100;
					4'b1010: Cathod <= 7'b0001000;
					4'b1011: Cathod <= 7'b1100000;
					4'b1100: Cathod <= 7'b0110001;
					4'b1101: Cathod <= 7'b1000010;
					4'b1110: Cathod <= 7'b0110000;
					default: Cathod <= 7'b0111000;
				endcase end
			2'b01: begin 
				Anode <= 4'b1101;
				//dot <= 1; 
				case (Num_1)
					4'b0000: Cathod <= 7'b0000001; 
					4'b0001: Cathod <= 7'b1001111;
					4'b0010: Cathod <= 7'b0010010;
					4'b0011: Cathod <= 7'b0000110;
					4'b0100: Cathod <= 7'b1001100;
					4'b0101: Cathod <= 7'b0100100;
					4'b0110: Cathod <= 7'b0100000;
					4'b0111: Cathod <= 7'b0001111;
					4'b1000: Cathod <= 7'b0000000;
					4'b1001: Cathod <= 7'b0000100;
					4'b1010: Cathod <= 7'b0001000;
					4'b1011: Cathod <= 7'b1100000;
					4'b1100: Cathod <= 7'b0110001;
					4'b1101: Cathod <= 7'b1000010;
					4'b1110: Cathod <= 7'b0110000;
					default: Cathod <= 7'b0111000;
				endcase end
			2'b10: begin 
				Anode <= 4'b1011;
				//dot <= 0; 
				case (Num_2)
					4'b0000: Cathod <= 7'b0000001; 
					4'b0001: Cathod <= 7'b1001111;
					4'b0010: Cathod <= 7'b0010010;
					4'b0011: Cathod <= 7'b0000110;
					4'b0100: Cathod <= 7'b1001100;
					4'b0101: Cathod <= 7'b0100100;
					4'b0110: Cathod <= 7'b0100000;
					4'b0111: Cathod <= 7'b0001111;
					4'b1000: Cathod <= 7'b0000000;
					4'b1001: Cathod <= 7'b0000100;
					4'b1010: Cathod <= 7'b0001000;
					4'b1011: Cathod <= 7'b1100000;
					4'b1100: Cathod <= 7'b0110001;
					4'b1101: Cathod <= 7'b1000010;
					4'b1110: Cathod <= 7'b0110000;
					default: Cathod <= 7'b0111000;
				endcase end
			2'b11: begin 
				Anode <= 4'b0111;
				//dot <= 1; 
				case (Num_3)
					4'b0000: Cathod <= 7'b0000001; 
					4'b0001: Cathod <= 7'b1001111;
					4'b0010: Cathod <= 7'b0010010;
					4'b0011: Cathod <= 7'b0000110;
					4'b0100: Cathod <= 7'b1001100;
					4'b0101: Cathod <= 7'b0100100;
					4'b0110: Cathod <= 7'b0100000;
					4'b0111: Cathod <= 7'b0001111;
					4'b1000: Cathod <= 7'b0000000;
					4'b1001: Cathod <= 7'b0000100;
					4'b1010: Cathod <= 7'b0001000;
					4'b1011: Cathod <= 7'b1100000;
					4'b1100: Cathod <= 7'b0110001;
					4'b1101: Cathod <= 7'b1000010;
					4'b1110: Cathod <= 7'b0110000;
					default: Cathod <= 7'b0111000;
				endcase end
		endcase
	end
endmodule

//////////////////////////////////////////////////////////////////////////

module clk_500_Hz(clk_50M, clk_500);
	input clk_50M;
	output clk_500;
	reg clk_500;
	reg[16:0] counter=0;
	always @ (posedge clk_50M)
	begin
		if (counter < 100000-1) begin
			counter<=counter+1;
			clk_500<=0;
		end else
		begin
			counter<=0;
			clk_500<=1;
		end
	end
endmodule

/////////////////////////////////////////////////////////////////////////////

module clk_10_Hz(clk_50M, clk_10, reset);
	input clk_50M, reset;
	output clk_10;
	reg clk_10;
	reg[22:0] counter=0;
	always @ (posedge clk_50M)
	begin
		if (reset==1) begin
			counter<=0;
			clk_10<=0;
		end else
		if (counter < 5000000-1) begin
			counter<=counter+1;
			clk_10<=0;
		end else
		begin
			counter<=0;
			clk_10<=1;
		end
	end
endmodule


