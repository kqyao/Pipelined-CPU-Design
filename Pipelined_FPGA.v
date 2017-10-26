`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    22:20:28 11/22/2016 
// Design Name: 
// Module Name:    Pipelined_SSD 
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
module Pipelined_SSD(Cathod, Anode, clk, SwitchClk, ShowRegIndex, ShowPC, reset);
	input clk, SwitchClk, ShowPC, reset;
	input [4:0] ShowRegIndex;
	output [6:0] Cathod;
	output [3:0] Anode;
	
	wire [6:0] Cathod;
	wire [3:0] Anode;
	wire clk_10, clk_500;
	wire [31:0] ShowNum;
	reg SwitchClk_10;
	reg reset_10;
	
	clk_10_Hz M_clk_10 (clk, clk_10, 1'b0);
	clk_500_Hz M_clk_500 (clk, clk_500);
	
	always @ (posedge clk_10) begin  //synchronizer
		SwitchClk_10 <= SwitchClk;
		reset_10 <= reset;
	end
	
	Pipelined M_Pipelined(ShowNum, SwitchClk_10, ShowRegIndex, ShowPC, reset_10);
	SSD_Driver M_SSD(ShowNum[3:0], ShowNum[7:4], ShowNum[11:8], ShowNum[15:12], Cathod, Anode, clk_500);
endmodule 

//////////////////////////////////////////////////////////////////////////////////

module Pipelined(ShowNum, SwitchClk_10, ShowRegIndex, ShowPC, reset);
	input SwitchClk_10, ShowPC, reset;
	input [4:0] ShowRegIndex;
	output [31:0] ShowNum;
	//output [31:0] IFID_Instruction;
	
	wire [31:0] ShowNum, PCAddFour, Instruction, ReadData1, ReadData2, Immediate_32, ShowRegOut, WriteData;
	wire [31:0] BranchAddr, ALUResult, ReadData2toEXMEM, MemData;
	wire [27:0] JumpAddr_28; 
	wire PCWrite, BranchMuxSignal, RegWrite, MemtoReg, Branch, BranchN, MemWrite, MemRead, Jump, RegDst, ALUSrc;
	wire IFIDWrite, IDEXFlush1, Zero, IFIDFlush, IDEXFlush2, EXMEMFlush; 
	wire [1:0] ALUOp; 
	wire [4:0] RegDstAddr;
	reg [31:0] IFID_PC, IFID_Instruction, IDEX_JumpAddr, IDEX_PC, IDEX_ReadData1, IDEX_ReadData2, IDEX_Immediate_32;
	reg [31:0] EXMEM_JumpAddr, EXMEM_BranchAddr, EXMEM_ALUResult, EXMEM_ReadData2, MEMWB_MemData, MEMWB_ALUData;
	reg [1:0] IDEX_WB, EXMEM_WB, MEMWB_WB;
	reg [4:0] IDEX_MEM, IDEX_Rt, IDEX_Rd, IDEX_Rs, EXMEM_MEM, EXMEM_RegDst, MEMWB_RegDst;
	reg [3:0] IDEX_EX;
	reg EXMEM_Zero;
	
	IFStage M_IF(PCAddFour, Instruction, PCWrite, EXMEM_MEM[0], EXMEM_JumpAddr, BranchMuxSignal, EXMEM_BranchAddr, SwitchClk_10, reset);
	IDStage M_ID(JumpAddr_28, RegWrite, MemtoReg, Branch, BranchN, MemWrite, MemRead, Jump, RegDst, ALUSrc, ALUOp, ReadData1, ReadData2, Immediate_32, PCWrite, IFIDWrite, IDEXFlush1, ShowRegOut, IFID_Instruction, MEMWB_RegDst, WriteData, MEMWB_WB[1], IDEX_Rt, IDEX_MEM[1], SwitchClk_10, reset, ShowRegIndex);
	EXStage M_EX(BranchAddr, Zero, ALUResult, ReadData2toEXMEM, RegDstAddr, IDEX_PC, IDEX_ReadData1, IDEX_ReadData2, IDEX_Immediate_32, IDEX_Rt, IDEX_Rd, IDEX_Rs, WriteData, EXMEM_ALUResult, EXMEM_RegDst, EXMEM_WB[1], MEMWB_RegDst, MEMWB_WB[1], IDEX_EX);
	MEMStage M_MEM(IFIDFlush, IDEXFlush2, EXMEMFlush, MemData, BranchMuxSignal, EXMEM_MEM, EXMEM_Zero, EXMEM_ALUResult, EXMEM_ReadData2, WriteData, EXMEM_RegDst, MEMWB_WB[0], MEMWB_RegDst, SwitchClk_10, reset);
	WBStage M_WB(WriteData, MEMWB_MemData, MEMWB_ALUData, MEMWB_WB[0]);
	
	always @(posedge SwitchClk_10, posedge reset) begin
		if (reset==1) begin
			IFID_PC<=0;
			IFID_Instruction<=0;
		end
		else if (IFIDFlush==1) begin
			IFID_PC<=0;
			IFID_Instruction<=0;
		end
		else if (IFIDWrite==0) begin
			IFID_PC<=IFID_PC;
			IFID_Instruction<=IFID_Instruction;
		end
		else begin
			IFID_PC<=PCAddFour;
			IFID_Instruction<=Instruction;
		end
	end
	
	always @(posedge SwitchClk_10, posedge reset) begin
		if (reset==1) begin
			IDEX_JumpAddr<=0; 
			IDEX_PC<=0;
			IDEX_ReadData1<=0;
			IDEX_ReadData2<=0;
			IDEX_Immediate_32<=0;
			IDEX_WB<=0;
			IDEX_MEM<=0; 
			IDEX_Rt<=0; 
			IDEX_Rd<=0; 
			IDEX_Rs<=0;
			IDEX_EX<=0;
		end
		else if (IDEXFlush1==1) begin
			IDEX_JumpAddr<=0; 
			IDEX_PC<=0;
			IDEX_ReadData1<=0;
			IDEX_ReadData2<=0;
			IDEX_Immediate_32<=0;
			IDEX_WB<=0;
			IDEX_MEM<=0; 
			IDEX_Rt<=0; 
			IDEX_Rd<=0; 
			IDEX_Rs<=0;
			IDEX_EX<=0;
		end
		else if (IDEXFlush2==1) begin
			IDEX_JumpAddr<=0; 
			IDEX_PC<=0;
			IDEX_ReadData1<=0;
			IDEX_ReadData2<=0;
			IDEX_Immediate_32<=0;
			IDEX_WB<=0;
			IDEX_MEM<=0; 
			IDEX_Rt<=0; 
			IDEX_Rd<=0; 
			IDEX_Rs<=0;
			IDEX_EX<=0;
		end
		else begin
			IDEX_JumpAddr<={IFID_PC[31:20], JumpAddr_28}; 
			IDEX_PC<=IFID_PC;
			IDEX_ReadData1<=ReadData1;
			IDEX_ReadData2<=ReadData2;
			IDEX_Immediate_32<=Immediate_32;
			IDEX_WB<={RegWrite, MemtoReg};
			IDEX_MEM<={Branch, BranchN, MemWrite, MemRead, Jump}; 
			IDEX_Rt<=IFID_Instruction[20:16]; 
			IDEX_Rd<=IFID_Instruction[15:11]; 
			IDEX_Rs<=IFID_Instruction[25:21];
			IDEX_EX<={RegDst, ALUSrc, ALUOp};
		end
	end
	
	always @(posedge SwitchClk_10, posedge reset) begin
		if (reset==1) begin
			EXMEM_JumpAddr<=0; 
			EXMEM_BranchAddr<=0; 
			EXMEM_ALUResult<=0; 
			EXMEM_ReadData2<=0;
			EXMEM_WB<=0;
			EXMEM_MEM<=0; 
			EXMEM_RegDst<=0;
			EXMEM_Zero<=0;
		end
		else if (EXMEMFlush==1) begin
			EXMEM_JumpAddr<=0; 
			EXMEM_BranchAddr<=0; 
			EXMEM_ALUResult<=0; 
			EXMEM_ReadData2<=0;
			EXMEM_WB<=0;
			EXMEM_MEM<=0; 
			EXMEM_RegDst<=0;
			EXMEM_Zero<=0;
		end
		else begin
			EXMEM_JumpAddr<=IDEX_JumpAddr;
			EXMEM_BranchAddr<=BranchAddr; 
			EXMEM_ALUResult<=ALUResult;
			EXMEM_ReadData2<=ReadData2toEXMEM;
			EXMEM_WB<=IDEX_WB;
			EXMEM_MEM<=IDEX_MEM; 
			EXMEM_RegDst<=RegDstAddr;
			EXMEM_Zero<=Zero; 
		end
	end
	
	always @(posedge SwitchClk_10, posedge reset) begin
		if (reset==1) begin
			MEMWB_MemData<=0;
			MEMWB_ALUData<=0;
			MEMWB_WB<=0;
			MEMWB_RegDst<=0;
		end
		else begin
			MEMWB_MemData<=MemData;
			MEMWB_ALUData<=EXMEM_ALUResult;
			MEMWB_WB<=EXMEM_WB;
			MEMWB_RegDst<=EXMEM_RegDst;
		end
	end
	
	ChooseOutput M_Output(ShowNum, PCAddFour-4, ShowRegOut, ShowPC);
endmodule

/////////////////////////////////////////////////////////////////////////////
//*************************************************************************//
/////////////////////////////////////////////////////////////////////////////

module IFStage(PCAddFour, Instruction, PCWrite, Jump, JumpAddress, Branch, BranchAddress, clock, reset);
	input PCWrite, Jump, Branch, clock, reset;
	input [31:0] JumpAddress, BranchAddress;
	output [31:0] PCAddFour, Instruction;
	
	wire [31:0] PCAddFour, Instruction, PCIn, BranchOut;
	reg [31:0] PCOut=0; 
	
	always @(posedge clock, posedge reset) begin
		if(reset==1) PCOut<=0;
		else if(PCWrite==0) PCOut<=PCOut;
		else PCOut<=PCIn;
	end
	
	InstructionMemory M_Imemory(Instruction, PCOut);
	AddFour M_AddFour(PCAddFour, PCOut);
	Mux_32bit_2to1 M_Mux_Branch(BranchOut, PCAddFour, BranchAddress, Branch);
	Mux_32bit_2to1 M_Mux_Jump(PCIn, BranchOut, JumpAddress, Jump);
endmodule

/////////////////////////////////////////////////////////////////////////////

module IDStage(JumpAddr_28, RegWrite, MemtoReg, Branch, BranchN, MemWrite, MemRead, Jump, RegDst, ALUSrc, ALUOp, ReadData1, ReadData2, Immediate_32, PCWrite, IFID_Write, IDEX_Flush1, ShowRegOut, Instruction, WriteReg, WriteData, MEMWB_RegWrite, IDEX_Rt, IDEX_MemRead, clock, reset, ShowRegIndex);
	input MEMWB_RegWrite, IDEX_MemRead, clock, reset;
	input [31:0] Instruction, WriteData;
	input [4:0] IDEX_Rt, WriteReg, ShowRegIndex;
	output [27:0] JumpAddr_28;
	output RegWrite, MemtoReg, Branch, BranchN, MemWrite, MemRead, Jump, RegDst, ALUSrc, PCWrite, IFID_Write, IDEX_Flush1;
	output [1:0] ALUOp;
	output [31:0] ReadData1, ReadData2, Immediate_32, ShowRegOut;
	
	wire [27:0] JumpAddr_28;
	wire RegWrite, MemtoReg, Branch, BranchN, MemWrite, MemRead, Jump, RegDst, ALUSrc, PCWrite, IFID_Write, IDEX_Flush1, Andi;
	wire [1:0] ALUOp;
	wire [31:0] ReadData1, ReadData2, Immediate_32, ShowRegOut, Immediate_Zero, Immediate_Sign;
	
	RegisterFile M_RegFile(ReadData1,ReadData2,ShowRegOut,Instruction[25:21],Instruction[20:16],WriteReg,WriteData,MEMWB_RegWrite,clock,ShowRegIndex, reset);
	Control M_Control(RegDst, Jump, Branch, BranchN, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite, Andi, Instruction[31:26]);
	ZeroExtend M_ZeroExtend(Immediate_Zero, Instruction[15:0]);
	SignExtend M_SignExtend(Immediate_Sign, Instruction[15:0]);
	Mux_32bit_2to1 M_Mux_Addi(Immediate_32, Immediate_Sign, Immediate_Zero, Andi);
	ShiftLeftTwo_26to28 M_Shift_Jump(JumpAddr_28, Instruction[25:0]);
	LoadUseHazardDetectionUnit M_LoadUse(PCWrite, IFID_Write, IDEX_Flush1, Instruction[31:26], Instruction[25:21], Instruction[20:16], IDEX_MemRead, IDEX_Rt);
endmodule

/////////////////////////////////////////////////////////////////////////////

module EXStage(BranchAddr, Zero, ALUResult, ReadData2, RegDst, IDEX_PC, IDEX_ReadData1, IDEX_ReadData2, IDEX_Immediate_32, IDEX_Rt, IDEX_Rd, IDEX_Rs, WriteData, EXMEM_ALUResult, EXMEM_RegDst, EXMEM_RegWrite, MEMWB_RegDst, MEMWB_RegWrite, IDEX_EX);
	input [31:0] IDEX_PC, IDEX_ReadData1, IDEX_ReadData2, IDEX_Immediate_32, WriteData, EXMEM_ALUResult;
	input [4:0] IDEX_Rt, IDEX_Rd, IDEX_Rs, EXMEM_RegDst, MEMWB_RegDst;
	input [3:0] IDEX_EX;
	input EXMEM_RegWrite, MEMWB_RegWrite;
	output [31:0] BranchAddr, ALUResult, ReadData2;
	output Zero;
	output [4:0] RegDst;
	
	wire [31:0] BranchAddr, ALUResult, ReadData2, BranchOffset, ALUInput1, ALUInput2;
	wire Zero;
	wire [4:0] RegDst;
	wire [1:0] ForwardA, ForwardB;
	wire [3:0] ALUControlSignal;
	
	Mux_5bit_2to1 M_Mux_RegDst(RegDst, IDEX_Rt, IDEX_Rd, IDEX_EX[3]);
	ForwardingUnit M_Forward(ForwardA, ForwardB, IDEX_Rs, IDEX_Rt, EXMEM_RegDst, EXMEM_RegWrite, MEMWB_RegDst, MEMWB_RegWrite);
	Alucontrol M_Alucontrol(ALUControlSignal,IDEX_EX[1:0],IDEX_Immediate_32[5:0]);
	ShiftLeftTwo_32to32 M_Shift_Immediate(BranchOffset, IDEX_Immediate_32);
	Adder_32bit M_Adder(BranchAddr, IDEX_PC, BranchOffset);
	Mux_32bit_3to1 M_Mux_ForwardA(ALUInput1, IDEX_ReadData1, WriteData, EXMEM_ALUResult, ForwardA);
	Mux_32bit_3to1 M_Mux_ForwardB(ReadData2, IDEX_ReadData2, WriteData, EXMEM_ALUResult, ForwardB);
	Mux_32bit_2to1 M_Mux_ALUSrc(ALUInput2, IDEX_Immediate_32, ReadData2, IDEX_EX[2]);
	ALU M_ALU(ALUResult,Zero,ALUControlSignal,ALUInput1,ALUInput2);
endmodule

/////////////////////////////////////////////////////////////////////////////

module MEMStage(IFIDFlush, IDEXFlush2, EXMEMFlush, MemData, BranchMuxSignal, EXMEM_MEM, EXMEM_Zero, EXMEM_ALUResult, EXMEM_ReadData2, WriteData, EXMEM_RegDst, MEMWB_MemtoReg, MEMWB_RegDst, clock, reset);
	input [4:0] EXMEM_MEM;
	input EXMEM_Zero, MEMWB_MemtoReg, clock, reset;
	input [31:0] EXMEM_ALUResult, EXMEM_ReadData2, WriteData;
	input [4:0] EXMEM_RegDst, MEMWB_RegDst;
	output IFIDFlush, IDEXFlush2, EXMEMFlush, BranchMuxSignal;
	output [31:0] MemData;
	
	wire IFIDFlush, IDEXFlush2, EXMEMFlush, ForwardC;
	wire [31:0] MemData, MemWriteData;
	
	BrenchDetection M_BranchDetection(BranchMuxSignal, EXMEM_MEM[4], EXMEM_MEM[3], EXMEM_Zero);
	ControlHazardDetectionUnit M_ControlHazard(IFIDFlush, IDEXFlush2, EXMEMFlush, BranchMuxSignal, EXMEM_MEM[0]);
	LoadStoreHazardDetectionUnit M_LoadStore(ForwardC, EXMEM_MEM[2], EXMEM_RegDst, MEMWB_MemtoReg, MEMWB_RegDst);
	Mux_32bit_2to1 M_Mux_WriteData(MemWriteData, EXMEM_ReadData2, WriteData, ForwardC);
	DataMemory M_Dmemory(MemData, EXMEM_ALUResult, MemWriteData, EXMEM_MEM[2], EXMEM_MEM[1], clock, reset);
endmodule

/////////////////////////////////////////////////////////////////////////////

module WBStage(WriteData, MEMWB_MemData, MEMWB_ALUData, MEMWB_MemtoReg);
	input [31:0] MEMWB_MemData, MEMWB_ALUData;
	input MEMWB_MemtoReg;
	output [31:0] WriteData;
	
	wire [31:0] WriteData;
	
	Mux_32bit_2to1 M_Mux_WriteData(WriteData, MEMWB_ALUData, MEMWB_MemData, MEMWB_MemtoReg);
endmodule

/////////////////////////////////////////////////////////////////////////////
//*************************************************************************//
/////////////////////////////////////////////////////////////////////////////

module RegisterFile(rddata1,rddata2,ShowRegOut, rdreg1,rdreg2,wrtreg,wrtdata,RegWrite,clock,ShowRegIndex, reset);
	input [4:0] rdreg1,rdreg2,wrtreg;
	input [31:0] wrtdata;
	input RegWrite, clock, reset;
	input [4:0] ShowRegIndex;
	output [31:0] rddata1,rddata2;
	output [31:0] ShowRegOut;

	reg [31:0] rddata1,rddata2;
	reg [31:0] RegFile [31:0];
	reg [31:0] ShowRegOut;
	//reg [5:0] i; 
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
	always @(ShowRegIndex, RegFile[ShowRegIndex]) begin
		ShowRegOut=RegFile[ShowRegIndex];
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

module Mux_32bit_3to1(Q, I0, I1, I2, s);
	input [31:0] I0, I1, I2;
	input [1:0] s;
	output [31:0] Q;
	
	reg [31:0] Q; 
	
	always @(I0, I1, I2, s) begin
		if (s==0) Q=I0;
		else if (s==1) Q=I1;
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
		/*Imemory[0] = 32'b00100000000010000000000000100000; //addi $t0, $zero, 32
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
		Imemory[31] =32'b00000000000000000000000000000000;*/
		
		Imemory[0]= 32'b00100010010100100000000000001011;    
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

module ForwardingUnit(ForwardA, ForwardB, IDEX_Rs, IDEX_Rt, EXMEM_RegDst, EXMEM_RegWrite, MEMWB_RegDst, MEMWB_RegWrite);
	input [4:0] IDEX_Rs, IDEX_Rt, EXMEM_RegDst, MEMWB_RegDst;
	input EXMEM_RegWrite, MEMWB_RegWrite;
	output [1:0] ForwardA, ForwardB;
	
	reg [1:0] ForwardA, ForwardB;
	
	always @(IDEX_Rs, IDEX_Rt, EXMEM_RegDst, EXMEM_RegWrite, MEMWB_RegDst, MEMWB_RegWrite) begin
		if (EXMEM_RegWrite==1 && EXMEM_RegDst!=0 && (EXMEM_RegDst==IDEX_Rs)) ForwardA=2'b10;
		else if (MEMWB_RegWrite==1 && MEMWB_RegDst!=0 && (MEMWB_RegDst==IDEX_Rs)) ForwardA=2'b01;
		else ForwardA=2'b00;
		if (EXMEM_RegWrite==1 && EXMEM_RegDst!=0 && (EXMEM_RegDst==IDEX_Rt)) ForwardB=2'b10;
		else if (MEMWB_RegWrite==1 && MEMWB_RegDst!=0 && (MEMWB_RegDst==IDEX_Rt)) ForwardB=2'b01;
		else ForwardB=2'b00;
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

module LoadUseHazardDetectionUnit(PCWrite, IFID_Write, IDEX_Flush1, IFID_Opcode, IFID_Rs, IFID_Rt, IDEX_MemRead, IDEX_Rt);
	input [5:0] IFID_Opcode;
	input [4:0] IFID_Rs, IFID_Rt, IDEX_Rt;
	input IDEX_MemRead;
	output PCWrite, IFID_Write, IDEX_Flush1;
	
	reg PCWrite, IFID_Write, IDEX_Flush1;
	
	always @(IFID_Opcode, IFID_Rs, IFID_Rt, IDEX_MemRead, IDEX_Rt) begin
		if ((IFID_Opcode!=2 && IDEX_MemRead==1 && IFID_Rs==IDEX_Rt) || ((IFID_Opcode==0 || IFID_Opcode==4 || IFID_Opcode==5) && IDEX_MemRead==1 && IFID_Rt==IDEX_Rt)) begin
			PCWrite=0; IFID_Write=0; IDEX_Flush1=1;
		end
		else begin
			PCWrite=1; IFID_Write=1; IDEX_Flush1=0; 
		end
	end
endmodule

//////////////////////////////////////////////////////////////////////////////////////

module ControlHazardDetectionUnit(IFIDFlush, IDEXFlush2, EXMEMFlush, Branch, Jump);
	input Branch, Jump;
	output IFIDFlush, IDEXFlush2, EXMEMFlush;
	
	reg IFIDFlush, IDEXFlush2, EXMEMFlush;
	
	always @(Branch, Jump) begin
		if (Branch==1 || Jump==1) begin
			IFIDFlush=1;
			IDEXFlush2=1;
			EXMEMFlush=1;
		end
		else begin
			IFIDFlush=0;
			IDEXFlush2=0;
			EXMEMFlush=0;
		end
	end
endmodule

//////////////////////////////////////////////////////////////////////////////////////

module LoadStoreHazardDetectionUnit(ForwardC, EXMEM_MemWrite, EXMEM_RegDst, MEMWB_MemtoReg, MEMWB_RegDst);
	input EXMEM_MemWrite, MEMWB_MemtoReg;
	input [4:0] EXMEM_RegDst, MEMWB_RegDst;
	output ForwardC;
	
	reg ForwardC;
	
	always @(EXMEM_MemWrite, EXMEM_RegDst, MEMWB_MemtoReg, MEMWB_RegDst) begin
		if(EXMEM_MemWrite==1 && MEMWB_MemtoReg==1 && EXMEM_RegDst==MEMWB_RegDst) ForwardC=1;
		else ForwardC=0;
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

