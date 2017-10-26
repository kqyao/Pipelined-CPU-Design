# Pipelined-CPU-Design
Course Project in UM-SJTU Joint Institute

## Problem Definition
**Single cycle:**   
A single cycle contains many components such as muxes, instruction memory, data memory, Alu and register files. We should first design the Verilog of all these components and then connect them to be a single cycle processor. The main progress of this problem is to first fetch an instruction from the Instruction Memory. Then decode the register into several parts, such as the corresponding registers, immediate offset, and some control bits. Next use the ALU to calculate the arithmetic results or logic results. If Data Memory is needed to be visited, we also write to or read from the Data Memory. Finally, we write the value we get back to the register file. The only component triggered by clock signal is the register storing PC, and it is the change of PC that causes the whole process. After the design, we should try to implement it on the computer using the Xilinx software. Show the value of PC and the values of all the used registers.   

**Pipelined:**  
Compared with single cycle processor, pipelined processor should contain four other registers, IF/ID, ID/EX, EX/MEM and MEM/WB. Moreover, we have to consider many hazards included in the lecture notes. And add some more controllers and hazard detection units to avoid the hazards. After dividing the overall circuit into 5 parts, we can increase our clock cycle rate and do the five parts together in a parallel structure. The main process is similar to the single cycle design except that we need to save almost all the things in the 4 more big registers other than only the PC register in the previous design. Since we do all the things in parallel, it is very likely that some consecutive instructions with data dependence or the control instruction to change the PC will cause some hazard. We need to design the corresponding detection unit to find these hazard and use some technic such as forwarding unit or inserting “nop” instructions to solve the problem.   
## System Partitioning 
**Instruction Memory**
We use this component to store the mips instructions. The input of this component is PC address, and the output of it is the according instructions.

**Register File**  
We use this component to store information in the 32 registers. The inputs of this is the address of the two registers which will be read and regwrite which is used to determine whether existing data to be written in the register file. If there exists some data which should be inserted into the file, writedata contains the information of the data and writereg contains the address of register.

**Shifter with or without Length Change**  
Shift the input right or left by n bits. It is efficient in multiplication. 

**Sign/Zero Extend**   
We use this component to extend a 16-bit number into a 32-bit number. Specially, the immediate part of “andi” instruction should be zero extended. 

**ALU**  
We use this component to deal with addition, subtraction and logic operations. The inputs of this component are data which will be processed and the control data which will be used to select the mode. Then the unit will output the results of the operations. 

**ALU Control Unit**   
Merge the information of the opcode and function. Return the correct control signal to ALU to select different operations. 

**Data Memory**  
We use this component to store the data in registers. The inputs of this component are the address of the data which will be read and the data which will be written. We use Aluop to control whether to use this component.

**Adder**  
Add two input together and output the result. 

**Mux**  
Choose one input for the multi-input source with a selection signal. 

**Branch Detection Unit**  
Using the signal zero and the “beq” or “bne” signal from the control unit, detect whether there’s a branch. 

**Single Cycle:**  
In this lab, we will use instruction memory to store the instruction and send the according instructions according to the input address to controller, register file and alu controller. First the controller will determine which components will be used in the later steps and control these components. For example, if the instruction code is called for “lw”. ALU, register file, data memory, Alu control and sign extender will be used.

**Pipelined:**  
We divide the single cycle processor into five parts to form pipelined processor. Different from the single cycle, pipelined processor need four other registers: IF/ID, ID/EX, EX/MEM and MEM/WB. We need these registers to store the control data.

**Hazards-avoiding design:**  
i)	Simple data hazard:   
We solve this problem by using the forwarding method. In this project, forwarding unit can solve the problems in the consecutive instructions with data dependency. We add some mux to choose the input as either the register file or the later stage registers. Also, the forwarding unit can solve the problem of “sw” after “lw”.    
ii)	Double hazard:  
We solve this problem by adding the condition that the less recent forwarding method shouldn’t be called.     
iii)	Load-use hazard:  
To solve this problem, we insert a nop just after “lw” instruction is found in EX stage which has some correspondence with the instruction in ID stage. Also, we need to hold the IFID register and PC to avoid the instruction losing.   
iv)	Control hazards:   
Since whether jump/branch or not is detected in MEM stage, before it is detected, we may have already fetched some instruction. Therefore, while changing the PC, we need to add a detection unit to detect the jump/branch to flush the value in the previous registers.    
