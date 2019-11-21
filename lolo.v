module Mux4to1_32bits (in1,in2,in3,in4,sel,out1);
input [31:0] in1,in2,in3,in4;
input sel;
output reg [31:0] out1;
always @(in1,in2,in3,in4,sel)
begin
case(sel)
0:out1<=in1;	
1:out1<=in2;
2:out1<=in3;
default:out1<=in4;
endcase
end
endmodule


module Mux2to1_32bits (in1,in2,sel,out1);
input [31:0] in1,in2;
input sel;
output reg [31:0] out1;
always @(in1,in2,sel)

/*repeat(1)@(posedge clk)
initial
begin
out1<=32'd0;
end*/

begin
case(sel)
1:out1<=in2;
default:out1<=in1;
endcase
end

endmodule

module Mux4to1_5bits (in1,in2,in3,in4,sel,out1);
input [4:0] in1,in2,in3,in4;
input sel;
output reg [4:0] out1;
always @(in1,in2,in3,in4,sel)
begin
case(sel)
0:out1<=in1;
1:out1<=in2;
2:out1<=in3;
default:out1<=in4;
endcase
end
endmodule
module Alu (a,b,c,aluctr,aluout,zero,);
input [31:0] b,a;
input [3:0] aluctr;
input[4:0] c; // shamt 
output reg [31:0] aluout;
output reg zero;
 
always @(a,b,c,aluctr)
begin
case(aluctr)
0: aluout <=a&b;
1: aluout <=a|b;
2: aluout <=a+b;
6: aluout <=a-b;
7: aluout <=a<b?1:0;
12: aluout <=~(a|b);
4'b1110:aluout <= b<<c; // check
4'b1111:aluout <= b<<16; 
endcase
if (aluout == 0)
begin
zero <= 1'b1;
end
else
begin
zero <= 1'b0;
end
end
endmodule

module Adder (x,y,sum);
input[31:0]x,y;
output reg[31:0] sum;
always @(x,y)
begin
sum<=x+y;
end
endmodule

module Register_File (Read_Register1,Read_Register2,Write_Register,Write_Data,RegWrite,clk,Read_Data1,Read_Data2);



input wire [4:0] Read_Register1,Read_Register2,Write_Register;
input wire [31:0] Write_Data;
input wire clk;
input wire RegWrite;
output reg [31:0] Read_Data1,Read_Data2;


reg [31:0] RF [0:31];
integer i;


initial
begin
for(i = 0;i<32;i = i+1)
begin
RF[i] = i;
end
end

always@(posedge clk)
begin
 Read_Data1 <=  RF[Read_Register1];
 Read_Data2 <=  RF[Read_Register2];
if((RegWrite)&&(Write_Register != 5'b00000)) begin
   RF[Write_Register] <= Write_Data ;

end
end

endmodule

module Shift_Left_2(in,out);
parameter size = 32;
input wire [size-1:0] in;
output reg [size-1:0] out;
always@(in)
begin
out <= in << 2;
end

endmodule

module Shift_Left_2_26to28(in,out);
parameter size_in = 26;
parameter size_out = 28;
input wire [size_in-1:0] in;
output reg [size_out-1:0] out;
always@(in)
begin
out <= in << 2;
end

endmodule
module Sign_Extend(in,out);

parameter in_size = 16;
parameter out_size = 32;

input wire [in_size-1:0] in;
output reg [out_size-1:0] out;

always @(in)
begin
out[in_size-1:0] <= in;
if (in[in_size-1] == 1'b0) out[out_size-1:in_size] <= 16'b0000000000000000;
else out[out_size-1:in_size] <= 16'b1111111111111111;
end
endmodule

module insmemory (instruction,address,clk);
output reg [31:0] instruction;
input  wire [31:0] address;
input clk;
reg [31:0] rom [0:1023];

initial
begin
$readmemh("D:\ins.txt",rom);
end

always @ (posedge clk)
begin

instruction <= rom [address];

end
 


endmodule

module PC (newaddress,address,clk);
//newaddress dh el addres el hytl3o , wl address dh l byd5olo b3d myzed 4 
output  reg [31:0] newaddress = 32'd0;
input  clk;
input  [31:0] address;


always @ (posedge clk)

begin newaddress <= address; end


endmodule 

module CPU (Instruction,RegDst,Branch,MemRead,MemtoReg,ALUOp,MemWrite,ALUSrc,RegWrite,Jump,clk);

input [31:26] Instruction;
input clk;

output reg Branch,MemRead,MemWrite,ALUSrc,RegWrite,Jump;
output reg [1:0] RegDst,MemtoReg;
output reg [2:0] ALUOp;

parameter R_format = 6'b000000;
parameter Addi = 6'b001000;
parameter Andi = 6'b001010;
parameter Beq =6'b000100;
parameter Bne =6'b001011;
parameter J =6'b000010;
parameter Jal =6'b000011;
parameter Jr =6'b000000;
parameter Lb =6'b001010;
parameter Lh =6'b001000;
parameter Lui= 6'b001111;
parameter Lw =6'b100011;
parameter Nor= 6'b000000;
parameter Or =6'b000000;
parameter Ori= 6'b001101;
parameter Sb =6'b001000;
parameter Sh =6'b001010;
parameter Sll= 6'b000000;
parameter Slt= 6'b000000;
parameter Slti= 6'b001000;
parameter Sra =6'b001010;
parameter Srl =6'b001000;
parameter Sub =6'b001010;
parameter Sw =6'b101011;
parameter Xor =6'b001010;
parameter Xori =6'b001000;


always @(posedge clk)
begin
case(Instruction)
//R-Format.
R_format : begin	
	RegDst <= 2'b01 ;
	Branch <= 1'b0 ;
	MemRead <= 1'b0 ;
	MemtoReg <=2'b00 ;
	MemWrite <=1'b0 ;
	ALUSrc <= 1'b0 ;
	RegWrite <= 1'b1 ;
	ALUOp <= 3'b010 ;
	Jump <= 1'b0 ;
	end

//addi.
Addi : begin
	RegDst <= 2'b00 ;
	Branch <= 1'b0 ;
	MemRead <= 1'b0 ;
	MemtoReg <=2'b00 ;
	MemWrite <=1'b0 ;
	ALUSrc <= 1'b1 ;
	RegWrite <= 1'b1 ;
	ALUOp <= 3'b000 ;
	 Jump <= 1'b0 ;
	end

//andi
/*2'd12 : begin
	RegDst = 2'b ;
	Branch = 1'b ;
	MemRead= 1'b ;
	MemtoReg =2'b ;
	MemWrite =1'b ;
	ALUSrc = 1'b ;
	RegWrite = 1'b ;
	ALUOp = 2'b ;
	Jump = 1'b0 ;
	end
*/
//beq.
Beq : begin
	RegDst<=2'bxx ;
	Branch<=1'b1 ;
	MemRead<=1'b0 ;
	MemtoReg<=2'bxx ;
	MemWrite<=1'b0 ;
	ALUSrc<=1'b0 ;
	RegWrite<=1'b0 ;
	ALUOp<=3'b001 ;
	Jump<=1'b0 ;
	end

//bne
/*2'd05 : begin
	RegDst = 2'b ;
	Branch = 1'b ;
	MemRead= 1'b ;
	MemtoReg =2'b ;
	MemWrite =1'b ;
	ALUSrc = 1'b ;
	RegWrite = 1'b ;
	ALUOp = 2'b ;
	Jump = 1'b0 ;
	end
*/
//j.
J : begin
	RegDst<=2'bxx ;
	Branch<=1'b0 ;
	MemRead<=1'b0 ;
	MemtoReg<=2'bxx ;
	MemWrite<=1'b0 ;
	ALUSrc<=1'bx ;
	RegWrite<=1'b0 ;
	Jump<=1'b1 ;
	ALUOp<=3'bxxx ;
	Jump<=1'b1 ;
	end

//jal.
Jal : 
begin
	RegDst<=2'b10 ;
	Branch<=1'b0 ;
	MemRead<=1'b0 ;
	MemtoReg<=2'b10 ;
	MemWrite<=1'b0 ;
	ALUSrc<=1'bx ;
	RegWrite<=1'b1 ;
	ALUOp<=3'bxxx ;
	Jump<=1'b1 ;
	end

//lb
/*2'd32 : begin
	RegDst = 2'b ;
	Branch = 1'b ;
	MemRead= 1'b ;
	MemtoReg =2'b ;
	MemWrite =1'b ;
	ALUSrc = 1'b ;
	RegWrite = 1'b ;
	ALUOp = 2'b ;
	Jump = 1'b0 ;
	end
*/
//lh
/*2'd33 : begin
	RegDst = 2'b ;
	Branch = 1'b ;
	MemRead= 1'b ;
	MemtoReg =2'b ;
	MemWrite =1'b ;
	ALUSrc = 1'b ;
	RegWrite = 1'b ;
	ALUOp = 2'b ;
	Jump = 1'b0 ;
	end
*/
//lui.
Lui : begin
	RegDst<=2'b00 ;
	Branch<=1'b0 ;
	MemRead<=1'b0 ;
	MemtoReg<=2'b00 ;
	MemWrite<=1'b0 ;
	ALUSrc<=1'b1 ;
	RegWrite<=1'b1 ;
	ALUOp<=3'b011 ;
	Jump<=1'b0 ;
	end

//lw.
Lw : begin
	RegDst<=2'b00 ;
	Branch<=1'b0 ;
	MemRead<=1'b1 ;
	MemtoReg<=2'b01 ;
	MemWrite<=1'b0 ;
	ALUSrc<=1'b1 ;
	RegWrite<=1'b1 ;
	ALUOp<=3'b000 ;
	Jump<=1'b0 ;
	end

//ori
Ori : begin
	RegDst<=2'b00 ;
	Branch<=1'b0 ;
	MemRead<=1'b0 ;
	MemtoReg<=2'b00 ;
	MemWrite<=1'b0 ;
	ALUSrc<=1'b1 ;
	RegWrite<=1'b1 ;
	ALUOp<=3'b100 ;//need this added in 
	Jump<=1'b0 ;
	end

//sb
/*2'd40 : begin
	RegDst = 2'b ;
	Branch = 1'b ;
	MemRead= 1'b ;
	MemtoReg =2'b ;
	MemWrite =1'b ;
	ALUSrc = 1'b ;
	RegWrite = 1'b ;
	ALUOp = 2'b ;
	Jump = 1'b0 ;
	end
*/
//sh
/*2'd41 : begin
	RegDst = 2'b ;
	Branch = 1'b ;
	MemRead= 1'b ;
	MemtoReg =2'b ;
	MemWrite =1'b ;
	ALUSrc = 1'b ;
	RegWrite = 1'b ;
	ALUOp = 2'b ;
	Jump = 1'b0 ;
	end 
*/
//slti
/*2'd10 : begin
	RegDst = 2'b ;
	Branch = 1'b ;
	MemRead= 1'b ;
	MemtoReg =2'b ;
	MemWrite =1'b ;
	ALUSrc = 1'b ;
	RegWrite = 1'b ;
	ALUOp = 2'b ;
	Jump = 1'b0 ;
	end
*/
//sw.
Sw : begin
	RegDst<=2'bxx ;
	Branch<=1'b0 ;
	MemRead<=1'b0 ;
	MemtoReg<=2'bxx ;
	MemWrite<=1'b1 ;
	ALUSrc<=1'b1 ;
	RegWrite<=1'b0 ;
	ALUOp<=3'b000 ;
	Jump<=1'b0 ;
	end

//xori
/*2'd14 : begin
	RegDst = 2'b ;
	Branch = 1'b ;
	MemRead= 1'b ;
	MemtoReg =2'b ;
	MemWrite =1'b ;
	ALUSrc = 1'b ;
	RegWrite = 1'b ;
	ALUOp = 2'b ;
	Jump = 1'b0 ;
	end
*/
endcase
end
endmodule

module DM(Clk,Address,Write_Data,Read_Data,Mem_Read,Mem_Write);
parameter d_size = 32; //data size
parameter m_size = 1024; //memory size
input wire [d_size-1:0] Address, Write_Data; //32 bit address and write data wire
input wire Mem_Read, Mem_Write; //1 bit memread and memwrite wires
output reg [d_size-1:0] Read_Data; //32 bit register read data
input wire  Clk;

reg[d_size-1:0] Memory[0:m_size-1]; //32kb reg memory\




always @(posedge Clk)
begin
if(Mem_Read) //giving priority to memread if both memread and memwrite are high
begin
Read_Data <= Memory[Address];
end
else if(Mem_Write)
begin
Memory[Address] <= Write_Data;
end
end
endmodule

module alucontrol (func,aluop,out,clk,muxcontrol);
input [5:0] func ;
input [2:0] aluop;
input clk;
output reg muxcontrol;
output reg [3:0] out;

always @ (func,aluop)

begin 

case (aluop)

3'b000 : begin out <= 4'b0010 ; muxcontrol <= 1'b1; end// hy3ml add lel load aw store word and addi
3'b001 : begin out <= 4'b0110 ; muxcontrol <= 1'b1; end // hy3ml subtract for branch equal
3'b011 : begin out <= 4'b1111 ;muxcontrol <= 1'b1;end // add 3shan bta3t lui
3'b100 : begin out <=4'b0001; muxcontrol <= 1'b1;end // for ori
3'b010 : // y3ni R-type
case (func)
6'b100000 :begin out <= 4'b0010 ;muxcontrol <= 1'b1; end // hy3ml add 
6'b100010 : begin out <= 4'b0110 ;muxcontrol <= 1'b1; end // hy3ml subtract
6'b100100 : begin out <= 4'b0000 ;muxcontrol <= 1'b1; end // hy3ml and
6'b100101 : begin out <= 4'b0001 ;muxcontrol <= 1'b1; end // hy3ml or
6'b101010 : begin out <= 4'b0111 ;muxcontrol <= 1'b1; end// hy3ml set on less than
6'b000000 : begin out <= 4'b1110 ;muxcontrol <= 1'b1; end // hy3ml sll
6'b001000 : begin muxcontrol <= 1'b0; end // for the jump register
//6'b100110 : begin out <= 4'b0001 ;muxcontrol <= 1'b1; end // hy3ml or
endcase
endcase

end


endmodule
module And(in1,in2,out);
input wire  in1;
input wire in2;
output reg  out;
always@(in1,in2)
begin
out <= in1&in2;
end
endmodule

module clk(clk);
output reg clk;
initial
begin 
clk =0;
end
always
begin
#30
 clk <= ~clk;
end
endmodule

module tb_MIPS_4 ();
wire [31:0] PC_IM;
wire [31:0] Instruction;
wire [1:0] RegDst;
wire [1:0] MemtoReg;
wire [31:0] DM_ReadData_MemtoReg_Mux;
wire [31:0] ALU_Result;
wire MemRead;
wire MemWrite;
wire [31:0] Read_data1 ;
wire [31:0] jumpmux_jrmux ;
wire JRmuxcontrol;
wire [31:0] JRmux_PCinput;
wire [31:0] PC_Plus_4;
wire [31:0] branch_adder_result;
wire Branch ;
wire zero_flag;
wire Branch_Mux_selector;
wire [31:0] Branch_mux_jump_mux;
wire [31:0] Jump_Address;
wire [27:0] Jump_Address28;
wire jump;
wire [31:0] Sign_extend32;
wire [31:0] Sll_out;
wire [31:0] Read_data2 ;
wire [31:0] MemtoRegMux_out;
wire ALUSrc;
wire [31:0]ALUSrcMux_Output ; // e7na 3dlna hna
wire [3:0] ALUctr_output ;
wire [2:0] AluOp;
wire RegWrite;
assign Jump_Address = {PC_Plus_4[31:28],Jump_Address28};
wire [4:0] Write_Reg;
wire clock;

wire [31:0] pcinput  ;
reg reset;
wire reset_wire;
assign reset_wire = reset;

initial
begin
reset = 1;
#92
reset = 0 ;
end

Mux2to1_32bits masterreset(
.in1(JRmux_PCinput),
.in2(32'd0),
.sel(reset_wire),
.out1(pcinput)
);



clk my_clk(
.clk(clock)
);

And my_and (
.in1(Branch),
.in2(zero_flag),
.out(Branch_Mux_selector)
);
PC my_pc (
.newaddress(PC_IM),
.address(pcinput),
.clk(clock)
);
insmemory my_insmem (
.instruction(Instruction),
.address(PC_IM),
.clk(clock)
);
CPU my_cpu (
.Instruction(Instruction[31:26]),
.RegDst(RegDst),
.Branch(Branch),
.MemRead(MemRead),
.MemtoReg(MemtoReg),
.ALUOp(AluOp),
.MemWrite(MemWrite),
.ALUSrc(ALUSrc),
.RegWrite(RegWrite),
.Jump(jump),
.clk(clock)
);
Register_File my_RF (
.Read_Register1(Instruction[25:21]),
.Read_Register2(Instruction[20:16]),
.Write_Register(Write_Reg),
.Write_Data(MemtoRegMux_out),
.RegWrite(RegWrite),
.clk(clock),
.Read_Data1(Read_data1),
.Read_Data2(Read_data2)
);

Mux4to1_5bits RegDst_Mux(
.in1(Instruction[20:16]),
.in2(Instruction[15:11]),
.in3(32'd31),
.in4(32'd0),
.sel(RegDst),
.out1(Write_Reg)
);
Mux4to1_32bits MemtoReg_mux(
.in1(ALU_Result),
.in2(DM_ReadData_MemtoReg_Mux),
.in3(PC_Plus_4),
.in4(32'd0),
.sel(MemtoReg),
.out1(MemtoRegMux_out)
);
Sign_Extend my_signextend (
.in(Instruction[15:0]),
.out(Sign_extend32)
);
Shift_Left_2_26to28 myshiftleft(
.in(Instruction[25:0]),
.out(Jump_Address28)

);
Alu my_alu (
.a(Read_data1),
.b(ALUSrcMux_Output),
.c(Instruction[10:6]),// shamt
.aluctr(ALUctr_output),
.aluout(ALU_Result),
.zero(zero_flag)

);
Adder branch_adder (
.x(PC_Plus_4),
.y(Sll_out),
.sum(branch_adder_result)
);
Adder PC_adder (
.x(PC_IM),
.y(32'd1),
.sum(PC_Plus_4)
);
Shift_Left_2 my_sll (
.in(Sign_extend32),
.out(Sll_out)
);

DM my_datamem(
.Clk(clock),
.Address(ALU_Result),
.Write_Data(Read_data2),
.Read_Data(DM_ReadData_MemtoReg_Mux),
.Mem_Read(MemRead),
.Mem_Write(MemWrite)
);
Mux2to1_32bits JumpR_Mux (
.in1(Read_data1),
.in2(jumpmux_jrmux),
.sel(JRmuxcontrol),
.out1(JRmux_PCinput)
);
Mux2to1_32bits Jump_Mux (
.in1(Branch_mux_jump_mux),
.in2(Jump_Address),
.sel(jump),
.out1(jumpmux_jrmux)
);
Mux2to1_32bits Branch_Mux (
.in1(PC_Plus_4),
.in2(branch_adder_result),
.sel(Branch_Mux_selector),
.out1(Branch_mux_jump_mux)
);
alucontrol my_aluctr(
.func(Instruction[5:0]),
.aluop(AluOp),
.out(ALUctr_output),
.clk(clock),
.muxcontrol(JRmuxcontrol)
);
Mux2to1_32bits alusrc_Mux (
.in1(Read_data2),
.in2(Sign_extend32),
.sel(ALUSrc),
.out1(ALUSrcMux_Output)
);

endmodule






