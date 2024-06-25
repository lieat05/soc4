`define XLEN 32
`define REG_IDX 5
`define CSR_IDX 12 
`define LZC_WIDTH 5
`define RGIDX_NUM 32
`define PC_DEFAULT 32'h30000000

`define INFOBUS_OP_WIDTH 3
`define INFOBUS_OP       2:0
`define INFOBUS_OP_ALU   3'd0
`define INFOBUS_OP_BJP   3'd1
`define INFOBUS_OP_LSU   3'd2
`define INFOBUS_OP_CSR   3'd3
`define INFOBUS_OP_MUL   3'd4
`define INFOBUS_OP_VPU   3'd5

`define INFOBUS_ALU_WIDTH 19
`define INFOBUS_ALU_RV32  3
`define INFOBUS_ALU_ADD   4
`define INFOBUS_ALU_SUB   5
`define INFOBUS_ALU_SLT   6
`define INFOBUS_ALU_SLTU  7
`define INFOBUS_ALU_XOR   8
`define INFOBUS_ALU_SLL   9
`define INFOBUS_ALU_SRL   10
`define INFOBUS_ALU_SRA   11
`define INFOBUS_ALU_OR    12
`define INFOBUS_ALU_AND   13
`define INFOBUS_ALU_LUI   14
`define INFOBUS_ALU_IMM   15
`define INFOBUS_ALU_PC    16
`define INFOBUS_ALU_NOP   17
`define INFOBUS_ALU_EBRK  18

`define INFOBUS_BJP_WIDTH  14
`define INFOBUS_BJP_RV32   3
`define INFOBUS_BJP_JUMP   4
`define INFOBUS_BJP_BPRDT  5
`define INFOBUS_BJP_BEQ    6
`define INFOBUS_BJP_BNE    7
`define INFOBUS_BJP_BLT    8
`define INFOBUS_BJP_BGT    9
`define INFOBUS_BJP_BLTU   10
`define INFOBUS_BJP_BGTU   11
`define INFOBUS_BJP_BXX    12
`define INFOBUS_BJP_MRET   13

`define INFOBUS_LSU_WIDTH  12       
`define INFOBUS_LSU_RV32   3
`define INFOBUS_LSU_LOAD   4
`define INFOBUS_LSU_STORE  5
`define INFOBUS_LSU_SIZE   7:6
`define INFOBUS_LSU_USIGN  8
`define INFOBUS_LSU_OP2IMM 9
`define INFOBUS_LSU_FENCE  10
`define INFOBUS_LSU_FENCEI 11

`define INFOBUS_CSR_WIDTH  27
`define INFOBUS_CSR_RV32   3
`define INFOBUS_CSR_CSRRW  4
`define INFOBUS_CSR_CSRRS  5
`define INFOBUS_CSR_CSRRC  6
`define INFOBUS_CSR_ECAL   7
`define INFOBUS_CSR_RS1IMM 8
`define INFOBUS_CSR_ZIMMM  13:9
`define INFOBUS_CSR_RS1IS0 14
`define INFOBUS_CSR_CSRIDX 26:15

`define INFOBUS_MUL_WIDTH  12
`define INFOBUS_MUL_RV32   3
`define INFOBUS_MUL_MUL    4
`define INFOBUS_MUL_MULH   5
`define INFOBUS_MUL_MULHSU 6
`define INFOBUS_MUL_MULHU  7
`define INFOBUS_MUL_DIV    8
`define INFOBUS_MUL_DIVU   9
`define INFOBUS_MUL_REM    10
`define INFOBUS_MUL_REMU   11
//`define INFOBUS_MUL_B2B    12

module lieat_axi_master(
  input              clock,
  input              reset,
  // ================================================================================================================================================
  // ICACHE
  // ================================================================================================================================================
  input  [`XLEN-1:0] icache_axi_araddr,
  input              icache_axi_arvalid,
  output             icache_axi_arready,

  output [63:0]      icache_axi_rdata,
  output             icache_axi_rvalid,
  input              icache_axi_rready,
  // ================================================================================================================================================
  // DCACHE
  // ================================================================================================================================================
  input              dcache_axi_arvalid,
  output             dcache_axi_arready,
  input  [`XLEN-1:0] dcache_axi_araddr,
  input  [2:0]       dcache_axi_arsize,
  //R channel
  output             dcache_axi_rvalid,
  input              dcache_axi_rready,
  output [63:0]      dcache_axi_rdata,
  //AW channel
  input              dcache_axi_awvalid,
  output             dcache_axi_awready,
  input  [`XLEN-1:0] dcache_axi_awaddr,
  input  [2:0]       dcache_axi_awsize,  
  //W channel
  input              dcache_axi_wvalid,
  output             dcache_axi_wready,
  input  [63:0]      dcache_axi_wdata,
  input  [7:0]       dcache_axi_wstrb,
  //B channel
  output             dcache_axi_bvalid,
  input              dcache_axi_bready,
  output [1:0]       dcache_axi_bresp,
  // ================================================================================================================================================
  // MASTER
  // ================================================================================================================================================
  input              io_master_awready,
  output             io_master_awvalid,
  output [31:0]      io_master_awaddr,
  output [3:0]       io_master_awid,
  output [7:0]       io_master_awlen,
  output [2:0]       io_master_awsize,
  output [1:0]       io_master_awburst,

  input              io_master_wready,
  output             io_master_wvalid,
  output [63:0]      io_master_wdata,
  output [7:0]       io_master_wstrb,
  output             io_master_wlast,

  output             io_master_bready,
  input              io_master_bvalid,
  input  [1:0]       io_master_bresp,
  input  [3:0]       io_master_bid,

  input              io_master_arready,
  output             io_master_arvalid,
  output [31:0]      io_master_araddr,
  output [3:0]       io_master_arid,
  output [7:0]       io_master_arlen,
  output [2:0]       io_master_arsize,
  output [1:0]       io_master_arburst,

  output             io_master_rready,
  input              io_master_rvalid,
  input  [1:0]       io_master_rresp,
  input  [63:0]      io_master_rdata,
  input              io_master_rlast,
  input  [3:0]       io_master_rid
);
wire unused_ok = &{io_master_bresp,io_master_bid,io_master_rresp,io_master_rlast,io_master_rdata};
wire icache_arvalid_hold;
wire icache_arvalid_hold_set = icache_axi_arvalid & ~dcache_axi_arvalid;
wire icache_arvalid_hold_clr = io_master_arvalid & io_master_arready;
wire icache_arvalid_hold_ena = icache_arvalid_hold_set | icache_arvalid_hold_clr;
wire icache_arvalid_hold_nxt = icache_arvalid_hold_set & ~icache_arvalid_hold_clr;
lieat_general_dfflr #(1) icache_arvalid_hold_dff(clock,reset,icache_arvalid_hold_ena,icache_arvalid_hold_nxt,icache_arvalid_hold);

wire master_ring;
wire master_ring_set = io_master_arvalid & io_master_arready;
wire master_ring_clr = io_master_rvalid  & io_master_rready;
wire master_ring_ena = master_ring_set | master_ring_clr;
wire master_ring_nxt = master_ring_set | ~master_ring_clr;
lieat_general_dfflr #(1) master_ring_dff(clock,reset,master_ring_ena,master_ring_nxt,master_ring);
// ================================================================================================================================================
// ONLY DCACHE
// ================================================================================================================================================
assign io_master_awvalid = dcache_axi_awvalid;
assign io_master_awaddr  = dcache_axi_awaddr;
assign io_master_awlen   = 8'b0;
assign io_master_awid    = 4'b0001;
assign io_master_awsize  = dcache_axi_awsize;
assign io_master_awburst = 2'b00;
assign io_master_wlast   = 1'b1;
assign io_master_wvalid  = dcache_axi_wvalid;
assign io_master_wdata   = dcache_axi_wdata;
assign io_master_wstrb   = dcache_axi_wstrb;
assign io_master_bready  = dcache_axi_bready;

assign dcache_axi_awready= io_master_awready;
assign dcache_axi_wready = io_master_wready;
assign dcache_axi_bvalid = io_master_bvalid;
assign dcache_axi_bresp  = io_master_bresp;
// ================================================================================================================================================
// DCACHE AND ICACHE:D FIRST THEN I
// ================================================================================================================================================
assign io_master_arvalid = (dcache_axi_arvalid | icache_axi_arvalid | icache_arvalid_hold) & ~master_ring;
assign io_master_araddr  = icache_arvalid_hold? icache_axi_araddr :
                           dcache_axi_arvalid ? dcache_axi_araddr :
                           icache_axi_arvalid ? icache_axi_araddr : 32'h0;
assign io_master_arid    = icache_arvalid_hold? 4'b0010           :
                           dcache_axi_arvalid ? 4'b0001           :
                           icache_axi_arvalid ? 4'b0010           : 4'b0000;
assign io_master_arlen   = 8'b0;
assign io_master_arsize  = icache_arvalid_hold? 3'b010            :
                           dcache_axi_arvalid ? dcache_axi_arsize :
                           icache_axi_arvalid ? 3'b010            : 3'b0;
assign io_master_arburst = 2'b0;
assign io_master_rready  = (io_master_rid == 4'b0001) ? dcache_axi_rready :
                           (io_master_rid == 4'b0010) ? icache_axi_rready : 1'b0;

assign icache_axi_arready = io_master_arready & (icache_arvalid_hold | ~dcache_axi_arvalid) & ~master_ring;
assign icache_axi_rvalid  = (io_master_rid == 4'b0010) & io_master_rvalid ;
assign icache_axi_rdata   = {`XLEN*2{(io_master_rid == 4'b0010)}} & io_master_rdata;

assign dcache_axi_arready = io_master_arready & ~icache_arvalid_hold & io_master_arvalid & ~master_ring;
assign dcache_axi_rvalid  = (io_master_rid == 4'b0001) & io_master_rvalid;
assign dcache_axi_rdata   = {`XLEN*2{(io_master_rid == 4'b0001)}} & {io_master_rdata};
endmodule
module lieat_exu_com_alu(
  input              clock,
  input              reset,

  input              alu_i_valid,
  input  [`XLEN-1:0] alu_i_pc,
  input  [`XLEN-1:0] alu_i_imm,
  input  [`XLEN-1:0] alu_i_src1,
  input  [`XLEN-1:0] alu_i_src2,
  input  [`XLEN-1:0] alu_i_infobus,

  output             alu_req,
  output [`XLEN-1:0] alu_req_op1,
  output [`XLEN-1:0] alu_req_op2,
  output             alu_req_add,
  output             alu_req_sub,
  output             alu_req_xor,
  output             alu_req_sll,
  output             alu_req_srl,
  output             alu_req_sra,
  output             alu_req_or ,
  output             alu_req_and,
  output             alu_req_slt,
  output             alu_req_sltu,
  output             alu_req_lui,
  input  [`XLEN-1:0] alu_req_res,
  output [`XLEN-1:0] alu_wbck_data,
  output             alu_wbck_ebreak
);
wire unused_ok = &{alu_i_infobus};
wire alu_ebreak  = alu_i_infobus[`INFOBUS_ALU_EBRK];
wire alu_op1sel  = alu_i_infobus[`INFOBUS_ALU_PC ];
wire alu_op2sel  = alu_i_infobus[`INFOBUS_ALU_IMM];

assign alu_req      = alu_i_valid;
assign alu_req_op1  = alu_op1sel ? alu_i_pc  : alu_i_src1;
assign alu_req_op2  = alu_op2sel ? alu_i_imm : alu_i_src2;
assign alu_req_add  = alu_i_infobus [`INFOBUS_ALU_ADD ];
assign alu_req_sub  = alu_i_infobus [`INFOBUS_ALU_SUB ];
assign alu_req_xor  = alu_i_infobus [`INFOBUS_ALU_XOR ];
assign alu_req_sll  = alu_i_infobus [`INFOBUS_ALU_SLL ];
assign alu_req_srl  = alu_i_infobus [`INFOBUS_ALU_SRL ];
assign alu_req_sra  = alu_i_infobus [`INFOBUS_ALU_SRA ];
assign alu_req_or   = alu_i_infobus [`INFOBUS_ALU_OR  ];
assign alu_req_and  = alu_i_infobus [`INFOBUS_ALU_AND ];
assign alu_req_slt  = alu_i_infobus [`INFOBUS_ALU_SLT ];
assign alu_req_sltu = alu_i_infobus [`INFOBUS_ALU_SLTU];
assign alu_req_lui  = alu_i_infobus [`INFOBUS_ALU_LUI ];
assign alu_wbck_data= alu_req_res;
assign alu_wbck_ebreak = alu_ebreak;
endmodule
module lieat_exu_com_bjp(
  input                 bjp_i_valid,
  input  [`XLEN-1:0]    bjp_i_pc,
  input  [`XLEN-1:0]    bjp_i_imm,
  input  [`XLEN-1:0]    bjp_i_src1,
  input  [`XLEN-1:0]    bjp_i_src2,
  input  [`XLEN-1:0]    bjp_i_infobus,
  
  output                bjp_req,
  output [`XLEN-1:0]    bjp_req_op1,
  output [`XLEN-1:0]    bjp_req_op2,
  output                bjp_req_cmp,
  output                bjp_req_beq,
  output                bjp_req_bne,
  output                bjp_req_blt,
  output                bjp_req_bgt,
  output                bjp_req_bltu,
  output                bjp_req_bgtu,
  output                bjp_req_add,
  input  [`XLEN-1:0]    bjp_req_res,
  output [`XLEN-1:0]    bjp_wbck_data,
  
  output                callback_en,
  output [`REG_IDX-1:0] callback_index,
  output                callback_result,
  output                callback_flush,
  output [`XLEN-1:0]    callback_truepc
);
wire unused_ok = &{bjp_i_infobus};
wire bjp_rv32 = bjp_i_infobus[`INFOBUS_BJP_RV32];
wire bjp_jump = bjp_i_infobus[`INFOBUS_BJP_JUMP];
wire bjp_prdt = bjp_i_infobus[`INFOBUS_BJP_BPRDT];
wire bjp_cmp  = bjp_req_res[0];
wire [31:0] offset = bjp_cmp ? bjp_i_imm : 32'd4;

assign bjp_req     = bjp_i_valid;
assign bjp_req_op1 = bjp_jump ? bjp_i_pc : bjp_i_src1;
assign bjp_req_op2 = bjp_jump ? (bjp_rv32 ? `XLEN'd4 : `XLEN'd2) : bjp_i_src2;
assign bjp_req_add = bjp_jump;
assign bjp_req_beq = bjp_i_infobus[`INFOBUS_BJP_BEQ];
assign bjp_req_bne = bjp_i_infobus[`INFOBUS_BJP_BNE];
assign bjp_req_blt = bjp_i_infobus[`INFOBUS_BJP_BLT];
assign bjp_req_bgt = bjp_i_infobus[`INFOBUS_BJP_BGT];
assign bjp_req_bltu= bjp_i_infobus[`INFOBUS_BJP_BLTU];
assign bjp_req_bgtu= bjp_i_infobus[`INFOBUS_BJP_BGTU];
assign bjp_req_cmp = bjp_i_infobus[`INFOBUS_BJP_BXX];
assign bjp_wbck_data = bjp_req_res;

assign callback_en     = bjp_req_cmp;
assign callback_index  = bjp_i_pc[6:2];
assign callback_result = bjp_cmp;
assign callback_flush  = bjp_req_cmp & (bjp_prdt ^ bjp_cmp);
assign callback_truepc = bjp_i_pc + offset;
endmodule
module lieat_exu_com_csrreg(
  input                 clock,
  input                 reset,

  input                 csr_ena,
  input                 csr_write,
  input                 csr_read,
  input  [`CSR_IDX-1:0] csr_idx,
  input  [`XLEN-1:0]    csr_wdata,
  output [`XLEN-1:0]    csr_rdata,
  input  [`CSR_IDX-1:0] csr_idx2,
  input  [`XLEN-1:0]    csr_wdata2,

  input                 ifu_csr_ren,
  input  [`CSR_IDX-1:0] ifu_csr_idx,
  output [`XLEN-1:0]    ifu_csr_rdata
);
/*
wire             sel_ustatus = (csr_idx == 12'h000);
wire             ustatus_wen = sel_ustatus & csr_wen;
wire             ustatus_ren = sel_ustatus & csr_ren;
wire [`XLEN-1:0] csr_ustatus;

wire             sel_mie     = (csr_idx == 12'h304);
wire             mie_wen     = sel_mie & csr_wen;
wire             mie_ren     = sel_mie & csr_ren;
wire [`XLEN-1:0] csr_mtvec;

wire             sel_mip     = (csr_idx == 12'h344);
wire             mip_wen     = sel_mip & csr_wen;
wire             mip_ren     = sel_mip & csr_ren;
wire [`XLEN-1:0] csr_mtvec;

wire             sel_mscratch= (csr_idx == 12'h340);
wire             mscratch_wen= sel_mscratch & csr_wen;
wire             mscratch_ren= sel_mscratch & csr_ren;
wire [`XLEN-1:0] csr_mtvec;

wire             sel_mbadaddr= (csr_idx == 12'h343);
wire             mbadaddr_wen= sel_mbadaddr & csr_wen;
wire             mbadaddr_ren= sel_mbadaddr & csr_ren;
wire [`XLEN-1:0] csr_mtvec;

wire sel_misa = (csr_idx == 12'h301);
wire [`E203_XLEN-1:0] csr_misa = {
    2'b1
   ,4'b0 //WIRI
   ,1'b0 //              25 Z Reserved
   ,1'b0 //              24 Y Reserved
   ,1'b0 //              23 X Non-standard extensions present
   ,1'b0 //              22 W Reserved
   ,1'b0 //              21 V Tentatively reserved for Vector extension 20 U User mode implemented
   ,1'b0 //              20 U User mode implemented
   ,1'b0 //              19 T Tentatively reserved for Transactional Memory extension
   ,1'b0 //              18 S Supervisor mode implemented
   ,1'b0 //              17 R Reserved
   ,1'b0 //              16 Q Quad-precision floating-point extension
   ,1'b0 //              15 P Tentatively reserved for Packed-SIMD extension
   ,1'b0 //              14 O Reserved
   ,1'b0 //              13 N User-level interrupts supported
   ,1'b1 // 12 M Integer Multiply/Divide extension
   ,1'b0 //              11 L Tentatively reserved for Decimal Floating-Point extension
   ,1'b0 //              10 K Reserved
   ,1'b0 //              9 J Reserved
   `ifdef E203_RFREG_NUM_IS_32
   ,1'b1 // 8 I RV32I/64I/128I base ISA
   `else
   ,1'b0
   `endif
   ,1'b0 //              7 H Hypervisor mode implemented
   ,1'b0 //              6 G Additional standard extensions present
  `ifndef E203_HAS_FPU//{
   ,1'b0 //              5 F Single-precision floating-point extension
  `endif//
   `ifdef E203_RFREG_NUM_IS_32
   ,1'b0 //              4 E RV32E base ISA
   `else
   ,1'b1 //              
   `endif
  `ifndef E203_HAS_FPU//{
   ,1'b0 //              3 D Double-precision floating-point extension
  `endif//
   ,1'b1 // 2 C Compressed extension
   ,1'b0 //              1 B Tentatively reserved for Bit operations extension
  `ifdef E203_SUPPORT_AMO//{
   ,1'b1 //              0 A Atomic extension
  `endif//E203_SUPPORT_AMO}
  `ifndef E203_SUPPORT_AMO//{
   ,1'b0 //              0 A Atomic extension
  `endif//}
                           };
wire sel_dcsr     = (csr_idx == 12'h7b0);
wire sel_dpc      = (csr_idx == 12'h7b1);
wire sel_dscratch = (csr_idx == 12'h7b2);                           
wire sel_mcycle    = (csr_idx == 12'hB00);
wire sel_mcycleh   = (csr_idx == 12'hB80);
wire sel_minstret  = (csr_idx == 12'hB02);
wire sel_minstreth = (csr_idx == 12'hB82);
wire sel_counterstop = (csr_idx == 12'hBFF);
wire sel_mcgstop = (csr_idx == 12'hBFE);
wire sel_itcmnohold = (csr_idx == 12'hBFD);
wire sel_mdvnob2b = (csr_idx == 12'hBF0);

*/
wire csr_wen = csr_ena & (~csr_ilgl) & csr_write;
wire csr_ren = csr_ena & (~csr_ilgl) & csr_read;
wire csr_ilgl = 1'b0;

//0x300 MRW mstatus Machine status register.
wire             sel_mstatus = (csr_idx == `CSR_IDX'h300);
wire             sel_mstatus2= (csr_idx2 == `CSR_IDX'h300);
wire             mstatus_wen = (sel_mstatus | sel_mstatus2) & csr_wen;
wire             mstatus_ren = sel_mstatus & csr_ren;
wire [`XLEN-1:0] mstatus_wdata = sel_mstatus2 ? csr_wdata2 : csr_wdata;
wire [`XLEN-1:0] csr_mstatus;
//0x305 MRW mtvec Machine trap-handler base address.
wire             sel_mtvec   = (csr_idx == `CSR_IDX'h305);
wire             sel_mtvec2  = (csr_idx2 == `CSR_IDX'h305);
wire             mtvec_wen   = (sel_mtvec | sel_mtvec2) & csr_wen;
wire             mtvec_ren   = sel_mtvec & csr_ren;
wire             mtvec_ifren = ifu_csr_ren & (ifu_csr_idx == `CSR_IDX'h305);
wire [`XLEN-1:0] mtvec_wdata = sel_mtvec2 ? csr_wdata2 : csr_wdata;
wire [`XLEN-1:0] csr_mtvec;
//0x341 MRW mepc Machine exception program counter.
wire             sel_mepc    = (csr_idx == `CSR_IDX'h341);
wire             sel_mepc2   = (csr_idx2 == `CSR_IDX'h341);
wire             mepc_wen    = (sel_mepc | sel_mepc2) & csr_wen;
wire             mepc_ren    = sel_mepc & csr_ren;
wire             mepc_ifren  = ifu_csr_ren & (ifu_csr_idx == `CSR_IDX'h341);
wire [`XLEN-1:0] mepc_wdata  = sel_mepc2 ? csr_wdata2 : csr_wdata;
wire [`XLEN-1:0] csr_mepc;
//0x342 MRW mcause Machine trap cause.
wire             sel_mcause  = (csr_idx == `CSR_IDX'h342);
wire             sel_mcause2 = (csr_idx2 == `CSR_IDX'h342);
wire             mcause_wen  = (sel_mcause | sel_mcause2) & csr_wen;
wire             mcause_ren  = sel_mcause & csr_ren;
wire [`XLEN-1:0] mcause_wdata= sel_mcause2 ? csr_wdata2 : csr_wdata;
wire [`XLEN-1:0] csr_mcause;

lieat_general_dfflr #(`XLEN) mstatus_dff(clock,reset,mstatus_wen,mstatus_wdata,csr_mstatus);
lieat_general_dfflr #(`XLEN) mtvec_dff(clock,reset,mtvec_wen,mtvec_wdata,csr_mtvec);
lieat_general_dfflr #(`XLEN) mepc_dff(clock,reset,mepc_wen,mepc_wdata,csr_mepc);
lieat_general_dfflr #(`XLEN) mcause_dff(clock,reset,mcause_wen,mcause_wdata,csr_mcause);

assign csr_rdata = ({`XLEN{mstatus_ren}} & csr_mstatus) |
                   ({`XLEN{mtvec_ren  }} & csr_mtvec  ) |
                   ({`XLEN{mepc_ren   }} & csr_mepc   ) |
                   ({`XLEN{mcause_ren }} & csr_mcause ) ;
assign ifu_csr_rdata = ({`XLEN{mepc_ifren}} & csr_mepc) | ({`XLEN{mtvec_ifren}} & csr_mtvec);
endmodule
module lieat_exu_com_csr(
  input                 csr_i_valid,
  input  [`XLEN-1:0]    csr_i_pc,
  input  [`XLEN-1:0]    csr_i_src1,
  input  [`XLEN-1:0]    csr_i_infobus,

  output                csr_req,
  output [`XLEN-1:0]    csr_req_op1,
  output [`XLEN-1:0]    csr_req_op2,
  output                csr_req_or,
  output                csr_req_and,
  input  [`XLEN-1:0]    csr_req_res,
  output [`XLEN-1:0]    csr_wbck_data,

  output                csr_ena,
  output                csr_write,
  output                csr_read,
  output [`CSR_IDX-1:0] csr_idx,
  output [`XLEN-1:0]    csr_wdata,
  input  [`XLEN-1:0]    csr_rdata,
  output [`CSR_IDX-1:0] csr_idx2,
  output [`XLEN-1:0]    csr_wdata2
);
wire unused_ok = &{csr_i_infobus};
wire       csr_csrrw  = csr_i_infobus[`INFOBUS_CSR_CSRRW ];
wire       csr_csrrs  = csr_i_infobus[`INFOBUS_CSR_CSRRS ];
wire       csr_csrrc  = csr_i_infobus[`INFOBUS_CSR_CSRRC ];
wire       csr_ecall  = csr_i_infobus[`INFOBUS_CSR_ECAL  ];
wire       csr_rs1imm = csr_i_infobus[`INFOBUS_CSR_RS1IMM];
wire [4:0] csr_imm    = csr_i_infobus[`INFOBUS_CSR_ZIMMM ];

assign csr_ena     = csr_i_valid;
assign csr_write   = csr_i_valid;
assign csr_read    = csr_i_valid;
assign csr_idx     = csr_ecall ? `CSR_IDX'h341 : csr_i_infobus[`INFOBUS_CSR_CSRIDX];
assign csr_idx2    = csr_ecall ? `CSR_IDX'h342 : `CSR_IDX'h0;
assign csr_wdata   = csr_ecall ? csr_i_pc : csr_req_res;
assign csr_wdata2  = {31'h0,csr_ecall};

assign csr_req     = csr_i_valid;
assign csr_req_op1 = csr_rs1imm ? {27'b0,csr_imm} : csr_i_src1;
assign csr_req_op2 = ({`XLEN{csr_csrrw}} & `XLEN'h0  ) | 
                     ({`XLEN{csr_csrrs}} & csr_rdata ) | 
                     ({`XLEN{csr_csrrc}} & csr_rdata );
assign csr_req_or  = csr_csrrs | csr_csrrw;
assign csr_req_and = csr_csrrc;
assign csr_wbck_data = csr_rdata;
endmodule
module lieat_exu_com(
  input                 clock,
  input                 reset,  

  input                 com_i_valid,
  output                com_i_ready,
  input [`XLEN-1:0]     com_i_pc,
  input [`XLEN-1:0]     com_i_imm,
  input [`XLEN-1:0]     com_i_src1,
  input [`XLEN-1:0]     com_i_src2,
  input [`XLEN-1:0]     com_i_infobus,
  input [`REG_IDX-1:0]  com_i_rd,
  input                 com_i_rdwen,

  output                com_o_valid,
  input                 com_o_ready,
  output [`XLEN-1:0]    com_o_pc,
  output                com_o_wen,
  output [`REG_IDX-1:0] com_o_rd,
  output [`XLEN-1:0]    com_o_data,
  output                com_o_ebreak,
  
  output                com_forward_valid,
  output [`XLEN-1:0]    com_forward_data,

  output                callback_en,
  output [`REG_IDX-1:0] callback_index,
  output                callback_result,
  output                callback_flush,
  output [`XLEN-1:0]    callback_truepc,

  input                 ifu_csr_ren,
  input  [`CSR_IDX-1:0] ifu_csr_idx,
  output [`XLEN-1:0]    ifu_csr_rdata
);
// ================================================================================================================================================
// INPUT SIGNAL
// ================================================================================================================================================
wire com_i_sh = com_i_valid & com_i_ready;
wire com_o_sh = com_o_valid & com_o_ready;
assign com_i_ready   = ~com_o_valid | com_o_sh;

wire com_rdwen;
wire [`XLEN-1:0] com_pc;
wire [`XLEN-1:0] com_imm;
wire [`XLEN-1:0] com_infobus;
wire [`REG_IDX-1:0] com_rd;
lieat_general_dfflr #(32) com_pc_dff(clock,reset,com_i_sh,com_i_pc,com_pc);
lieat_general_dfflr #(32) com_imm_dff(clock,reset,com_i_sh,com_i_imm,com_imm);
lieat_general_dfflr #(32) com_infobus_dff(clock,reset,com_i_sh,com_i_infobus,com_infobus);
lieat_general_dfflr #(5)  com_rd_dff(clock,reset,com_i_sh,com_i_rd,com_rd);
lieat_general_dfflr #(1)  com_rdwen_dff(clock,reset,com_i_sh,com_i_rdwen,com_rdwen);
// ================================================================================================================================================
// STATE CONTROL
// ================================================================================================================================================
wire com_o_valid_set = com_i_sh;
wire com_o_valid_clr = com_o_sh;
wire com_o_valid_ena = com_o_valid_set | com_o_valid_clr;
wire com_o_valid_nxt = com_o_valid_set | ~com_o_valid_clr;
lieat_general_dfflr #(1) com_o_valid_dff(clock,reset,com_o_valid_ena,com_o_valid_nxt,com_o_valid);

wire com_req_valid;
wire com_req_valid_set = com_i_sh;
wire com_req_valid_clr = com_req_valid;
wire com_req_valid_ena = com_req_valid_set | com_req_valid_clr;
wire com_req_valid_nxt = com_req_valid_set | ~com_req_valid_clr;
lieat_general_dfflr #(1) com_req_valid_dff(clock,reset,com_req_valid_ena,com_req_valid_nxt,com_req_valid);
// ================================================================================================================================================
// INPUT DISP AND OUTPUT SEL
// ================================================================================================================================================
wire             alu_i_valid   = com_req_valid & (com_infobus[`INFOBUS_OP] == 3'd0);
wire [`XLEN-1:0] alu_i_pc      = {`XLEN{alu_i_valid}} & com_pc;
wire [`XLEN-1:0] alu_i_imm     = {`XLEN{alu_i_valid}} & com_imm;
wire [`XLEN-1:0] alu_i_src1    = {`XLEN{alu_i_valid}} & com_i_src1;
wire [`XLEN-1:0] alu_i_src2    = {`XLEN{alu_i_valid}} & com_i_src2;
wire [`XLEN-1:0] alu_i_infobus = {`XLEN{alu_i_valid}} & com_infobus;

wire             bjp_i_valid   = com_req_valid & (com_infobus[`INFOBUS_OP] == 3'd1);
wire [`XLEN-1:0] bjp_i_pc      = {`XLEN{bjp_i_valid}} & com_pc;
wire [`XLEN-1:0] bjp_i_imm     = {`XLEN{bjp_i_valid}} & com_imm;
wire [`XLEN-1:0] bjp_i_src1    = {`XLEN{bjp_i_valid}} & com_i_src1;
wire [`XLEN-1:0] bjp_i_src2    = {`XLEN{bjp_i_valid}} & com_i_src2;
wire [`XLEN-1:0] bjp_i_infobus = {`XLEN{bjp_i_valid}} & com_infobus;

wire             csr_i_valid   = com_req_valid & (com_infobus[`INFOBUS_OP] == 3'd3);
wire [`XLEN-1:0] csr_i_pc      = {`XLEN{csr_i_valid}} & com_pc;
wire [`XLEN-1:0] csr_i_src1    = {`XLEN{csr_i_valid}} & com_i_src1;
wire [`XLEN-1:0] csr_i_infobus = {`XLEN{csr_i_valid}} & com_infobus;

wire [`XLEN-1:0] alu_wbck_data;
wire [`XLEN-1:0] bjp_wbck_data;
wire [`XLEN-1:0] csr_wbck_data;
wire [`XLEN-1:0] com_wbck_data_nxt = alu_wbck_data | bjp_wbck_data | csr_wbck_data;
wire [`XLEN-1:0] com_wbck_data_reg;
lieat_general_dffr #(1) com_forward_valid_dff(clock,reset,com_o_valid & ~com_o_ready,com_forward_valid);
lieat_general_dfflr #(`XLEN) com_o_data_dff(clock,reset,com_req_valid,com_wbck_data_nxt,com_wbck_data_reg);

assign com_o_pc   = com_pc;
assign com_o_wen  = com_rdwen;
assign com_o_rd   = com_rd;
assign com_o_data = com_req_valid ? com_wbck_data_nxt : com_wbck_data_reg;
assign com_forward_data = com_wbck_data_reg;
// ================================================================================================================================================
// ALU
// ================================================================================================================================================
lieat_exu_com_alu alu(
  .clock(clock),
  .reset(reset),

  .alu_i_valid(alu_i_valid),
  .alu_i_pc(alu_i_pc),
  .alu_i_imm(alu_i_imm),
  .alu_i_src1(alu_i_src1),
  .alu_i_src2(alu_i_src2),
  .alu_i_infobus(alu_i_infobus),

  .alu_req(alu_req),
  .alu_req_op1(alu_req_op1),
  .alu_req_op2(alu_req_op2),
  .alu_req_add(alu_req_add),
  .alu_req_sub(alu_req_sub),
  .alu_req_xor(alu_req_xor),
  .alu_req_sll(alu_req_sll),
  .alu_req_srl(alu_req_srl),
  .alu_req_sra(alu_req_sra),
  .alu_req_or(alu_req_or),
  .alu_req_and(alu_req_and),
  .alu_req_slt(alu_req_slt),
  .alu_req_sltu(alu_req_sltu),
  .alu_req_lui(alu_req_lui),
  .alu_req_res(alu_req_res),
  .alu_wbck_data(alu_wbck_data),
  .alu_wbck_ebreak(com_o_ebreak)
);
// ================================================================================================================================================
// BJP
// ================================================================================================================================================
lieat_exu_com_bjp bjp(
  .bjp_i_valid(bjp_i_valid),
  .bjp_i_pc(bjp_i_pc),
  .bjp_i_imm(bjp_i_imm),
  .bjp_i_src1(bjp_i_src1),
  .bjp_i_src2(bjp_i_src2),
  .bjp_i_infobus(bjp_i_infobus),

  .bjp_req(bjp_req),
  .bjp_req_op1(bjp_req_op1),
  .bjp_req_op2(bjp_req_op2),
  .bjp_req_cmp(bjp_req_cmp),
  .bjp_req_beq(bjp_req_beq),
  .bjp_req_bne(bjp_req_bne),
  .bjp_req_blt(bjp_req_blt),
  .bjp_req_bgt(bjp_req_bgt),
  .bjp_req_bltu(bjp_req_bltu),
  .bjp_req_bgtu(bjp_req_bgtu),
  .bjp_req_add(bjp_req_add),
  .bjp_req_res(bjp_req_res),
  .bjp_wbck_data(bjp_wbck_data),

  .callback_en(callback_en),
  .callback_index(callback_index),
  .callback_result(callback_result),
  .callback_flush(callback_flush),
  .callback_truepc(callback_truepc)
);
// ================================================================================================================================================
// CSR
// ================================================================================================================================================
lieat_exu_com_csr csr(
  .csr_i_valid(csr_i_valid),
  .csr_i_pc(csr_i_pc),
  .csr_i_src1(csr_i_src1),
  .csr_i_infobus(csr_i_infobus),

  .csr_req(csr_req),
  .csr_req_op1(csr_req_op1),
  .csr_req_op2(csr_req_op2),
  .csr_req_or(csr_req_or),
  .csr_req_and(csr_req_and),
  .csr_req_res(csr_req_res),
  .csr_wbck_data(csr_wbck_data),

  .csr_ena(csr_ena),
  .csr_write(csr_write),
  .csr_read(csr_read),
  .csr_idx(csr_idx),
  .csr_wdata(csr_wdata),
  .csr_rdata(csr_rdata),
  .csr_idx2(csr_idx2),
  .csr_wdata2(csr_wdata2)
);
// ================================================================================================================================================
// SHARE COMMUTE
// ================================================================================================================================================
wire             alu_req;
wire             alu_req_add;
wire             alu_req_sub;
wire             alu_req_xor;
wire             alu_req_sll;
wire             alu_req_srl;
wire             alu_req_sra;
wire             alu_req_or;
wire             alu_req_and;
wire             alu_req_slt;
wire             alu_req_sltu;
wire             alu_req_lui;
wire [`XLEN-1:0] alu_req_op1;
wire [`XLEN-1:0] alu_req_op2;
wire [`XLEN-1:0] alu_req_res;

wire             bjp_req;
wire             bjp_req_cmp;
wire             bjp_req_add;
wire             bjp_req_beq;
wire             bjp_req_bne;
wire             bjp_req_blt;
wire             bjp_req_bgt;
wire             bjp_req_bltu;
wire             bjp_req_bgtu;
wire [`XLEN-1:0] bjp_req_op1;
wire [`XLEN-1:0] bjp_req_op2;
wire [`XLEN-1:0] bjp_req_res;

wire             csr_req;
wire             csr_req_or;
wire             csr_req_and;
wire [`XLEN-1:0] csr_req_op1;
wire [`XLEN-1:0] csr_req_op2;
wire [`XLEN-1:0] csr_req_res;

lieat_exu_share share(
  .alu_req(alu_req),
  .alu_req_op1(alu_req_op1),
  .alu_req_op2(alu_req_op2),
  .alu_req_add(alu_req_add),
  .alu_req_sub(alu_req_sub),
  .alu_req_xor(alu_req_xor),
  .alu_req_sll(alu_req_sll),
  .alu_req_srl(alu_req_srl),
  .alu_req_sra(alu_req_sra),  
  .alu_req_or(alu_req_or),
  .alu_req_and(alu_req_and),
  .alu_req_slt(alu_req_slt),
  .alu_req_sltu(alu_req_sltu),
  .alu_req_lui(alu_req_lui),
  .alu_req_result(alu_req_res),

  .bjp_req(bjp_req),
  .bjp_req_op1(bjp_req_op1),
  .bjp_req_op2(bjp_req_op2),
  .bjp_req_cmp(bjp_req_cmp),
  .bjp_req_add(bjp_req_add),
  .bjp_req_beq(bjp_req_beq),
  .bjp_req_bne(bjp_req_bne),
  .bjp_req_blt(bjp_req_blt),
  .bjp_req_bgt(bjp_req_bgt),
  .bjp_req_bltu(bjp_req_bltu),
  .bjp_req_bgtu(bjp_req_bgtu),
  .bjp_req_result(bjp_req_res),

  .csr_req(csr_req),
  .csr_req_op1(csr_req_op1),
  .csr_req_op2(csr_req_op2),
  .csr_req_or(csr_req_or),
  .csr_req_and(csr_req_and),
  .csr_req_result(csr_req_res)
);
// ================================================================================================================================================
// CSR REG
// ================================================================================================================================================
wire                csr_ena;
wire                csr_write;
wire                csr_read;
wire [`CSR_IDX-1:0] csr_idx;
wire [`XLEN-1:0]    csr_wdata;
wire [`XLEN-1:0]    csr_rdata;
wire [`CSR_IDX-1:0] csr_idx2;
wire [`XLEN-1:0]    csr_wdata2;

lieat_exu_com_csrreg csrreg(
  .clock(clock),
  .reset(reset),
  .csr_ena(csr_ena),
  .csr_write(csr_write),
  .csr_read(csr_read),
  .csr_idx(csr_idx),
  .csr_wdata(csr_wdata),
  .csr_rdata(csr_rdata),
  .csr_idx2(csr_idx2),
  .csr_wdata2(csr_wdata2),
  .ifu_csr_ren(ifu_csr_ren),
  .ifu_csr_idx(ifu_csr_idx),
  .ifu_csr_rdata(ifu_csr_rdata)
);
endmodule
module lieat_exu_dcache #(
  parameter CACHE_WAY = 2,
  parameter INDEX_LEN = 6,//cache has 2 ways: 1KB
  parameter CACHE_SIZE = 64,//each way has 64 blocks
  parameter TAG_WIDTH  = 24,
  parameter OFFSET_LEN = 2//each block has 4 bytes
)(
  input                clock,
  input                reset,

  input                lsu_req_valid,
  output reg           lsu_req_ready,
  input                lsu_req_ren,
  input                lsu_req_wen,
  input  [`XLEN-1:0]   lsu_req_addr,
  input  [2:0]         lsu_req_flag,
  input  [`XLEN-1:0]   lsu_req_wdata,
  output               lsu_rsp_valid,
  input                lsu_rsp_ready,
  output [`XLEN-1:0]   lsu_rsp_rdata,
  input                lsu_req_fencei,
  //AR channel
  output               dcache_axi_arvalid,
  input                dcache_axi_arready,
  output [`XLEN-1:0]   dcache_axi_araddr,
  output [2:0]         dcache_axi_arsize,
  //R channel
  input                dcache_axi_rvalid,
  output               dcache_axi_rready,
  input  [`XLEN*2-1:0] dcache_axi_rdata,
  //AW channel
  output               dcache_axi_awvalid,
  input                dcache_axi_awready,
  output [`XLEN-1:0]   dcache_axi_awaddr,
  output [2:0]         dcache_axi_awsize,
  //W channel
  output               dcache_axi_wvalid,
  input                dcache_axi_wready,
  output [`XLEN*2-1:0] dcache_axi_wdata,
  output [7:0]         dcache_axi_wstrb,
  //B channel
  input                dcache_axi_bvalid,
  output               dcache_axi_bready,
  input  [1:0]         dcache_axi_bresp
);
wire req_sh = lsu_req_valid & lsu_req_ready;
wire rsp_sh = lsu_rsp_valid & lsu_rsp_ready;
wire aw_sh  = dcache_axi_awvalid & dcache_axi_awready;
wire w_sh   = dcache_axi_wvalid & dcache_axi_wready;
wire ar_sh  = dcache_axi_arvalid & dcache_axi_arready;
wire r_sh   = dcache_axi_rvalid & dcache_axi_rready;
wire b_sh   = dcache_axi_bvalid & dcache_axi_bready;

wire [2:0]dcache_len = (lsu_req_flag[1:0] == 2'b00) ? 3'b000 :
                       (lsu_req_flag[1:0] == 2'b01) ? 3'b001 : 
                       (lsu_req_flag[1:0] == 2'b10) ? 3'b010 : 3'b000;

wire [31:0] sel_rdata = hit0              ? dcache_sram_rdata[31:0] :
                        hit1              ? dcache_sram_rdata[63:32]:
                        dcache_axi_rvalid ? dcache_axi_rdata32      : 32'h0;

wire [31:0] dcache_axi_rdata32 = lsu_req_addr[2] ? dcache_axi_rdata[63:32] : dcache_axi_rdata[31:0];
wire [31:0] dcache_axi_wdata32 = (req_sh | sync_valid) ? dcache_axi_wdata32_pre : dcache_axi_wdata32_reg;
wire [31:0] dcache_axi_wdata32_reg;
wire [31:0] dcache_axi_wdata32_pre = write_outaddr            ? lsu_req_wdata                                                   :
                                     read_miss_need_renew     ? (lru_sel1 ? dcache_sram_rdata[63:32] : dcache_sram_rdata[31:0]) :
                                     write_miss_need_renew    ? (lru_sel1 ? dcache_sram_rdata[63:32] : dcache_sram_rdata[31:0]) :
                                     sync_valid               ? (count[0] ? dcache_sram_rdata[63:32] : dcache_sram_rdata[31:0]) : 32'h0;
lieat_general_dfflr #(32) dcache_axi_wdata32_dff(clock,reset,req_sh,dcache_axi_wdata32_pre,dcache_axi_wdata32_reg);
// ================================================================================================================================================
// FENCEI
// ================================================================================================================================================
wire [INDEX_LEN:0] count;
wire [INDEX_LEN:0] count_nxt = (req_sh & lsu_req_fencei) ? 7'b0000000 : (count+1);
wire count_ena = (state_r[STATE_SYNC_BIT] & (dcache_axi_bvalid | sync_clean)) | (req_sh & lsu_req_fencei);
wire count_over = (count == 7'b1111111);
lieat_general_dfflr #(7) sync_count_dff(clock,reset,count_ena,count_nxt,count);

wire sync_ena;
wire sync_ena_set = count_ena;
wire sync_ena_clr = (sync_ena & dcache_axi_awvalid & dcache_axi_awready);
wire sync_ena_ena = sync_ena_set | sync_ena_clr;
wire sync_ena_nxt = sync_ena_set | ~sync_ena_clr;
lieat_general_dfflr #(1) sync_ena_dff(clock,reset,sync_ena_ena,sync_ena_nxt,sync_ena);
wire sync_clean   = (cache_tags_lru_vld[count[6:1]][count[0]][4] == 1'b0);
wire sync_valid   = sync_ena & ~sync_clean; 
// ================================================================================================================================================
// CHANNEL: INTERACTIVE WITH LSU
// ================================================================================================================================================
reg [3:0]state_r;
reg [3:0]state_nxt;
reg [2:0]state_rr;
reg [2:0]state_rnxt;
reg [4:0]state_wr;
reg [4:0]state_wnxt;

localparam STATE_IDLE = 4'b0001;
localparam STATE_VALD = 4'b0010;
localparam STATE_BUSY = 4'b0100;
localparam STATE_SYNC = 4'b1000;
localparam STATE_IDLE_BIT = 0;
localparam STATE_VALD_BIT = 1;
localparam STATE_BUSY_BIT = 2;
localparam STATE_SYNC_BIT = 3;

localparam STATER_ID = 3'b001;
localparam STATER_AR = 3'b010;
localparam STATER_R  = 3'b100;
localparam STATER_ID_BIT = 0;
localparam STATER_AR_BIT = 1;
localparam STATER_R_BIT  = 2;

localparam STATEW_ID  = 5'b00001;
localparam STATEW_AWW = 5'b00010;
localparam STATEW_AW  = 5'b00100;
localparam STATEW_W   = 5'b01000;
localparam STATEW_B   = 5'b10000;
localparam STATEW_ID_BIT = 0;
localparam STATEW_AWW_BIT= 1;
localparam STATEW_AW_BIT = 2;
localparam STATEW_W_BIT  = 3;
localparam STATEW_B_BIT  = 4;

always @(*) begin
  case(state_wr)
    STATEW_ID :state_wnxt = (aw_sh & w_sh)     ? STATEW_B  :
                            aw_sh              ? STATEW_W  :
                            w_sh               ? STATEW_AW :
                            dcache_axi_awvalid ? STATEW_AWW: STATEW_ID;
    STATEW_AWW:state_wnxt = (aw_sh & w_sh)     ? STATEW_B  :
                            aw_sh              ? STATEW_W  :
                            w_sh               ? STATEW_AW : STATEW_AWW;
    STATEW_AW :state_wnxt = aw_sh              ? STATEW_B  : STATEW_AW;
    STATEW_W  :state_wnxt = w_sh               ? STATEW_B  : STATEW_W;
    STATEW_B  :state_wnxt = b_sh               ? STATEW_ID : STATEW_B;
    default:  state_wnxt = STATEW_ID;
  endcase
    case(state_rr)
    STATER_ID: state_rnxt = ar_sh              ? STATER_R  :
                           dcache_axi_arvalid  ? STATER_AR : STATER_ID;
    STATER_AR: state_rnxt = ar_sh              ? STATER_R  : STATER_AR;
    STATER_R : state_rnxt = r_sh               ? STATER_ID : STATER_R; 
    default:  state_rnxt = STATER_ID;
  endcase
    case(state_r)
    STATE_IDLE: state_nxt = rsp_sh                      ? STATE_IDLE :
                            (req_sh & lsu_req_fencei)   ? STATE_SYNC :
                            req_sh & dcache_axi_nondaxi ? STATE_VALD :
                            dcache_axi_arvalid          ? STATE_BUSY :
                            dcache_axi_awvalid          ? STATE_BUSY : STATE_IDLE;
    STATE_VALD: state_nxt = rsp_sh                      ? STATE_IDLE : STATE_VALD;
    STATE_BUSY: state_nxt = rsp_sh                      ? STATE_IDLE :
                            dcache_axi_allvalid         ? STATE_VALD : STATE_BUSY;
    STATE_SYNC: state_nxt = count_over                  ? STATE_IDLE : STATE_SYNC;
    default:    state_nxt = STATE_IDLE;
  endcase
end

lieat_general_dffrd #(
  .DW(4),
  .DEFAULT(STATE_IDLE)
) dcache_state(clock,reset,state_nxt,state_r);
lieat_general_dffrd #(
  .DW(3),
  .DEFAULT(STATER_ID)
) dcache_stater(clock,reset,state_rnxt,state_rr);
lieat_general_dffrd #(
  .DW(5),
  .DEFAULT(STATEW_ID)
) dcache_statew(clock,reset,state_wnxt,state_wr);
// ================================================================================================================================================
// CONTROL
// ================================================================================================================================================
wire addr_inside = (lsu_req_addr[31:28] == 4'b0011) | (lsu_req_addr[31]);//0x80000000 and 0x30000000~0x3fffffff

wire hit = hit0 | hit1;
wire hit0 = addr_inside & (cache_tags_lru_vld[index][0][TAG_WIDTH+4:5] == tag);
wire hit1 = addr_inside & (cache_tags_lru_vld[index][1][TAG_WIDTH+4:5] == tag);

wire miss = addr_inside & (~hit);
wire lru_sel1  = ~lru_sel0;
wire lru_sel0  = (cache_tags_lru_vld[index][0][3:0] > cache_tags_lru_vld[index][1][3:0]);
wire lru_clean = (lru_sel0 & ~cache_tags_lru_vld[index][0][4]) | (lru_sel1 & ~cache_tags_lru_vld[index][1][4]);
wire lru_dirty = (lru_sel0 &  cache_tags_lru_vld[index][0][4]) | (lru_sel1 &  cache_tags_lru_vld[index][1][4]);

wire read_outaddr          = lsu_req_ren & ~addr_inside;
wire write_outaddr         = lsu_req_wen & ~addr_inside;

wire read_hit              = lsu_req_ren & hit;
wire read_miss             = lsu_req_ren & miss;
wire read_miss_need_renew  = read_miss  & lru_dirty;
wire write_hit             = lsu_req_wen & hit;
wire write_miss            = lsu_req_wen & miss;
wire write_miss_need_renew = write_miss & lru_dirty;
wire write_miss_nond_renew = write_miss & lru_clean;
wire write_miss_need_read  = write_miss & ((dcache_len == 3'b000) | (dcache_len == 3'b001));
wire write_miss_nond_read  = write_miss & (dcache_len == 3'b010);

wire dcache_axi_nondaxi    = write_hit | read_hit | (write_miss_nond_renew & write_miss_nond_read);
wire dcache_axi_allvalid = ((state_wr[STATEW_B_BIT] & b_sh) & (state_rr[STATER_ID_BIT])) |
                           ((state_rr[STATER_R_BIT] & r_sh) & (state_wr[STATEW_ID_BIT])) |
                           ((state_wr[STATEW_B_BIT] & b_sh) & (state_rr[STATER_R_BIT] & r_sh));
// ================================================================================================================================================
// CHANNEL: INTERACTIVE WITH LSU
// ================================================================================================================================================
assign lsu_rsp_valid = (dcache_axi_nondaxi & req_sh)                                       | 
                       (state_r[STATE_VALD_BIT])                                           | 
                       (dcache_axi_allvalid & ~state_r[STATE_SYNC_BIT])                    |
                       (state_r[STATE_SYNC_BIT] & count_over & (dcache_axi_bvalid | sync_clean));
assign lsu_req_ready = (state_r[STATE_IDLE_BIT]);
assign lsu_rsp_rdata = ({`XLEN{(lsu_req_flag == 3'b010)                     }} & sel_rdata               )               |
                       ({`XLEN{(lsu_req_flag == 3'b101) & (bytesel == 2'b00)}} & {16'b0,sel_rdata[15: 0]})               | 
                       ({`XLEN{(lsu_req_flag == 3'b101) & (bytesel == 2'b10)}} & {16'b0,sel_rdata[31:16]})               |
                       ({`XLEN{(lsu_req_flag == 3'b001) & (bytesel == 2'b00)}} & {{16{sel_rdata[15]}},sel_rdata[15: 0]}) | 
                       ({`XLEN{(lsu_req_flag == 3'b001) & (bytesel == 2'b10)}} & {{16{sel_rdata[31]}},sel_rdata[31:16]}) |
                       ({`XLEN{(lsu_req_flag == 3'b100) & (bytesel == 2'b00)}} & {24'b0,sel_rdata[ 7: 0]})               |
                       ({`XLEN{(lsu_req_flag == 3'b100) & (bytesel == 2'b01)}} & {24'b0,sel_rdata[15: 8]})               |
                       ({`XLEN{(lsu_req_flag == 3'b100) & (bytesel == 2'b10)}} & {24'b0,sel_rdata[23:16]})               |
                       ({`XLEN{(lsu_req_flag == 3'b100) & (bytesel == 2'b11)}} & {24'b0,sel_rdata[31:24]})               | 
                       ({`XLEN{(lsu_req_flag == 3'b000) & (bytesel == 2'b00)}} & {{24{sel_rdata[7]}},sel_rdata[ 7: 0]})  |
                       ({`XLEN{(lsu_req_flag == 3'b000) & (bytesel == 2'b01)}} & {{24{sel_rdata[15]}},sel_rdata[15: 8]}) |
                       ({`XLEN{(lsu_req_flag == 3'b000) & (bytesel == 2'b10)}} & {{24{sel_rdata[23]}},sel_rdata[23:16]}) |
                       ({`XLEN{(lsu_req_flag == 3'b000) & (bytesel == 2'b11)}} & {{24{sel_rdata[31]}},sel_rdata[31:24]}) ;
// ================================================================================================================================================
// CHANNEL: INTERACTIVE WITH DRAM
// ================================================================================================================================================
assign dcache_axi_arvalid = (req_sh | state_rr[STATER_AR_BIT]) & (read_outaddr | read_miss | write_miss_need_read);
assign dcache_axi_araddr  = read_outaddr          ? lsu_req_addr            : {lsu_req_addr[31:2],2'b0};
assign dcache_axi_arsize  = read_outaddr          ? dcache_len              : 3'b010;

assign dcache_axi_awvalid = (req_sh | state_wr[STATEW_AWW_BIT] | state_wr[STATEW_AW_BIT] | state_r[STATE_SYNC_BIT]) & (write_outaddr | read_miss_need_renew | write_miss_need_renew | sync_valid);
assign dcache_axi_awaddr  = write_outaddr         ? lsu_req_addr             :
                            read_miss_need_renew  ? {cache_tags_lru_vld[index][lru_sel1][TAG_WIDTH+4:5],index,2'b0} :
                            write_miss_need_renew ? {cache_tags_lru_vld[index][lru_sel1][TAG_WIDTH+4:5],index,2'b0} :
                            sync_valid            ? {cache_tags_lru_vld[count[6:1]][count[0]][TAG_WIDTH+4:5],count[6:1],2'b0} : 32'h0;
assign dcache_axi_awsize  = write_outaddr         ? dcache_len               : 
                            read_miss_need_renew  ? 3'b010                   : 
                            write_miss_need_renew ? 3'b010                   : 
                            sync_valid            ? 3'b010                   : 3'b011;
assign dcache_axi_wvalid  = (req_sh | state_wr[STATEW_AWW_BIT] | state_wr[STATEW_W_BIT] | state_r[STATE_SYNC_BIT]) & (write_outaddr | read_miss_need_renew | write_miss_need_renew | sync_valid);
assign dcache_axi_wdata   = ({64{(dcache_axi_awsize == 3'b000) & (dcache_axi_awaddr[2:0] == 3'b000)}} & {56'h0,{dcache_axi_wdata32[ 7: 0]}})       |
                            ({64{(dcache_axi_awsize == 3'b000) & (dcache_axi_awaddr[2:0] == 3'b001)}} & {48'h0,{dcache_axi_wdata32[ 7: 0]}, 8'h0}) |
                            ({64{(dcache_axi_awsize == 3'b000) & (dcache_axi_awaddr[2:0] == 3'b010)}} & {40'h0,{dcache_axi_wdata32[ 7: 0]},16'h0}) |
                            ({64{(dcache_axi_awsize == 3'b000) & (dcache_axi_awaddr[2:0] == 3'b011)}} & {32'h0,{dcache_axi_wdata32[ 7: 0]},24'h0}) |
                            ({64{(dcache_axi_awsize == 3'b000) & (dcache_axi_awaddr[2:0] == 3'b100)}} & {24'h0,{dcache_axi_wdata32[ 7: 0]},32'h0}) |
                            ({64{(dcache_axi_awsize == 3'b000) & (dcache_axi_awaddr[2:0] == 3'b101)}} & {16'h0,{dcache_axi_wdata32[ 7: 0]},40'h0}) |
                            ({64{(dcache_axi_awsize == 3'b000) & (dcache_axi_awaddr[2:0] == 3'b110)}} & { 8'h0,{dcache_axi_wdata32[ 7: 0]},48'h0}) |
                            ({64{(dcache_axi_awsize == 3'b000) & (dcache_axi_awaddr[2:0] == 3'b111)}} & {      {dcache_axi_wdata32[ 7: 0]},56'h0}) |
                            ({64{(dcache_axi_awsize == 3'b001) & (dcache_axi_awaddr[2:0] == 3'b000)}} & {48'h0,{dcache_axi_wdata32[15: 0]}      }) |
                            ({64{(dcache_axi_awsize == 3'b001) & (dcache_axi_awaddr[2:0] == 3'b010)}} & {32'h0,{dcache_axi_wdata32[15: 0]},16'h0}) |
                            ({64{(dcache_axi_awsize == 3'b001) & (dcache_axi_awaddr[2:0] == 3'b100)}} & {16'h0,{dcache_axi_wdata32[15: 0]},32'h0}) |
                            ({64{(dcache_axi_awsize == 3'b001) & (dcache_axi_awaddr[2:0] == 3'b110)}} & {      {dcache_axi_wdata32[15: 0]},48'h0}) |
                            ({64{(dcache_axi_awsize == 3'b010) & (dcache_axi_awaddr[2:0] == 3'b000)}} & {32'h0,{dcache_axi_wdata32[31: 0]}      }) |
                            ({64{(dcache_axi_awsize == 3'b010) & (dcache_axi_awaddr[2:0] == 3'b100)}} & {      {dcache_axi_wdata32[31: 0]},32'h0}) ;
assign dcache_axi_wstrb   = ({8{(dcache_axi_awsize == 3'b000) & (dcache_axi_awaddr[2:0] == 3'b000)}} & 8'b00000001) |
                            ({8{(dcache_axi_awsize == 3'b000) & (dcache_axi_awaddr[2:0] == 3'b001)}} & 8'b00000010) |
                            ({8{(dcache_axi_awsize == 3'b000) & (dcache_axi_awaddr[2:0] == 3'b010)}} & 8'b00000100) |
                            ({8{(dcache_axi_awsize == 3'b000) & (dcache_axi_awaddr[2:0] == 3'b011)}} & 8'b00001000) |
                            ({8{(dcache_axi_awsize == 3'b000) & (dcache_axi_awaddr[2:0] == 3'b100)}} & 8'b00010000) |
                            ({8{(dcache_axi_awsize == 3'b000) & (dcache_axi_awaddr[2:0] == 3'b101)}} & 8'b00100000) |
                            ({8{(dcache_axi_awsize == 3'b000) & (dcache_axi_awaddr[2:0] == 3'b110)}} & 8'b01000000) |
                            ({8{(dcache_axi_awsize == 3'b000) & (dcache_axi_awaddr[2:0] == 3'b111)}} & 8'b10000000) |
                            ({8{(dcache_axi_awsize == 3'b001) & (dcache_axi_awaddr[2:0] == 3'b000)}} & 8'b00000011) |
                            ({8{(dcache_axi_awsize == 3'b001) & (dcache_axi_awaddr[2:0] == 3'b010)}} & 8'b00001100) |
                            ({8{(dcache_axi_awsize == 3'b001) & (dcache_axi_awaddr[2:0] == 3'b100)}} & 8'b00110000) |
                            ({8{(dcache_axi_awsize == 3'b001) & (dcache_axi_awaddr[2:0] == 3'b110)}} & 8'b11000000) |
                            ({8{(dcache_axi_awsize == 3'b010) & (dcache_axi_awaddr[2:0] == 3'b000)}} & 8'b00001111) |
                            ({8{(dcache_axi_awsize == 3'b010) & (dcache_axi_awaddr[2:0] == 3'b100)}} & 8'b11110000) ;
assign dcache_axi_rready  = lsu_rsp_ready;
assign dcache_axi_bready  = lsu_rsp_ready;
// ================================================================================================================================================
// CACHE MODULE
// ================================================================================================================================================
wire [TAG_WIDTH-1:0] tag      = lsu_req_addr[`XLEN-1:OFFSET_LEN+INDEX_LEN];
wire [INDEX_LEN-1:0] index    = lsu_req_addr[OFFSET_LEN+INDEX_LEN-1:OFFSET_LEN];

wire [1:0] bytesel_nxt = lsu_req_addr[OFFSET_LEN-1:0];
wire [1:0] bytesel_r;
wire [1:0] bytesel = req_sh ? bytesel_nxt : bytesel_r;
lieat_general_dfflr #(2) bytesel_dff(clock,reset,req_sh,bytesel_nxt,bytesel_r);

reg [TAG_WIDTH+4:0] cache_tags_lru_vld  [CACHE_SIZE-1:0][CACHE_WAY-1:0];//TAG_WIDTH + VALID_BIT1 + LRU_BIT 4

always@(posedge clock or posedge reset) begin
  if(reset)begin
    for(int i = 0;i < CACHE_SIZE; i = i + 1)begin
      for(int j = 0; j < CACHE_WAY; j = j + 1)begin
        cache_tags_lru_vld[i][j] <= 0;
      end
    end
  end
  else if(addr_inside) begin
    if(read_hit) begin
        cache_tags_lru_vld[index][hit1][3:0] <= 4'b0;
        cache_tags_lru_vld[index][hit0][3:0] <= (cache_tags_lru_vld[index][hit0][3:0] == 4'b1111) ? 4'b1111 : (cache_tags_lru_vld[index][hit0][3:0] + 1);
    end
    else if(read_miss & dcache_axi_rvalid) begin
      cache_tags_lru_vld[index][lru_sel1] <= {tag,5'b00000};
      cache_tags_lru_vld[index][lru_sel0][3:0] <= cache_tags_lru_vld[index][lru_sel0][3:0]+1;
    end
    else if(write_hit) begin
        cache_tags_lru_vld[index][hit1] <= {tag,5'b10000};
        cache_tags_lru_vld[index][hit0][3:0] <= cache_tags_lru_vld[index][1][3:0]+1;
    end
    else if(write_miss) begin
      if((dcache_len == 3'b010)) begin
          cache_tags_lru_vld[index][lru_sel1] <= {tag,5'b10000};
          cache_tags_lru_vld[index][lru_sel0][3:0] <= cache_tags_lru_vld[index][1][3:0]+1;
        end
      if(dcache_axi_rvalid) begin
        cache_tags_lru_vld[index][lru_sel1] <= {tag,5'b10000};
        cache_tags_lru_vld[index][lru_sel0][3:0] <= cache_tags_lru_vld[index][1][3:0]+1;
      end
    end
  end
end

wire [63:0] read_miss_sram_data = lru_sel1 ? {dcache_axi_rdata32,dcache_sram_rdata[31:0]} : {dcache_sram_rdata[63:32],dcache_axi_rdata32};
wire [63:0] write_hit_sram_data = 
({64{((dcache_len == 3'b000) & (bytesel == 2'b00) & (hit0))}} & {dcache_sram_rdata[63: 8],lsu_req_wdata[7:0]                        }) |
({64{((dcache_len == 3'b000) & (bytesel == 2'b01) & (hit0))}} & {dcache_sram_rdata[63:16],lsu_req_wdata[7:0],dcache_sram_rdata[ 7:0]}) |
({64{((dcache_len == 3'b000) & (bytesel == 2'b10) & (hit0))}} & {dcache_sram_rdata[63:24],lsu_req_wdata[7:0],dcache_sram_rdata[15:0]}) |
({64{((dcache_len == 3'b000) & (bytesel == 2'b11) & (hit0))}} & {dcache_sram_rdata[63:32],lsu_req_wdata[7:0],dcache_sram_rdata[23:0]}) |
({64{((dcache_len == 3'b000) & (bytesel == 2'b00) & (hit1))}} & {dcache_sram_rdata[63:40],lsu_req_wdata[7:0],dcache_sram_rdata[31:0]}) |
({64{((dcache_len == 3'b000) & (bytesel == 2'b01) & (hit1))}} & {dcache_sram_rdata[63:48],lsu_req_wdata[7:0],dcache_sram_rdata[39:0]}) |
({64{((dcache_len == 3'b000) & (bytesel == 2'b10) & (hit1))}} & {dcache_sram_rdata[63:56],lsu_req_wdata[7:0],dcache_sram_rdata[47:0]}) |
({64{((dcache_len == 3'b000) & (bytesel == 2'b11) & (hit1))}} & {                         lsu_req_wdata[7:0],dcache_sram_rdata[55:0]}) |
({64{((dcache_len == 3'b001) & (bytesel == 2'b00) & (hit0))}} & {dcache_sram_rdata[63:16],lsu_req_wdata[15:0]                        }) |
({64{((dcache_len == 3'b001) & (bytesel == 2'b10) & (hit0))}} & {dcache_sram_rdata[63:32],lsu_req_wdata[15:0],dcache_sram_rdata[15:0]}) |
({64{((dcache_len == 3'b001) & (bytesel == 2'b00) & (hit1))}} & {dcache_sram_rdata[63:48],lsu_req_wdata[15:0],dcache_sram_rdata[31:0]}) |
({64{((dcache_len == 3'b001) & (bytesel == 2'b10) & (hit1))}} & {                         lsu_req_wdata[15:0],dcache_sram_rdata[47:0]}) |
({64{((dcache_len == 3'b010) & (bytesel == 2'b00) & (hit0))}} & {dcache_sram_rdata[63:32],lsu_req_wdata[31:0]                        }) |
({64{((dcache_len == 3'b010) & (bytesel == 2'b00) & (hit1))}} & {                         lsu_req_wdata[31:0],dcache_sram_rdata[31:0]}) ;
wire [63:0] write_miss_sram_data = 
({64{((dcache_len == 3'b000) & (bytesel == 2'b00) & (lru_sel0))}} & {dcache_sram_rdata[63:32],dcache_axi_rdata32[31: 8],lsu_req_wdata[7:0]                          }) |
({64{((dcache_len == 3'b000) & (bytesel == 2'b01) & (lru_sel0))}} & {dcache_sram_rdata[63:32],dcache_axi_rdata32[31:16],lsu_req_wdata[7:0],dcache_axi_rdata32[ 7:0] }) |
({64{((dcache_len == 3'b000) & (bytesel == 2'b10) & (lru_sel0))}} & {dcache_sram_rdata[63:32],dcache_axi_rdata32[31:24],lsu_req_wdata[7:0],dcache_axi_rdata32[15:0] }) |
({64{((dcache_len == 3'b000) & (bytesel == 2'b11) & (lru_sel0))}} & {dcache_sram_rdata[63:32],                          lsu_req_wdata[7:0],dcache_axi_rdata32[23:0] }) |
({64{((dcache_len == 3'b000) & (bytesel == 2'b00) & (lru_sel1))}} & {dcache_axi_rdata32[31: 8],lsu_req_wdata[7:0],                         dcache_sram_rdata[31: 0] }) |
({64{((dcache_len == 3'b000) & (bytesel == 2'b01) & (lru_sel1))}} & {dcache_axi_rdata32[31:16],lsu_req_wdata[7:0],dcache_axi_rdata32[7:0] ,dcache_sram_rdata[31: 0] }) |
({64{((dcache_len == 3'b000) & (bytesel == 2'b10) & (lru_sel1))}} & {dcache_axi_rdata32[31:24],lsu_req_wdata[7:0],dcache_axi_rdata32[15:0],dcache_sram_rdata[31: 0] }) |
({64{((dcache_len == 3'b000) & (bytesel == 2'b11) & (lru_sel1))}} & {                          lsu_req_wdata[7:0],dcache_axi_rdata32[23:0],dcache_sram_rdata[31: 0] }) |
({64{((dcache_len == 3'b001) & (bytesel == 2'b00) & (lru_sel0))}} & {dcache_sram_rdata[63:32],dcache_axi_rdata32[31:16],lsu_req_wdata[15:0]                         }) |
({64{((dcache_len == 3'b001) & (bytesel == 2'b10) & (lru_sel0))}} & {dcache_sram_rdata[63:32],                          lsu_req_wdata[15:0],dcache_axi_rdata32[15:0]}) |
({64{((dcache_len == 3'b001) & (bytesel == 2'b00) & (lru_sel1))}} & {                         dcache_axi_rdata32[15: 0],lsu_req_wdata[15:0],dcache_sram_rdata[31:0] }) |
({64{((dcache_len == 3'b001) & (bytesel == 2'b10) & (lru_sel1))}} & {                         lsu_req_wdata[15: 0],dcache_axi_rdata32[15:0],dcache_sram_rdata[31:0] }) |
({64{((dcache_len == 3'b010) & (bytesel == 2'b00) & (lru_sel0))}} & {dcache_sram_rdata[63:32],lsu_req_wdata[31:0]})                                                    |
({64{((dcache_len == 3'b010) & (bytesel == 2'b00) & (lru_sel1))}} & {lsu_req_wdata[31:0],dcache_sram_rdata[31:0]}) ;

wire dcache_sram_cen = 1'b1;
wire dcache_sram_wen = (read_miss & dcache_axi_rvalid) |
                       (write_hit)                     |
                       (write_miss & (dcache_axi_rvalid | (dcache_len == 3'b010)));
wire [ 5:0] dcache_sram_addr= state_r[STATE_SYNC_BIT] ? count[6:1] : index;
wire [63:0] dcache_sram_data= (read_miss & dcache_axi_rvalid) ? read_miss_sram_data :
                              (write_hit)                     ? write_hit_sram_data :
                              (write_miss)                    ? write_miss_sram_data : 64'h0;
wire [63:0] dcache_sram_rdata;
lieat_general_64x64_sram dcache_sram(clock,reset,dcache_sram_cen,dcache_sram_wen,dcache_sram_addr,dcache_sram_data,dcache_sram_rdata);

endmodule
module lieat_exu_div(
  input              clock,
  input              reset,
  input              div_i_valid,
  output             div_i_ready,
  input              div_i_signed,
  input  [`XLEN-1:0] div_i_dividend,
  input  [`XLEN-1:0] div_i_divisor,

  output             div_o_valid,
  input              div_o_ready,
  output [`XLEN-1:0] div_o_quot,
  output [`XLEN-1:0] div_o_rem,
  output             div_o_divisor0
);
wire unused_ok = &{nrdnt_rem_nxt[2:0],nrdnt_rem_plus_d_nxt[2:0],lzc_diff_nxt[4:0],dividend_lzc[5],lzc_diff[5]};
//state: idle pre0 pre1 iter pos0 pos1
localparam QUOT_NEG_2 = 0;
localparam QUOT_NEG_1 = 1;
localparam QUOT_ZERO  = 2;
localparam QUOT_POS_1 = 3;
localparam QUOT_POS_2 = 4;
localparam QUOT_ONEHOT_ZERO  = 5'b0_0100;
localparam QUOT_ONEHOT_POS_1 = 5'b0_1000;
localparam QUOT_ONEHOT_POS_2 = 5'b1_0000;

localparam STATE_IDLE_BIT = 0;
localparam STATE_PRE0_BIT = 1;
localparam STATE_PRE1_BIT = 2;
localparam STATE_ITER_BIT = 3;
localparam STATE_POS0_BIT = 4;
localparam STATE_POS1_BIT = 5;

localparam STATE_IDLE = 6'b000001;//quot_neg rem_neg
localparam STATE_PRE0 = 6'b000010;
localparam STATE_PRE1 = 6'b000100;
localparam STATE_ITER = 6'b001000;
localparam STATE_POS0 = 6'b010000;
localparam STATE_POS1 = 6'b100000;
// ================================================================================================================================================
// STATE Ctrl Logic
// ================================================================================================================================================
wire div_i_sh = div_i_valid & div_i_ready;
wire div_o_sh = div_o_valid & div_o_ready;
reg [5:0] state_r;
reg [5:0] state_nxt;

always @(*) begin
  case(state_r)
    STATE_IDLE:state_nxt = div_i_sh   ? STATE_PRE0 : STATE_IDLE;
    STATE_PRE0:state_nxt = STATE_PRE1;
    STATE_PRE1:state_nxt = skip_iter  ? STATE_POS0 : STATE_ITER;
    STATE_ITER:state_nxt = final_iter ? STATE_POS0 : STATE_ITER;
    STATE_POS0:state_nxt = STATE_POS1;
    STATE_POS1:state_nxt = div_o_sh   ? STATE_IDLE : STATE_POS1;
    default:state_nxt = STATE_IDLE;
  endcase
end
lieat_general_dffrd #(
  .DW(6),
  .DEFAULT(STATE_IDLE)
) div_state(clock,reset,state_nxt,state_r);
// ================================================================================================================================================
// SPECIAL CASE
// ================================================================================================================================================
wire [`LZC_WIDTH:0] lzc_diff     = {1'b0, divisor_lzc[0 +: `LZC_WIDTH]}     - {1'b0, dividend_lzc[0 +: `LZC_WIDTH]};
wire [`LZC_WIDTH:0] lzc_diff_nxt = {1'b0, divisor_lzc_nxt[0 +: `LZC_WIDTH]} - {1'b0, dividend_lzc_nxt[0 +: `LZC_WIDTH]};

wire divisor_is_zero = divisor_lzc[`LZC_WIDTH];//valid from PRE1
wire divisor_is_one  = (divisor_lzc_nxt[`LZC_WIDTH-1:0] == 5'b11111);//valid when POS0

wire dividend_too_big;//dividend is too big and divisor is too small
wire dividend_too_big_en  = state_r[STATE_PRE0_BIT];
wire dividend_too_big_nxt = divisor_is_one & dividend_abs[`XLEN-1];
lieat_general_dfflr #(1) dividend_too_big_dff(clock,reset,dividend_too_big_en,dividend_too_big_nxt,dividend_too_big);

wire dividend_too_small;//dividend < divisor
wire dividend_too_small_en = state_r[STATE_PRE0_BIT];
wire dividend_too_small_nxt = lzc_diff_nxt[`LZC_WIDTH] | dividend_lzc_nxt[`LZC_WIDTH];
lieat_general_dfflr #(1) dividend_too_small_dff(clock,reset,dividend_too_small_en,dividend_too_small_nxt,dividend_too_small);
wire skip_iter = dividend_too_small | divisor_is_zero | dividend_too_big;//no iter
// ================================================================================================================================================
// OUTPUT SIGN
// ================================================================================================================================================
wire rem_neg;
wire rem_neg_nxt  = dividend_neg;
wire rem_neg_ena  = div_i_sh;
wire quot_neg;
wire quot_neg_nxt = state_r[STATE_IDLE_BIT] ? (dividend_neg ^ divisor_neg) : 1'b0;
wire quot_neg_ena = div_i_sh | (state_r[STATE_PRE1_BIT] & divisor_is_zero);
lieat_general_dfflr #(1) quot_neg_dff(clock,reset,quot_neg_ena,quot_neg_nxt,quot_neg);
lieat_general_dfflr #(1) rem_neg_dff(clock,reset,rem_neg_ena,rem_neg_nxt,rem_neg);
// ================================================================================================================================================
// ABS
// ================================================================================================================================================
wire dividend_neg = div_i_signed & div_i_dividend[`XLEN-1];
wire divisor_neg  = div_i_signed & div_i_divisor[`XLEN-1];
wire [`XLEN-1:0] inv1 = -(state_r[STATE_IDLE_BIT] ? div_i_dividend : iter_quot);          //dividend or iter_quot
wire [`XLEN-1:0] inv2 = -(state_r[STATE_IDLE_BIT] ? div_i_divisor  : iter_quot_minus_1);  //divisor  or iter_quot_minus_1

wire [`XLEN-1:0] dividend_abs_pre = dividend_neg ? inv1 : div_i_dividend;
wire [`XLEN-1:0] divisor_abs_pre  = divisor_neg  ? inv2 : div_i_divisor;
wire [`XLEN-1:0] normalized_dividend = dividend_abs[`XLEN-1:0] << dividend_lzc_nxt[`LZC_WIDTH-1:0];//reg state:PRE0
wire [`XLEN-1:0] normalized_divisor  = divisor_abs[`XLEN-1:0]  << divisor_lzc_nxt[`LZC_WIDTH-1:0];//reg state:PRE0

wire [`XLEN:0] divisor_abs;
wire [`XLEN:0] dividend_abs;
wire           abs_ena          = div_i_sh | state_r[STATE_PRE0_BIT] | state_r[STATE_POS0_BIT];
wire [`XLEN:0] divisor_abs_nxt  = ({(`XLEN+1){state_r[STATE_IDLE_BIT]}} & {1'b0,divisor_abs_pre    }) |
                                  ({(`XLEN+1){state_r[STATE_PRE0_BIT]}} & {1'b0,normalized_divisor }) |
                                  ({(`XLEN+1){state_r[STATE_POS0_BIT]}} & nrdnt_rem_plus_d_nxt[3 +: `XLEN+1]);
wire [`XLEN:0] dividend_abs_nxt = ({(`XLEN+1){state_r[STATE_IDLE_BIT]}} & {1'b0,dividend_abs_pre   }) |
                                  ({(`XLEN+1){state_r[STATE_PRE0_BIT]}} & {1'b0,normalized_dividend}) |
                                  ({(`XLEN+1){state_r[STATE_POS0_BIT]}} & nrdnt_rem_nxt[3 +: `XLEN+1]);
lieat_general_dfflr #(`XLEN+1) dividend_abs_dff(clock,reset,abs_ena,dividend_abs_nxt,dividend_abs);
lieat_general_dfflr #(`XLEN+1) divisor_abs_dff(clock,reset,abs_ena,divisor_abs_nxt,divisor_abs);
// ================================================================================================================================================
// LZC and Normalize
// ================================================================================================================================================
wire [`LZC_WIDTH:0] divisor_lzc;
wire [`LZC_WIDTH:0] dividend_lzc;
wire [`LZC_WIDTH:0] divisor_lzc_nxt;
wire [`LZC_WIDTH:0] dividend_lzc_nxt;

lieat_general_lzc lzc_dividend (
	.in_i(dividend_abs[`XLEN-1:0]),
	.cnt_o(dividend_lzc_nxt[`LZC_WIDTH-1:0]),
	.empty_o(dividend_lzc_nxt[`LZC_WIDTH])
);

lieat_general_lzc lzc_divisor (
	.in_i(divisor_abs[`XLEN-1:0]),
	.cnt_o(divisor_lzc_nxt[`LZC_WIDTH-1:0]),
	.empty_o(divisor_lzc_nxt[`LZC_WIDTH])
);

wire lzc_ena               = state_r[STATE_PRE0_BIT];
lieat_general_dfflr #(`LZC_WIDTH+1) dividend_lzc_dff(clock,reset,lzc_ena,dividend_lzc_nxt,dividend_lzc);
lieat_general_dfflr #(`LZC_WIDTH+1) divisor_lzc_dff(clock,reset,lzc_ena,divisor_lzc_nxt,divisor_lzc);
// ================================================================================================================================================
// SHIFTER
// ================================================================================================================================================
wire [`LZC_WIDTH-1:0] shifter_num = state_r[STATE_PRE1_BIT] ? dividend_lzc[`LZC_WIDTH-1:0] : ((dividend_too_small | divisor_is_zero) ? {(`LZC_WIDTH){1'b0}} : divisor_lzc[`LZC_WIDTH-1:0]);
wire [`XLEN-1:0]      shifter_din = state_r[STATE_PRE1_BIT] ? dividend_abs[`XLEN-1:0] : pre_shifted_rem[`XLEN-1:0];
wire                  shifter_extend_msb = state_r[STATE_POS1_BIT] & rem_neg & pre_shifted_rem[`XLEN];

wire [`XLEN-1:0] shifter_res_s0 = shifter_num[0] ? {{(1 ){shifter_extend_msb}},shifter_din[`XLEN-1:1]} : shifter_din;
wire [`XLEN-1:0] shifter_res_s1 = shifter_num[1] ? {{(2 ){shifter_extend_msb}},shifter_res_s0[`XLEN-1:2]} : shifter_res_s0;
wire [`XLEN-1:0] shifter_res_s2 = shifter_num[2] ? {{(4 ){shifter_extend_msb}},shifter_res_s1[`XLEN-1:4]} : shifter_res_s1;
wire [`XLEN-1:0] shifter_res_s3 = shifter_num[3] ? {{(8 ){shifter_extend_msb}},shifter_res_s2[`XLEN-1:8]} : shifter_res_s2;
wire [`XLEN-1:0] shifter_res    = shifter_num[4] ? {{(16){shifter_extend_msb}},shifter_res_s3[`XLEN-1:16]} : shifter_res_s3;
// ================================================================================================================================================
// Choose the parameters for CMP, according to the value of the normalized_d[(WIDTH - 2) -: 3]
// ================================================================================================================================================
wire [4:0] qds_para_neg1;
wire [2:0] qds_para_neg0;
wire [1:0] qds_para_pos1;
wire [4:0] qds_para_pos2;
wire       special_divisor;
wire [4:0] qds_para_neg1_nxt = 
  ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b000}} & 5'b0_1101)  // 000: m[-1] = -13, -m[-1] = +13 = 00_1101 -> ext(-m[-1]) = 00_11010
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b001}} & 5'b0_1111)  // 001: m[-1] = -15, -m[-1] = +15 = 00_1111 -> ext(-m[-1]) = 00_11110
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b010}} & 5'b1_0000)  // 010: m[-1] = -16, -m[-1] = +16 = 01_0000 -> ext(-m[-1]) = 01_00000
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b011}} & 5'b1_0010)  // 011: m[-1] = -17, -m[-1] = +17 = 01_0001 -> ext(-m[-1]) = 01_00010
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b100}} & 5'b1_0011)  // 100: m[-1] = -19, -m[-1] = +19 = 01_0011 -> ext(-m[-1]) = 01_00110
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b101}} & 5'b1_0100)  // 101: m[-1] = -20, -m[-1] = +20 = 01_0100 -> ext(-m[-1]) = 01_01000
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b110}} & 5'b1_0110)  // 110: m[-1] = -22, -m[-1] = +22 = 01_0110 -> ext(-m[-1]) = 01_01100
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b111}} & 5'b1_1000); // 111: m[-1] = -24, -m[-1] = +24 = 01_1000 -> ext(-m[-1]) = 01_10000
wire [2:0] qds_para_neg0_nxt = 
  ({(3){divisor_abs[(`XLEN-2) -: 3] == 3'b000}} & 3'b010)  // 000: m[-0] = -4, -m[-0] = +4 = 000_0100
| ({(3){divisor_abs[(`XLEN-2) -: 3] == 3'b001}} & 3'b011)  // 001: m[-0] = -6, -m[-0] = +6 = 000_0110
| ({(3){divisor_abs[(`XLEN-2) -: 3] == 3'b010}} & 3'b011)  // 010: m[-0] = -6, -m[-0] = +6 = 000_0110
| ({(3){divisor_abs[(`XLEN-2) -: 3] == 3'b011}} & 3'b011)  // 011: m[-0] = -6, -m[-0] = +6 = 000_0110
| ({(3){divisor_abs[(`XLEN-2) -: 3] == 3'b100}} & 3'b011)  // 100: m[-0] = -6, -m[-0] = +6 = 000_0110
| ({(3){divisor_abs[(`XLEN-2) -: 3] == 3'b101}} & 3'b100)  // 101: m[-0] = -8, -m[-0] = +8 = 000_1000
| ({(3){divisor_abs[(`XLEN-2) -: 3] == 3'b110}} & 3'b100)  // 110: m[-0] = -8, -m[-0] = +8 = 000_1000
| ({(3){divisor_abs[(`XLEN-2) -: 3] == 3'b111}} & 3'b100); // 111: m[-0] = -8, -m[-0] = +8 = 000_1000
wire [1:0] qds_para_pos1_nxt = 
  ({(2){divisor_abs[(`XLEN-2) -: 3] == 3'b000}} & 2'b10)  // 000: m[+1] = +4, -m[+1] = -4 = 111_1100
| ({(2){divisor_abs[(`XLEN-2) -: 3] == 3'b001}} & 2'b10)  // 001: m[+1] = +4, -m[+1] = -4 = 111_1100
| ({(2){divisor_abs[(`XLEN-2) -: 3] == 3'b010}} & 2'b10)  // 010: m[+1] = +4, -m[+1] = -4 = 111_1100
| ({(2){divisor_abs[(`XLEN-2) -: 3] == 3'b011}} & 2'b10)  // 011: m[+1] = +4, -m[+1] = -4 = 111_1100
| ({(2){divisor_abs[(`XLEN-2) -: 3] == 3'b100}} & 2'b01)  // 100: m[+1] = +6, -m[+1] = -6 = 111_1010
| ({(2){divisor_abs[(`XLEN-2) -: 3] == 3'b101}} & 2'b01)  // 101: m[+1] = +6, -m[+1] = -6 = 111_1010
| ({(2){divisor_abs[(`XLEN-2) -: 3] == 3'b110}} & 2'b01)  // 110: m[+1] = +6, -m[+1] = -6 = 111_1010
| ({(2){divisor_abs[(`XLEN-2) -: 3] == 3'b111}} & 2'b00); // 111: m[+1] = +8, -m[+1] = -8 = 111_1000
wire [4:0] qds_para_pos2_nxt = 
  ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b000}} & 5'b1_0100)  // 000: m[+2] = +12, -m[+2] = -12 = 11_0100 -> ext(-m[+2]) = 11_01000
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b001}} & 5'b1_0010)  // 001: m[+2] = +14, -m[+2] = -14 = 11_0010 -> ext(-m[+2]) = 11_00100
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b010}} & 5'b1_0001)  // 010: m[+2] = +15, -m[+2] = -15 = 11_0001 -> ext(-m[+2]) = 11_00010
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b011}} & 5'b1_0000)  // 011: m[+2] = +16, -m[+2] = -16 = 11_0000 -> ext(-m[+2]) = 11_00000
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b100}} & 5'b0_1110)  // 100: m[+2] = +18, -m[+2] = -18 = 10_1110 -> ext(-m[+2]) = 10_11100
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b101}} & 5'b0_1100)  // 101: m[+2] = +20, -m[+2] = -20 = 10_1100 -> ext(-m[+2]) = 10_11000
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b110}} & 5'b0_1010)  // 110: m[+2] = +22, -m[+2] = -22 = 10_1010 -> ext(-m[+2]) = 10_10100
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b111}} & 5'b0_1010); // 111: m[+2] = +22, -m[+2] = -22 = 10_1010 -> ext(-m[+2]) = 10_10100
wire special_divisor_nxt = (divisor_abs[(`XLEN-2) -: 3] == 3'b000) | (divisor_abs[(`XLEN-2) -: 3] == 3'b100);
wire qds_para_en = state_r[STATE_PRE1_BIT];
lieat_general_dfflr #(5) qds_para_neg1_dff(clock,reset,qds_para_en,qds_para_neg1_nxt,qds_para_neg1);
lieat_general_dfflr #(3) qds_para_neg0_dff(clock,reset,qds_para_en,qds_para_neg0_nxt,qds_para_neg0);
lieat_general_dfflr #(2) qds_para_pos1_dff(clock,reset,qds_para_en,qds_para_pos1_nxt,qds_para_pos1);
lieat_general_dfflr #(5) qds_para_pos2_dff(clock,reset,qds_para_en,qds_para_pos2_nxt,qds_para_pos2);
lieat_general_dfflr #(1) special_divisor_dff(clock,reset,qds_para_en,special_divisor_nxt,special_divisor);
// ================================================================================================================================================
// Get iter_num, and some initial value for different regs.
// ================================================================================================================================================
wire r_shift = lzc_diff[0];
wire [`XLEN+3:0] rem_init_value = {3'b0,r_shift ? {1'b0, dividend_abs[`XLEN-1:0]} : {dividend_abs[`XLEN-1:0], 1'b0}};
wire [`XLEN+3:0] rem_sum_init_value  = (dividend_too_small | divisor_is_zero) ? {1'b0, shifter_res, 3'b0} : 
                                        dividend_too_big                      ? {(`XLEN+4){1'b0}}         : rem_init_value;
wire [`XLEN+3:0] rem_carry_init_value = {`XLEN+4{1'b0}};
wire [`LZC_WIDTH-1:0] pre_rem_trunc_14 = {1'b0,rem_init_value[`XLEN -: `LZC_WIDTH-1]};
wire [`LZC_WIDTH-1:0] pre_m_pos_1 = 
  ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b000}} & 5'b0_0100)  // 000: m[+1] =  +4 = 0_0100;
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b001}} & 5'b0_0100)  // 001: m[+1] =  +4 = 0_0100;
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b010}} & 5'b0_0100)  // 010: m[+1] =  +4 = 0_0100;
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b011}} & 5'b0_0100)  // 011: m[+1] =  +4 = 0_0100;
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b100}} & 5'b0_0110)  // 100: m[+1] =  +6 = 0_0110;
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b101}} & 5'b0_0110)  // 101: m[+1] =  +6 = 0_0110;
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b110}} & 5'b0_0110)  // 110: m[+1] =  +6 = 0_0110;
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b111}} & 5'b0_1000); // 111: m[+1] =  +8 = 0_1000;
wire [`LZC_WIDTH-1:0] pre_m_pos_2 = 
  ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b000}} & 5'b0_1100)  // 000: m[+2] = +12 = 0_1100;
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b001}} & 5'b0_1110)  // 001: m[+2] = +14 = 0_1110;
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b010}} & 5'b0_1111)  // 010: m[+2] = +15 = 0_1111;
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b011}} & 5'b1_0000)  // 011: m[+2] = +16 = 1_0000;
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b100}} & 5'b1_0010)  // 100: m[+2] = +18 = 1_0010;
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b101}} & 5'b1_0100)  // 101: m[+2] = +20 = 1_0100;
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b110}} & 5'b1_0110)  // 110: m[+2] = +22 = 1_0110;
| ({(5){divisor_abs[(`XLEN-2) -: 3] == 3'b111}} & 5'b1_0110); // 111: m[+2] = +22 = 1_0110;
wire [1:0] pre_cmp_res = {(pre_rem_trunc_14 >= pre_m_pos_1), (pre_rem_trunc_14 >= pre_m_pos_2)};
wire [`LZC_WIDTH-1:0] prev_quot_digit;
wire [`LZC_WIDTH-1:0] prev_quot_digit_init_value = pre_cmp_res[0] ? QUOT_ONEHOT_POS_2 : pre_cmp_res[1] ? QUOT_ONEHOT_POS_1 : QUOT_ONEHOT_ZERO;
wire prev_quot_digit_en = state_r[STATE_PRE1_BIT] | state_r[STATE_ITER_BIT];
wire [`LZC_WIDTH-1:0] prev_quot_digit_nxt= state_r[STATE_PRE1_BIT] ? prev_quot_digit_init_value : quot_digit_iter_end;
lieat_general_dfflr #(5) prev_quot_digit_dff(clock,reset,prev_quot_digit_en,prev_quot_digit_nxt,prev_quot_digit);
// ================================================================================================================================================
//  ITER
// ================================================================================================================================================
wire [`XLEN-1:0] iter_cycle_quot_nxt = 
  ({(`XLEN){prev_quot_digit[QUOT_POS_2]}} & {iter_quot			  [`XLEN-3:0], 2'b10})
| ({(`XLEN){prev_quot_digit[QUOT_POS_1]}} & {iter_quot			  [`XLEN-3:0], 2'b01})
| ({(`XLEN){prev_quot_digit[QUOT_ZERO ]}} & {iter_quot			  [`XLEN-3:0], 2'b00})
| ({(`XLEN){prev_quot_digit[QUOT_NEG_1]}} & {iter_quot_minus_1[`XLEN-3:0], 2'b11})
| ({(`XLEN){prev_quot_digit[QUOT_NEG_2]}} & {iter_quot_minus_1[`XLEN-3:0], 2'b10});
wire [`XLEN-1:0] iter_cycle_quot_minus_1_nxt = 
  ({(`XLEN){prev_quot_digit[QUOT_POS_2]}} & {iter_quot			  [`XLEN-3:0], 2'b01})
| ({(`XLEN){prev_quot_digit[QUOT_POS_1]}} & {iter_quot			  [`XLEN-3:0], 2'b00})
| ({(`XLEN){prev_quot_digit[QUOT_ZERO ]}} & {iter_quot_minus_1[`XLEN-3:0], 2'b11})
| ({(`XLEN){prev_quot_digit[QUOT_NEG_1]}} & {iter_quot_minus_1[`XLEN-3:0], 2'b10})
| ({(`XLEN){prev_quot_digit[QUOT_NEG_2]}} & {iter_quot_minus_1[`XLEN-3:0], 2'b01});

wire [`XLEN-1:0] iter_quot;
wire [`XLEN-1:0] iter_quot_minus_1;
wire iter_quot_en = state_r[STATE_PRE1_BIT] | state_r[STATE_ITER_BIT] | state_r[STATE_POS0_BIT];
wire [`XLEN-1:0] iter_quot_nxt =         state_r[STATE_PRE1_BIT] ? (divisor_is_zero ? {(`XLEN){1'b1}} : dividend_too_big ? dividend_abs[`XLEN-1:0] : {(`XLEN){1'b0}}) :
                                         state_r[STATE_ITER_BIT] ? iter_cycle_quot_nxt        : (quot_neg ? inv1 : iter_quot);
wire [`XLEN-1:0] iter_quot_minus_1_nxt = state_r[STATE_PRE1_BIT] ? (divisor_is_zero ? {(`XLEN){1'b1}} : dividend_too_big ? dividend_abs[`XLEN-1:0] : {(`XLEN){1'b0}}) :
                                         state_r[STATE_ITER_BIT] ? iter_cycle_quot_minus_1_nxt : (quot_neg ? inv2 : iter_quot_minus_1);
lieat_general_dfflr #(`XLEN) iter_quot_dff(clock,reset,iter_quot_en,iter_quot_nxt,iter_quot);
lieat_general_dfflr #(`XLEN) iter_quot_minus_1_dff(clock,reset,iter_quot_en,iter_quot_minus_1_nxt,iter_quot_minus_1);

wire [`LZC_WIDTH-2:0] iter_num;
wire                  iter_num_en = state_r[STATE_PRE1_BIT] | state_r[STATE_ITER_BIT];
wire [`LZC_WIDTH-2:0] iter_num_nxt = state_r[STATE_PRE1_BIT] ? (lzc_diff[`LZC_WIDTH-1:1] + {{(`LZC_WIDTH - 2){1'b0}}, lzc_diff[0]}) : (iter_num - {{(`LZC_WIDTH - 2){1'b0}}, 1'b1});
lieat_general_dfflr #(`LZC_WIDTH-1) iter_num_dff(clock,reset,iter_num_en,iter_num_nxt,iter_num);                               
wire final_iter = (iter_num == {(`LZC_WIDTH-1){1'b0}});

wire [`XLEN+3:0] rem_sum;
wire [`XLEN+3:0] rem_carry;
wire             rem_en = state_r[STATE_PRE1_BIT] | state_r[STATE_ITER_BIT];
wire [`XLEN+3:0] rem_sum_nxt   = state_r[STATE_PRE1_BIT] ? rem_sum_init_value   : rem_sum_iter_end;
wire [`XLEN+3:0] rem_carry_nxt = state_r[STATE_PRE1_BIT] ? rem_carry_init_value : rem_carry_iter_end;
lieat_general_dfflr #(`XLEN+4) rem_sum_dff(clock,reset,rem_en,rem_sum_nxt,rem_sum);
lieat_general_dfflr #(`XLEN+4) rem_carry_dff(clock,reset,rem_en,rem_carry_nxt,rem_carry);

wire [`LZC_WIDTH-1:0] quot_digit_iter_end;
wire [`XLEN+3:0] rem_sum_iter_end;
wire [`XLEN+3:0] rem_carry_iter_end;
lieat_general_radix_4_qds #(
	.WIDTH(`XLEN)
) u_qds (
	.rem_sum_i(rem_sum),
	.rem_carry_i(rem_carry),
	.divisor_i(divisor_abs[`XLEN-1:0]),
	.qds_para_neg_1_i(qds_para_neg1),
	.qds_para_neg_0_i(qds_para_neg0),
	.qds_para_pos_1_i(qds_para_pos1),
	.qds_para_pos_2_i(qds_para_pos2),
	.special_divisor_i(special_divisor),
	.prev_quot_digit_i(prev_quot_digit),
	.quot_digit_o(quot_digit_iter_end)
);

wire [`XLEN+3:0] csa_3_2_x1 = {rem_sum  [0 +: (`XLEN+2)], 2'b0};
wire [`XLEN+3:0] csa_3_2_x2 = {rem_carry[0 +: (`XLEN+2)], 2'b0};
wire [`XLEN+3:0] csa_3_2_x3 = 
  ({(`XLEN+4){prev_quot_digit[QUOT_NEG_2]}} & {divisor_abs[`XLEN-1:0], 4'b0})
| ({(`XLEN+4){prev_quot_digit[QUOT_NEG_1]}} & {1'b0, divisor_abs[`XLEN-1:0], 3'b0})
| ({(`XLEN+4){prev_quot_digit[QUOT_POS_1]}} & ~{1'b0, divisor_abs[`XLEN-1:0], 3'b0})
| ({(`XLEN+4){prev_quot_digit[QUOT_POS_2]}} & ~{divisor_abs[`XLEN-1:0], 4'b0});
wire csa_3_2_carry_unused;
lieat_general_compress32 #(
	.WIDTH(`XLEN+4)
) u_csa_3_2 (
	.x1(csa_3_2_x1),
	.x2(csa_3_2_x2),
	.x3(csa_3_2_x3),
	.sum_o(rem_sum_iter_end),
	.carry_o({rem_carry_iter_end[1 +: (`XLEN+3)], csa_3_2_carry_unused})
);
assign rem_carry_iter_end[0] = (prev_quot_digit[QUOT_POS_1] | prev_quot_digit[QUOT_POS_2]);

// ================================================================================================================================================
// Post Process
// ================================================================================================================================================
// If(rem <= 0), 
// rem = (-rem_sum) + (-rem_carry) = ~rem_sum + ~rem_carry + 2'b10;
// If(rem <= 0), 
// rem_plus_d = ~rem_sum + ~rem_carry + ~normalized_d + 2'b11;
wire [`XLEN+3:0] nrdnt_rem_nxt = 
  ({(`XLEN+4){rem_neg}} ^ rem_sum)
+ ({(`XLEN+4){rem_neg}} ^ rem_carry)
+ {{(`XLEN+2){1'b0}}, rem_neg, 1'b0};

wire [`XLEN+3:0] nrdnt_rem_plus_d_nxt = 
  ({(`XLEN+4){rem_neg}} ^ rem_sum)
+ ({(`XLEN+4){rem_neg}} ^ rem_carry)
+ ({(`XLEN+4){rem_neg}} ^ {1'b0, divisor_abs[`XLEN-1:0], 3'b0})
+ {{(`XLEN+2){1'b0}}, rem_neg, rem_neg};

// Let's assume:
// quo/quo_pre is the ABS value.
// If (rem >= 0), 
// need_corr = 0 <-> "rem_pre" belongs to [ 0, +D), quo = quo_pre - 0, rem = (rem_pre + 0) >> divisor_lzc;
// need_corr = 1 <-> "rem_pre" belongs to (-D,  0), quo = quo_pre - 1, rem = (rem_pre + D) >> divisor_lzc;
// If (rem <= 0), 
// need_corr = 0 <-> "rem_pre" belongs to (-D,  0], quo = quo_pre - 0, rem = (rem_pre - 0) >> divisor_lzc;
// need_corr = 1 <-> "rem_pre" belongs to ( 0, +D), quo = quo_pre - 1, rem = (rem_pre - D) >> divisor_lzc;
wire [`XLEN:0] nrdnt_rem 			    = dividend_abs;
wire [`XLEN:0] nrdnt_rem_plus_d 	= divisor_abs;
wire nrdnt_rem_is_zero 	= ~(|nrdnt_rem);
wire need_corr = (~divisor_is_zero & ~dividend_too_big) & (rem_neg ? (~nrdnt_rem[`XLEN] & ~nrdnt_rem_is_zero) : nrdnt_rem[`XLEN]);
wire [`XLEN:0] pre_shifted_rem = need_corr ? nrdnt_rem_plus_d : nrdnt_rem;
wire [`XLEN-1:0] final_rem = shifter_res;
wire [`XLEN-1:0] final_quot = need_corr ? iter_quot_minus_1 : iter_quot;
// ================================================================================================================================================
// output signals
// ================================================================================================================================================
assign div_i_ready    = state_r[STATE_IDLE_BIT];
assign div_o_valid    = state_r[STATE_POS1_BIT];
assign div_o_divisor0 = divisor_is_zero;
assign div_o_quot     = final_quot;
assign div_o_rem      = final_rem;
endmodule
module lieat_exu_lsu(
  input                 clock,
  input                 reset,

  input                 lsu_i_valid,
  output                lsu_i_ready,

  input [`XLEN-1:0]     lsu_i_pc,
  input [`XLEN-1:0]     lsu_i_imm,
  input [`XLEN-1:0]     lsu_i_src1,
  input [`XLEN-1:0]     lsu_i_src2,
  input [`XLEN-1:0]     lsu_i_infobus,
  input [`REG_IDX-1:0]  lsu_i_rd,
  input                 lsu_i_rdwen,

  output                lsu_req_valid,
  input                 lsu_req_ready,
  output                lsu_req_ren,
  output                lsu_req_wen,
  output [`XLEN-1:0]    lsu_req_addr,
  output [2:0]          lsu_req_flag,
  output [`XLEN-1:0]    lsu_req_wdata,
  output                lsu_req_fencei,

  input                 lsu_rsp_valid,
  output                lsu_rsp_ready,
  input  [`XLEN-1:0]    lsu_rsp_rdata,

  output                lsu_o_valid,
  input                 lsu_o_ready,
  output [`XLEN-1:0]    lsu_o_pc,
  output                lsu_o_wen,
  output [`REG_IDX-1:0] lsu_o_rd,
  output [`XLEN-1:0]    lsu_o_data,
  output                lsu_o_mmio,
  output                lsu_o_fencei_finish
);
wire unused_ok = &{lsu_infobus};
wire lsu_i_sh = lsu_i_valid & lsu_i_ready;
wire lsu_o_sh = lsu_o_valid & lsu_o_ready;
wire lsu_req_sh = lsu_req_valid & lsu_req_ready;

wire [`XLEN-1:0] lsu_pc;
wire [`XLEN-1:0] lsu_imm;
wire [`XLEN-1:0] lsu_infobus;
wire [`REG_IDX-1:0] lsu_rd;
wire lsu_rdwen;
lieat_general_dfflr #(`XLEN)    lsu_pc_dff(clock,reset,lsu_i_sh,lsu_i_pc,lsu_pc);
lieat_general_dfflr #(`XLEN)    lsu_imm_dff(clock,reset,lsu_i_sh,lsu_i_imm,lsu_imm);
lieat_general_dfflr #(`XLEN)    lsu_infobus_dff(clock,reset,lsu_i_sh,lsu_i_infobus,lsu_infobus);
lieat_general_dfflr #(`REG_IDX) lsu_rd_dff(clock,reset,lsu_i_sh,lsu_i_rd,lsu_rd);
lieat_general_dfflr #(1)        lsu_rdwen_dff(clock,reset,lsu_i_sh,lsu_i_rdwen,lsu_rdwen);

wire [1:0] lsu_sizesel  = lsu_infobus[`INFOBUS_LSU_SIZE];
wire       lsu_usignsel = lsu_infobus[`INFOBUS_LSU_USIGN];
wire       lsu_load     = lsu_infobus[`INFOBUS_LSU_LOAD];
wire       lsu_store    = lsu_infobus[`INFOBUS_LSU_STORE];
wire       lsu_fencei   = lsu_infobus[`INFOBUS_LSU_FENCEI];
//channel:LSU to DCACHE
wire lsu_req_valid_set = lsu_i_sh;
wire lsu_req_valid_clr = lsu_req_sh;
wire lsu_req_valid_ena = lsu_req_valid_set | lsu_req_valid_clr;
wire lsu_req_valid_nxt = lsu_req_valid_set | (~lsu_req_valid_clr);
lieat_general_dfflr #(1) lsu_req_valid_dff(clock,reset,lsu_req_valid_ena,lsu_req_valid_nxt,lsu_req_valid);

wire lsu_src_valid;
wire lsu_src_valid_set = lsu_i_sh;
wire lsu_src_valid_clr = lsu_src_valid;
wire lsu_src_valid_ena = lsu_src_valid_set | lsu_src_valid_clr;
wire lsu_src_valid_nxt = lsu_src_valid_set | (~lsu_src_valid_clr);
lieat_general_dfflr #(1) lsu_src_valid_dff(clock,reset,lsu_src_valid_ena,lsu_src_valid_nxt,lsu_src_valid);

wire[`XLEN-1:0] lsu_req_addr_nxt = lsu_i_src1 + lsu_imm;
reg [`XLEN-1:0] lsu_req_addr_reg;
reg [`XLEN-1:0] lsu_req_wdata_reg;
lieat_general_dfflr #(`XLEN) lsu_addr_dff(clock,reset,lsu_src_valid,lsu_req_addr_nxt,lsu_req_addr_reg);
lieat_general_dfflr #(`XLEN) lsu_wdata_dff(clock,reset,lsu_src_valid,lsu_i_src2,lsu_req_wdata_reg);

assign lsu_rsp_ready = lsu_o_ready;
assign lsu_req_ren   = lsu_load;
assign lsu_req_wen   = lsu_store;
assign lsu_req_flag  = {lsu_usignsel,lsu_sizesel};
assign lsu_req_addr  = lsu_src_valid ? lsu_req_addr_nxt : lsu_req_addr_reg;
assign lsu_req_wdata = lsu_src_valid ? lsu_i_src2       : lsu_req_wdata_reg;
assign lsu_req_fencei= lsu_fencei;

assign lsu_i_ready = (~lsu_req_valid & lsu_req_ready) | lsu_o_sh;
assign lsu_o_valid   = lsu_rsp_valid;
assign lsu_o_pc   = lsu_pc;
assign lsu_o_wen  = lsu_rdwen;
assign lsu_o_rd   = lsu_rd;
assign lsu_o_data = lsu_rsp_rdata;
assign lsu_o_mmio = (lsu_req_addr[31] != 1'b1) & (lsu_req_addr[31:28] != 4'b0011);
assign lsu_o_fencei_finish = lsu_req_fencei & lsu_rsp_valid & lsu_rsp_ready;
endmodule
module lieat_exu_muldiv(
  input                    clock,
  input                    reset,
  
  input                 muldiv_i_valid,
  output                muldiv_i_ready,
  input  [`XLEN-1:0]    muldiv_i_pc,
  input  [`XLEN-1:0]    muldiv_i_src1,
  input  [`XLEN-1:0]    muldiv_i_src2,
  input  [`XLEN-1:0]    muldiv_i_infobus,
  input  [`REG_IDX-1:0] muldiv_i_rd,
  input                 muldiv_i_rdwen,

  output                muldiv_o_valid,
  input                 muldiv_o_ready,
  output [`XLEN-1:0]    muldiv_o_pc,
  output                muldiv_o_wen,
  output [`REG_IDX-1:0] muldiv_o_rd,
  output [`XLEN-1:0]    muldiv_o_data
);
wire unused_ok = &{muldiv_infobus,div_o_divisor0};
wire muldiv_i_sh   = muldiv_i_valid & muldiv_i_ready;
wire muldiv_o_sh   = muldiv_o_valid & muldiv_o_ready;

wire [`XLEN-1:0] muldiv_pc;
wire [`XLEN-1:0] muldiv_infobus;
wire [`REG_IDX-1:0] muldiv_rd;
wire muldiv_rdwen;
lieat_general_dfflr #(`XLEN)    muldiv_pc_dff(clock,reset,muldiv_i_sh,muldiv_i_pc,muldiv_pc);
lieat_general_dfflr #(`XLEN)    muldiv_infobus_dff(clock,reset,muldiv_i_sh,muldiv_i_infobus,muldiv_infobus);
lieat_general_dfflr #(`REG_IDX) muldiv_rd_dff(clock,reset,muldiv_i_sh,muldiv_i_rd,muldiv_rd);
lieat_general_dfflr #(1)        muldiv_rdwen_dff(clock,reset,muldiv_i_sh,muldiv_i_rdwen,muldiv_rdwen);

wire muldiv_req_sh = mul_req_sh | div_req_sh;
wire div_req_sh    = div_req_valid & div_req_ready;
wire mul_req_sh    = mul_req_valid & mul_req_ready;

wire muldiv_req_valid;
wire muldiv_req_valid_set = muldiv_i_sh;
wire muldiv_req_valid_clr = muldiv_req_sh;
wire muldiv_req_valid_ena = muldiv_req_valid_set | muldiv_req_valid_clr;
wire muldiv_req_valid_nxt = muldiv_req_valid_set | ~muldiv_req_valid_clr;
lieat_general_dfflr #(1) muldiv_req_valid_dff(clock,reset,muldiv_req_valid_ena,muldiv_req_valid_nxt,muldiv_req_valid);
wire muldiv_req_ready = mul_req_ready & div_req_ready;
assign muldiv_i_ready = (~muldiv_req_valid & muldiv_req_ready) | muldiv_o_sh;

wire mul_req_valid = muldiv_req_valid & op_mul;
wire div_req_valid = muldiv_req_valid & op_div;
wire mul_req_ready;
wire div_req_ready;

wire op_mul = muldiv_mul | muldiv_mulh | muldiv_mulhsu | muldiv_mulhu;
wire op_div = muldiv_div | muldiv_divu | muldiv_rem    | muldiv_remu;
wire muldiv_mul    = muldiv_infobus[`INFOBUS_MUL_MUL   ];
wire muldiv_mulh   = muldiv_infobus[`INFOBUS_MUL_MULH  ];
wire muldiv_mulhsu = muldiv_infobus[`INFOBUS_MUL_MULHSU];
wire muldiv_mulhu  = muldiv_infobus[`INFOBUS_MUL_MULHU ];
wire muldiv_div    = muldiv_infobus[`INFOBUS_MUL_DIV   ];
wire muldiv_divu   = muldiv_infobus[`INFOBUS_MUL_DIVU  ];
wire muldiv_rem    = muldiv_infobus[`INFOBUS_MUL_REM   ];
wire muldiv_remu   = muldiv_infobus[`INFOBUS_MUL_REMU  ];

wire             mul_i_signed1 = muldiv_mulh | muldiv_mulhsu;
wire             mul_i_signed2 = muldiv_mulh;
wire [`XLEN-1:0] mul_i_multiplicand = muldiv_i_src1;
wire [`XLEN-1:0] mul_i_multiplier   = muldiv_i_src2;
wire [`XLEN-1:0] mul_o_resl;
wire [`XLEN-1:0] mul_o_resh;
wire             mul_o_ready = muldiv_o_ready;
wire             mul_o_valid;

wire             div_i_signed   = muldiv_div | muldiv_rem;
wire [`XLEN-1:0] div_i_dividend = muldiv_i_src1;
wire [`XLEN-1:0] div_i_divisor  = muldiv_i_src2;
wire [`XLEN-1:0] div_o_quot;
wire [`XLEN-1:0] div_o_rem;
wire             div_o_divisor0;
wire             div_o_ready = muldiv_o_ready;
wire             div_o_valid;

lieat_exu_mul mul(
  .clock(clock),
  .reset(reset),
  .mul_i_valid(mul_req_valid),
  .mul_i_ready(mul_req_ready),
  .mul_i_signed1(mul_i_signed1),
  .mul_i_signed2(mul_i_signed2),
  .mul_i_multiplicand(mul_i_multiplicand),
  .mul_i_multiplier(mul_i_multiplier),
  .mul_o_valid(mul_o_valid),
  .mul_o_ready(mul_o_ready),
  .mul_o_resh(mul_o_resh),
  .mul_o_resl(mul_o_resl)
);

lieat_exu_div div(
  .clock(clock),
  .reset(reset),
  .div_i_valid(div_req_valid),
  .div_i_ready(div_req_ready),
  .div_i_signed(div_i_signed),
  .div_i_dividend(div_i_dividend),
  .div_i_divisor(div_i_divisor),
  .div_o_valid(div_o_valid),
  .div_o_ready(div_o_ready),
  .div_o_quot(div_o_quot),
  .div_o_rem(div_o_rem),
  .div_o_divisor0(div_o_divisor0)
);
wire mul_low  = muldiv_mul;
wire mul_high = muldiv_mulh | muldiv_mulhsu | muldiv_mulhu;
wire div_quot = muldiv_div | muldiv_divu;
wire div_rem  = muldiv_rem | muldiv_remu;

assign muldiv_o_pc    = muldiv_pc;
assign muldiv_o_rd    = muldiv_rd;
assign muldiv_o_wen   = muldiv_rdwen;
assign muldiv_o_data = ({`XLEN{mul_low }} & mul_o_resl) | 
                       ({`XLEN{mul_high}} & mul_o_resh) |
                       ({`XLEN{div_quot}} & div_o_quot) |
                       ({`XLEN{div_rem }} & div_o_rem) ;
assign muldiv_o_valid = mul_o_valid | div_o_valid;
endmodule
module lieat_exu_mul(
  input              clock,
  input              reset,

  input              mul_i_valid,
  output             mul_i_ready,
  input              mul_i_signed1,
  input              mul_i_signed2,
  input  [`XLEN-1:0] mul_i_multiplicand,
  input  [`XLEN-1:0] mul_i_multiplier,

  output             mul_o_valid,
  input              mul_o_ready,
  output [`XLEN-1:0] mul_o_resh,
  output [`XLEN-1:0] mul_o_resl
);
localparam STATE_IDLE = 6'b000001;//quot_neg rem_neg
localparam STATE_PRE  = 6'b000010;
localparam STATE_CYC0 = 6'b000100;
localparam STATE_CYC1 = 6'b001000;
localparam STATE_CYC2 = 6'b010000;
localparam STATE_POS  = 6'b100000;

localparam STATE_IDLE_BIT = 0;
localparam STATE_PRE_BIT  = 1;
localparam STATE_CYC0_BIT = 2;
localparam STATE_CYC1_BIT = 3;
localparam STATE_CYC2_BIT = 4;
localparam STATE_POS_BIT  = 5;
// ================================================================================================================================================
// STATE Ctrl Logic
// ================================================================================================================================================
wire mul_i_sh = mul_i_valid & mul_i_ready;
wire mul_o_sh = mul_o_valid & mul_o_ready;
assign mul_i_ready = state_r[STATE_IDLE_BIT];
assign mul_o_valid = state_r[STATE_POS_BIT];
reg [5:0] state_r;
reg [5:0] state_nxt;

always @(*) begin
  case(state_r)
    STATE_IDLE:state_nxt = mul_i_sh ? (mul_i_zero ? STATE_POS : STATE_PRE) : STATE_IDLE;
    STATE_PRE :state_nxt = STATE_CYC0;
    STATE_CYC0:state_nxt = STATE_CYC1;
    STATE_CYC1:state_nxt = STATE_CYC2;
    STATE_CYC2:state_nxt = STATE_POS ;
    STATE_POS :state_nxt = mul_o_sh   ? STATE_IDLE : STATE_POS;
    default:state_nxt = STATE_IDLE;
  endcase
end
lieat_general_dffrd #(
  .DW(6),
  .DEFAULT(STATE_IDLE)
) mul_state(clock,reset,state_nxt,state_r);
// ================================================================================================================================================
// PRE PREPARE
// ================================================================================================================================================
wire mul_i_zero = (mul_i_multiplicand == 32'h0) | (mul_i_multiplier == 32'h0);
wire multiplicand_neg = mul_i_signed1 & mul_i_multiplicand[`XLEN-1];
wire multiplier_neg   = mul_i_signed2 & mul_i_multiplier[`XLEN-1];
wire [`XLEN-1:0] multiplicand_inv = -mul_i_multiplicand;
wire [`XLEN-1:0] multiplier_inv   = -mul_i_multiplier;

wire res_neg;
wire res_neg_ena = mul_i_sh;
wire res_neg_nxt = multiplicand_neg ^ multiplier_neg;
lieat_general_dfflr #(1) res_neg_dff(clock,reset,res_neg_ena,res_neg_nxt,res_neg);

wire abs_ena = mul_i_sh;
wire [`XLEN-1:0] multiplicand_abs;
wire [`XLEN-1:0] multiplier_abs;
wire [`XLEN-1:0] multiplicand_abs_nxt = multiplicand_neg ? multiplicand_inv : mul_i_multiplicand;
wire [`XLEN-1:0] multiplier_abs_nxt   = multiplier_neg   ? multiplier_inv   : mul_i_multiplier;
lieat_general_dfflr #(`XLEN) multiplicand_abs_dff(clock,reset,abs_ena,multiplicand_abs_nxt,multiplicand_abs);
lieat_general_dfflr #(`XLEN) multiplier_abs_dff(clock,reset,abs_ena,multiplier_abs_nxt,multiplier_abs);//generate booth
// ================================================================================================================================================
// CYCLE
// ================================================================================================================================================
wire [`XLEN+2:0] booth_gene = {2'b0,multiplier_abs,1'b0};
wire [8:0] booth_sel = ({9{state_r[STATE_PRE_BIT ]}} & booth_gene[32:24]) |
                       ({9{state_r[STATE_CYC0_BIT]}} & booth_gene[24:16]) |
                       ({9{state_r[STATE_CYC1_BIT]}} & booth_gene[16: 8]) |
                       ({9{state_r[STATE_CYC2_BIT]}} & booth_gene[ 8: 0]);

wire [2:0] booth_bits_0 = booth_sel[2:0];
wire [2:0] booth_bits_1 = booth_sel[4:2];
wire [2:0] booth_bits_2 = booth_sel[6:4];
wire [2:0] booth_bits_3 = booth_sel[8:6];
wire [2:0] booth_bits_extend = booth_gene[34:32];

wire [`XLEN*2-1:0] partial_product_0 = ({`XLEN*2{(booth_bits_0 == 3'b001) | (booth_bits_0 == 3'b010)}} & {32'h0,multiplicand_abs}) | 
                                       ({`XLEN*2{(booth_bits_0 == 3'b101) | (booth_bits_0 == 3'b110)}} & {32'hffffffff,(-multiplicand_abs)}) | 
                                       ({`XLEN*2{(booth_bits_0 == 3'b011)                           }} & {31'b0,multiplicand_abs,1'b0}) | 
                                       ({`XLEN*2{(booth_bits_0 == 3'b100)                           }} & {31'h7fffffff,(-multiplicand_abs),1'b0});
wire [`XLEN*2-1:0] partial_product_1 = ({`XLEN*2{(booth_bits_1 == 3'b001) | (booth_bits_1 == 3'b010)}} & {32'h0,multiplicand_abs}) | 
                                       ({`XLEN*2{(booth_bits_1 == 3'b101) | (booth_bits_1 == 3'b110)}} & {32'hffffffff,(-multiplicand_abs)}) | 
                                       ({`XLEN*2{(booth_bits_1 == 3'b011)                           }} & {31'b0,multiplicand_abs,1'b0}) | 
                                       ({`XLEN*2{(booth_bits_1 == 3'b100)                           }} & {31'h7fffffff,(-multiplicand_abs),1'b0});
wire [`XLEN*2-1:0] partial_product_2 = ({`XLEN*2{(booth_bits_2 == 3'b001) | (booth_bits_2 == 3'b010)}} & {32'h0,multiplicand_abs}) | 
                                       ({`XLEN*2{(booth_bits_2 == 3'b101) | (booth_bits_2 == 3'b110)}} & {32'hffffffff,(-multiplicand_abs)}) | 
                                       ({`XLEN*2{(booth_bits_2 == 3'b011)                           }} & {31'b0,multiplicand_abs,1'b0}) | 
                                       ({`XLEN*2{(booth_bits_2 == 3'b100)                           }} & {31'h7fffffff,(-multiplicand_abs),1'b0});
wire [`XLEN*2-1:0] partial_product_3 = ({`XLEN*2{(booth_bits_3 == 3'b001) | (booth_bits_3 == 3'b010)}} & {32'h0,multiplicand_abs}) | 
                                       ({`XLEN*2{(booth_bits_3 == 3'b101) | (booth_bits_3 == 3'b110)}} & {32'hffffffff,(-multiplicand_abs)}) | 
                                       ({`XLEN*2{(booth_bits_3 == 3'b011)                           }} & {31'b0,multiplicand_abs,1'b0}) | 
                                       ({`XLEN*2{(booth_bits_3 == 3'b100)                           }} & {31'h7fffffff,(-multiplicand_abs),1'b0});
wire [`XLEN*2-1:0] partial_product_extend = {`XLEN*2{state_r[STATE_PRE_BIT]}} & (
                                            ({`XLEN*2{(booth_bits_extend == 3'b001) | (booth_bits_extend == 3'b010)}} & {32'h0,multiplicand_abs}) | 
                                            ({`XLEN*2{(booth_bits_extend == 3'b101) | (booth_bits_extend == 3'b110)}} & {32'hffffffff,(-multiplicand_abs)}) | 
                                            ({`XLEN*2{(booth_bits_extend == 3'b011)                                }} & {31'b0,multiplicand_abs,1'b0}) | 
                                            ({`XLEN*2{(booth_bits_extend == 3'b100)                                }} & {31'h7fffffff,(-multiplicand_abs),1'b0}));
// ================================================================================================================================================
// COMPRESS ADD
// ================================================================================================================================================
wire [`XLEN*2-1:0] intermediate_sum1;
wire [`XLEN*2-1:0] intermediate_sum2;
wire [`XLEN*2-1:0] intermediate_sum3;
wire [`XLEN*2-1:0] intermediate_sum4;
wire [`XLEN*2-1:0] intermediate_sum5;
wire [`XLEN*2-1:0] intermediate_sum6;
wire [`XLEN*2-1:0] intermediate_sum7;
wire [`XLEN*2-1:0] intermediate_sum8;

lieat_general_compress32 #(`XLEN*2) mul_add1(partial_product_0,partial_product_1 << 2,partial_product_2 << 4,intermediate_sum1,intermediate_sum2);
lieat_general_compress32 #(`XLEN*2) mul_add2(partial_product_3 << 6, partial_product_extend << 8, mul_res << 8,intermediate_sum3,intermediate_sum4);
lieat_general_compress32 #(`XLEN*2) mul_add3(intermediate_sum1,intermediate_sum2,intermediate_sum3,intermediate_sum5,intermediate_sum6);
lieat_general_compress32 #(`XLEN*2) mul_add4(intermediate_sum4,intermediate_sum5,intermediate_sum6,intermediate_sum7,intermediate_sum8);
// ================================================================================================================================================
// OUTPUT SIGNAL
// ================================================================================================================================================
wire [`XLEN*2-1:0] mul_res;
wire [`XLEN*2-1:0] mul_res_cycle = intermediate_sum7 + intermediate_sum8;
wire [`XLEN*2-1:0] mul_res_nxt = state_r[STATE_IDLE_BIT] ? `XLEN*2'b0    :
                                 state_r[STATE_POS_BIT]  ? mul_res       : 
                                 state_r[STATE_CYC2_BIT] ? (res_neg ? -mul_res_cycle : mul_res_cycle) : mul_res_cycle ;
lieat_general_dfflr #(2*`XLEN) mul_res_dff(clock,reset,1'b1,mul_res_nxt,mul_res);
assign mul_o_resh  = mul_res[63:32];
assign mul_o_resl  = mul_res[31:0];
endmodule
module lieat_exu_share(
  input              alu_req,
  input [`XLEN-1:0]  alu_req_op1,
  input [`XLEN-1:0]  alu_req_op2,
  input              alu_req_add,
  input              alu_req_sub,
  input              alu_req_xor,
  input              alu_req_sll,
  input              alu_req_srl,
  input              alu_req_sra,  
  input              alu_req_or,
  input              alu_req_and,
  input              alu_req_slt,
  input              alu_req_sltu,
  input              alu_req_lui,
  output [`XLEN-1:0] alu_req_result,

  input              bjp_req,
  input              bjp_req_cmp,
  input              bjp_req_beq,
  input              bjp_req_bne,
  input              bjp_req_blt,
  input              bjp_req_bgt,
  input              bjp_req_bltu,
  input              bjp_req_bgtu,
  input [`XLEN-1:0]  bjp_req_op1,
  input [`XLEN-1:0]  bjp_req_op2,
  input              bjp_req_add,
  output [`XLEN-1:0] bjp_req_result,

  input              csr_req,
  input [`XLEN-1:0]  csr_req_op1,
  input [`XLEN-1:0]  csr_req_op2,
  input              csr_req_or,
  input              csr_req_and,
  output [`XLEN-1:0] csr_req_result
);
wire [`XLEN-1:0] op1 = ({`XLEN{alu_req}} & alu_req_op1) | ({`XLEN{bjp_req}} & bjp_req_op1) | ({`XLEN{csr_req}} & csr_req_op1);
wire [`XLEN-1:0] op2 = ({`XLEN{alu_req}} & alu_req_op2) | ({`XLEN{bjp_req}} & bjp_req_op2) | ({`XLEN{csr_req}} & csr_req_op2);
wire op_add = (alu_req & alu_req_add) | (bjp_req & bjp_req_add);
wire op_sub = (alu_req & alu_req_sub) | op_cmp_bgt | op_cmp_bgtu | op_cmp_blt | op_cmp_bltu;
wire op_xor = (alu_req & alu_req_xor) | op_cmp_beq | op_cmp_bne;
wire op_sll = (alu_req & alu_req_sll);
wire op_srl = (alu_req & alu_req_srl);
wire op_sra = (alu_req & alu_req_sra);
wire op_or  = (alu_req & alu_req_or ) | (csr_req & csr_req_or);
wire op_and = (alu_req & alu_req_and) | (csr_req & csr_req_and);
wire op_slt = (alu_req & alu_req_slt);
wire op_sltu= (alu_req & alu_req_sltu);
wire op_lui = (alu_req & alu_req_lui);
wire op_unsign = op_sltu | op_cmp_bltu | op_cmp_bgtu;

wire op_cmp_beq = bjp_req & bjp_req_beq;
wire op_cmp_bne = bjp_req & bjp_req_bne;
wire op_cmp_blt = bjp_req & bjp_req_blt;
wire op_cmp_bgt = bjp_req & bjp_req_bgt;
wire op_cmp_bltu= bjp_req & bjp_req_bltu;
wire op_cmp_bgtu= bjp_req & bjp_req_bgtu;
//adder:add sub slt sltu
wire adder_ena   = adder_add | adder_sub;
wire adder_add   = op_add;
wire adder_sub   = op_sub | op_slt | op_sltu;
wire [`XLEN:0] adder_op1 = ({`XLEN+1{adder_ena}} & {(~op_unsign) & op1[`XLEN-1],op1});
wire [`XLEN:0] adder_op2 = ({`XLEN+1{adder_ena}} & {(~op_unsign) & op2[`XLEN-1],op2});
wire [`XLEN:0] adder_in1 = adder_op1;
wire [`XLEN:0] adder_in2 = adder_sub ? (~adder_op2 + 1'b1) : adder_op2;
wire [`XLEN:0] adder_res = adder_in1 + adder_in2;

wire op_lt       = op_slt | op_sltu;
wire slt_cmp     = op_lt & adder_res[`XLEN];
wire [`XLEN-1:0] lter_res = {31'b0,slt_cmp};
//xor or and
wire [`XLEN-1:0] xorer_res = op1 ^ op2;
wire [`XLEN-1:0] orer_res  = op1 | op2;
wire [`XLEN-1:0] ander_res = op1 & op2;
//cmper:
wire neq = (|xorer_res);
wire cmp_bne_res = op_cmp_bne & neq;
wire cmp_beq_res = op_cmp_beq & ~neq;
wire cmp_blt_res = op_cmp_blt & adder_res[`XLEN];
wire cmp_bgt_res = op_cmp_bgt & ~adder_res[`XLEN];
wire cmp_bltu_res= op_cmp_bltu& adder_res[`XLEN];
wire cmp_bgtu_res= op_cmp_bgtu& ~adder_res[`XLEN];
wire cmp_res     = cmp_bne_res | cmp_beq_res | cmp_blt_res | cmp_bgt_res | cmp_bltu_res | cmp_bgtu_res;
//shifter:sll srl sra
wire shifter_ena   = op_sll | op_srl | op_sra;
wire shifter_right = op_srl | op_sra;
wire [`XLEN-1:0]    shifter_op1 = ({`XLEN{shifter_ena}} & op1);
wire [`REG_IDX-1:0] shifter_op2 = ({`REG_IDX{shifter_ena}} & op2[4:0]);
wire [`XLEN-1:0]    shifter_in1 = (shifter_right ? {
    shifter_op1[00],shifter_op1[01],shifter_op1[02],shifter_op1[03],
    shifter_op1[04],shifter_op1[05],shifter_op1[06],shifter_op1[07],
    shifter_op1[08],shifter_op1[09],shifter_op1[10],shifter_op1[11],
    shifter_op1[12],shifter_op1[13],shifter_op1[14],shifter_op1[15],
    shifter_op1[16],shifter_op1[17],shifter_op1[18],shifter_op1[19],
    shifter_op1[20],shifter_op1[21],shifter_op1[22],shifter_op1[23],
    shifter_op1[24],shifter_op1[25],shifter_op1[26],shifter_op1[27],
    shifter_op1[28],shifter_op1[29],shifter_op1[30],shifter_op1[31]
                               } : shifter_op1);
wire [4:0] shifter_in2 = shifter_op2;
wire [`XLEN-1:0] shifter_res = (shifter_in1 << shifter_in2);
wire [`XLEN-1:0] mask        = (~(`XLEN'b0)) >> shifter_in2;
wire [`XLEN-1:0] sraer_res     = (srler_res & mask) | ({32{shifter_op1[31]}} & (~mask));
wire [`XLEN-1:0] sller_res     = shifter_res;
wire [`XLEN-1:0] srler_res     = {
    shifter_res[00],shifter_res[01],shifter_res[02],shifter_res[03],
    shifter_res[04],shifter_res[05],shifter_res[06],shifter_res[07],
    shifter_res[08],shifter_res[09],shifter_res[10],shifter_res[11],
    shifter_res[12],shifter_res[13],shifter_res[14],shifter_res[15],
    shifter_res[16],shifter_res[17],shifter_res[18],shifter_res[19],
    shifter_res[20],shifter_res[21],shifter_res[22],shifter_res[23],
    shifter_res[24],shifter_res[25],shifter_res[26],shifter_res[27],
    shifter_res[28],shifter_res[29],shifter_res[30],shifter_res[31]};
//lui
wire [`XLEN-1:0] luier_res = op2;

wire [`XLEN-1:0] result = ({`XLEN{op_add}} & adder_res[`XLEN-1:0]) |
                          ({`XLEN{op_sub}} & adder_res[`XLEN-1:0]) |
                          ({`XLEN{op_xor}} & xorer_res) |
                          ({`XLEN{op_or }} & orer_res ) |
                          ({`XLEN{op_and}} & ander_res) |
                          ({`XLEN{op_srl}} & srler_res) |
                          ({`XLEN{op_sll}} & sller_res) |
                          ({`XLEN{op_sra}} & sraer_res) |
                          ({`XLEN{op_lt }} & lter_res ) |
                          ({`XLEN{op_lui}} & luier_res);

assign alu_req_result = ({`XLEN{alu_req}} & result);
assign bjp_req_result = ({`XLEN{bjp_req_add}} & result) | {31'b0,bjp_req_cmp & cmp_res};
assign csr_req_result = ({`XLEN{csr_req}} & result);
endmodule
module lieat_exu(
  input                 clock,
  input                 reset,
  
  input [`XLEN-1:0]     ex_i_pc,
  input [`XLEN-1:0]     ex_i_imm,
  input [`XLEN-1:0]     ex_i_infobus,
  input [`REG_IDX-1:0]  ex_i_rs1,
  input [`REG_IDX-1:0]  ex_i_rs2,
  input [`REG_IDX-1:0]  ex_i_rd,
  input                 ex_i_rdwen,

  output [`REG_IDX-1:0] ex_reg_rs1,
  output [`REG_IDX-1:0] ex_reg_rs2,
  input [`XLEN-1:0]     ex_reg_src1,
  input [`XLEN-1:0]     ex_reg_src2,
  
  input                 com_i_valid,
  output                com_i_ready,
  input                 lsu_i_valid,
  output                lsu_i_ready,
  input                 muldiv_i_valid,
  output                muldiv_i_ready,

  output                longi_wbck,
  output [1:0]          longi_wbck_op,
  input                 oitf_waw_dep,

  output [`XLEN-1:0]    wbck_o_pc,
  output                wbck_o_valid,
  output                wbck_o_en,
  output [`REG_IDX-1:0] wbck_o_rd,
  output [`XLEN-1:0]    wbck_o_data,
  output                wbck_o_lsu,
  output                wbck_o_ebreak,
  
  output                dcache_axi_arvalid,
  input                 dcache_axi_arready,
  output [`XLEN-1:0]    dcache_axi_araddr,
  output [2:0]          dcache_axi_arsize,
  input                 dcache_axi_rvalid,
  output                dcache_axi_rready,
  input  [63:0]         dcache_axi_rdata,
  output                dcache_axi_awvalid,
  input                 dcache_axi_awready,
  output [`XLEN-1:0]    dcache_axi_awaddr,
  output [2:0]          dcache_axi_awsize,
  output                dcache_axi_wvalid,
  input                 dcache_axi_wready,
  output [63:0]         dcache_axi_wdata,
  output [7:0]          dcache_axi_wstrb,
  input                 dcache_axi_bvalid,
  output                dcache_axi_bready,
  input  [1:0]          dcache_axi_bresp,

  output                callback_en,
  output [4:0]          callback_index,
  output                callback_result,
  output                callback_flush,
  output [`XLEN-1:0]    callback_truepc,
  output                callback_fenceifinish,

  input                 ifu_csr_ren,
  input  [11:0]         ifu_csr_idx,
  output [`XLEN-1:0]    ifu_csr_rdata,

  input  [`REG_IDX-1:0] jalr_rs1,
  input  [`XLEN-1:0]    jalr_reg_src1,
  output [`XLEN-1:0]    jalr_src1
);
// ================================================================================================================================================
// INPUT SIGNAL
// ================================================================================================================================================
wire ex_i_sh = com_i_sh | lsu_i_sh | muldiv_i_sh;
wire com_i_sh = com_i_valid & com_i_ready;
wire lsu_i_sh = lsu_i_valid & lsu_i_ready;
wire muldiv_i_sh = muldiv_i_valid & muldiv_i_ready;
lieat_general_dfflr #(`REG_IDX) ex_rs1_dff(clock,reset,ex_i_sh,ex_i_rs1,ex_reg_rs1);
lieat_general_dfflr #(`REG_IDX) ex_rs2_dff(clock,reset,ex_i_sh,ex_i_rs2,ex_reg_rs2);
// ================================================================================================================================================
// FORWARD SRC
// ================================================================================================================================================
wire com_forward_valid;
wire [`XLEN-1:0] com_forward_data;
wire com_forward_rs1_valid  = com_forward_valid & (com_o_rd == ex_reg_rs1);
wire com_forward_rs2_valid  = com_forward_valid & (com_o_rd == ex_reg_rs2);
wire com_forward_jalr_valid = com_o_valid & com_o_wen & (com_o_rd == jalr_rs1);
wire [`XLEN-1:0] ex_src1    = com_forward_rs1_valid ? com_forward_data : ex_reg_src1;
wire [`XLEN-1:0] ex_src2    = com_forward_rs2_valid ? com_forward_data : ex_reg_src2;
assign jalr_src1            = com_forward_jalr_valid ? com_o_data : jalr_reg_src1;
// ================================================================================================================================================
// COM
// ================================================================================================================================================
lieat_exu_com com(
  .clock(clock),
  .reset(reset),
  
  .com_i_valid(com_i_valid),
  .com_i_ready(com_i_ready),
  .com_i_pc(ex_i_pc),
  .com_i_imm(ex_i_imm),
  .com_i_src1(ex_src1),
  .com_i_src2(ex_src2),
  .com_i_infobus(ex_i_infobus),
  .com_i_rd(ex_i_rd),
  .com_i_rdwen(ex_i_rdwen),

  .com_o_valid(com_o_valid),
  .com_o_ready(com_o_ready),
  .com_o_pc(com_o_pc),
  .com_o_wen(com_o_wen),
  .com_o_rd(com_o_rd),
  .com_o_data(com_o_data),
  .com_o_ebreak(com_o_ebreak),
  .com_forward_valid(com_forward_valid),
  .com_forward_data(com_forward_data),

  .callback_en(callback_en),
  .callback_index(callback_index),
  .callback_result(callback_result),
  .callback_flush(callback_flush),
  .callback_truepc(callback_truepc),

  .ifu_csr_ren(ifu_csr_ren),
  .ifu_csr_idx(ifu_csr_idx),
  .ifu_csr_rdata(ifu_csr_rdata)
);
// ================================================================================================================================================
// LSU
// ================================================================================================================================================
lieat_exu_lsu lsu(
  .clock(clock),
  .reset(reset),
  .lsu_i_valid(lsu_i_valid),
  .lsu_i_ready(lsu_i_ready),
  .lsu_i_pc(ex_i_pc),
  .lsu_i_imm(ex_i_imm),
  .lsu_i_src1(ex_src1),
  .lsu_i_src2(ex_src2),
  .lsu_i_infobus(ex_i_infobus),
  .lsu_i_rd(ex_i_rd),
  .lsu_i_rdwen(ex_i_rdwen),

  .lsu_req_valid(lsu_req_valid),
  .lsu_req_ready(lsu_req_ready),
  .lsu_req_ren(lsu_req_ren),
  .lsu_req_wen(lsu_req_wen),
  .lsu_req_addr(lsu_req_addr),
  .lsu_req_flag(lsu_req_flag),
  .lsu_req_wdata(lsu_req_wdata),
  .lsu_req_fencei(lsu_req_fencei),

  .lsu_rsp_valid(lsu_rsp_valid),
  .lsu_rsp_ready(lsu_rsp_ready),
  .lsu_rsp_rdata(lsu_rsp_rdata),

  .lsu_o_valid(lsu_o_valid),
  .lsu_o_ready(lsu_o_ready),
  .lsu_o_pc(lsu_o_pc),
  .lsu_o_wen(lsu_o_wen),
  .lsu_o_rd(lsu_o_rd),
  .lsu_o_data(lsu_o_data),
  .lsu_o_mmio(lsu_o_mmio),
  .lsu_o_fencei_finish(callback_fenceifinish)
);
// ================================================================================================================================================
// MULDIV
// ================================================================================================================================================
lieat_exu_muldiv muldiv(
  .clock(clock),
  .reset(reset),
  .muldiv_i_valid(muldiv_i_valid),
  .muldiv_i_ready(muldiv_i_ready),
  .muldiv_i_pc(ex_i_pc),
  .muldiv_i_src1(ex_src1),
  .muldiv_i_src2(ex_src2),
  .muldiv_i_infobus(ex_i_infobus),
  .muldiv_i_rd(ex_i_rd),
  .muldiv_i_rdwen(ex_i_rdwen),

  .muldiv_o_valid(muldiv_o_valid),
  .muldiv_o_ready(muldiv_o_ready),
  .muldiv_o_pc(muldiv_o_pc),
  .muldiv_o_wen(muldiv_o_wen),
  .muldiv_o_rd(muldiv_o_rd),
  .muldiv_o_data(muldiv_o_data)
);
// ================================================================================================================================================
// WBCK
// ================================================================================================================================================
wire com_o_valid;
wire com_o_ready;
wire com_o_wen;
wire [`XLEN-1:0] com_o_pc;
wire [`XLEN-1:0] com_o_data;
wire [`REG_IDX-1:0] com_o_rd;
wire com_o_ebreak;

wire lsu_o_valid;
wire lsu_o_ready;
wire lsu_o_wen;
wire [`XLEN-1:0] lsu_o_pc;
wire [`XLEN-1:0] lsu_o_data;
wire [`REG_IDX-1:0] lsu_o_rd;
wire lsu_o_mmio;

wire muldiv_o_valid;
wire muldiv_o_ready;
wire muldiv_o_wen;
wire [`XLEN-1:0] muldiv_o_pc;
wire [`XLEN-1:0] muldiv_o_data;
wire [`REG_IDX-1:0] muldiv_o_rd;

lieat_exu_wbu wbu(
  .clock(clock),
  .reset(reset),

  .com_wbck_valid(com_o_valid),
  .com_wbck_ready(com_o_ready),
  .com_wbck_pc(com_o_pc),
  .com_wbck_en(com_o_wen),
  .com_wbck_rd(com_o_rd),
  .com_wbck_data(com_o_data),
  .com_wbck_ebreak(com_o_ebreak),

  .lsu_wbck_valid(lsu_o_valid),
  .lsu_wbck_ready(lsu_o_ready),
  .lsu_wbck_pc(lsu_o_pc),
  .lsu_wbck_en(lsu_o_wen),
  .lsu_wbck_rd(lsu_o_rd),
  .lsu_wbck_data(lsu_o_data),
  .lsu_wbck_mmio(lsu_o_mmio),

  .muldiv_wbck_valid(muldiv_o_valid),
  .muldiv_wbck_ready(muldiv_o_ready),
  .muldiv_wbck_pc(muldiv_o_pc),
  .muldiv_wbck_en(muldiv_o_wen),
  .muldiv_wbck_rd(muldiv_o_rd),
  .muldiv_wbck_data(muldiv_o_data),

  .longi_wbck(longi_wbck),
  .longi_wbck_op(longi_wbck_op),
  .oitf_waw_dep(oitf_waw_dep),

  .wbck_o_valid(wbck_o_valid),
  .wbck_o_pc(wbck_o_pc),
  .wbck_o_en(wbck_o_en),
  .wbck_o_rd(wbck_o_rd),
  .wbck_o_data(wbck_o_data),
  .wbck_o_lsu(wbck_o_lsu),
  .wbck_o_ebreak(wbck_o_ebreak)
);
// ================================================================================================================================================
// LSU TO DCACHE:REQ AND RSP
// ================================================================================================================================================
wire             lsu_req_valid;
wire             lsu_req_ready;
wire             lsu_req_ren;
wire             lsu_req_wen;
wire [`XLEN-1:0] lsu_req_addr;
wire [2:0]       lsu_req_flag;
wire             lsu_req_fencei;
wire [`XLEN-1:0] lsu_req_wdata;
wire             lsu_rsp_valid;
wire             lsu_rsp_ready;
wire [`XLEN-1:0] lsu_rsp_rdata;

lieat_exu_dcache dcache(
  .clock(clock),
  .reset(reset),
  .lsu_req_valid(lsu_req_valid),
  .lsu_req_ready(lsu_req_ready),
  .lsu_req_ren(lsu_req_ren),
  .lsu_req_wen(lsu_req_wen),
  .lsu_req_addr(lsu_req_addr),
  .lsu_req_flag(lsu_req_flag),
  .lsu_req_wdata(lsu_req_wdata),
  .lsu_rsp_valid(lsu_rsp_valid),
  .lsu_rsp_ready(lsu_rsp_ready),
  .lsu_rsp_rdata(lsu_rsp_rdata),
  .lsu_req_fencei(lsu_req_fencei),
  .dcache_axi_arvalid(dcache_axi_arvalid),
  .dcache_axi_arready(dcache_axi_arready),
  .dcache_axi_araddr(dcache_axi_araddr),
  .dcache_axi_arsize(dcache_axi_arsize),
  .dcache_axi_rvalid(dcache_axi_rvalid),
  .dcache_axi_rready(dcache_axi_rready),
  .dcache_axi_rdata(dcache_axi_rdata),
  .dcache_axi_awvalid(dcache_axi_awvalid),
  .dcache_axi_awready(dcache_axi_awready),
  .dcache_axi_awaddr(dcache_axi_awaddr),
  .dcache_axi_awsize(dcache_axi_awsize),
  .dcache_axi_wvalid(dcache_axi_wvalid),
  .dcache_axi_wready(dcache_axi_wready),
  .dcache_axi_wdata(dcache_axi_wdata),
  .dcache_axi_wstrb(dcache_axi_wstrb),
  .dcache_axi_bvalid(dcache_axi_bvalid),
  .dcache_axi_bready(dcache_axi_bready),
  .dcache_axi_bresp(dcache_axi_bresp)
);

endmodule
module lieat_exu_wbu(
  input                 clock,
  input                 reset,

  input                 com_wbck_valid,
  output                com_wbck_ready,
  input [`XLEN-1:0]     com_wbck_pc,
  input                 com_wbck_en,
  input [`REG_IDX-1:0]  com_wbck_rd,
  input [`XLEN-1:0]     com_wbck_data,
  input                 com_wbck_ebreak,

  input                 lsu_wbck_valid,
  output                lsu_wbck_ready,
  input [`XLEN-1:0]     lsu_wbck_pc,
  input                 lsu_wbck_en,
  input [`XLEN-1:0]     lsu_wbck_data,
  input [`REG_IDX-1:0]  lsu_wbck_rd,
  input                 lsu_wbck_mmio,

  input                 muldiv_wbck_valid,
  output                muldiv_wbck_ready,
  input [`XLEN-1:0]     muldiv_wbck_pc,
  input                 muldiv_wbck_en,
  input [`XLEN-1:0]     muldiv_wbck_data,
  input [`REG_IDX-1:0]  muldiv_wbck_rd,

  output                longi_wbck,
  output [1:0]          longi_wbck_op,
  input                 oitf_waw_dep,

  output                wbck_o_valid,
  output [`XLEN-1:0]    wbck_o_pc,
  output                wbck_o_en,
  output [`REG_IDX-1:0] wbck_o_rd,
  output [`XLEN-1:0]    wbck_o_data,
  output                wbck_o_lsu,
  output                wbck_o_ebreak
);
assign muldiv_wbck_ready = ~ignore_waw_dep;
assign lsu_wbck_ready = ~muldiv_wbck_valid & ~ignore_waw_dep;
assign com_wbck_ready = (~lsu_wbck_valid & ~muldiv_wbck_valid & ~oitf_waw_dep) | ignore_waw_dep;

wire ignore_waw_dep;
wire ignore_waw_dep_set = ~oitf_waw_dep & com_wbck_valid & longi_wbck;
wire ignore_waw_dep_clr = ignore_waw_dep;
wire ignore_waw_dep_ena = ignore_waw_dep_set | ignore_waw_dep_clr;
wire ignore_waw_dep_nxt = ignore_waw_dep_set & ~ignore_waw_dep_clr;
lieat_general_dfflr #(1) ignore_waw_dep_dff(clock,reset,ignore_waw_dep_ena,ignore_waw_dep_nxt,ignore_waw_dep);

assign longi_wbck   = lsu_wbck_valid | muldiv_wbck_valid;
assign longi_wbck_op= ignore_waw_dep ? 2'b11 : muldiv_wbck_valid ? 2'b01 : lsu_wbck_valid ? 2'b00 : 2'b11;
assign wbck_o_valid= (lsu_wbck_valid | ((com_wbck_valid & ~oitf_waw_dep) | ignore_waw_dep) | muldiv_wbck_valid);
assign wbck_o_en   = ignore_waw_dep    ? com_wbck_en :
                     muldiv_wbck_valid ? muldiv_wbck_en : 
                     lsu_wbck_valid    ? lsu_wbck_en :
                     com_wbck_valid    ? com_wbck_en : 1'b0;
assign wbck_o_pc   = ignore_waw_dep    ? com_wbck_pc :
                     muldiv_wbck_valid ? muldiv_wbck_pc : 
                     lsu_wbck_valid    ? lsu_wbck_pc :
                     com_wbck_valid    ? com_wbck_pc : `XLEN'b0;
assign wbck_o_rd   = ignore_waw_dep    ? com_wbck_rd :
                     muldiv_wbck_valid ? muldiv_wbck_rd : 
                     lsu_wbck_valid    ? lsu_wbck_rd :
                     com_wbck_valid    ? com_wbck_rd : `REG_IDX'b0;
assign wbck_o_data = ignore_waw_dep    ? com_wbck_data    :
                     muldiv_wbck_valid ? muldiv_wbck_data : 
                     lsu_wbck_valid    ? lsu_wbck_data    :
                     com_wbck_valid    ? com_wbck_data    : `XLEN'b0;
assign wbck_o_lsu  = lsu_wbck_mmio & ~ignore_waw_dep & ~muldiv_wbck_valid & lsu_wbck_valid & lsu_wbck_en;
assign wbck_o_ebreak = (ignore_waw_dep | com_wbck_valid) & com_wbck_ebreak;
endmodule
module lieat_general_64x64_sram(
  input         clock,
  input         reset,
  input         CEn,
  input         WEn,
  input  [5:0]  A,
  input  [63:0] D,
  output [63:0] Q
);
wire [63:0] sram [63:0];
wire [63:0] sram_wen;

genvar i;
generate
  for(i = 0; i < 64; i = i + 1) begin
    assign sram_wen[i] = ((A == i) & CEn & WEn);
    lieat_general_dfflr #(64) x64_sram(clock,reset,sram_wen[i],D,sram[i]); 
  end
endgenerate
assign Q = {64{CEn}} & sram[A];
endmodule
module lieat_general_compress32 #(
	parameter WIDTH = 16
)(
	input  logic [WIDTH-1:0] x1,
	input  logic [WIDTH-1:0] x2,
	input  logic [WIDTH-1:0] x3,
	
	output logic [WIDTH-1:0] sum_o,
	output logic [WIDTH-1:0] carry_o
);
assign sum_o[WIDTH-1:0] = x1 ^ x2 ^ x3;
assign carry_o[WIDTH-1:0] = {(x1[WIDTH-2:0] & x2[WIDTH-2:0]) | (x1[WIDTH-2:0] & x3[WIDTH-2:0]) | (x2[WIDTH-2:0] & x3[WIDTH-2:0]), 1'b0};
endmodule
module lieat_general_dfflrs # (//l:使用loaden使能 r:允许reset异步复位 s:默认输出全1
    parameter DW = 32
)(
  input                     clock,
  input                     reset,
  input                     loaden,
  input         [DW-1:0]    din,
  output        [DW-1:0]    qout
);

reg          reg_s1;
reg [DW-1:0] reg_qout;

always @(posedge clock or posedge reset) begin
  if (reset)begin
    reg_s1   <= 1'b0;
    reg_qout <= {DW{1'b1}};
  end
  else begin
    reg_s1   <= 1'b1;
    if (loaden == 1'b1) reg_qout <= reg_s1 ? din : {DW{1'b1}};
  end
end

assign qout = reg_qout;

endmodule
module lieat_general_dfflr # (//l:使用loaden使能 r:允许reset异步复位 s:默认输出全1
    parameter DW = 32
)(
  input                     clock,
  input                     reset,
  input                     loaden,
  input         [DW-1:0]    din,
  output        [DW-1:0]    qout
);

reg          reg_s1;
reg [DW-1:0] reg_qout;

always @(posedge clock or posedge reset)
begin
  if (reset)begin
    reg_s1   <= 1'b0;
    reg_qout <= {DW{1'b0}};
  end
  else begin
    reg_s1   <= 1'b1;
    if (loaden == 1'b1) reg_qout <= reg_s1 ? din : {DW{1'b0}};
  end
end

assign qout = reg_qout;

endmodule
module lieat_general_dffrd # (//l:使用loaden使能 r:允许reset异步复位 d:自定义默认输出
    parameter DW = 32,
    parameter DEFAULT = 32'h80000000
)(
  input                     clock,
  input                     reset,
  input         [DW-1:0]    din,
  output        [DW-1:0]    qout
);

reg          reg_s1;
reg [DW-1:0] reg_qout;

always @(posedge clock or posedge reset)
begin
  if (reset) begin
    reg_s1   <= 1'b0;
    reg_qout <= DEFAULT;
  end
  else begin
    reg_s1   <= 1'b1;
    reg_qout <= reg_s1 ? din : DEFAULT;
  end
end

assign qout = reg_qout;

endmodule
module lieat_general_dffrs # (//l:使用loaden使能 r:允许reset异步复位 s:默认输出全1
    parameter DW = 32
)(
  input                     clock,
  input                     reset,
  input         [DW-1:0]    din,
  output        [DW-1:0]    qout
);

reg          reg_s1;
reg [DW-1:0] reg_qout;

always @(posedge clock or posedge reset)
begin
  if (reset)begin
    reg_s1   <= 1'b0;
    reg_qout <= {DW{1'b1}};
  end
  else begin
    reg_s1   <= 1'b1;
    reg_qout <= reg_s1 ? din : {DW{1'b1}};
  end
end

assign qout = reg_qout;

endmodule
module lieat_general_dffr # (//l:使用loaden使能 r:允许reset异步复位 s:默认输出全1
    parameter DW = 32
)(
  input                     clock,
  input                     reset,
  input         [DW-1:0]    din,
  output        [DW-1:0]    qout
);

reg          reg_s1;
reg [DW-1:0] reg_qout;

always @(posedge clock or posedge reset)
begin
  if (reset)begin
    reg_s1   <= 1'b0;
    reg_qout <= {DW{1'b0}};
  end
  else begin
    reg_s1   <= 1'b1;
    reg_qout <= reg_s1 ? din : {DW{1'b0}};
  end
end

assign qout = reg_qout;

endmodule
module lieat_general_fifo # (
  parameter DP = 8,
  parameter DW = 32,
  parameter MASK = 1
)(
  input             clock,
  input            reset,
  
  output     oitf_empty,
  input         i_valid,
  output        i_ready,
  input [DW-1:0] i_data,

  output        o_valid,
  input         o_ready,
  output[DW-1:0] o_data
);

genvar i;
generate
  if(DP == 0) begin
    assign o_data = i_data;
    assign i_ready = o_ready;
    assign o_valid = i_valid;
    assign oitf_empty = 1'b1;
  end
  else begin
    //state control
    wire [DP-1:0] fifo_state;
    wire [DP-1:0] fifo_state_nxt;
    wire i_shakehand = i_valid & i_ready; 
    wire o_shakehand = o_valid & o_ready;    
    wire fifo_state_rise = i_shakehand & ~o_shakehand;
    wire fifo_state_fall = ~i_shakehand & o_shakehand;
    assign fifo_state_nxt = fifo_state_rise ? {fifo_state[DP-2:0],1'b1} :
                            fifo_state_fall ? {1'b0,fifo_state[DP-1:1]} : fifo_state;
    lieat_general_dfflr #(DP) fifo_state_dfflr (clock,reset,fifo_state_rise|fifo_state_fall,fifo_state_nxt,fifo_state);
    assign i_ready = ~fifo_state[DP-1] | o_shakehand;//can accept when fifo is not full or it is popping 
    assign o_valid = fifo_state[0] & (fifo_wptr != 0);
    assign oitf_empty = ~fifo_state[0];
    //ptr control
    wire [DP-1:0] fifo_wptr;
    wire [DP-1:0] fifo_wptr_nxt;
    wire [DP-1:0] fifo_rptr;
    wire [DP-1:0] fifo_rptr_nxt;
    if(DP == 1)begin
      assign fifo_rptr_nxt = 1'b1;
      assign fifo_wptr_nxt = 1'b1;
    end
    else begin
      assign fifo_rptr_nxt = fifo_rptr[DP-1] ? {{DP-1{1'b0}},1'b1} : {fifo_rptr[DP-2:0],1'b0};
      assign fifo_wptr_nxt = fifo_wptr[DP-1] ? {{DP-1{1'b0}},1'b1} : {fifo_wptr[DP-2:0],1'b0};
    end
    lieat_general_dfflrs #(1) fifo_rptr_0 (clock,reset,o_shakehand,fifo_rptr_nxt[0],fifo_rptr[0]);
    lieat_general_dfflrs #(1) fifo_wptr_0 (clock,reset,i_shakehand,fifo_wptr_nxt[0],fifo_wptr[0]);
    if(DP != 1) begin
      lieat_general_dfflr #(DP-1) fifo_rptr_DP (clock,reset,o_shakehand,fifo_rptr_nxt[DP-1:1],fifo_rptr[DP-1:1]);
      lieat_general_dfflr #(DP-1) fifo_wptr_DP (clock,reset,i_shakehand,fifo_wptr_nxt[DP-1:1],fifo_wptr[DP-1:1]);
    end
    //data control
    wire [DW-1:0] fifo_reg [DP-1:0];
    wire [DP-1:0] fifo_reg_en;
    wire [DW-1:0] fifo_out;
    for(i = 0; i < DP; i = i + 1) begin
      assign fifo_reg_en[i] = i_shakehand & fifo_wptr[i];
      lieat_general_dffl #(DW) fifo_rdata (clock,fifo_reg_en[i],i_data,fifo_reg[i]);
    end
    integer j;
    reg [DW-1:0] fifo_out;
    always @(*)begin
      fifo_out = {DW{1'b0}};
      for(j = 0; j < DP; j = j + 1)
      fifo_out = fifo_out | ({DW{fifo_rptr[j]}} & fifo_reg[j]);
    end
    if(MASK == 1) assign o_data = fifo_out & {DW{o_valid}};
    else assign o_data = fifo_out;
    end
  endgenerate
endmodule
module lieat_general_lzc(
  input  logic [31:0] in_i,
  output logic [4:0]  cnt_o,
  output logic        empty_o
);
wire [31:0] lzc_is_which;
wire [4:0] lzc_0_7;
wire [4:0] lzc_8_15;
wire [4:0] lzc_16_23;
wire [4:0] lzc_24_31;
genvar i;
generate 
  assign lzc_is_which[0]  = in_i[31];
  assign lzc_is_which[1]  = ~in_i[31];
  for (i = 2; i < 32; i = i + 1) begin
    assign lzc_is_which[i] = (in_i[31:32-i] == 0);
  end
endgenerate
  assign lzc_0_7 = lzc_is_which[7] ? 5'b00111 : lzc_is_which[6] ? 5'b00110 : lzc_is_which[5] ? 5'b00101 : lzc_is_which[4] ? 5'b00100 :
lzc_is_which[3] ? 5'b00011 : lzc_is_which[2] ? 5'b00010 : lzc_is_which[1] ? 5'b00001 : lzc_is_which[0] ? 5'b00000 : 5'b00000;
  assign lzc_8_15 = lzc_is_which[15] ? 5'b01111 : lzc_is_which[14] ? 5'b01110 : lzc_is_which[13] ? 5'b01101 : lzc_is_which[12] ? 5'b01100 :
lzc_is_which[11] ? 5'b01011 : lzc_is_which[10] ? 5'b01010 : lzc_is_which[9 ] ? 5'b01001 : lzc_is_which[8 ] ? 5'b01000 : 5'b00000;
  assign lzc_16_23 = lzc_is_which[23] ? 5'b10111 : lzc_is_which[22] ? 5'b10110 : lzc_is_which[21] ? 5'b10101 : lzc_is_which[20] ? 5'b10100 :
lzc_is_which[19] ? 5'b10011 : lzc_is_which[18] ? 5'b10010 : lzc_is_which[17] ? 5'b10001 : lzc_is_which[16] ? 5'b10000 : 5'b00000;
  assign lzc_24_31 = lzc_is_which[31] ? 5'b11111 : lzc_is_which[30] ? 5'b11110 : lzc_is_which[29] ? 5'b11101 : lzc_is_which[28] ? 5'b11100 :
lzc_is_which[27] ? 5'b11011 : lzc_is_which[26] ? 5'b11010 : lzc_is_which[25] ? 5'b11001 : lzc_is_which[24] ? 5'b11000 : 5'b00000;
  assign cnt_o = lzc_is_which[24] ? lzc_24_31 :
                 lzc_is_which[16] ? lzc_16_23 :
                 lzc_is_which[8 ] ? lzc_8_15  : lzc_0_7;
  assign empty_o = (in_i == 32'h0);
endmodule
module lieat_general_radix_4_qds #(
	// Put some parameters here, which can be changed by other modules.
	parameter WIDTH = 32,
	// ATTENTION: Don't change the below paras !!!
	// ITN = InTerNal
	parameter ITN_WIDTH = WIDTH + 4,
	parameter QUOT_ONEHOT_WIDTH = 5
)(
	input  logic [ITN_WIDTH-1:0] rem_sum_i,
	input  logic [ITN_WIDTH-1:0] rem_carry_i,
	input  logic [WIDTH-1:0] divisor_i,
	input  logic [5-1:0] qds_para_neg_1_i,
	input  logic [3-1:0] qds_para_neg_0_i,
	input  logic [2-1:0] qds_para_pos_1_i,
	input  logic [5-1:0] qds_para_pos_2_i,
	input  logic special_divisor_i,
	input  logic [QUOT_ONEHOT_WIDTH-1:0] prev_quot_digit_i,
	output logic [QUOT_ONEHOT_WIDTH-1:0] quot_digit_o
);
wire unused_ok =&{divisor_mul_neg_8,divisor_mul_neg_4,divisor_mul_8,divisor_mul_4,rem_carry_mul_16,rem_sum_mul_16};
// ================================================================================================================================================
// (local) parameters begin

localparam QUOT_NEG_2 = 0;
localparam QUOT_NEG_1 = 1;
//localparam QUOT_ZERO  = 2;
localparam QUOT_POS_1 = 3;
localparam QUOT_POS_2 = 4;

// sd = sign detector
logic [(ITN_WIDTH + 4)-1:0] rem_sum_mul_16;
logic [(ITN_WIDTH + 4)-1:0] rem_carry_mul_16;
logic [7-1:0] rem_sum_mul_16_trunc_2_5;
logic [7-1:0] rem_carry_mul_16_trunc_2_5;
logic [7-1:0] rem_sum_mul_16_trunc_3_4;
logic [7-1:0] rem_carry_mul_16_trunc_3_4;

// Since we need to do "16 * rem_sum + 16 * rem_carry - m[i] - 4 * q * D" (i = -1, 0, +1, +2) to select the next quot, so we choose to remember the 
// inversed value of parameters described in the paper.
logic [7-1:0] para_m_neg_1_trunc_2_5;
logic [7-1:0] para_m_neg_0_trunc_3_4;
logic [7-1:0] para_m_pos_1_trunc_3_4;
logic [7-1:0] para_m_pos_2_trunc_2_5;

logic [ITN_WIDTH-1:0] divisor;
logic [(ITN_WIDTH + 2)-1:0] divisor_mul_4;
logic [(ITN_WIDTH + 2)-1:0] divisor_mul_8;
logic [(ITN_WIDTH + 2)-1:0] divisor_mul_neg_4;
logic [(ITN_WIDTH + 2)-1:0] divisor_mul_neg_8;
logic [7-1:0] divisor_mul_4_trunc_2_5;
logic [7-1:0] divisor_mul_4_trunc_3_4;
logic [7-1:0] divisor_mul_8_trunc_2_5;
logic [7-1:0] divisor_mul_8_trunc_3_4;
logic [7-1:0] divisor_mul_neg_4_trunc_2_5;
logic [7-1:0] divisor_mul_neg_4_trunc_3_4;
logic [7-1:0] divisor_mul_neg_8_trunc_2_5;
logic [7-1:0] divisor_mul_neg_8_trunc_3_4;
logic [7-1:0] divisor_for_sd_trunc_3_4;
logic [7-1:0] divisor_for_sd_trunc_2_5;

logic sd_m_neg_1_sign;
logic sd_m_neg_0_sign;
logic sd_m_pos_1_sign;
logic sd_m_pos_2_sign;

// signals end
// ================================================================================================================================================

// After "16 * " operation, the decimal point is still between "[ITN_WIDTH-1]" and "[ITN_WIDTH-2]".
assign rem_sum_mul_16 = {rem_sum_i, 4'b0};
assign rem_carry_mul_16 = {rem_carry_i, 4'b0};

assign rem_sum_mul_16_trunc_2_5 = rem_sum_mul_16[(ITN_WIDTH    ) -: 7];
assign rem_sum_mul_16_trunc_3_4 = rem_sum_mul_16[(ITN_WIDTH + 1) -: 7];
assign rem_carry_mul_16_trunc_2_5 = rem_carry_mul_16[(ITN_WIDTH    ) -: 7];
assign rem_carry_mul_16_trunc_3_4 = rem_carry_mul_16[(ITN_WIDTH + 1) -: 7];
// ================================================================================================================================================
// Calculate the parameters for CMP.
// ================================================================================================================================================
assign para_m_neg_1_trunc_2_5 = {1'b0, qds_para_neg_1_i, 1'b0};

assign para_m_neg_0_trunc_3_4 = {3'b0, qds_para_neg_0_i, special_divisor_i};

assign para_m_pos_1_trunc_3_4 = {4'b1111, qds_para_pos_1_i, special_divisor_i};

assign para_m_pos_2_trunc_2_5 = {1'b1, qds_para_pos_2_i, 1'b0};

// ================================================================================================================================================
// Calculate "-4 * q * D" for CMP.
// ================================================================================================================================================
assign divisor = {1'b0, divisor_i, 3'b0};
assign divisor_mul_4 = {divisor, 2'b0};
assign divisor_mul_8 = {divisor[ITN_WIDTH-2:0], 3'b0};
// Using "~" is enough here.
assign divisor_mul_neg_4 = ~{divisor, 2'b0};
assign divisor_mul_neg_8 = ~{divisor[ITN_WIDTH-2:0], 1'b0, 2'b0};

// The decimal point is between "[ITN_WIDTH-1]" and "[ITN_WIDTH-2]".
assign divisor_mul_4_trunc_2_5 = divisor_mul_4[(ITN_WIDTH    ) -: 7];
assign divisor_mul_4_trunc_3_4 = divisor_mul_4[(ITN_WIDTH + 1) -: 7];
assign divisor_mul_8_trunc_2_5 = divisor_mul_8[(ITN_WIDTH    ) -: 7];
assign divisor_mul_8_trunc_3_4 = divisor_mul_8[(ITN_WIDTH + 1) -: 7];
assign divisor_mul_neg_4_trunc_2_5 = divisor_mul_neg_4[(ITN_WIDTH    ) -: 7];
assign divisor_mul_neg_4_trunc_3_4 = divisor_mul_neg_4[(ITN_WIDTH + 1) -: 7];
assign divisor_mul_neg_8_trunc_2_5 = divisor_mul_neg_8[(ITN_WIDTH    ) -: 7];
assign divisor_mul_neg_8_trunc_3_4 = divisor_mul_neg_8[(ITN_WIDTH + 1) -: 7];

// sd = Sign Detector
assign divisor_for_sd_trunc_2_5 = 
  ({(7){prev_quot_digit_i[QUOT_NEG_2]}} & divisor_mul_8_trunc_2_5)
| ({(7){prev_quot_digit_i[QUOT_NEG_1]}} & divisor_mul_4_trunc_2_5)
| ({(7){prev_quot_digit_i[QUOT_POS_1]}} & divisor_mul_neg_4_trunc_2_5)
| ({(7){prev_quot_digit_i[QUOT_POS_2]}} & divisor_mul_neg_8_trunc_2_5);
assign divisor_for_sd_trunc_3_4 = 
  ({(7){prev_quot_digit_i[QUOT_NEG_2]}} & divisor_mul_8_trunc_3_4)
| ({(7){prev_quot_digit_i[QUOT_NEG_1]}} & divisor_mul_4_trunc_3_4)
| ({(7){prev_quot_digit_i[QUOT_POS_1]}} & divisor_mul_neg_4_trunc_3_4)
| ({(7){prev_quot_digit_i[QUOT_POS_2]}} & divisor_mul_neg_8_trunc_3_4);

// ================================================================================================================================================
// Calculate sign and code the res.
// ================================================================================================================================================
lieat_general_radix_4_sign_detector
u_sd_m_neg_1 (
	.rem_sum_msb_i(rem_sum_mul_16_trunc_2_5),
	.rem_carry_msb_i(rem_carry_mul_16_trunc_2_5),
	.parameter_i(para_m_neg_1_trunc_2_5),
	.divisor_i(divisor_for_sd_trunc_2_5),
	.sign_o(sd_m_neg_1_sign)
);
lieat_general_radix_4_sign_detector
u_sd_m_neg_0 (
	.rem_sum_msb_i(rem_sum_mul_16_trunc_3_4),
	.rem_carry_msb_i(rem_carry_mul_16_trunc_3_4),
	.parameter_i(para_m_neg_0_trunc_3_4),
	.divisor_i(divisor_for_sd_trunc_3_4),
	.sign_o(sd_m_neg_0_sign)
);
lieat_general_radix_4_sign_detector
u_sd_m_pos_1 (
	.rem_sum_msb_i(rem_sum_mul_16_trunc_3_4),
	.rem_carry_msb_i(rem_carry_mul_16_trunc_3_4),
	.parameter_i(para_m_pos_1_trunc_3_4),
	.divisor_i(divisor_for_sd_trunc_3_4),
	.sign_o(sd_m_pos_1_sign)
);
lieat_general_radix_4_sign_detector
u_sd_m_pos_2 (
	.rem_sum_msb_i(rem_sum_mul_16_trunc_2_5),
	.rem_carry_msb_i(rem_carry_mul_16_trunc_2_5),
	.parameter_i(para_m_pos_2_trunc_2_5),
	.divisor_i(divisor_for_sd_trunc_2_5),
	.sign_o(sd_m_pos_2_sign)
);

lieat_general_radix_4_sign_coder
u_sign_coder (
	.sd_m_neg_1_sign_i(sd_m_neg_1_sign),
	.sd_m_neg_0_sign_i(sd_m_neg_0_sign),
	.sd_m_pos_1_sign_i(sd_m_pos_1_sign),
	.sd_m_pos_2_sign_i(sd_m_pos_2_sign),
	.quot_o(quot_digit_o)
);


endmodule
module lieat_general_radix_4_sign_coder #(
	// Put some parameters here, which can be changed by other modules.
	
)(
	input  logic sd_m_neg_1_sign_i,
	input  logic sd_m_neg_0_sign_i,
	input  logic sd_m_pos_1_sign_i,
	input  logic sd_m_pos_2_sign_i,
	output logic [5-1:0] quot_o
);

// ==================================================================================================================================================
// (local) params
// ==================================================================================================================================================

localparam QUOT_NEG_2 = 0;
localparam QUOT_NEG_1 = 1;
localparam QUOT_ZERO  = 2;
localparam QUOT_POS_1 = 3;
localparam QUOT_POS_2 = 4;

logic [4-1:0] sign;

// ==================================================================================================================================================
// main codes
// ==================================================================================================================================================

// Just look at "TABLE 2" in 
// "Digit-Recurrence Dividers with Reduced Logical Depth", Elisardo Antelo.
assign sign = {sd_m_pos_2_sign_i, sd_m_pos_1_sign_i, sd_m_neg_0_sign_i, sd_m_neg_1_sign_i};
assign quot_o[QUOT_POS_2] = (sign[3:1] == 3'b000);
assign quot_o[QUOT_POS_1] = (sign[3:1] == 3'b100);
assign quot_o[QUOT_ZERO ] = (sign[2:1] == 2'b10);
assign quot_o[QUOT_NEG_1] = (sign[2:0] == 3'b110);
assign quot_o[QUOT_NEG_2] = (sign[2:0] == 3'b111);

endmodule
// ========================================================================================================
// File Name			: radix_4_sign_detector.sv
// Author				: Yifei He
// How to Contact		: hyf_sysu@qq.com
// Created Time    		: 2021-07-20 16:08:45
// Last Modified Time 	: 2021-09-10 14:32:35
// ========================================================================================================
// Description	:
// Please Look at the reference for more details.
// ========================================================================================================

// ========================================================================================================
// Copyright (C) 2021, Yifei He. All Rights Reserved.
// This file is licensed under BSD 3-Clause License.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// Redistribution and use in source and binary forms, with or without modification, 
// are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this list of 
// conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of 
// conditions and the following disclaimer in the documentation and/or other materials provided 
// with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its contributors may be used 
// to endorse or promote products derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
// OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
// THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND 
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ========================================================================================================

// include some definitions here

module lieat_general_radix_4_sign_detector #(
	// Put some parameters here, which can be changed by other modules.
	
)(
	input  logic[7-1:0] rem_sum_msb_i,
	input  logic[7-1:0] rem_carry_msb_i,
	input  logic[7-1:0] parameter_i,
	input  logic[7-1:0] divisor_i,
	// input  cin_i,
	output logic sign_o
);
logic [6-1:0] unused_bit;

assign {sign_o, unused_bit} = rem_sum_msb_i + rem_carry_msb_i + parameter_i + divisor_i;
// assign {sign_o, unused_bit} = rem_sum_msb_i + rem_carry_msb_i + parameter_i + divisor_i + {6'b0, cin_i};

endmodule
module lieat_idu_dec(
  input  [`XLEN-1:0]    inst,
  input                 prdt_taken,
  
  output                id_rv32,
  output                id_rs1en,
  output                id_rs2en,
  output                id_rdwen,
  output [`REG_IDX-1:0] id_rs1,
  output [`REG_IDX-1:0] id_rs2,
  output [`REG_IDX-1:0] id_rd,
  output [`XLEN-1:0]    id_imm,
  output [`XLEN-1:0]    id_infobus,
  output                id_ilgl
);

wire [6:0] opcode= rv32_inst[6:0];
wire opcode_1_0_11  = (opcode[1:0] == 2'b11);
wire opcode_4_2_000 = (opcode[4:2] == 3'b000);
wire opcode_4_2_001 = (opcode[4:2] == 3'b001);
//wire opcode_4_2_010 = (opcode[4:2] == 3'b010);
wire opcode_4_2_011 = (opcode[4:2] == 3'b011);
wire opcode_4_2_100 = (opcode[4:2] == 3'b100);
wire opcode_4_2_101 = (opcode[4:2] == 3'b101);
//wire opcode_4_2_110 = (opcode[4:2] == 3'b110);
wire opcode_4_2_111 = (opcode[4:2] == 3'b111);
wire opcode_6_5_00  = (opcode[6:5] == 2'b00);
wire opcode_6_5_01  = (opcode[6:5] == 2'b01);
//wire opcode_6_5_10  = (opcode[6:5] == 2'b10);
wire opcode_6_5_11  = (opcode[6:5] == 2'b11);
//--------------------rv32--------------------
assign id_rv32 = (~opcode_4_2_111) & opcode_1_0_11;
wire [31:0] rv32_inst = inst;

wire [4:0] rv32_rs2   = rv32_inst[24:20];
wire [4:0] rv32_rs1   = rv32_inst[19:15];
wire [4:0] rv32_rd    = rv32_inst[11:7];

wire rv32_rs1x0 = (rv32_rs1 == 5'b00000);
wire rv32_rs2x0 = (rv32_rs2 == 5'b00000);
wire rv32_rdx0  = (rv32_rd  == 5'b00000);

wire [6:0] rv32_func7 = rv32_inst[31:25];
wire rv32_func7_0000000 = (rv32_func7 == 7'b0000000);
wire rv32_func7_0000001 = (rv32_func7 == 7'b0000001);
wire rv32_func7_0100000 = (rv32_func7 == 7'b0100000);
wire rv32_func7_0100001 = (rv32_func7 == 7'b0100001);

wire [2:0] rv32_func3 = rv32_inst[14:12];
wire rv32_func3_000 = (rv32_func3 == 3'b000);
wire rv32_func3_001 = (rv32_func3 == 3'b001);
wire rv32_func3_010 = (rv32_func3 == 3'b010);
wire rv32_func3_011 = (rv32_func3 == 3'b011);
wire rv32_func3_100 = (rv32_func3 == 3'b100);
wire rv32_func3_101 = (rv32_func3 == 3'b101);
wire rv32_func3_110 = (rv32_func3 == 3'b110);
wire rv32_func3_111 = (rv32_func3 == 3'b111);

wire rv32_load     = opcode_6_5_00 & opcode_4_2_000 & opcode_1_0_11;
//wire rv32_lb       = rv32_load   & rv32_func3_000;
//wire rv32_lh       = rv32_load   & rv32_func3_001;
//wire rv32_lw       = rv32_load   & rv32_func3_010;
//wire rv32_lbu      = rv32_load   & rv32_func3_100;
//wire rv32_lhu      = rv32_load   & rv32_func3_101;

wire rv32_store    = opcode_6_5_01 & opcode_4_2_000 & opcode_1_0_11;
//wire rv32_sb       = rv32_store  & rv32_func3_000;
//wire rv32_sh       = rv32_store  & rv32_func3_001;
//wire rv32_sw       = rv32_store  & rv32_func3_010;

//wire rv32_madd   = opcode_6_5_10 & opcode_4_2_000 & opcode_1_0_11; 

wire rv32_branch   = opcode_6_5_11 & opcode_4_2_000 & opcode_1_0_11; 
wire rv32_beq      = rv32_branch & rv32_func3_000;
wire rv32_bne      = rv32_branch & rv32_func3_001;
wire rv32_blt      = rv32_branch & rv32_func3_100;
wire rv32_bgt      = rv32_branch & rv32_func3_101;
wire rv32_bltu     = rv32_branch & rv32_func3_110;
wire rv32_bgtu     = rv32_branch & rv32_func3_111;

//wire rv32_loadfp = opcode_6_5_00 & opcode_4_2_001 & opcode_1_0_11; 
//wire rv32_storefp= opcode_6_5_01 & opcode_4_2_001 & opcode_1_0_11; 
//wire rv32_msub   = opcode_6_5_10 & opcode_4_2_001 & opcode_1_0_11; 
wire rv32_jalr     = opcode_6_5_11 & opcode_4_2_001 & opcode_1_0_11; 

//wire rv32_custom0= opcode_6_5_00 & opcode_4_2_010 & opcode_1_0_11; 
//wire rv32_custom1= opcode_6_5_01 & opcode_4_2_010 & opcode_1_0_11; 
//wire rv32_nmsub  = opcode_6_5_10 & opcode_4_2_010 & opcode_1_0_11; 
//wire rv32_resrvd0= opcode_6_5_11 & opcode_4_2_010 & opcode_1_0_11; 

wire rv32_miscmem  = opcode_6_5_00 & opcode_4_2_011 & opcode_1_0_11;
wire rv32_fence    = rv32_miscmem & rv32_func3_000;
wire rv32_fencei   = rv32_miscmem & rv32_func3_001;
//wire rv32_amo    = opcode_6_5_01 & opcode_4_2_011 & opcode_1_0_11; 
//wire rv32_nmadd  = opcode_6_5_10 & opcode_4_2_011 & opcode_1_0_11; 
wire rv32_jal      = opcode_6_5_11 & opcode_4_2_011 & opcode_1_0_11; 

wire rv32_op_imm   = opcode_6_5_00 & opcode_4_2_100 & opcode_1_0_11; 
wire rv32_addi     = rv32_op_imm & rv32_func3_000;
wire rv32_slti     = rv32_op_imm & rv32_func3_010;
wire rv32_sltiu    = rv32_op_imm & rv32_func3_011;
wire rv32_xori     = rv32_op_imm & rv32_func3_100;
wire rv32_ori      = rv32_op_imm & rv32_func3_110;
wire rv32_andi     = rv32_op_imm & rv32_func3_111;
wire rv32_slli     = rv32_op_imm & rv32_func3_001 & (rv32_func7_0000000 | rv32_func7_0000001);
wire rv32_srli     = rv32_op_imm & rv32_func3_101 & (rv32_func7_0000000 | rv32_func7_0000001);
wire rv32_srai     = rv32_op_imm & rv32_func3_101 & (rv32_func7_0100000 | rv32_func7_0100001);

wire rv32_nop      = rv32_addi & rv32_rs1x0 & rv32_rdx0 & (~(|rv32_inst[31:20]));

wire rv32_op       = opcode_6_5_01 & opcode_4_2_100 & opcode_1_0_11; 
wire rv32_add      = rv32_op & rv32_func3_000 & rv32_func7_0000000;
wire rv32_sub      = rv32_op & rv32_func3_000 & rv32_func7_0100000;
wire rv32_sll      = rv32_op & rv32_func3_001 & rv32_func7_0000000;
wire rv32_slt      = rv32_op & rv32_func3_010 & rv32_func7_0000000;
wire rv32_sltu     = rv32_op & rv32_func3_011 & rv32_func7_0000000;
wire rv32_xor      = rv32_op & rv32_func3_100 & rv32_func7_0000000;
wire rv32_srl      = rv32_op & rv32_func3_101 & rv32_func7_0000000;
wire rv32_sra      = rv32_op & rv32_func3_101 & rv32_func7_0100000;
wire rv32_or       = rv32_op & rv32_func3_110 & rv32_func7_0000000;
wire rv32_and      = rv32_op & rv32_func3_111 & rv32_func7_0000000;
wire rv32_mul      = rv32_op & rv32_func3_000 & rv32_func7_0000001;
wire rv32_mulh     = rv32_op & rv32_func3_001 & rv32_func7_0000001;
wire rv32_mulhsu   = rv32_op & rv32_func3_010 & rv32_func7_0000001;
wire rv32_mulhu    = rv32_op & rv32_func3_011 & rv32_func7_0000001;
wire rv32_div      = rv32_op & rv32_func3_100 & rv32_func7_0000001;
wire rv32_divu     = rv32_op & rv32_func3_101 & rv32_func7_0000001;
wire rv32_rem      = rv32_op & rv32_func3_110 & rv32_func7_0000001;
wire rv32_remu     = rv32_op & rv32_func3_111 & rv32_func7_0000001;

//wire rv32_op_fp  = opcode_6_5_10 & opcode_4_2_100 & opcode_1_0_11; 

wire rv32_system   = opcode_6_5_11 & opcode_4_2_100 & opcode_1_0_11; 
wire rv32_ecall    = rv32_system & rv32_func3_000 & (rv32_inst[31:20] == 12'b000000000000);
wire rv32_ebreak   = rv32_system & rv32_func3_000 & (rv32_inst[31:20] == 12'b000000000001);
wire rv32_mret     = rv32_system & rv32_func3_000 & (rv32_inst[31:20] == 12'b001100000010);
//wire rv32_dret     = rv32_system & rv32_func3_000 & (rv32_instr[31:20] == 12'b0111_1011_0010);
wire rv32_wfi      = rv32_system & rv32_func3_000 & (rv32_inst[31:20] == 12'b0001_0000_0101);
wire rv32_csr      = (rv32_system & (~rv32_func3_000)) | rv32_ecall;//not main opcode
wire rv32_csrrw    = rv32_system & rv32_func3_001; 
wire rv32_csrrs    = rv32_system & rv32_func3_010; 
wire rv32_csrrc    = rv32_system & rv32_func3_011; 
wire rv32_csrrwi   = rv32_system & rv32_func3_101; 
wire rv32_csrrsi   = rv32_system & rv32_func3_110; 
wire rv32_csrrci   = rv32_system & rv32_func3_111; 

wire rv32_auipc    = opcode_6_5_00 & opcode_4_2_101 & opcode_1_0_11; 
wire rv32_lui      = opcode_6_5_01 & opcode_4_2_101 & opcode_1_0_11; 
//wire rv32_resved1= opcode_6_5_10 & opcode_4_2_101 & opcode_1_0_11; 
//wire rv32_resved2= opcode_6_5_11 & opcode_4_2_101 & opcode_1_0_11; 

//wire rv32_opimm_32= opcode_6_5_00 & opcode_4_2_110 & opcode_1_0_11; 
//wire rv32_op_32  = opcode_6_5_01 & opcode_4_2_110 & opcode_1_0_11; 
//wire rv32_custom2= opcode_6_5_10 & opcode_4_2_110 & opcode_1_0_11; 
//wire rv32_custom3= opcode_6_5_11 & opcode_4_2_110 & opcode_1_0_11; 

//--------------------ILEGAL--------------------
wire rv32_shift_ilgl = (rv32_slli | rv32_srli | rv32_srai) & (rv32_inst[25]);//should be 0
wire rv32_all01_ilgl = (inst == 32'h0) | (inst == 32'hffffffff);
wire op_ilgl         = ~(alu_op | bjp_op | lsu_op | csr_op | mul_op);


//lui auipc jal fence fencei ecall ebreak csrrwi csrrsi csrrci no need rs1 
wire rv32_need_rs1 = (~rv32_rs1x0) & 
                     (~rv32_lui)    & 
                     (~rv32_auipc)  & 
                     (~rv32_jal)    & 
                     (~rv32_fence)  & 
                     (~rv32_fencei) & 
                     (~rv32_ecall)  & 
                     (~rv32_ebreak) &
                     (~rv32_csrrwi) & 
                     (~rv32_csrrsi) & 
                     (~rv32_csrrci);
//branch store rv32_op need rs2
wire rv32_need_rs2 = (~rv32_rs2x0) & ((rv32_branch) | (rv32_store) | (rv32_op));
//branch store fence fence_i ecall ebreak no need rd
wire rv32_need_rd  = (~rv32_rdx0)  & 
                     (~rv32_branch) & 
                     (~rv32_store)  & 
                     (~rv32_fence)  & 
                     (~rv32_fencei) & 
                     (~rv32_ecall)  & 
                     (~rv32_ebreak);

wire rv32_i = rv32_op_imm | rv32_jalr | rv32_load;
wire rv32_u = rv32_lui | rv32_auipc;
wire rv32_s = rv32_store;
wire rv32_b = rv32_branch;
wire rv32_j = rv32_jal;
wire rv32_need_imm = rv32_i | rv32_u | rv32_s | rv32_b | rv32_j;
wire need_imm      = id_rv32 ? rv32_need_imm : 1'b0;

wire [31:0] rv32_immi = {{20{rv32_inst[31]}},rv32_inst[31:20]};
wire [31:0] rv32_immu = {rv32_inst[31:12],12'b0};
wire [31:0] rv32_imms = {{20{rv32_inst[31]}},rv32_inst[31:25],rv32_inst[11:7]};
wire [31:0] rv32_immb = {{20{rv32_inst[31]}},rv32_inst[7],rv32_inst[30:25],rv32_inst[11:8],1'b0};
wire [31:0] rv32_immj = {{12{rv32_inst[31]}},rv32_inst[19:12],rv32_inst[20],rv32_inst[30:21],1'b0};
wire [31:0] rv32_imm  = rv32_i ? rv32_immi :
                        rv32_u ? rv32_immu :
                        rv32_s ? rv32_imms :
                        rv32_b ? rv32_immb :
                        rv32_j ? rv32_immj : 32'h0;

//--------------------IFOBUS--------------------
wire [`INFOBUS_ALU_WIDTH-1:0] alu_infobus;
wire alu_op = (~rv32_shift_ilgl) & ( rv32_op_imm | rv32_op & (~rv32_func7_0000001) | rv32_auipc | rv32_lui | rv32_nop | rv32_wfi | rv32_ebreak);
assign alu_infobus[`INFOBUS_OP        ] = `INFOBUS_OP_ALU;
assign alu_infobus[`INFOBUS_ALU_RV32  ] = id_rv32;
assign alu_infobus[`INFOBUS_ALU_ADD   ] = rv32_add | rv32_addi | rv32_auipc;
assign alu_infobus[`INFOBUS_ALU_SUB   ] = rv32_sub;
assign alu_infobus[`INFOBUS_ALU_SLT   ] = rv32_slt | rv32_slti;
assign alu_infobus[`INFOBUS_ALU_SLTU  ] = rv32_sltu | rv32_sltiu;
assign alu_infobus[`INFOBUS_ALU_XOR   ] = rv32_xor | rv32_xori;
assign alu_infobus[`INFOBUS_ALU_SLL   ] = rv32_sll | rv32_slli;
assign alu_infobus[`INFOBUS_ALU_SRL   ] = rv32_srl | rv32_srli;
assign alu_infobus[`INFOBUS_ALU_SRA   ] = rv32_sra | rv32_srai;
assign alu_infobus[`INFOBUS_ALU_OR    ] = rv32_or  | rv32_ori;
assign alu_infobus[`INFOBUS_ALU_AND   ] = rv32_and | rv32_andi;
assign alu_infobus[`INFOBUS_ALU_LUI   ] = rv32_lui;
assign alu_infobus[`INFOBUS_ALU_IMM   ] = need_imm;
assign alu_infobus[`INFOBUS_ALU_PC    ] = rv32_auipc;
assign alu_infobus[`INFOBUS_ALU_NOP   ] = rv32_nop;
assign alu_infobus[`INFOBUS_ALU_EBRK  ] = rv32_ebreak;

wire [`INFOBUS_BJP_WIDTH-1:0] bjp_infobus;
wire bjp_op = rv32_jal | rv32_jalr | rv32_branch | rv32_mret;
assign bjp_infobus[`INFOBUS_OP        ] = `INFOBUS_OP_BJP;
assign bjp_infobus[`INFOBUS_BJP_RV32  ] = id_rv32;
assign bjp_infobus[`INFOBUS_BJP_JUMP  ] = rv32_jal | rv32_jalr;
assign bjp_infobus[`INFOBUS_BJP_BPRDT ] = prdt_taken;
assign bjp_infobus[`INFOBUS_BJP_BEQ   ] = rv32_beq;
assign bjp_infobus[`INFOBUS_BJP_BNE   ] = rv32_bne;
assign bjp_infobus[`INFOBUS_BJP_BLT   ] = rv32_blt; 
assign bjp_infobus[`INFOBUS_BJP_BGT   ] = rv32_bgt ;
assign bjp_infobus[`INFOBUS_BJP_BLTU  ] = rv32_bltu;
assign bjp_infobus[`INFOBUS_BJP_BGTU  ] = rv32_bgtu;
assign bjp_infobus[`INFOBUS_BJP_BXX   ] = rv32_branch;
assign bjp_infobus[`INFOBUS_BJP_MRET  ] = rv32_mret;

wire [`INFOBUS_LSU_WIDTH-1:0] lsu_infobus;
wire lsu_op = rv32_load | rv32_store | rv32_fence | rv32_fencei;
wire [1:0] lsu_info_size  = id_rv32 ? rv32_func3[1:0] : 2'b10;
wire       lsu_info_usign = id_rv32 ? rv32_func3[2]   : 1'b0;
assign lsu_infobus[`INFOBUS_OP        ] = `INFOBUS_OP_LSU;
assign lsu_infobus[`INFOBUS_LSU_RV32  ] = id_rv32;
assign lsu_infobus[`INFOBUS_LSU_LOAD  ] = rv32_load;
assign lsu_infobus[`INFOBUS_LSU_STORE ] = rv32_store;
assign lsu_infobus[`INFOBUS_LSU_SIZE  ] = lsu_info_size;
assign lsu_infobus[`INFOBUS_LSU_USIGN ] = lsu_info_usign;
assign lsu_infobus[`INFOBUS_LSU_OP2IMM] = need_imm;
assign lsu_infobus[`INFOBUS_LSU_FENCE ] = rv32_fence;
assign lsu_infobus[`INFOBUS_LSU_FENCEI] = rv32_fencei;

wire [`INFOBUS_CSR_WIDTH-1:0] csr_infobus;
wire csr_op = rv32_csr;
assign csr_infobus[`INFOBUS_OP        ] = `INFOBUS_OP_CSR;
assign csr_infobus[`INFOBUS_CSR_RV32  ] = id_rv32;
assign csr_infobus[`INFOBUS_CSR_CSRRW ] = rv32_csrrw | rv32_csrrwi;
assign csr_infobus[`INFOBUS_CSR_CSRRS ] = rv32_csrrs | rv32_csrrsi;
assign csr_infobus[`INFOBUS_CSR_CSRRC ] = rv32_csrrc | rv32_csrrci;
assign csr_infobus[`INFOBUS_CSR_RS1IMM] = rv32_csrrwi | rv32_csrrsi | rv32_csrrci;
assign csr_infobus[`INFOBUS_CSR_ECAL  ] = rv32_ecall;
assign csr_infobus[`INFOBUS_CSR_ZIMMM ] = rv32_rs1;
assign csr_infobus[`INFOBUS_CSR_RS1IS0] = rv32_rs1x0;
assign csr_infobus[`INFOBUS_CSR_CSRIDX] = rv32_inst[31:20];


wire [`INFOBUS_MUL_WIDTH-1:0] mul_infobus;
wire mul_op = rv32_op & rv32_func7_0000001;
assign mul_infobus[`INFOBUS_OP        ] = `INFOBUS_OP_MUL;
assign mul_infobus[`INFOBUS_MUL_RV32  ] = id_rv32    ;
assign mul_infobus[`INFOBUS_MUL_MUL   ] = rv32_mul    ;   
assign mul_infobus[`INFOBUS_MUL_MULH  ] = rv32_mulh   ;
assign mul_infobus[`INFOBUS_MUL_MULHSU] = rv32_mulhsu ;
assign mul_infobus[`INFOBUS_MUL_MULHU ] = rv32_mulhu  ;
assign mul_infobus[`INFOBUS_MUL_DIV   ] = rv32_div    ;
assign mul_infobus[`INFOBUS_MUL_DIVU  ] = rv32_divu   ;
assign mul_infobus[`INFOBUS_MUL_REM   ] = rv32_rem    ;
assign mul_infobus[`INFOBUS_MUL_REMU  ] = rv32_remu   ;
//--------------------OUTPUT--------------------
assign id_rs1     = id_rv32 ? rv32_rs1      : 0;
assign id_rs2     = id_rv32 ? rv32_rs2      : 0;
assign id_rd      = id_rv32 ? rv32_rd       : 0;
assign id_rs1en   = id_rv32 ? rv32_need_rs1 : 0;
assign id_rs2en   = id_rv32 ? rv32_need_rs2 : 0;
assign id_rdwen   = id_rv32 ? rv32_need_rd  : 0;

assign id_imm     = id_rv32 ? rv32_imm : 32'h0;
assign id_infobus = alu_op     ? {{(`XLEN-`INFOBUS_ALU_WIDTH){1'b0}},alu_infobus} :
                    bjp_op     ? {{(`XLEN-`INFOBUS_BJP_WIDTH){1'b0}},bjp_infobus} :
                    lsu_op     ? {{(`XLEN-`INFOBUS_LSU_WIDTH){1'b0}},lsu_infobus} :
                    csr_op     ? {{(`XLEN-`INFOBUS_CSR_WIDTH){1'b0}},csr_infobus} :
                    mul_op     ? {{(`XLEN-`INFOBUS_MUL_WIDTH){1'b0}},mul_infobus} : 0;
assign id_ilgl   = rv32_all01_ilgl | rv32_shift_ilgl | op_ilgl;
endmodule
module lieat_idu_depend#(
  DEPTH = 3 
)(
  input                clock,
  input                reset,
  input                disp_ena,
  input [2:0]          disp_op,
  input                disp_rs1en,
  input                disp_rs2en,
  input                disp_rdwen,
  input [`REG_IDX-1:0] disp_rs1,
  input [`REG_IDX-1:0] disp_rs2,
  input [`REG_IDX-1:0] disp_rd,

  input                remove_ena,
  input [1:0]          remove_op,

  input [`REG_IDX-1:0] ifu_dep_rs1,
  input [`REG_IDX-1:0] wbu_dep_rd,

  output               disp_dep,
  output               wbck_dep,
  output               ifu_dep,
  output               longi_empty
);
// ================================================================================================================================================
// PTR
// ================================================================================================================================================
wire [1:0] wptr = (disp_op   == 3'b010) ? 2'b00 : (disp_op   == 3'b100) ? 2'b01 : 2'b11;//lsu muldiv
wire [1:0] rptr = remove_op;
// ================================================================================================================================================
// OITF SIGNAL
// ================================================================================================================================================
wire [DEPTH-1:0] oitf;//lsu muldiv others
wire [DEPTH-1:0] oitf_set;
wire [DEPTH-1:0] oitf_clr;
wire [DEPTH-1:0] oitf_ena;
wire [DEPTH-1:0] oitf_nxt;
wire [DEPTH-1:0] oitf_rdwen;
wire [`REG_IDX-1:0] oitf_rd [DEPTH-1:0];

wire [DEPTH-1:0] oitf_rs1_matchrd;
wire [DEPTH-1:0] oitf_rs2_matchrd;
wire [DEPTH-1:0] oitf_rd_matchrd;
wire [DEPTH-1:0] oitf_ifu_matchrd;
wire [DEPTH-1:0] wbck_rd_matchrd;
genvar i;
generate 
  for(i = 0;i < DEPTH; i = i + 1) begin
    assign oitf_set[i] = disp_ena   & (wptr == i);
    assign oitf_clr[i] = remove_ena & (rptr == i);
    assign oitf_ena[i] = oitf_set[i] | oitf_clr[i];
    assign oitf_nxt[i] = oitf_set[i] | (~oitf_clr[i]);

    lieat_general_dfflr #(1) oitf_dff (clock,reset,oitf_ena[i],oitf_nxt[i],oitf[i]);
    lieat_general_dfflr #(1) oitf_rdwen_dff (clock,reset,oitf_set[i],disp_rdwen,oitf_rdwen[i]);
    lieat_general_dfflr #(`REG_IDX) oitf_rd_dff (clock,reset,oitf_set[i],disp_rd,oitf_rd[i]);
    
    assign oitf_rs1_matchrd[i] = ~oitf_clr[i] & oitf[i] & disp_rs1en & oitf_rdwen[i] & (disp_rs1 == oitf_rd[i]);
    assign oitf_rs2_matchrd[i] = ~oitf_clr[i] & oitf[i] & disp_rs2en & oitf_rdwen[i] & (disp_rs2 == oitf_rd[i]);
    assign oitf_rd_matchrd[i]  = ~oitf_clr[i] & oitf[i] & disp_rdwen & oitf_rdwen[i] & (disp_rd  == oitf_rd[i]);
    assign wbck_rd_matchrd[i]  = ~oitf_clr[i] & oitf[i] & oitf_rdwen[i] & (wbu_dep_rd   == oitf_rd[i]);
    assign oitf_ifu_matchrd[i] = oitf[i] & oitf_rdwen[i] & (ifu_dep_rs1 == oitf_rd[i]);
  end
endgenerate
// ================================================================================================================================================
// OUTPUT SIGNAL
// ================================================================================================================================================
wire oitf_o_rs1_matchrd = | oitf_rs1_matchrd;
wire oitf_o_rs2_matchrd = | oitf_rs2_matchrd;
wire oitf_o_rd_matchrd  = (| oitf_rd_matchrd) & ~wptr[1];
wire wbck_o_rd_matchrd  = | wbck_rd_matchrd;
assign ifu_dep = | oitf_ifu_matchrd;
assign disp_dep = oitf_o_rs1_matchrd | oitf_o_rs2_matchrd | oitf_o_rd_matchrd;
assign wbck_dep = wbck_o_rd_matchrd;
assign longi_empty = ~(| oitf);
endmodule

module lieat_idu_disp(
  input       clock,
  input       reset,
  input       id_i_valid,
  output      id_i_ready,

  input [2:0] disp_op,
  input       disp_condition,
  input       flush_req,

  output      disp_com_valid,
  input       disp_com_ready,  
  output      disp_lsu_valid,
  input       disp_lsu_ready,
  output      disp_muldiv_valid,
  input       disp_muldiv_ready,
  output      longi_disp
);
// ================================================================================================================================================
// VALID-READY SHAKEHAND
// ================================================================================================================================================
wire id_i_sh = id_i_valid & id_i_ready;
wire id_o_sh = disp_com_sh | disp_lsu_sh | disp_muldiv_sh;
wire disp_com_sh = disp_com_valid & disp_com_ready;
wire disp_lsu_sh = disp_lsu_valid & disp_lsu_ready;
wire disp_muldiv_sh = disp_muldiv_valid & disp_muldiv_ready;
// ================================================================================================================================================
// OP_SEL
// ================================================================================================================================================
wire op_alu = (disp_op == 3'd0);
wire op_bjp = (disp_op == 3'd1);
wire op_lsu = (disp_op == 3'd2);
wire op_csr = (disp_op == 3'd3);
wire op_mul = (disp_op == 3'd4);
wire op_com = op_alu | op_bjp | op_csr;

wire disp_valid_pre;
wire disp_valid_set = id_i_sh;
wire disp_valid_clr = id_o_sh;
wire disp_valid_ena = disp_valid_set | disp_valid_clr | flush_req;
wire disp_valid_nxt = (disp_valid_set | (~disp_valid_clr)) & (~flush_req);
lieat_general_dfflr #(1) disp_valid_dff(clock,reset,disp_valid_ena,disp_valid_nxt,disp_valid_pre);
wire disp_valid = disp_valid_pre & disp_condition & (~flush_req);//case:valid but oitf_raw_dep
// ================================================================================================================================================
// OUTPUT SIGNAL
// ================================================================================================================================================
assign longi_disp = disp_lsu_sh | disp_muldiv_sh;
assign disp_com_valid = disp_valid & op_com;
assign disp_lsu_valid = disp_valid & op_lsu;
assign disp_muldiv_valid = disp_valid & op_mul;
assign id_i_ready = (~disp_valid_pre) | id_o_sh;
endmodule
module lieat_idu (
  input                 clock,
  input                 reset,
  
  input                 id_i_valid,
  output                id_i_ready,
  input  [`XLEN-1:0]    id_i_pc,
  input  [`XLEN-1:0]    id_i_inst,
  input                 id_i_prdt_taken,
  
  output                id_o_com_valid,
  input                 id_o_com_ready,
  output                id_o_lsu_valid,
  input                 id_o_lsu_ready,
  output                id_o_muldiv_valid,
  input                 id_o_muldiv_ready,
  output [`XLEN-1:0]    id_o_pc,
  output [`XLEN-1:0]    id_o_imm,
  output [`XLEN-1:0]    id_o_infobus,
  output [`REG_IDX-1:0] id_o_rs1,
  output [`REG_IDX-1:0] id_o_rs2,
  output [`REG_IDX-1:0] id_o_rd,
  output                id_o_rdwen,

  input  [`REG_IDX-1:0] jalr_rs1,
  input                 wait_for_ifetch,
  output                jalr_dep,

  input                 flush_req,
  input  [`REG_IDX-1:0] wbck_rd,
  output                wbck_dep,
  input                 longi_wbck,
  input  [1:0]          longi_wbck_op,
  output                longi_empty
);
wire unused_ok = &{dec_rv32,dec_ilgl};
// ================================================================================================================================================
// INPUT SIGNAL
// ================================================================================================================================================
wire id_i_sh = id_i_valid & id_i_ready;

wire [`XLEN-1:0] pc;
wire [`XLEN-1:0] inst;
wire             prdt_taken;
lieat_general_dfflr #(`XLEN) id_pc_dff(clock,reset,id_i_sh,id_i_pc,pc);
lieat_general_dfflr #(`XLEN) id_inst_dff(clock,reset,id_i_sh,id_i_inst,inst);
lieat_general_dfflr #(1)     id_prdt_taken_dff(clock,reset,id_i_sh,id_i_prdt_taken,prdt_taken);
// ================================================================================================================================================
// DECODE MODULE
// ================================================================================================================================================
wire                dec_rs1en;
wire                dec_rs2en;
wire                dec_rdwen;
wire [`REG_IDX-1:0] dec_rs1;
wire [`REG_IDX-1:0] dec_rs2;
wire [`REG_IDX-1:0] dec_rd;
wire [`XLEN-1:0]    dec_imm;
wire [`XLEN-1:0]    dec_infobus;
wire                dec_rv32;
wire                dec_ilgl;

lieat_idu_dec decode(
  .inst(inst),
  .prdt_taken(prdt_taken),
  .id_rv32(dec_rv32),
  .id_rs1en(dec_rs1en),
  .id_rs2en(dec_rs2en),
  .id_rdwen(dec_rdwen),
  .id_rs1(dec_rs1),
  .id_rs2(dec_rs2),
  .id_rd(dec_rd),
  .id_imm(dec_imm),
  .id_infobus(dec_infobus),
  .id_ilgl(dec_ilgl)
);
// ================================================================================================================================================
// LONG_INST OITF MODULE
// ================================================================================================================================================
wire longi_disp;
wire disp_dep;
wire disp_condition = (~disp_dep);

lieat_idu_depend depend(
  .clock(clock),
  .reset(reset),
  .disp_ena(longi_disp),
  .disp_op(disp_op),
  .disp_rs1en(dec_rs1en),
  .disp_rs2en(dec_rs2en),
  .disp_rdwen(dec_rdwen),
  .disp_rs1(dec_rs1),
  .disp_rs2(dec_rs2),
  .disp_rd(dec_rd),

  .remove_ena(longi_wbck),
  .remove_op(longi_wbck_op),

  .ifu_dep_rs1(jalr_rs1),
  .wbu_dep_rd(wbck_rd),
  .disp_dep(disp_dep),
  .wbck_dep(wbck_dep),
  .ifu_dep(ifu_longi_dep),
  .longi_empty(longi_empty)
);
// ================================================================================================================================================
// DISP MODULE
// ================================================================================================================================================
wire [2:0] disp_op = dec_infobus[`INFOBUS_OP];
lieat_idu_disp disp(
  .clock(clock),
  .reset(reset),

  .id_i_valid(id_i_valid),
  .id_i_ready(id_i_ready),
  .disp_op(disp_op),

  .disp_com_valid(id_o_com_valid),
  .disp_com_ready(id_o_com_ready),
  .disp_lsu_valid(id_o_lsu_valid),
  .disp_lsu_ready(id_o_lsu_ready),
  .disp_muldiv_valid(id_o_muldiv_valid),
  .disp_muldiv_ready(id_o_muldiv_ready),

  .longi_disp(longi_disp),
  .disp_condition(disp_condition),
  .flush_req(flush_req)
);
// ================================================================================================================================================
// OUTPUT SIGNAL
// ================================================================================================================================================
wire ifu_longi_dep;
wire jalr_id_dep  = (jalr_rs1 == dec_rd) & dec_rdwen & (id_o_com_valid | id_o_lsu_valid | id_o_muldiv_valid) & (~wait_for_ifetch);
assign jalr_dep = ifu_longi_dep | jalr_id_dep;

assign id_o_pc = pc;
assign id_o_imm = dec_imm;
assign id_o_infobus = dec_infobus;
assign id_o_rs1 = dec_rs1;
assign id_o_rs2 = dec_rs2;
assign id_o_rd = dec_rd;
assign id_o_rdwen = dec_rdwen;
endmodule
module lieat_ifu_agu(
input              clock,
input              reset,

input              rst_req,
input              flush_req,
input  [`XLEN-1:0] flush_pc,
input              ifetch_req,
input              bxx_taken,

input              inst_jal,
input              inst_jalr,
input              inst_bxx,
input              inst_ecall,
input              inst_mret,

input  [`XLEN-1:0] imm_branch,
input  [`XLEN-1:0] ifu_csr_rdata,

input              jalr_dep,
input  [`XLEN-1:0] jalr_src1,

output             jalr_need_wait,
output             prdt_taken,
output [`XLEN-1:0] pc
);
assign prdt_taken = (inst_jal | inst_jalr | bxx_taken | inst_mret | inst_ecall);
assign jalr_need_wait = inst_jalr & jalr_dep;

wire             pc_ena       = ifetch_req | rst_req | flush_req;
wire [`XLEN-1:0] pc_offset    = 32'd4;
wire [`XLEN-1:0] taken_pc_op1 = (inst_ecall | inst_mret) ? ifu_csr_rdata : (inst_jal | inst_bxx) ? pc : jalr_src1;
wire [`XLEN-1:0] taken_pc_op2 = (inst_ecall | inst_mret) ? 32'h0         : imm_branch;
wire [`XLEN-1:0] pc_op1       = prdt_taken ? taken_pc_op1  : pc;
wire [`XLEN-1:0] pc_op2       = prdt_taken ? taken_pc_op2  : pc_offset;
wire [`XLEN-1:0] pc_nxt_pre   = pc_op1 + pc_op2;
wire [`XLEN-1:0] pc_nxt       = rst_req    ? `PC_DEFAULT : 
                                flush_req  ? flush_pc    : 
                                ifetch_req ? (pc_nxt_pre & 32'hfffffffe) : 32'h0;
lieat_general_dfflr #(`XLEN) pc_dff (clock,reset,pc_ena,pc_nxt,pc);
endmodule
module lieat_ifu_bpudec(
  input  [`XLEN-1:0]    inst,
  output [`REG_IDX-1:0] rs1,
  output [`XLEN-1:0]    imm_branch,
  output                inst_jal,
  output                inst_jalr,
  output                inst_bxx,
  output                inst_ecall,
  output                inst_mret,
  output                inst_fencei
);
assign rs1 = inst[19:15];
wire [6:0] opcode   = inst[6:0];

wire opcode_6_5_11  = (opcode[6:5] == 2'b11);
wire opcode_6_5_00  = (opcode[6:5] == 2'b00);
wire opcode_4_2_000 = (opcode[4:2] == 3'b000);
wire opcode_4_2_001 = (opcode[4:2] == 3'b001);
wire opcode_4_2_011 = (opcode[4:2] == 3'b011);
wire opcode_4_2_100 = (opcode[4:2] == 3'b100);
wire opcode_1_0_11  = (opcode[1:0] == 2'b11);

assign inst_jal     = opcode_6_5_11 & opcode_4_2_011 & opcode_1_0_11;
assign inst_jalr    = opcode_6_5_11 & opcode_4_2_001 & opcode_1_0_11;
assign inst_bxx     = opcode_6_5_11 & opcode_4_2_000 & opcode_1_0_11;//beq bne blt bge bltu bgeu
assign inst_ecall   = opcode_6_5_11 & opcode_4_2_100 & opcode_1_0_11 & (inst[14:12] == 3'b000) & (inst[31:20] == 12'b000000000000);
assign inst_mret    = opcode_6_5_11 & opcode_4_2_100 & opcode_1_0_11 & (inst[14:12] == 3'b000) & (inst[31:20] == 12'b001100000010);
assign inst_fencei  = opcode_6_5_00 & opcode_4_2_011 & opcode_1_0_11 & (inst[14:12] == 3'b001);

assign imm_branch   = (inst_jal)  ? imm_jal  :
                      (inst_jalr) ? imm_jalr :
                      (inst_bxx)  ? imm_bxx  : 32'h0;

wire [`XLEN-1:0] imm_jal = {{12{inst[31]}},inst[19:12],inst[20],inst[30:21],1'b0};
wire [`XLEN-1:0] imm_jalr= {{20{inst[31]}},inst[31:20]};
wire [`XLEN-1:0] imm_bxx = {{20{inst[31]}},inst[7],inst[30:25],inst[11:8],1'b0};

endmodule
module lieat_ifu_bpuprdt # (
  parameter INDEX_NUM  = 32,
  parameter BHR_SIZE   = 2,
  parameter PHT_SIZE   = 4
)(
  input       clock,
  input       reset,
  
  input [4:0] index,
  input       inst_bxx,
  output      bxx_taken,

  input       callback_result,
  input [4:0] callback_index,
  input       callback_en
);

reg  [BHR_SIZE-1:0] branch_history_table[INDEX_NUM-1:0];//BHR
wire [BHR_SIZE-1:0] branch_history = branch_history_table[index];
wire [BHR_SIZE-1:0] callback_result_history = branch_history_table[callback_index];

reg [1:0] pattern_history_table[INDEX_NUM-1:0][PHT_SIZE-1:0];//PHT

always@(posedge clock or posedge reset) begin
  if(reset)begin
    for(int i = 0;i < INDEX_NUM; i = i + 1)begin
      for(int j = 0; j < PHT_SIZE; j = j + 1)begin
        pattern_history_table[i][j] <= 2'b01;
      end
      branch_history_table[i] <= 2'b00;
    end
  end
  else if(callback_en) begin
    case (pattern_history_table[callback_index][callback_result_history])
    2'b00: pattern_history_table[callback_index][callback_result_history] <= (callback_result) ? 2'b01 : 2'b00;
    2'b01: pattern_history_table[callback_index][callback_result_history] <= (callback_result) ? 2'b10 : 2'b00;
    2'b10: pattern_history_table[callback_index][callback_result_history] <= (callback_result) ? 2'b11 : 2'b01;
    2'b11: pattern_history_table[callback_index][callback_result_history] <= (callback_result) ? 2'b11 : 2'b10;
    endcase
    branch_history_table[callback_index] <= {branch_history[0],callback_result};
  end
end
assign bxx_taken = inst_bxx & pattern_history_table[index][branch_history][1];
endmodule
module lieat_ifu_bpu(
  input                 clock,
  input                 reset,

  output [`XLEN-1:0]    pc,
  input  [`XLEN-1:0]    inst,
  output                prdt_taken,

  input                 ifetch_req,
  input                 rst_req,
  input                 flush_req,
  input  [`XLEN-1:0]    flush_pc,

  output [`REG_IDX-1:0] jalr_rs1,
  input  [`XLEN-1:0]    jalr_src1,
  input                 jalr_dep,
  output                jalr_need_wait,
  output                need_fencei,

  input                 bxx_callback_result,
  input  [4:0]          bxx_callback_index,
  input                 bxx_callback_en,

  output                ifu_csr_ren,
  output [11:0]         ifu_csr_idx,
  input  [`XLEN-1:0]    ifu_csr_rdata
);
// ================================================================================================================================================
// DECODE MODULE
// ================================================================================================================================================
wire inst_jal;
wire inst_jalr;
wire inst_bxx;
wire inst_ecall;
wire inst_mret;
wire [`XLEN-1:0] imm_branch;

lieat_ifu_bpudec bpudec(
  .inst(inst),
  .rs1(jalr_rs1),
  .imm_branch(imm_branch),
  .inst_jal(inst_jal),
  .inst_jalr(inst_jalr),
  .inst_bxx(inst_bxx),
  .inst_ecall(inst_ecall),
  .inst_mret(inst_mret),
  .inst_fencei(need_fencei)
);
assign ifu_csr_ren = inst_ecall | inst_mret;
assign ifu_csr_idx = inst_ecall ? 12'h305 : inst_mret ? 12'h341 : 12'h0;
// ================================================================================================================================================
//  PRDT MODULE
// ================================================================================================================================================
wire bxx_taken;
wire [4:0] pc_index = pc[6:2];

lieat_ifu_bpuprdt bpuprdt(
  .clock(clock),
  .reset(reset),
  .index(pc_index),
  .inst_bxx(inst_bxx),
  .bxx_taken(bxx_taken),
  .callback_result(bxx_callback_result),
  .callback_index(bxx_callback_index),
  .callback_en(bxx_callback_en)
);

lieat_ifu_agu agu(
  .clock(clock),
  .reset(reset),
  .rst_req(rst_req),
  .flush_req(flush_req),
  .flush_pc(flush_pc),
  .ifetch_req(ifetch_req),
  .bxx_taken(bxx_taken),

  .inst_jal(inst_jal),
  .inst_jalr(inst_jalr),
  .inst_bxx(inst_bxx),
  .inst_ecall(inst_ecall),
  .inst_mret(inst_mret),

  .jalr_dep(jalr_dep),
  .jalr_src1(jalr_src1),
  .imm_branch(imm_branch),
  .ifu_csr_rdata(ifu_csr_rdata),

  .jalr_need_wait(jalr_need_wait),
  .prdt_taken(prdt_taken),
  .pc(pc)
);
endmodule
module lieat_ifu_icache # (
  parameter CACHE_WAY  = 2,//cache has 2 ways: 1KB
  parameter INDEX_LEN  = 6, //each way has 64 blocks
  parameter CACHE_SIZE = 64,
  parameter TAG_WIDTH  = 24,
  parameter OFFSET_LEN = 2 //each block has 8 bytes
)(
  input               clock,
  input               reset,

  input               flush_req,
  input               fencei_req,
  
  input               ifetch_req_valid,
  output              ifetch_req_ready,
  input  [`XLEN-1:0]  ifetch_req_pc,

  output              ifetch_rsp_valid,
  input               ifetch_rsp_ready,
  output [`XLEN-1:0]  ifetch_rsp_inst,

  output [`XLEN-1:0]  icache_axi_araddr,
  output              icache_axi_arvalid,
  input               icache_axi_arready,

  input  [`XLEN*2-1:0]icache_axi_rdata,
  input               icache_axi_rvalid,
  output              icache_axi_rready
);
wire ar_sh  = icache_axi_arvalid & icache_axi_arready;
wire r_sh   = icache_axi_rvalid & icache_axi_rready;
wire rsp_sh = ifetch_rsp_valid & ifetch_rsp_ready;
// ================================================================================================================================================
// STATE CONTROL
// ================================================================================================================================================
reg [5:0] state_r;
reg [5:0] state_nxt;
localparam STATE_IDE = 6'b000001;
localparam STATE_AR  = 6'b000010;
localparam STATE_R   = 6'b000100;
localparam STATE_ARF = 6'b001000;
localparam STATE_RF  = 6'b010000;
localparam STATE_VLD = 6'b100000;

localparam STATE_IDE_BIT = 0;
localparam STATE_AR_BIT  = 1;
localparam STATE_R_BIT   = 2;
localparam STATE_ARF_BIT = 3;
localparam STATE_RF_BIT  = 4;
localparam STATE_VLD_BIT = 5;

always @(*) begin
  case(state_r)
    STATE_IDE: state_nxt = rsp_sh                           	? STATE_IDE   :
                           (hit & ifetch_req_valid & flush_req) ? STATE_IDE   :
                           (hit & ifetch_req_valid) 		? STATE_VLD   :
                           (ar_sh & flush_req)              	? STATE_RF    :
                           (icache_axi_arvalid & flush_req) 	? STATE_ARF   :
                           ar_sh                            	? STATE_R     :
                           icache_axi_arvalid               	? STATE_AR    : STATE_IDE;
    STATE_AR : state_nxt = (ar_sh & flush_req)              	? STATE_RF    :
                           (icache_axi_arvalid & flush_req) 	? STATE_ARF   :
                           ar_sh                            	? STATE_R     : STATE_AR;
    STATE_R  : state_nxt = (flush_req & icache_axi_rvalid)  	? STATE_IDE   :
                           (flush_req)                      	? STATE_RF    : 
                           (icache_axi_rvalid & rsp_sh)     	? STATE_IDE   :
                           (icache_axi_rvalid)              	? STATE_VLD   : STATE_R;
    STATE_ARF: state_nxt = ar_sh                            	? STATE_RF    : STATE_ARF;
    STATE_RF : state_nxt = icache_axi_rvalid                	? STATE_IDE   : STATE_RF;
    STATE_VLD: state_nxt = flush_req                        	? STATE_IDE   :
                           rsp_sh                           	? STATE_IDE   : STATE_VLD;
    default:state_nxt    = STATE_IDE;
  endcase
end

lieat_general_dffrd #(
  .DW(6),
  .DEFAULT(STATE_IDE)
) icache_state(clock,reset,state_nxt,state_r);
// ================================================================================================================================================
// CHANNEL: INTERACTIVE WITH IFETCH
// ================================================================================================================================================
assign ifetch_rsp_valid = (state_r[STATE_R_BIT] & icache_axi_rvalid & ~flush_req) |
                          (state_r[STATE_VLD_BIT] & ~flush_req)                   |
                          (hit & ifetch_req_valid & ~flush_req);
assign ifetch_req_ready = ((state_r[STATE_IDE_BIT]) | rsp_sh);
assign ifetch_rsp_inst  = (hit_sel0)          ? icache_sram_rdata[31:0]  :
                          (hit_sel1)          ? icache_sram_rdata[63:32] :
                          (icache_axi_rvalid) ? icache_axi_rdata[31:0]   : 32'h0;
// ================================================================================================================================================
// CHANNEL: INTERACTIVE WITH DRAM
// ================================================================================================================================================
assign icache_axi_arvalid = (state_r[STATE_IDE_BIT] & miss & ifetch_req_valid & ~flush_req) |
                            (state_r[STATE_AR_BIT])                                         |
                            (state_r[STATE_ARF_BIT]);
assign icache_axi_araddr  = (icache_axi_arvalid) ? ifetch_req_pc : 32'h0;
assign icache_axi_rready  = 1'b1;
// ================================================================================================================================================
// CACHE MODULE
// ================================================================================================================================================
wire [TAG_WIDTH-1:0] tag      = ifetch_req_pc[`XLEN-1:OFFSET_LEN+INDEX_LEN];
wire [INDEX_LEN-1:0] index    = ifetch_req_pc[OFFSET_LEN+INDEX_LEN-1:OFFSET_LEN];

reg [TAG_WIDTH+4:0] cache_tags_lru_vld [CACHE_SIZE-1:0][CACHE_WAY-1:0];;//TAG_WIDTH + VALID_BIT1 + LRU_BIT 4

wire hit_sel0 = (cache_tags_lru_vld[index][0][TAG_WIDTH+4:4] == {tag,1'b1});
wire hit_sel1 = (cache_tags_lru_vld[index][1][TAG_WIDTH+4:4] == {tag,1'b1});
wire hit  = hit_sel0 | hit_sel1;
wire miss_sel1 = cache_tags_lru_vld[index][0][3:0] < cache_tags_lru_vld[index][1][3:0];
wire miss = ~hit;

always@(posedge clock or posedge reset) begin
  if(reset)begin
    for(int i = 0;i < CACHE_SIZE; i = i + 1)begin
      for(int j = 0; j < CACHE_WAY; j = j + 1)begin
        cache_tags_lru_vld[i][j] <= 0;
      end
    end
  end
  else if(fencei_req) begin
    for(int i = 0;i < CACHE_SIZE; i = i + 1)begin
      for(int j = 0; j < CACHE_WAY; j = j + 1)begin
        cache_tags_lru_vld[i][j] <= 0;
      end
    end
  end
  else if(hit & ifetch_req_valid) begin
    cache_tags_lru_vld[index][hit_sel1][3:0] <= 4'b0;
    cache_tags_lru_vld[index][~hit_sel1][3:0] <= (cache_tags_lru_vld[index][~hit_sel1][3:0] == 4'b1111) ? 4'b1111 : (cache_tags_lru_vld[index][~hit_sel1][3:0] + 1);
  end
  else if(icache_axi_rvalid & state_r[STATE_R_BIT] & ~flush_req) begin
    cache_tags_lru_vld[index][miss_sel1] <= {tag,5'b10000};
    cache_tags_lru_vld[index][~miss_sel1][3:0] <= cache_tags_lru_vld[index][~miss_sel1][3:0]+1;
  end
end
wire icache_sram_reset = reset | fencei_req;
wire icache_sram_cen = 1'b1;
wire icache_sram_wen = icache_axi_rvalid & state_r[STATE_R_BIT] & ~flush_req;
wire [ 5:0] icache_sram_addr= index;
wire [63:0] icache_sram_data= miss_sel1 ? {icache_axi_rdata[31:0],icache_sram_rdata[31:0]} : {icache_sram_rdata[63:32],icache_axi_rdata[31:0]};
wire [63:0] icache_sram_rdata;
lieat_general_64x64_sram icache_sram(clock,icache_sram_reset,icache_sram_cen,icache_sram_wen,icache_sram_addr,icache_sram_data,icache_sram_rdata);
endmodule
module lieat_ifu(
  input                 clock,
  input                 reset,
  //TO IDU
  output                if_o_valid,
  input                 if_o_ready,
  output [`XLEN-1:0]    if_o_pc,
  output [`XLEN-1:0]    if_o_inst,
  output                if_o_prdt_taken,
  //TO DRAM
  output                icache_axi_arvalid,
  input                 icache_axi_arready,
  output [`XLEN-1:0]    icache_axi_araddr,
  input                 icache_axi_rvalid,
  output                icache_axi_rready,
  input  [63:0]         icache_axi_rdata,
  //FROM EXU:branch callback
  input                 callback_en,
  input  [4:0]          callback_index,
  input                 callback_result,
  input                 callback_flush,
  input  [`XLEN-1:0]    callback_truepc,
  input                 callback_fencei,
  //TO EXU:csr write
  output                ifu_csr_ren,
  output [11:0]         ifu_csr_idx,
  input  [`XLEN-1:0]    ifu_csr_rdata,
  //FROM REGFILE:jalr fetch src1
  output [`REG_IDX-1:0] jalr_rs1,
  input  [`XLEN-1:0]    jalr_src1,
  input                 jalr_dep,
  output                wait_for_ifetch
);
// ================================================================================================================================================
// VALID-READY HANDSHAKE
// ================================================================================================================================================
wire ifetch_req_sh = ifetch_req_valid & ifetch_req_ready;
wire if_o_sh       = if_o_valid & if_o_ready;
// ================================================================================================================================================
// WAIT FOR CONDITION:FENCEI OR SRC1
// ================================================================================================================================================
wire fencei_req;//when ifu fetch fencei,next inst cant be fetched until exu write back fencei
wire jalr_need_wait;//wait for src1
wire fencei_need_wait;
wire fencei_need_wait_set = fencei_req & if_o_sh;
wire fencei_need_wait_clr = callback_fencei;
wire fencei_need_wait_ena = fencei_need_wait_set | fencei_need_wait_clr;
wire fencei_need_wait_nxt = fencei_need_wait_set | ~fencei_need_wait_clr;
lieat_general_dfflr #(1) fencei_need_wait_dff(clock,reset,fencei_need_wait_ena,fencei_need_wait_nxt,fencei_need_wait);

wire ifu_need_wait       = jalr_need_wait | fencei_need_wait | fencei_req;
wire wait_for_ifetch_set = ifu_need_wait & if_o_sh;//case: output to IDU when waiting
wire wait_for_ifetch_clr = wait_for_ifetch & ~ifu_need_wait;//call once
wire wait_for_ifetch_ena = wait_for_ifetch_set | wait_for_ifetch_clr;
wire wait_for_ifetch_nxt = wait_for_ifetch_set | (~wait_for_ifetch_clr);
lieat_general_dfflr #(1) wait_for_ifetch_dff (clock,reset,wait_for_ifetch_ena,wait_for_ifetch_nxt,wait_for_ifetch);
// ================================================================================================================================================
// FETCH PC
// ================================================================================================================================================
wire rst_req;
lieat_general_dffrs #(1) rst_req_dffrs(clock,reset,1'b0,rst_req);
wire flush_req    = callback_flush;
wire ifetch_req   = (if_o_sh | wait_for_ifetch) & ~ifu_need_wait;

lieat_ifu_bpu bpu(
  .clock(clock),
  .reset(reset),

  .pc(pc),
  .inst(inst),
  .prdt_taken(prdt_taken),

  .jalr_rs1(jalr_rs1),
  .jalr_src1(jalr_src1),
  .jalr_dep(jalr_dep),
  .jalr_need_wait(jalr_need_wait),

  .ifetch_req(ifetch_req),
  .rst_req(rst_req),
  .flush_req(callback_flush),
  .flush_pc(callback_truepc),

  .bxx_callback_result(callback_result),
  .bxx_callback_index(callback_index),
  .bxx_callback_en(callback_en),

  .ifu_csr_ren(ifu_csr_ren),
  .ifu_csr_idx(ifu_csr_idx),
  .ifu_csr_rdata(ifu_csr_rdata),

  .need_fencei(fencei_req)
);
// ================================================================================================================================================
// FETCH INST
// ================================================================================================================================================
wire ifetch_req_valid;
wire ifetch_req_ready;
wire ifetch_req_valid_set = rst_req | flush_req | ifetch_req;
wire ifetch_req_valid_clr = ifetch_req_sh;
wire ifetch_req_valid_ena = ifetch_req_valid_set | ifetch_req_valid_clr;
wire ifetch_req_valid_nxt = ifetch_req_valid_set | (~ifetch_req_valid_clr);
lieat_general_dfflr #(1) ifetch_valid_req_dff (clock,reset,ifetch_req_valid_ena,ifetch_req_valid_nxt,ifetch_req_valid);

lieat_ifu_icache icache(
  .clock(clock),
  .reset(reset),
  .flush_req(flush_req),
  .fencei_req(fencei_req),

  .ifetch_req_valid(ifetch_req_valid),
  .ifetch_req_ready(ifetch_req_ready),
  .ifetch_req_pc(pc),

  .ifetch_rsp_valid(if_o_valid),
  .ifetch_rsp_ready(if_o_ready),
  .ifetch_rsp_inst(inst),

  .icache_axi_arvalid(icache_axi_arvalid),
  .icache_axi_arready(icache_axi_arready),
  .icache_axi_araddr(icache_axi_araddr),

  .icache_axi_rvalid(icache_axi_rvalid),
  .icache_axi_rready(icache_axi_rready),
  .icache_axi_rdata(icache_axi_rdata)
);
// ================================================================================================================================================
// OUTPUT SINGAL
// ================================================================================================================================================
wire [`XLEN-1:0] pc;
wire [`XLEN-1:0] inst;
wire             prdt_taken;
assign if_o_pc   = pc;
assign if_o_inst = inst;
assign if_o_prdt_taken = prdt_taken;
endmodule
module lieat_regfile(
  input                 clock,
  input                 reset,

  input  [`REG_IDX-1:0] ifu_rs1,
  output  [`XLEN-1:0]   ifu_src1,

  input  [`REG_IDX-1:0] exu_rs1,
  input  [`REG_IDX-1:0] exu_rs2,
  output [`XLEN-1:0]    exu_src1,
  output [`XLEN-1:0]    exu_src2,
  
  input  [`XLEN-1:0]    wb_pc,
  input                 wb_valid,
  input                 wb_en,
  input  [`REG_IDX-1:0] wb_rd,
  input  [`XLEN-1:0]    wb_data,
  input                 wb_lsu,//DIFFTEST
  input                 wb_ebreak,
  input                 longi_empty//DIFFTEST
);
wire [`XLEN-1:0] regs [`RGIDX_NUM-1:0];
wire [`RGIDX_NUM-1:0] reg_wen;

genvar i;
generate
  for(i = 0; i < `RGIDX_NUM; i = i + 1) begin
    if(i == 0) begin
      assign reg_wen[i] = 1'b0;
      assign regs[i] = (`XLEN'b0 & {`XLEN{reg_wen[i]}});
    end
    else begin
      assign reg_wen[i] = (wb_rd == i) & wb_en;
      lieat_general_dfflr #(`XLEN) regfile (clock,reset,reg_wen[i],wb_data,regs[i]); 
    end
  end
endgenerate

assign ifu_src1 = regs[ifu_rs1];
assign exu_src1 = regs[exu_rs1];
assign exu_src2 = regs[exu_rs2];

wire diff_ena;
wire diff_skip;
wire diff_sync;
wire diff_valid = diff_ena & wb_valid;
wire diff_ebreak;
wire [`XLEN-1:0] diff_pc;
wire [`XLEN-1:0] diff_sync_data;
wire [`REG_IDX-1:0] diff_sync_rd;

lieat_general_dffr  #(1) diff_valid_delay(clock,reset,wb_valid | diff_ena,diff_ena);
lieat_general_dfflr #(1) diff_skip_delay(clock,reset,diff_valid,~longi_empty,diff_skip);
lieat_general_dfflr #(1) diff_sync_delay(clock,reset,diff_valid,wb_lsu,diff_sync);
lieat_general_dfflr #(1) diff_ebreak_delay(clock,reset,diff_valid,wb_ebreak,diff_ebreak);
lieat_general_dfflr #(`REG_IDX) diff_sync_rd_delay(clock,reset,diff_valid,wb_rd,diff_sync_rd);
lieat_general_dfflr #(`XLEN) diff_sync_data_delay(clock,reset,diff_valid,wb_data,diff_sync_data);
lieat_general_dfflr #(`XLEN) diff_pc_delay(clock,reset,wb_valid,wb_pc,diff_pc);

import "DPI-C" function void difftest_dut_regs(input int diff_pc, input int z0, input int ra, input int sp, input int gp, input int tp, input int t0, input int t1, input int t2,
                                              input int fp, input int s1, input int a0, input int a1, input int a2, input int a3, input int a4, input int a5, 
                                              input int a6, input int a7, input int s2, input int s3, input int s4, input int s5, input int s6, input int s7, 
                                              input int s8, input int s9, input int s10, input int a11, input int t3, input int t4, input int t5, input int t6);

import "DPI-C" function void difftest_dut_sync(input int diff_tag,input int diff_sync_data);
import "DPI-C" function void ebreak();
always @(posedge clock or posedge reset) begin
  difftest_dut_regs(diff_pc,regs[0],regs[1],regs[2],regs[3],regs[4],regs[5],regs[6],regs[7],
                    regs[8],regs[9],regs[10],regs[11],regs[12],regs[13],regs[14],regs[15],
                    regs[16],regs[17],regs[18],regs[19],regs[20],regs[21],regs[22],regs[23],
                    regs[24],regs[25],regs[26],regs[27],regs[28],regs[29],regs[30],regs[31]);
  difftest_dut_sync({24'b0,diff_valid,diff_skip,diff_sync,diff_sync_rd},diff_sync_data);
  if(diff_ebreak) ebreak();
end
endmodule

module ysyx_22040000(
  input         clock,
  input         reset,
  input         io_interrupt,
  input         io_master_awready,
  output        io_master_awvalid,
  output [3:0]  io_master_awid,
  output [31:0] io_master_awaddr,
  output [7:0]  io_master_awlen,
  output [2:0]  io_master_awsize,
  output [1:0]  io_master_awburst,

  input         io_master_wready,
  output        io_master_wvalid,
  output [63:0] io_master_wdata,
  output [7:0]  io_master_wstrb,
  output        io_master_wlast,

  output        io_master_bready,
  input         io_master_bvalid,
  input  [3:0]  io_master_bid,
  input  [1:0]  io_master_bresp,

  input         io_master_arready,
  output        io_master_arvalid,
  output [3:0]  io_master_arid,
  output [31:0] io_master_araddr,
  output [7:0]  io_master_arlen,
  output [2:0]  io_master_arsize,
  output [1:0]  io_master_arburst,

  output        io_master_rready,
  input         io_master_rvalid,
  input  [3:0]  io_master_rid,
  input  [1:0]  io_master_rresp,
  input  [63:0] io_master_rdata,
  input         io_master_rlast,

  output        io_slave_awready,
  input         io_slave_awvalid,
  input [3:0]   io_slave_awid,
  input [31:0]  io_slave_awaddr,
  input [7:0]   io_slave_awlen,
  input [2:0]   io_slave_awsize,
  input [1:0]   io_slave_awburst,

  output        io_slave_wready,
  input         io_slave_wvalid,
  input [63:0]  io_slave_wdata,
  input [7:0]   io_slave_wstrb,
  input         io_slave_wlast,

  input         io_slave_bready,
  output        io_slave_bvalid,
  output [3:0]  io_slave_bid,
  output [1:0]  io_slave_bresp,

  output        io_slave_arready,
  input         io_slave_arvalid,
  input [3:0]   io_slave_arid,
  input [31:0]  io_slave_araddr,
  input [7:0]   io_slave_arlen,
  input [2:0]   io_slave_arsize,
  input [1:0]   io_slave_arburst,

  input         io_slave_rready,
  output        io_slave_rvalid,
  output [3:0]  io_slave_rid,
  output [1:0]  io_slave_rresp,
  output [63:0] io_slave_rdata,
  output        io_slave_rlast,

  output [5:0]  io_sram0_addr,
  output        io_sram0_cen,
  output        io_sram0_wen,
  output [127:0]io_sram0_wmask,
  output [127:0]io_sram0_wdata,
  input  [127:0]io_sram0_rdata,
  output [5:0]  io_sram1_addr,
  output        io_sram1_cen,
  output        io_sram1_wen,
  output [127:0]io_sram1_wmask,
  output [127:0]io_sram1_wdata,
  input  [127:0]io_sram1_rdata,
  output [5:0]  io_sram2_addr,
  output        io_sram2_cen,
  output        io_sram2_wen,
  output [127:0]io_sram2_wmask,
  output [127:0]io_sram2_wdata,
  input  [127:0]io_sram2_rdata,
  output [5:0]  io_sram3_addr,
  output        io_sram3_cen,
  output        io_sram3_wen,
  output [127:0]io_sram3_wmask,
  output [127:0]io_sram3_wdata,
  input  [127:0]io_sram3_rdata,
  output [5:0]  io_sram4_addr,
  output        io_sram4_cen,
  output        io_sram4_wen,
  output [127:0]io_sram4_wmask,
  output [127:0]io_sram4_wdata,
  input  [127:0]io_sram4_rdata,
  output [5:0]  io_sram5_addr,
  output        io_sram5_cen,
  output        io_sram5_wen,
  output [127:0]io_sram5_wmask,
  output [127:0]io_sram5_wdata,
  input  [127:0]io_sram5_rdata,
  output [5:0]  io_sram6_addr,
  output        io_sram6_cen,
  output        io_sram6_wen,
  output [127:0]io_sram6_wmask,
  output [127:0]io_sram6_wdata,
  input  [127:0]io_sram6_rdata,
  output [5:0]  io_sram7_addr,
  output        io_sram7_cen,
  output        io_sram7_wen,
  output [127:0]io_sram7_wmask,
  output [127:0]io_sram7_wdata,
  input  [127:0]io_sram7_rdata
);
assign io_slave_awready = 1'b0;
assign io_slave_wready = 1'b0;
assign io_slave_bvalid = 1'b0;
assign io_slave_bid = 4'b0;
assign io_slave_bresp = 2'b0;
assign io_slave_arready = 1'b0;
assign io_slave_rvalid = 1'b0;
assign io_slave_rid = 4'b0;
assign io_slave_rresp = 2'b0;
assign io_slave_rdata = 64'b0;
assign io_slave_rlast = 1'b0;

assign io_sram0_addr = 6'b0;
assign io_sram1_addr = 6'b0;
assign io_sram2_addr = 6'b0;
assign io_sram3_addr = 6'b0;
assign io_sram4_addr = 6'b0;
assign io_sram5_addr = 6'b0;
assign io_sram6_addr = 6'b0;
assign io_sram7_addr = 6'b0;
assign io_sram0_cen  = 1'b0;
assign io_sram1_cen  = 1'b0;
assign io_sram2_cen  = 1'b0;
assign io_sram3_cen  = 1'b0;
assign io_sram4_cen  = 1'b0;
assign io_sram5_cen  = 1'b0;
assign io_sram6_cen  = 1'b0;
assign io_sram7_cen  = 1'b0;
assign io_sram0_wen  = 1'b0;
assign io_sram1_wen  = 1'b0;
assign io_sram2_wen  = 1'b0;
assign io_sram3_wen  = 1'b0;
assign io_sram4_wen  = 1'b0;
assign io_sram5_wen  = 1'b0;
assign io_sram6_wen  = 1'b0;
assign io_sram7_wen  = 1'b0;
assign io_sram0_wmask = 128'b0;
assign io_sram1_wmask = 128'b0;
assign io_sram2_wmask = 128'b0;
assign io_sram3_wmask = 128'b0;
assign io_sram4_wmask = 128'b0;
assign io_sram5_wmask = 128'b0;
assign io_sram6_wmask = 128'b0;
assign io_sram7_wmask = 128'b0;
assign io_sram0_wdata = 128'b0;
assign io_sram1_wdata = 128'b0;
assign io_sram2_wdata = 128'b0;
assign io_sram3_wdata = 128'b0;
assign io_sram4_wdata = 128'b0;
assign io_sram5_wdata = 128'b0;
assign io_sram6_wdata = 128'b0;
assign io_sram7_wdata = 128'b0;
wire unused_ok =&{io_slave_awvalid,io_slave_awid,io_slave_awaddr,io_slave_awlen,io_slave_awsize,io_slave_awburst,io_slave_wvalid,io_slave_wdata,io_slave_wstrb,io_slave_wlast,io_slave_bready,io_slave_arvalid,io_slave_arid,io_slave_araddr,io_slave_arlen,io_slave_arsize,io_slave_arburst,io_slave_rready,
io_sram0_rdata,io_sram1_rdata,io_sram2_rdata,io_sram3_rdata,io_sram4_rdata,io_sram5_rdata,io_sram6_rdata,io_sram7_rdata};
// ================================================================================================================================================
// IFU
// ================================================================================================================================================
wire             if_o_valid;
wire             if_o_ready;
wire [`XLEN-1:0] if_o_pc;
wire [`XLEN-1:0] if_o_inst;
wire             if_o_prdt_taken;

wire             ifu_csr_ren;
wire [11:0]      ifu_csr_idx;
wire [`XLEN-1:0] ifu_csr_rdata;
wire             unused_ok = io_interrupt;
lieat_ifu ifu(
  .clock(clock),
  .reset(reset),
  //TO IDU
  .if_o_valid(if_o_valid),
  .if_o_ready(if_o_ready),
  .if_o_pc(if_o_pc),
  .if_o_inst(if_o_inst),
  .if_o_prdt_taken(if_o_prdt_taken),
  //TO DRAM
  .icache_axi_arvalid(icache_axi_arvalid),
  .icache_axi_arready(icache_axi_arready),
  .icache_axi_araddr(icache_axi_araddr),
  .icache_axi_rvalid(icache_axi_rvalid),
  .icache_axi_rready(icache_axi_rready),
  .icache_axi_rdata(icache_axi_rdata),
  //FROM EXU:branch callback
  .callback_en(callback_en),
  .callback_index(callback_index),
  .callback_result(callback_result),
  .callback_flush(callback_flush),
  .callback_truepc(callback_truepc),
  .callback_fencei(callback_fenceifinish),
  //TO EXU:csr write
  .ifu_csr_ren(ifu_csr_ren),
  .ifu_csr_idx(ifu_csr_idx),
  .ifu_csr_rdata(ifu_csr_rdata),
  //FROM REGFILE:jalr fetch src1
  .jalr_rs1(if_reg_rs1),
  .jalr_src1(jalr_src1),
  .jalr_dep(jalr_dep),
  .wait_for_ifetch(wait_for_ifetch)
);
// ================================================================================================================================================
// IDU
// ================================================================================================================================================
wire                id_o_com_valid;
wire                id_o_com_ready;
wire                id_o_lsu_valid;
wire                id_o_lsu_ready;
wire                id_o_muldiv_valid;
wire                id_o_muldiv_ready;

wire [`XLEN-1:0]    id_o_pc;
wire [`XLEN-1:0]    id_o_imm;
wire [`XLEN-1:0]    id_o_infobus;
wire [`REG_IDX-1:0] id_o_rs1;
wire [`REG_IDX-1:0] id_o_rs2;
wire [`REG_IDX-1:0] id_o_rd;
wire                id_o_rdwen;

wire                jalr_dep;
wire                wait_for_ifetch;

lieat_idu idu(
  .clock(clock),
  .reset(reset),
  //FROM IFU
  .id_i_valid(if_o_valid),
  .id_i_ready(if_o_ready),
  .id_i_pc(if_o_pc),
  .id_i_inst(if_o_inst),
  .id_i_prdt_taken(if_o_prdt_taken),
  //TO EXU
  .id_o_com_valid(id_o_com_valid),
  .id_o_com_ready(id_o_com_ready),
  .id_o_lsu_valid(id_o_lsu_valid),
  .id_o_lsu_ready(id_o_lsu_ready),
  .id_o_muldiv_valid(id_o_muldiv_valid),
  .id_o_muldiv_ready(id_o_muldiv_ready),
  .id_o_pc(id_o_pc),
  .id_o_imm(id_o_imm),
  .id_o_infobus(id_o_infobus),
  .id_o_rs1(id_o_rs1),
  .id_o_rs2(id_o_rs2),
  .id_o_rd(id_o_rd),
  .id_o_rdwen(id_o_rdwen),
  //FROM EXU:flush
  .jalr_rs1(if_reg_rs1),
  .wait_for_ifetch(wait_for_ifetch),
  .jalr_dep(jalr_dep),
  .flush_req(callback_flush),
  //FROM EXU:depend record
  .wbck_dep(wbck_dep),
  .wbck_rd(wbck_o_rd),
  .longi_wbck(longi_wbck),
  .longi_wbck_op(longi_wbck_op),
  .longi_empty(longi_empty)
);
// ================================================================================================================================================
// EXU
// ================================================================================================================================================
wire                wbck_dep;
wire                wbck_o_valid;
wire                wbck_o_en;
wire [`XLEN-1:0]    wbck_o_pc;
wire [`REG_IDX-1:0] wbck_o_rd;
wire [`XLEN-1:0]    wbck_o_data;
wire                wbck_o_lsu;
wire                wbck_o_ebreak;

wire                callback_en;
wire [4:0]          callback_index;
wire                callback_result;
wire                callback_flush;
wire [`XLEN-1:0]    callback_truepc;
wire                callback_fenceifinish;

wire                longi_empty;
wire                longi_wbck;
wire [1:0]          longi_wbck_op;

wire [`XLEN-1:0]    jalr_src1;
lieat_exu exu(
  .clock(clock),
  .reset(reset),
  //FROM IDU
  .ex_i_pc(id_o_pc),
  .ex_i_imm(id_o_imm),
  .ex_i_infobus(id_o_infobus),
  .ex_i_rs1(id_o_rs1),
  .ex_i_rs2(id_o_rs2),
  .ex_i_rd(id_o_rd),
  .ex_i_rdwen(id_o_rdwen),
  //INTERACT REGFILE:FETCH SRC
  .ex_reg_rs1(ex_reg_rs1),
  .ex_reg_rs2(ex_reg_rs2),
  .ex_reg_src1(ex_reg_src1),
  .ex_reg_src2(ex_reg_src2),
  //FROM IDU
  .com_i_valid(id_o_com_valid),
  .com_i_ready(id_o_com_ready),
  .lsu_i_valid(id_o_lsu_valid),
  .lsu_i_ready(id_o_lsu_ready),
  .muldiv_i_valid(id_o_muldiv_valid),
  .muldiv_i_ready(id_o_muldiv_ready),
  //WBCK
  .wbck_o_valid(wbck_o_valid),
  .wbck_o_pc(wbck_o_pc),
  .wbck_o_en(wbck_o_en),
  .wbck_o_rd(wbck_o_rd),
  .wbck_o_data(wbck_o_data),
  .wbck_o_lsu(wbck_o_lsu),
  .wbck_o_ebreak(wbck_o_ebreak),
  //FROM IDU:LONGI DEP
  .longi_wbck(longi_wbck),
  .longi_wbck_op(longi_wbck_op),
  .oitf_waw_dep(wbck_dep),
  //DCACHE:AXI SRAM
  .dcache_axi_arvalid(dcache_axi_arvalid),
  .dcache_axi_arready(dcache_axi_arready),
  .dcache_axi_araddr(dcache_axi_araddr),
  .dcache_axi_arsize(dcache_axi_arsize),
  .dcache_axi_rvalid(dcache_axi_rvalid),
  .dcache_axi_rready(dcache_axi_rready),
  .dcache_axi_rdata(dcache_axi_rdata),
  .dcache_axi_awvalid(dcache_axi_awvalid),
  .dcache_axi_awready(dcache_axi_awready),
  .dcache_axi_awaddr(dcache_axi_awaddr),
  .dcache_axi_awsize(dcache_axi_awsize),
  .dcache_axi_wvalid(dcache_axi_wvalid),
  .dcache_axi_wready(dcache_axi_wready),
  .dcache_axi_wdata(dcache_axi_wdata),
  .dcache_axi_wstrb(dcache_axi_wstrb),
  .dcache_axi_bvalid(dcache_axi_bvalid),
  .dcache_axi_bready(dcache_axi_bready),
  .dcache_axi_bresp(dcache_axi_bresp),
  //TO IFU:PRDT CALLBACK
  .callback_en(callback_en),
  .callback_index(callback_index),
  .callback_result(callback_result),
  .callback_flush(callback_flush),
  .callback_truepc(callback_truepc),
  .callback_fenceifinish(callback_fenceifinish),
  //FROM IFU:CSR WRITE
  .ifu_csr_ren(ifu_csr_ren),
  .ifu_csr_idx(ifu_csr_idx),
  .ifu_csr_rdata(ifu_csr_rdata),
  //TO IFU:FORWARD SRC1
  .jalr_rs1(if_reg_rs1),
  .jalr_reg_src1(if_reg_src1),
  .jalr_src1(jalr_src1)
);
// ================================================================================================================================================
// Regfile and Difftest
// ================================================================================================================================================
wire [`REG_IDX-1:0] if_reg_rs1;
wire [`REG_IDX-1:0] ex_reg_rs1;
wire [`REG_IDX-1:0] ex_reg_rs2;
wire [`XLEN-1:0]    if_reg_src1;
wire [`XLEN-1:0]    ex_reg_src1;
wire [`XLEN-1:0]    ex_reg_src2;

lieat_regfile regfile(
  .clock(clock),
  .reset(reset),

  .ifu_rs1(if_reg_rs1),
  .exu_rs1(ex_reg_rs1),
  .exu_rs2(ex_reg_rs2),
  .ifu_src1(if_reg_src1),
  .exu_src1(ex_reg_src1),
  .exu_src2(ex_reg_src2),

  .wb_pc(wbck_o_pc),
  .wb_valid(wbck_o_valid),
  .wb_en(wbck_o_en),
  .wb_rd(wbck_o_rd),
  .wb_data(wbck_o_data),
  .wb_lsu(wbck_o_lsu),
  .wb_ebreak(wbck_o_ebreak),
  .longi_empty(longi_empty)
);
// ================================================================================================================================================
// AXI MASTER
// ================================================================================================================================================
wire             icache_axi_arvalid;
wire             icache_axi_arready;
wire [`XLEN-1:0] icache_axi_araddr;
wire             icache_axi_rvalid;
wire             icache_axi_rready;
wire [63:0]      icache_axi_rdata;
wire             dcache_axi_arvalid;
wire             dcache_axi_arready;
wire [`XLEN-1:0] dcache_axi_araddr;
wire [2:0]       dcache_axi_arsize;
wire             dcache_axi_rvalid;
wire             dcache_axi_rready;
wire [63:0]      dcache_axi_rdata;
wire             dcache_axi_awvalid;
wire             dcache_axi_awready;
wire [`XLEN-1:0] dcache_axi_awaddr;
wire [2:0]       dcache_axi_awsize;
wire             dcache_axi_wvalid;
wire             dcache_axi_wready;
wire [63:0]      dcache_axi_wdata;
wire [7:0]       dcache_axi_wstrb;
wire             dcache_axi_bvalid;
wire             dcache_axi_bready;
wire [1:0]       dcache_axi_bresp;

lieat_axi_master axi_master(
  .clock(clock),
  .reset(reset),
  .icache_axi_arvalid(icache_axi_arvalid),
  .icache_axi_arready(icache_axi_arready),
  .icache_axi_araddr(icache_axi_araddr),
  .icache_axi_rvalid(icache_axi_rvalid),
  .icache_axi_rready(icache_axi_rready),
  .icache_axi_rdata(icache_axi_rdata),
  .dcache_axi_arvalid(dcache_axi_arvalid),
  .dcache_axi_arready(dcache_axi_arready),
  .dcache_axi_araddr(dcache_axi_araddr),
  .dcache_axi_arsize(dcache_axi_arsize),
  .dcache_axi_rvalid(dcache_axi_rvalid),
  .dcache_axi_rready(dcache_axi_rready),
  .dcache_axi_rdata(dcache_axi_rdata),
  .dcache_axi_awvalid(dcache_axi_awvalid),
  .dcache_axi_awready(dcache_axi_awready),
  .dcache_axi_awaddr(dcache_axi_awaddr),
  .dcache_axi_awsize(dcache_axi_awsize),
  .dcache_axi_wvalid(dcache_axi_wvalid),
  .dcache_axi_wready(dcache_axi_wready),
  .dcache_axi_wdata(dcache_axi_wdata),
  .dcache_axi_wstrb(dcache_axi_wstrb),
  .dcache_axi_bvalid(dcache_axi_bvalid),
  .dcache_axi_bready(dcache_axi_bready),
  .dcache_axi_bresp(dcache_axi_bresp),

  .io_master_awready(io_master_awready),
  .io_master_awvalid(io_master_awvalid),
  .io_master_awaddr(io_master_awaddr),
  .io_master_awid(io_master_awid),
  .io_master_awlen(io_master_awlen),
  .io_master_awsize(io_master_awsize),
  .io_master_awburst(io_master_awburst),

  .io_master_wready(io_master_wready),
  .io_master_wvalid(io_master_wvalid),
  .io_master_wdata(io_master_wdata),
  .io_master_wstrb(io_master_wstrb),
  .io_master_wlast(io_master_wlast),

  .io_master_bready(io_master_bready),
  .io_master_bvalid(io_master_bvalid),
  .io_master_bresp(io_master_bresp),
  .io_master_bid(io_master_bid),

  .io_master_arready(io_master_arready),
  .io_master_arvalid(io_master_arvalid),
  .io_master_araddr(io_master_araddr),
  .io_master_arid(io_master_arid),
  .io_master_arlen(io_master_arlen),
  .io_master_arsize(io_master_arsize),
  .io_master_arburst(io_master_arburst),

  .io_master_rready(io_master_rready),
  .io_master_rvalid(io_master_rvalid),
  .io_master_rresp(io_master_rresp),
  .io_master_rdata(io_master_rdata),
  .io_master_rlast(io_master_rlast),
  .io_master_rid(io_master_rid)
);
endmodule

