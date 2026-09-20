module mycpu_top(
    input  wire        clk,
    input  wire        resetn,
    // inst sram interface
    // exp7：新增指令 SRAM 片选信号。
    output wire        inst_sram_en,
    // exp7：写使能由 1 位扩展为 4 位字节写使能。
    output wire [ 3:0] inst_sram_we,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata,
    // data sram interface
    // exp7：新增数据 SRAM 片选信号。
    output wire        data_sram_en,
    // exp7：写使能由 1 位扩展为 4 位字节写使能。
    output wire [ 3:0] data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input  wire [31:0] data_sram_rdata,
    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata
);
reg         reset;
always @(posedge clk) reset <= ~resetn;

// exp7 第3步：为五级流水建立有效位
// fs_valid->IF, ds_valid->ID, es_valid->EX, ms_valid->MEM, ws_valid->WB
// 复位时全部清零；正常运行时逐级传递，用于标记该级内容是否来自一条有效指令。
reg         fs_valid;
reg         ds_valid;
reg         es_valid;
reg         ms_valid;
reg         ws_valid;
reg         branch_flush;
reg  [31:0] inst_pc_q;
// 分支在 ID 级确定后，下一拍冲刷已经取到的顺序路径指令。
reg  [31:0] ifid_pc;
reg  [31:0] ifid_inst;
reg  [11:0] idex_alu_op;
reg  [31:0] idex_pc;
reg  [31:0] idex_alu_src1;
reg  [31:0] idex_alu_src2;
reg  [31:0] idex_store_data;
reg  [ 4:0] idex_dest;
reg         idex_gr_we;
reg         idex_mem_we;
reg         idex_res_from_mem;
// EX/MEM 流水寄存器：统一保存数据 SRAM 请求所需字段。
reg  [31:0] exmem_alu_result;
reg  [31:0] exmem_store_data;
reg  [ 4:0] exmem_dest;
reg         exmem_gr_we;
reg         exmem_mem_we;
reg         exmem_res_from_mem;
reg  [31:0] exmem_pc;
// MEM/WB 流水寄存器：保存最终写回结果及调试信息。
reg  [31:0] memwb_pc;
reg  [31:0] memwb_result;
reg  [ 4:0] memwb_dest;
reg         memwb_gr_we;
wire        hazard_stall;
wire        use_rj;
wire        use_rkd;
wire        hazard_load_use;
wire        br_taken;
reg  [31:0] pc;
wire [11:0] alu_op;
wire        res_from_mem;
wire        gr_we;
wire        mem_we;
wire [ 4:0] dest;
wire [31:0] rkd_value;
wire [31:0] rj_value_raw;
wire [31:0] rkd_value_raw;
wire [31:0] rj_value_fwd;
wire [31:0] rkd_value_fwd;
wire [31:0] alu_src1;
wire [31:0] alu_src2;
wire [31:0] alu_result;
wire [31:0] final_result;
always @(posedge clk) begin
    if (reset) begin
        fs_valid <= 1'b0;
        ds_valid <= 1'b0;
        es_valid <= 1'b0;
        ms_valid <= 1'b0;
        ws_valid <= 1'b0;
        branch_flush <= 1'b0;
        inst_pc_q <= 32'b0;
        ifid_pc  <= 32'b0;
        ifid_inst <= 32'b0;
        idex_alu_op <= 12'b0;
        idex_pc <= 32'b0;
        idex_alu_src1 <= 32'b0;
        idex_alu_src2 <= 32'b0;
        idex_store_data <= 32'b0;
        idex_dest <= 5'b0;
        idex_gr_we <= 1'b0;
        idex_mem_we <= 1'b0;
        idex_res_from_mem <= 1'b0;
        exmem_alu_result <= 32'b0;
        exmem_store_data <= 32'b0;
        exmem_dest <= 5'b0;
        exmem_gr_we <= 1'b0;
        exmem_mem_we <= 1'b0;
        exmem_res_from_mem <= 1'b0;
        exmem_pc <= 32'b0;
        memwb_pc <= 32'b0;
        memwb_result <= 32'b0;
        memwb_dest <= 5'b0;
        memwb_gr_we <= 1'b0;
    end
    else begin
        fs_valid <= 1'b1;      // 当前无阻塞、无冲刷，IF 级每拍都在取一条有效指令
        ds_valid <= hazard_stall ? ds_valid : (fs_valid && !branch_flush && !br_taken);
        es_valid <= hazard_stall ? 1'b0 : ds_valid;
        ms_valid <= es_valid;
        ws_valid <= ms_valid;
        branch_flush <= hazard_stall ? 1'b0 : br_taken;
        // Block RAM 同步读：本拍返回的数据对应上一拍发出的 pc 请求。
        inst_pc_q <= hazard_stall ? inst_pc_q : pc;
        ifid_pc  <= hazard_stall ? ifid_pc : inst_pc_q;
        ifid_inst <= hazard_stall ? ifid_inst : inst_sram_rdata;
        if (ds_valid && !hazard_stall) begin
            idex_alu_op <= alu_op;
            idex_pc <= ifid_pc;
            idex_alu_src1 <= alu_src1;
            idex_alu_src2 <= alu_src2;
            idex_store_data <= rkd_value;
            idex_dest <= dest;
            idex_gr_we <= gr_we;
            idex_mem_we <= mem_we;
            idex_res_from_mem <= res_from_mem;
        end
        else begin
            idex_gr_we <= 1'b0;
            idex_mem_we <= 1'b0;
            idex_res_from_mem <= 1'b0;
        end
        if (es_valid) begin
            exmem_alu_result <= alu_result;
            exmem_store_data <= idex_store_data;
            exmem_dest <= idex_dest;
            exmem_gr_we <= idex_gr_we;
            exmem_mem_we <= idex_mem_we;
            exmem_res_from_mem <= idex_res_from_mem;
            exmem_pc <= idex_pc;
        end
        else begin
            exmem_gr_we <= 1'b0;
            exmem_mem_we <= 1'b0;
            exmem_res_from_mem <= 1'b0;
        end
        if (ms_valid) begin
            memwb_pc <= exmem_pc;
            memwb_result <= final_result;
            memwb_dest <= exmem_dest;
            memwb_gr_we <= exmem_gr_we;
        end
        else begin
            memwb_gr_we <= 1'b0;
        end
    end
end

// 当前组合解码逻辑对应 ID 级指令；保留该别名，避免空泡进入控制副作用。
wire        id_valid = ds_valid;

wire [31:0] seq_pc;
wire [31:0] nextpc;
wire [31:0] br_target;
wire [31:0] inst;

wire        src1_is_pc;
wire        src2_is_imm;
wire        dst_is_r1;
wire        src_reg_is_rd;
wire [31:0] rj_value;
wire [31:0] imm;
wire [31:0] br_offs;
wire [31:0] jirl_offs;

wire [ 5:0] op_31_26;
wire [ 3:0] op_25_22;
wire [ 1:0] op_21_20;
wire [ 4:0] op_19_15;
wire [ 4:0] rd;
wire [ 4:0] rj;
wire [ 4:0] rk;
wire [11:0] i12;
wire [19:0] i20;
wire [15:0] i16;
wire [25:0] i26;

wire [63:0] op_31_26_d;
wire [15:0] op_25_22_d;
wire [ 3:0] op_21_20_d;
wire [31:0] op_19_15_d;

wire        inst_add_w;
wire        inst_sub_w;
wire        inst_slt;
wire        inst_sltu;
wire        inst_nor;
wire        inst_and;
wire        inst_or;
wire        inst_xor;
wire        inst_slli_w;
wire        inst_srli_w;
wire        inst_srai_w;
wire        inst_addi_w;
wire        inst_ld_w;
wire        inst_st_w;
wire        inst_jirl;
wire        inst_b;
wire        inst_bl;
wire        inst_beq;
wire        inst_bne;
wire        inst_lu12i_w;

wire        need_ui5;
wire        need_si12;
wire        need_si16;
wire        need_si20;
wire        need_si26;
wire        src2_is_4;

wire [ 4:0] rf_raddr1;
wire [31:0] rf_rdata1;
wire [ 4:0] rf_raddr2;
wire [31:0] rf_rdata2;
wire        rf_we   ;
wire [ 4:0] rf_waddr;
wire [31:0] rf_wdata;

wire        rj_eq_rd;

wire [31:0] mem_result;
// 第8步：分支在 ID 级判定，并将结果反馈到 IF 的 PC 选择器。
wire        branch_instr;

assign seq_pc       = pc + 32'h4;
assign nextpc       = hazard_stall ? pc : (br_taken ? br_target : seq_pc);

always @(posedge clk) begin
    if (reset) begin
        pc <= 32'h1bfffffc;     //trick: to make nextpc be 0x1c000000 during reset 
    end
    else begin
        pc <= nextpc;
    end
end

// exp7：指令 SRAM 保持选中，指令写使能保持关闭。
// Hold the synchronous instruction RAM output together with PC/IFID while ID
// is stalled, so the pending instruction is not overwritten or misaligned.
assign inst_sram_en    = !hazard_stall;
assign inst_sram_we    = 4'b0;
assign inst_sram_addr  = pc;
assign inst_sram_wdata = 32'b0;
// ID 级只使用 IF/ID 寄存器中的指令，避免直接使用尚未对齐的 RAM 输出。
assign inst            = ifid_inst;

assign op_31_26  = inst[31:26];
assign op_25_22  = inst[25:22];
assign op_21_20  = inst[21:20];
assign op_19_15  = inst[19:15];

assign rd   = inst[ 4: 0];
assign rj   = inst[ 9: 5];
assign rk   = inst[14:10];

assign i12  = inst[21:10];
assign i20  = inst[24: 5];
assign i16  = inst[25:10];
assign i26  = {inst[ 9: 0], inst[25:10]};

decoder_6_64 u_dec0(.in(op_31_26 ), .out(op_31_26_d ));
decoder_4_16 u_dec1(.in(op_25_22 ), .out(op_25_22_d ));
decoder_2_4  u_dec2(.in(op_21_20 ), .out(op_21_20_d ));
decoder_5_32 u_dec3(.in(op_19_15 ), .out(op_19_15_d ));

assign inst_add_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h00];
assign inst_sub_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h02];
assign inst_slt    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h04];
assign inst_sltu   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h05];
assign inst_nor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h08];
assign inst_and    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h09];
assign inst_or     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0a];
assign inst_xor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0b];
assign inst_slli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h01];
assign inst_srli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h09];
assign inst_srai_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h11];
assign inst_addi_w = op_31_26_d[6'h00] & op_25_22_d[4'ha];
assign inst_ld_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h2];
assign inst_st_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h6];
assign inst_jirl   = op_31_26_d[6'h13];
assign inst_b      = op_31_26_d[6'h14];
assign inst_bl     = op_31_26_d[6'h15];
assign inst_beq    = op_31_26_d[6'h16];
assign inst_bne    = op_31_26_d[6'h17];
assign inst_lu12i_w= op_31_26_d[6'h05] & ~inst[25];

assign alu_op[ 0] = inst_add_w | inst_addi_w | inst_ld_w | inst_st_w
                    | inst_jirl | inst_bl;
assign alu_op[ 1] = inst_sub_w;
assign alu_op[ 2] = inst_slt;
assign alu_op[ 3] = inst_sltu;
assign alu_op[ 4] = inst_and;
assign alu_op[ 5] = inst_nor;
assign alu_op[ 6] = inst_or;
assign alu_op[ 7] = inst_xor;
assign alu_op[ 8] = inst_slli_w;
assign alu_op[ 9] = inst_srli_w;
assign alu_op[10] = inst_srai_w;
assign alu_op[11] = inst_lu12i_w;

assign need_ui5   =  inst_slli_w | inst_srli_w | inst_srai_w;
assign need_si12  =  inst_addi_w | inst_ld_w | inst_st_w;
assign need_si16  =  inst_jirl | inst_beq | inst_bne;
assign need_si20  =  inst_lu12i_w;
assign need_si26  =  inst_b | inst_bl;
assign src2_is_4  =  inst_jirl | inst_bl;

assign imm = src2_is_4 ? 32'h4                      :
             need_si20 ? {i20[19:0], 12'b0}         :
             need_ui5  ? {27'b0, i12[4:0]}          :
                         {{20{i12[11]}}, i12[11:0]} ;

assign br_offs = need_si26 ? {{ 4{i26[25]}}, i26[25:0], 2'b0} :
                             {{14{i16[15]}}, i16[15:0], 2'b0} ;

assign jirl_offs = {{14{i16[15]}}, i16[15:0], 2'b0};

assign src_reg_is_rd = inst_beq | inst_bne | inst_st_w;

assign src1_is_pc    = inst_jirl | inst_bl;

assign src2_is_imm   = inst_slli_w |
                       inst_srli_w |
                       inst_srai_w |
                       inst_addi_w |
                       inst_ld_w   |
                       inst_st_w   |
                       inst_lu12i_w|
                       inst_jirl   |
                       inst_bl     ;

assign res_from_mem  = inst_ld_w;
assign dst_is_r1     = inst_bl;
assign gr_we         = ~inst_st_w & ~inst_beq & ~inst_bne & ~inst_b;
assign mem_we        = inst_st_w;
assign dest          = dst_is_r1 ? 5'd1 : rd;

assign rf_raddr1 = rj;
assign rf_raddr2 = src_reg_is_rd ? rd :rk;
regfile u_regfile(
    .clk    (clk      ),
    .raddr1 (rf_raddr1),
    .rdata1 (rf_rdata1),
    .raddr2 (rf_raddr2),
    .rdata2 (rf_rdata2),
    .we     (rf_we    ),
    .waddr  (rf_waddr ),
    .wdata  (rf_wdata )
    );

assign rj_value_raw  = rf_rdata1;
assign rkd_value_raw = rf_rdata2;

// Forward completed ALU/load values back to ID.  EX has the highest
// priority, followed by MEM and WB, matching the youngest producer.
assign rj_value_fwd = (es_valid && idex_gr_we && !idex_res_from_mem &&
                       idex_dest != 5'd0 && rj == idex_dest) ? alu_result :
                      (ms_valid && exmem_gr_we && exmem_dest != 5'd0 &&
                       rj == exmem_dest) ?
                        (exmem_res_from_mem ? mem_result : exmem_alu_result) :
                      (ws_valid && memwb_gr_we && memwb_dest != 5'd0 &&
                       rj == memwb_dest) ? memwb_result : rj_value_raw;
assign rkd_value_fwd = (es_valid && idex_gr_we && !idex_res_from_mem &&
                        idex_dest != 5'd0 && rf_raddr2 == idex_dest) ? alu_result :
                       (ms_valid && exmem_gr_we && exmem_dest != 5'd0 &&
                        rf_raddr2 == exmem_dest) ?
                         (exmem_res_from_mem ? mem_result : exmem_alu_result) :
                       (ws_valid && memwb_gr_we && memwb_dest != 5'd0 &&
                        rf_raddr2 == memwb_dest) ? memwb_result : rkd_value_raw;
assign rj_value  = rj_value_fwd;
assign rkd_value = rkd_value_fwd;

assign rj_eq_rd = (rj_value == rkd_value);
assign branch_instr = inst_beq | inst_bne | inst_jirl | inst_bl | inst_b;
// Only source fields actually consumed by the decoded instruction take part
// in hazard detection.  Register zero never creates a dependency.
assign use_rj = inst_add_w | inst_sub_w | inst_slt | inst_sltu |
                inst_nor | inst_and | inst_or | inst_xor |
                inst_slli_w | inst_srli_w | inst_srai_w | inst_addi_w |
                inst_ld_w | inst_st_w | inst_jirl | inst_beq | inst_bne;
assign use_rkd = inst_add_w | inst_sub_w | inst_slt | inst_sltu |
                 inst_nor | inst_and | inst_or | inst_xor |
                 inst_st_w | inst_beq | inst_bne;
// A load value is not available until the MEM stage.  Keep exactly one
// bubble for a load-use dependency; all ALU dependencies are forwarded.
assign hazard_load_use = es_valid && idex_gr_we && idex_res_from_mem &&
                         (idex_dest != 5'd0) &&
                         ((use_rj && (rj == idex_dest)) ||
                          (use_rkd && (rf_raddr2 == idex_dest)));
assign hazard_stall = id_valid && hazard_load_use;
assign br_taken = id_valid && !hazard_stall &&
                  ( (inst_beq  &&  rj_eq_rd)
                  || (inst_bne  && !rj_eq_rd)
                  || inst_jirl
                  || inst_bl
                  || inst_b );
// PC 相对分支使用 IF/ID 中保存的 PC；jirl 使用寄存器值加偏移量。
assign br_target = (branch_instr && !inst_jirl) ? (ifid_pc + br_offs) :
                                                   /*inst_jirl*/ (rj_value + jirl_offs);

// The ALU shift operations use src2 as the value and src1[4:0] as the
// shift amount, so immediate shifts need their operands swapped here.
assign alu_src1 = need_ui5 ? imm :
                  src1_is_pc ? ifid_pc : rj_value;
assign alu_src2 = need_ui5 ? rj_value :
                  src2_is_imm ? imm : rkd_value;

alu u_alu(
    .alu_op     (idex_alu_op),
    .alu_src1   (idex_alu_src1),
    .alu_src2   (idex_alu_src2),
    .alu_result (alu_result)
    );

// exp7：数据 SRAM 保持选中，执行 st.w 时使能全部四个字节通道。
assign data_sram_en    = es_valid && (idex_mem_we || idex_res_from_mem);
// 数据写请求属于当前解码/执行中的指令，空泡不得产生写请求。
assign data_sram_we    = {4{es_valid && idex_mem_we}};
assign data_sram_addr  = alu_result;
assign data_sram_wdata = idex_store_data;

assign mem_result   = data_sram_rdata;
assign final_result = exmem_res_from_mem ? mem_result : exmem_alu_result;

// 写回端只有 WB 级有效指令才能更新寄存器。
assign rf_we    = memwb_gr_we && ws_valid;
assign rf_waddr = memwb_dest;
assign rf_wdata = memwb_result;

// debug info generate
assign debug_wb_pc       = memwb_pc;
assign debug_wb_rf_we   = {4{rf_we}};//修改变量名
assign debug_wb_rf_wnum  = memwb_dest;
assign debug_wb_rf_wdata = memwb_result;

endmodule
