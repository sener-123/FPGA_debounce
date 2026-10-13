//****************************************Copyright (c)***********************************//
//原子哥在线教学平台：www.yuanzige.com
//技术支持：http://www.openedv.com/forum.php
//淘宝店铺：https://zhengdianyuanzi.tmall.com
//关注微信公众平台微信号："正点原子"，免费获取ZYNQ & FPGA & STM32 & LINUX资料。
//版权所有，盗版必究。
//Copyright(C) 正点原子 2023-2033
//All rights reserved                                  
//----------------------------------------------------------------------------------------
// File name:           ddr3_ctrl_top
// Created by:          正点原子
// Created date:        2023年2月3日14:17:02
// Version:             V1.0
// Descriptions:        ddr3_ctrl_top
//
//----------------------------------------------------------------------------------------
//****************************************************************************************///

module ddr3_ctrl_top(
    input                   clk             ,
    input                   wd_clk          ,
    input                   rd_clk          ,
    input                   rst_n           ,
    // 外部数据接口   
    input                   wd_en           ,   //写使能
    input       [15:0]      wd_data         ,   //写数据
    input                   rd_en           ,   //读使能
    output      [15:0]      rd_data         ,   //读数据
    input                   wr_load         ,   //输入源更新信号
    input                   rd_load         ,   //输出源更新信号
    input                   ddr3_pingpang_en,   //DDR3乒乓操作
    input                   ddr3_read_valid ,
    // ddr地址长度控制接口                  
    input       [26:0]      addr_rd_min     ,
    input       [26:0]      addr_rd_max     ,
    input       [9:0]       rd_burst_len    ,
    input       [26:0]      addr_wd_min     ,
    input       [26:0]      addr_wd_max     ,
    input       [9:0]       wd_burst_len    ,
    output  reg             ddr3_init_done  ,   //ddr3初始化完成标志
    //DDR控制器接口
    output                  mem_rst_n       ,                       
    output                  mem_ck          ,
    output                  mem_ck_n        ,
    output                  mem_cke         ,
    output                  mem_ras_n       ,
    output                  mem_cas_n       ,
    output                  mem_we_n        , 
    output                  mem_odt         ,
    output                  mem_cs_n        ,
    output 	    [14:0]      mem_a           ,   
    output 	    [2:0]       mem_ba          ,   
    inout 	    [3:0]       mem_dqs         ,
    inout 	    [3:0]       mem_dqs_n       ,
    inout 	    [31:0]      mem_dq          ,
    output 	    [3:0]       mem_dm               
);
//wire define
wire            ui_clk          ;
wire [10:0]     rfifo_wcount    ;
wire [10:0]     wfifo_rcount    ;
wire [26:0]     wd_addr         ;
wire [26:0]     rd_addr         ;
wire [9:0]      wd_len          ;
wire [9:0]      rd_len          ;
wire [255:0]    rfifo_wdata     ;
wire            rfifo_wen       ;
wire [255:0]    wfifo_rdata     ;
wire            wfifo_ren       ;
wire            rd_finish       ;
wire            wd_finish       ;
wire            rd_req          ;
wire            wd_req          ;
// Write Address                
wire [3:0]     axi_awid         ;
wire [26:0]    axi_awaddr       ;
wire [3:0]     axi_awlen        ;
wire           axi_awvalid      ;
wire           axi_awready      ;
// write data
wire [255:0]   axi_wdata        ;
wire [31:0]    axi_wstrb        ;
wire           axi_wlast        ;
wire           axi_wready       ;
// read address                 
wire [3:0]     axi_arid         ;
wire [26:0]    axi_araddr       ;
wire [3:0]     axi_arlen        ;
wire           axi_arvalid      ;
wire           axi_arready      ;
wire           ddr_init_done    ;
// read data                    
wire [3:0]     axi_rid          ;
wire [255:0]   axi_rdata        ;
wire           axi_rlast        ;
wire           axi_rvalid       ;

reg            ddr3_init_done_c0;
reg            ddr3_init_done_c1;

//*****************************************************
//**                    main code
//*****************************************************

// 稳定初始化信号
always@ (posedge ui_clk or negedge rst_n ) begin
    if(!rst_n)begin
        ddr3_init_done_c0 <= 1'b0;
        ddr3_init_done_c1 <= 1'b0;
    end
	else begin
        ddr3_init_done_c0 <= ddr_init_done;
        ddr3_init_done_c1 <= ddr3_init_done_c0;
    end
end
always@(posedge ui_clk or negedge rst_n)begin
    if(!rst_n)
        ddr3_init_done <= 1'b0;
    else if(ddr3_init_done_c0 && ~ddr3_init_done_c1)	
		ddr3_init_done <= 1'b1;
    else
        ddr3_init_done <= ddr3_init_done;
end

// DDR3读写控制模块 
ddr3_rw_ctrl u_ddr3_rw_ctrl(
    .rst_n                  (rst_n      ),
    .clk                    (ui_clk     ),
    .addr_rd_min            (addr_rd_min),
    .addr_rd_max            (addr_rd_max),
    .rd_burst_len           (rd_burst_len),
    .addr_wd_min            (addr_wd_min),
    .addr_wd_max            (addr_wd_max),
    .wd_burst_len           (wd_burst_len),
    .rfifo_wcount           (rfifo_wcount),
    .wfifo_rcount           (wfifo_rcount),
    .ddr3_init_done         (ddr3_init_done),
    .wd_finish              (wd_finish  ),
    .wd_req                 (wd_req     ),
    .wd_addr                (wd_addr    ),
    .wd_len                 (wd_len     ),
    .rd_finish              (rd_finish  ),
    .rd_req                 (rd_req     ),
    .rd_addr                (rd_addr    ),
    .rd_len                 (rd_len     ),
    .rd_load                (rd_load    ),
    .wr_load                (wr_load    ),
    .ddr3_pingpang_en       (ddr3_pingpang_en),
    .ddr3_read_valid        (ddr3_read_valid)
);

// axi 控制模块
aq_axi_master u_aq_axi_master
	(
	.rst_n                  (rst_n  && ddr3_init_done),
	.clk                    (ui_clk        ),
    // Write address channel
	.m_axi_awid             (axi_awid      ),                             
	.m_axi_awaddr           (axi_awaddr    ),                             
	.m_axi_awlen            (axi_awlen     ),                                                         
	.m_axi_awvalid          (axi_awvalid   ),                             
	.m_axi_awready          (axi_awready   ),
    // Write data channel
	.m_axi_wdata            (axi_wdata     ),//O                             
	.m_axi_wstrb            (axi_wstrb     ),                             
	.m_axi_wlast            (axi_wlast     ),                                                         
	.m_axi_wready           (axi_wready    ),
    // Read address channel
	.m_axi_arid             (axi_arid      ),                             
	.m_axi_araddr           (axi_araddr    ),                             
	.m_axi_arlen            (axi_arlen     ),                                                     
	.m_axi_arvalid          (axi_arvalid   ),                             
	.m_axi_arready          (axi_arready   ),
    // Read data channel
	.m_axi_rdata            (axi_rdata     ), //I                                                        
	.m_axi_rlast            (axi_rlast     ),                                                          
	.m_axi_rvalid           (axi_rvalid    ),
    // User control interface
	.wr_start               (wd_req        ),      
	.wr_adrs                ({wd_addr,3'd0}),         
	.wr_len                 ({wd_len,3'd0} ),         
	.wr_ready               ( ),         
	.wr_fifo_re             (wfifo_ren     ),         
	.wr_fifo_empty          (1'b0          ),         
	.wr_fifo_aempty         (1'b0          ),
	.wr_fifo_data           (wfifo_rdata   ),//I
	.wr_done                (wd_finish     ),
	.rd_start               (rd_req        ),	  
	.rd_adrs                ({rd_addr,3'd0}),
	.rd_len                 ({rd_len,3'd0} ),
	.rd_ready               ( ),
	.rd_fifo_we             (rfifo_wen     ),
	.rd_fifo_full           (1'b0          ),
	.rd_fifo_afull          (1'b0          ),
	.rd_fifo_data           (rfifo_wdata   ),//O
	.rd_done                (rd_finish     )
);

// DDR3 IP core
ddr3_ip u_ddr3_ip (
    .ref_clk               	(clk),                  // input
    .resetn                	(rst_n),                // input
    .core_clk              	(ui_clk),               // output
    .pll_lock              	( ),                    // output
    .phy_pll_lock          	( ),                    // output
    .gpll_lock             	( ),                    // output
    .rst_gpll_lock         	( ),                    // output
    .ddrphy_cpd_lock       	( ),                    // output
    .ddr_init_done         	(ddr_init_done),        // output
	
    .axi_awaddr            	(axi_awaddr),           // input [26:0]
    .axi_awuser_ap         	(0),                    // input
    .axi_awuser_id         	(0),                    // input [3:0]
    .axi_awlen             	(axi_awlen),            // input [3:0]
    .axi_awready           	(axi_awready),          // output
    .axi_awvalid           	(axi_awvalid),          // input
		
    .axi_wdata             	(axi_wdata),            // input [255:0]
    .axi_wstrb             	(axi_wstrb),            // input [31:0]
    .axi_wready            	(axi_wready),           // output
    .axi_wusero_id         	( ),                    // output [3:0]
    .axi_wusero_last       	(axi_wlast),            // output
    
    .axi_araddr            	(axi_araddr),           // input [26:0]
    .axi_aruser_ap         	(0),                    // input
    .axi_aruser_id         	(0),                    // input [3:0]
    .axi_arlen             	(axi_arlen),            // input [3:0]
    .axi_arready           	(axi_arready),          // output
    .axi_arvalid           	(axi_arvalid),          // input
		
    .axi_rdata             	(axi_rdata),            // output [255:0]
    .axi_rid               	( ),                    // output [3:0]
    .axi_rlast             	(axi_rlast),            // output
    .axi_rvalid            	(axi_rvalid),           // output
		
    .apb_clk               	(0),                    // input
    .apb_rst_n             	(0),                    // input
    .apb_sel               	(0),                    // input
    .apb_enable            	(0),                    // input
    .apb_addr              	(0),                    // input [7:0]
    .apb_write             	(0),                    // input
    .apb_ready             	( ),                    // output
    .apb_wdata             	(0),                    // input [15:0]
    .apb_rdata             	( ),                    // output [15:0]
		
    .mem_cs_n              	(mem_cs_n),             // output
    .mem_rst_n             	(mem_rst_n),            // output
    .mem_ck                	(mem_ck),               // output
    .mem_ck_n              	(mem_ck_n),             // output
    .mem_cke               	(mem_cke),              // output
    .mem_ras_n             	(mem_ras_n),            // output
    .mem_cas_n             	(mem_cas_n),            // output
    .mem_we_n              	(mem_we_n),             // output
    .mem_odt               	(mem_odt),              // output
    .mem_a                 	(mem_a),                // output [13:0]
    .mem_ba                	(mem_ba),               // output [2:0]
    .mem_dqs               	(mem_dqs),              // inout [3:0]
    .mem_dqs_n             	(mem_dqs_n),            // inout [3:0]
    .mem_dq                	(mem_dq),               // inout [31:0]
    .mem_dm                	(mem_dm),               // output [3:0]
  
    //.dfi_address(0),                              // input [59:0]
    //.dfi_bank(0),                                 // input [11:0]
    //.dfi_cs_n(0),                                 // input [3:0]
    //.dfi_cas_n(0),                                // input [3:0]
    //.dfi_ras_n(0),                                // input [3:0]
    //.dfi_we_n(0),                                 // input [3:0]
    //.dfi_cke(0),                                  // input [3:0]
    //.dfi_odt(0),                                  // input [3:0]
    //.dfi_wrdata_en(0),                            // input [3:0]
    //.dfi_wrdata(0),                               // input [255:0]
    //.dfi_wrdata_mask(0),                          // input [31:0]
    //.dfi_rddata( ),                               // output [255:0]
    //.dfi_rddata_valid( ),                         // output
    //.dfi_reset_n(1),                              // input
    //.dfi_phyupd_req( ),                           // output
    //.dfi_phyupd_ack(0),                           // input
    //.dfi_error( ),                                // output
    .dbg_gate_start         	(0),               	// input
    .dbg_cpd_start          	(1),               	// input
    .dbg_ddrphy_rst_n       	(1),               	// input
    .dbg_gpll_scan_rst      	(0),               	// input
    .samp_position_dyn_adj  	(0),               	// input
    .init_samp_position_even	(0),               	// input [31:0]
    .init_samp_position_odd 	(0),               	// input [31:0]
    .wrcal_position_dyn_adj 	(0),               	// input
    .init_wrcal_position    	(0),               	// input [31:0]
    .force_read_clk_ctrl    	(0),               	// input
    .init_slip_step         	(0),               	// input [15:0]
    .init_read_clk_ctrl     	(0),               	// input [11:0]
    .debug_calib_ctrl       	( ),               	// output [33:0]
    .dbg_slice_state        	( ),               	// output [87:0]
    .debug_data             	( ),               	// output [275:0]
    .dbg_dll_upd_state      	( ),               	// output [1:0]
    .debug_gpll_dps_phase       ( ),               	// output [8:0]
    .dbg_rst_dps_state          ( ),               	// output [2:0]
    .dbg_tran_err_rst_cnt       ( ),               	// output [5:0]
    .dbg_ddrphy_init_fail       ( ),               	// output
    .debug_cpd_offset_adj       (0),               	// input
    .debug_cpd_offset_dir       (0),               	// input
    .debug_cpd_offset           (0),               	// input [9:0]
    .debug_dps_cnt_dir0         ( ),               	// output [9:0]
    .debug_dps_cnt_dir1         ( ),               	// output [9:0]
    .ck_dly_en                  (1),               	// input
    .init_ck_dly_step           (0),               	// input [7:0]
    .ck_dly_set_bin             ( ),               	// output [7:0]
    .align_error                ( ),               	// output
    .debug_rst_state            ( ),               	// output [3:0]
    .debug_cpd_state            ( )                	// output [3:0]
);
 
// FIFO控制模块
ctrl_fifo u_ctrl_fifo(
    .rst_n                  (rst_n      ),
    .wd_clk                 (wd_clk     ),
    .rd_clk                 (rd_clk     ),
    .clk_100                (ui_clk     ),        
    .wfifo_wr_en            (wd_en      ),
    .wfifo_wr_data          (wd_data    ),
    .wfifo_rd_en            (wfifo_ren  ),
    .rd_load                (rd_load    ),
    .wr_load                (wr_load    ),
    .wfifo_rd_data          (wfifo_rdata),
    .wfifo_rcount           (wfifo_rcount),               
    .rfifo_wr_en            (rfifo_wen  ),
    .rfifo_wr_data          (rfifo_wdata),
    .rfifo_wcount           (rfifo_wcount),
    .rfifo_rd_en            (rd_en      ),
    .rfifo_rd_data          (rd_data    )
);
endmodule