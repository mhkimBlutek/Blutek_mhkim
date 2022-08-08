//
// Copyright (c) 2014 Fumate co.,ltd All Rights Reserved.
//
//  2F Jin Sung B/D 534-6 Noeun-dong, Yuseong-gu, Daejeon, KOREA 305-325
//
////////////////////////////////////////////////////////////////////////////////
//
//  File name  : cli_cmd.h
//
//  Language  : C
//  Compiler  : 
//  Target    : 
//
////////////////////////////////////////////////////////////////////////////////
//
//  Descriptions
//
//  
//
////////////////////////////////////////////////////////////////////////////////
//
//  The revision history
//
//  2014.02
//    Created by Shin (Tab : 2 space)
//
////////////////////////////////////////////////////////////////////////////////

#ifndef CLI_CMD_H_
#define CLI_CMD_H_

// * INCLUDE
#include <stdio.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

// * DEFINES
typedef unsigned int   u32;
typedef unsigned short u16;
typedef unsigned char  u8;

#define INPUT_CMD_PARAM_MAX 10
#define INPUT_STR_LENGTH    40

#define IPC_PRINTF_BUF_MAX 1024





// * CMD DEFINES
#define CMD_SHOW_INVENTORY        "show in"
#define CMD_SHOW_TEMPERATURE      "show temp"
#define CMD_ENABLE_MODEM          "enable modem"
#define CMD_STOP_MODEM            "no modem"
#define CMD_RF_CTRL               "rf [on|off|1_on|1_off|2_on|2_off]"
#define CMD_SHOW_XADC_ZYNQ_DUMP   "show xadc_zynq dump"
#define CMD_SHOW_XADC_K7_DUMP     "show xadc_k7 dump"
#define CMD_READ_PLL1             "show pll1 [H]"
#define CMD_SET_PLL1              "set pll1 [H] [H]"
#define CMD_READ_PLL1_DUMP        "show pll1 dump"
#define CMD_READ_PLL2             "show pll2 [H]"
#define CMD_SET_PLL2              "set pll2 [H] [H]"
#define CMD_READ_PLL2_DUMP        "show pll2 dump"
#define CMD_READ_PLL2             "show pll2 [H]"
#define CMD_READ_ADC1             "show adc1 [H]"
#define CMD_SET_ADC1              "set adc1 [H] [H]"
#define CMD_READ_ADC1_DUMP        "show adc1 dump"
#define CMD_READ_ADC2             "show adc2 [H]"
#define CMD_SET_ADC2              "set adc2 [H] [H]"
#define CMD_READ_ADC2_DUMP        "show adc2 dump"
#define CMD_READ_DAC1             "show dac1 [H]"
#define CMD_SET_DAC1              "set dac1 [H] [H]"
#define CMD_READ_DAC1_DUMP        "show dac1 dump"
#define CMD_READ_DAC2             "show dac2 [H]"
#define CMD_SET_DAC2              "set dac2 [H] [H]"
#define CMD_READ_DAC2_DUMP        "show dac2 dump"
#define CMD_READ_FPGA             "show fpga [H]"
#define CMD_SET_FPGA              "set fpga [H] [H]"
#define CMD_READ_FPGA_DUMP        "show fpga dump [H]"
#define CMD_READ_FPGA_DECODE      "show fpga decode [H]"
#define CMD_READ_CPLD             "show cpld [H]"
#define CMD_SET_CPLD              "set cpld [H] [H]"
#define CMD_READ_CPLD_DUMP        "show cpld dump"
#define CMD_ENABLE_FA2            "enable fa2"
#define CMD_DISABLE_FA2           "disable fa2"
#define CMD_SET_CLI_FA_MODE       "set cli [S]"
#define CMD_NWK_TEST              "nwk_test [D]"
#define CMD_ETH_SNED_DBG          "dbg eth_send [H]"
#define CMD_ETH_RECV_DBG          "dbg eth_recv [H]"
#define CMD_FPGA_RESET            "fpga reset [on|off]"
#define CMD_PLL_RESET             "pll reset [on|off]"
#define CMD_DAC_RESET             "dac reset [on|off]"
#define CMD_JTAG_RESET            "jtag reset [on|off]"
#define CMD_MS_DAC_INIT           "ms dac init"
#define CMD_MS_ADC_INIT           "ms adc init"
#define CMD_MS_PLL_INIT           "ms pll init"
#define CMD_SHOW_INFO             "info"
#define CMD_SHOW_MSID             "show msid"
#define CMD_CLR_INFO              "clr info"
#define CMD_SHOW_VER              "show ver"
#define CMD_SHOW_SYNC             "show sync"
#define CMD_SET_NODE_MODE         "enable [S] [D]"
#define CMD_SHOW_RF               "show rf"
#define CMD_SHOW_PWR              "show power"
#define CMD_UM_TEST               "set um [H]"
#define CMD_SHOW_IOMANAGER_DEBUG  "show iom"
#define CMD_SET_IOMANAGER_DEBUG   "set iom [D]"
#define CMD_SHOW_ETH_PACKET       "show pkt"
#define CMD_ADD_ETH_PORT          "add port [t|u1|u2] [D]"
#define CMD_DEL_ETH_PORT          "del port [t|u1|u2] [D]"
#define CMD_RESET_PKT_CNT         "reset pkt"
#define CMD_BIT_MODE              "set bit_mode [on|off]"
#define CMD_SET_ERROR_BIT         "set bit [D]"
#define CMD_SHOW_SEND_ETH_PACKET  "show spkt"
#define CMD_RESET_SEND_PKT_CNT    "reset spkt"
#define CMD_SHOW_PACKET_RATE      "show rate"
#define CMD_RESET_PACKET_RATE     "reset rate"
#define CMD_SET_RF_FREQ            "set freq [D]"
#define CMD_NWK_TOP_MODE           "nwk tmode [off|hop2|hop1-2|hop3|hop1]"
#define CMD_NWK_2HOP_MODE          "nwk hop [D] [D] [D]"
#define CMD_PL_LOOPBACK            "set loopback [on|off]"
#define CMD_MAC_DETER             "deter [on|off]"
#define CMD_CHECK_MAP_ALIVE       "en_chk dl [on|off]"
#define CMD_CHECK_LQ_ALIVE        "en_chk ah [on|off]"
#define CMD_CHECK_LQ_RPT_ALVIE    "en_chk ul [on|off]"
#define CMD_FORCE_HOP_CMD         "fhop [D] [D] [D]"
#define CMD_SET_PORT_BLOCK        "set port stat [t|u1|u2] [D] [on|off|fw]"
#define CMD_SET_ALL_PORT_BLOCK    "set all port stat [on|off]"
#define CMD_SHOW_PORT_INFO        "show port info"
#define CMD_REMOTE_PORT_CTRL_USC1 "usc1 port [t|u1|u2] [D] [on|off]"
#define CMD_REMOTE_PORT_CTRL_USC2 "usc2 port [t|u1|u2] [D] [on|off]"
#define CMD_REMOTE_PORT_CTRL_ALL  "all port [D] [on|off]"
#define CMD_ANT_ANGLE_TEST_CTRL   "ant ctrl [D]"
#define CMD_AES_CTRL              "aes [on|off]"
#define CMD_AES_CAPTURE           "aes cap"
#define CMD_AES_MS_CTRL           "aes ms [on|off]"
#define CMD_MS1_PORT_ADD_PORT      "ms1 add port [t|u1|u2] [D]"      
#define CMD_MS1_PORT_DEL_PORT      "ms1 del port [t|u1|u2] [D]"  
#define CMD_MS2_PORT_ADD_PORT      "ms2 add port [t|u1|u2] [D]"      
#define CMD_MS2_PORT_DEL_PORT      "ms2 del port [t|u1|u2] [D]"  
#define CMD_MS_TPC_P_OFFSET        "tpc_p [D]"
#define CMD_MS_TPC_M_OFFSET        "tpc_m [D]"
#define CMD_EXT_DEV_RESET          "extdev reset"

//Add 
typedef unsigned short u16;
#define INPUT_STR_LENGTH    40

typedef struct _CLI_CMDS{
 char cli_cmds[INPUT_STR_LENGTH];
 void *cb_func;
}CLI_CMDS;


// * CMD FUNCTIONS
void cmd_show_inventory( u32 **argv );
void cmd_show_temperature( u32 **argv );
void cmd_enable_modem( u32 **argv );
void cmd_stop_modem( u32 **argv );
void cmd_rf_ctrl( u32 **argv );
void cmd_show_xadc_zynq_dump( u32 **argv);
void cmd_show_xadc_k7_dump( u32 **argv);
void cmd_read_pll1( u32 **argv);
void cmd_set_pll1( u32 **argv);
void cmd_read_pll1_dump( u32 **argv);
void cmd_read_pll2_dump( u32 **argv);
void cmd_set_pll2( u32 **argv);
void cmd_read_pll2( u32 **argv);
void cmd_read_adc2_dump( u32 **argv);
void cmd_set_adc2( u32 **argv);
void cmd_read_adc2( u32 **argv); 
void cmd_read_adc1_dump( u32 **argv);
void cmd_set_adc1( u32 **argv);
void cmd_read_adc1( u32 **argv);
void cmd_read_dac2_dump( u32 **argv);
void cmd_set_dac2( u32 **argv);  
void cmd_read_dac2( u32 **argv); 
void cmd_read_dac1_dump( u32 **argv); 
void cmd_set_dac1( u32 **argv);  
void cmd_read_dac1( u32 **argv);
void cmd_read_fpga( u32 **argv);
void cmd_set_fpga( u32 **argv);
void cmd_read_fpga_dump( u32 **argv);
void cmd_read_fpga_decode( u32 **argv);
void cmd_read_cpld( u32 **argv);
void cmd_set_cpld( u32 **argv);
void cmd_read_cpld_dump( u32 **argv);
void cmd_disable_fa2( u32 **argv);
void cmd_enable_fa2( u32 **argv);
void cmd_set_cli_fa_mode(u32 **argv);
void cmd_network_test( u32 **argv);
void cmd_recv_eth_dbg(u32 **argv);
void cmd_send_eth_dbg(u32 **argv);
void cmd_ms_pll_init(u32 **argv);
void cmd_ms_adc_init(u32 **argv);
void cmd_ms_dac_init(u32 **argv);
void cmd_fpga_reset(u32 **argv);
void cmd_pll_reset(u32 **argv);
void cmd_dac_reset(u32 **argv);
void cmd_jtag_reset(u32 **argv);
void cmd_show_information( u32 **argv);
void cmd_show_msid( u32 **argv);
void cmd_clear_inforamtion( u32 **argv);
void cmd_show_version( u32 **argv);
void cmd_show_sync( u32 **argv);
void cmd_set_node_mode( u32 **argv);
void cmd_read_rf( u32 **argv);
void cmd_show_power( u32 **argv );
void cmd_set_um_test(u32 **argv);
void cmd_show_rf(u32 **argv);
void cmd_set_ioManager_debug( u32 **argv);
void cmd_show_ioManager_debug( u32 **argv);
void cmd_show_eth_packet( u32 **argv);
void cmd_del_eth_packet_port( u32 **argv );
void cmd_add_eth_packet_port( u32 **argv );
void cmd_reset_eth_packet_port_cnt( u32 **argv );
void cmd_set_error_bit(u32 **argv);
void cmd_set_bit_mode(u32 **argv);
void cmd_show_send_eth_packet( u32 **argv);
void cmd_reset_eth_send_packet_port_cnt( u32 **argv );
void cmd_show_packet_rate(u32 **argv );
void cmd_reset_packet_rate(u32 **argv );
void cmd_set_rf_freq(u32 **argv);
void cmd_nwk_set_mode(u32 **argv);
void cmd_nwk_2hop_mode(u32 **argv);
void cmd_pl_loopback(u32 **argv);
void cmd_mac_deter_ctrl(u32 **argv);
void cmd_en_chk_dl(u32 **argv);
void cmd_en_chk_ah(u32 **argv);
void cmd_en_chk_ul(u32 **argv);
void cmd_force_hop( u32 **argv);
void cmd_set_port_block( u32 **argv);
void cmd_set_all_port_block( u32 **argv);
void cmd_show_port_info( u32 **argv);
void cmd_remote_port_ctrl_usc1( u32 **argv);
void cmd_remote_port_ctrl_usc2( u32 **argv);
void cmd_remote_all_port_ctrl( u32 **argv);
void cmd_set_ant_ctrl(u32 **argv);
void cmd_set_aes_test(u32 **argv);
void cmd_aes_capture( u32 **argv);
void cmd_aes_ctrl( u32 **argv);
void cmd_aes_ms_ctrl( u32 **argv);
void cmd_m2_add_ctrl_port( u32 **argv );
void cmd_m1_add_ctrl_port( u32 **argv );
void cmd_m2_del_ctrl_port( u32 **argv );
void cmd_m1_del_ctrl_port( u32 **argv );
void cmd_ms_tpc_offset_p( u32 **argv);
void cmd_ms_tpc_offset_m( u32 **argv);
void cmd_ext_dev_reset( u32 **argv);


#endif /*CLI_CMD_H_*/ 
