////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2014 Fumate co.,ltd All Rights Reserved.
//
//  2F Jin Sung B/D 534-6 Noeun-dong, Yuseong-gu, Daejeon, KOREA 305-325
//
////////////////////////////////////////////////////////////////////////////////
//
//  File name  : cli_cmd.c
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

#include <malloc.h>
#include "cli_cmd.h"
 
//CLI_CMDS *pcli_cmds = 0;
CLI_CMDS pcli_cmds[150] = {};
int reg_cmd_cnt =0;

void reg_cli_cmd(char *cmd,void *cb_func){  

  u16 len = strlen(cmd);

	memcpy( (pcli_cmds[reg_cmd_cnt]).cli_cmds, cmd, len );

  //�������� ���� �߰� �� NULL ó��
  (pcli_cmds[reg_cmd_cnt]).cli_cmds[len] = ' ';
  //(pcli_cmds[reg_cmd_cnt]).cli_cmds[len+1] = NULL;  
  
  (pcli_cmds[reg_cmd_cnt]).cb_func = cb_func;


#ifdef FOR_DEBUG
  printf("reg >> %s\n",(pcli_cmds[reg_cmd_cnt]).cli_cmds);
  printf("addr >> 0x%x\n",(u32)(pcli_cmds[reg_cmd_cnt]).cb_func);
#endif

  reg_cmd_cnt++;
  
}

//CMD 
void cli_cmd_init( void ){
   
  reg_cli_cmd(CMD_SHOW_INVENTORY,&cmd_show_inventory);
  reg_cli_cmd(CMD_SHOW_TEMPERATURE,&cmd_show_temperature);
  reg_cli_cmd(CMD_ENABLE_MODEM,&cmd_enable_modem);  
  reg_cli_cmd(CMD_STOP_MODEM,&cmd_stop_modem);   
  reg_cli_cmd(CMD_RF_CTRL,&cmd_rf_ctrl);    
  reg_cli_cmd(CMD_SHOW_XADC_ZYNQ_DUMP,&cmd_show_xadc_zynq_dump);
  reg_cli_cmd(CMD_SHOW_XADC_K7_DUMP,&cmd_show_xadc_k7_dump);  
  reg_cli_cmd(CMD_READ_PLL1,&cmd_read_pll1);    
  reg_cli_cmd(CMD_SET_PLL1,&cmd_set_pll1);
  reg_cli_cmd(CMD_READ_PLL1_DUMP,&cmd_read_pll1_dump);
  reg_cli_cmd(CMD_READ_PLL2,&cmd_read_pll2);
  reg_cli_cmd(CMD_SET_PLL2,&cmd_set_pll2); 
  reg_cli_cmd(CMD_READ_PLL2_DUMP,&cmd_read_pll2_dump);
  reg_cli_cmd(CMD_READ_ADC1,&cmd_read_adc1);
  reg_cli_cmd(CMD_SET_ADC1,&cmd_set_adc1);
  reg_cli_cmd(CMD_READ_ADC1_DUMP,&cmd_read_adc1_dump);   
  reg_cli_cmd(CMD_READ_PLL2,&cmd_read_adc2);    
  reg_cli_cmd(CMD_SET_PLL2,&cmd_set_adc2);
  reg_cli_cmd(CMD_READ_PLL2_DUMP,&cmd_read_adc2_dump);   
  reg_cli_cmd(CMD_READ_DAC1,&cmd_read_dac1);    
  reg_cli_cmd(CMD_SET_DAC1,&cmd_set_dac1);
  reg_cli_cmd(CMD_READ_DAC1_DUMP,&cmd_read_dac1_dump);    
  reg_cli_cmd(CMD_READ_DAC2,&cmd_read_dac2);    
  reg_cli_cmd(CMD_SET_DAC2,&cmd_set_dac2);
  reg_cli_cmd(CMD_READ_DAC2_DUMP,&cmd_read_dac2_dump);    
  reg_cli_cmd(CMD_READ_FPGA,&cmd_read_fpga);    
  reg_cli_cmd(CMD_SET_FPGA,&cmd_set_fpga);
  reg_cli_cmd(CMD_READ_FPGA_DUMP,&cmd_read_fpga_dump);   
  reg_cli_cmd(CMD_READ_FPGA_DECODE,&cmd_read_fpga_decode);     
  reg_cli_cmd(CMD_READ_CPLD,&cmd_read_cpld);    
  reg_cli_cmd(CMD_SET_CPLD,&cmd_set_cpld);
  reg_cli_cmd(CMD_READ_CPLD_DUMP,&cmd_read_cpld_dump);   
  reg_cli_cmd(CMD_ENABLE_FA2,&cmd_enable_fa2);
  reg_cli_cmd(CMD_DISABLE_FA2,&cmd_disable_fa2);  
  reg_cli_cmd(CMD_SET_CLI_FA_MODE,&cmd_set_cli_fa_mode);
  reg_cli_cmd(CMD_NWK_TEST,&cmd_network_test);
  reg_cli_cmd(CMD_ETH_SNED_DBG,&cmd_send_eth_dbg);
  reg_cli_cmd(CMD_ETH_RECV_DBG,&cmd_recv_eth_dbg);
  reg_cli_cmd(CMD_FPGA_RESET,&cmd_fpga_reset);
  reg_cli_cmd(CMD_MS_DAC_INIT,&cmd_ms_dac_init);
  reg_cli_cmd(CMD_MS_ADC_INIT,&cmd_ms_adc_init);  
  reg_cli_cmd(CMD_MS_PLL_INIT,&cmd_ms_pll_init);  
  reg_cli_cmd(CMD_SHOW_MSID,&cmd_show_msid);    
  reg_cli_cmd(CMD_SHOW_INFO,&cmd_show_information);    
  reg_cli_cmd(CMD_CLR_INFO,&cmd_clear_inforamtion);    
  reg_cli_cmd(CMD_SHOW_VER,&cmd_show_version);    
  reg_cli_cmd(CMD_SHOW_SYNC,&cmd_show_sync);    
  reg_cli_cmd(CMD_SET_NODE_MODE,&cmd_set_node_mode);  
  reg_cli_cmd(CMD_SHOW_PWR,&cmd_show_power);  
  reg_cli_cmd(CMD_PLL_RESET,&cmd_pll_reset);  
  reg_cli_cmd(CMD_DAC_RESET,&cmd_dac_reset);    
  reg_cli_cmd(CMD_JTAG_RESET,&cmd_jtag_reset); 
  reg_cli_cmd(CMD_UM_TEST,&cmd_set_um_test);
  reg_cli_cmd(CMD_SHOW_RF,&cmd_show_rf);
  reg_cli_cmd(CMD_SHOW_IOMANAGER_DEBUG,&cmd_show_ioManager_debug);
  reg_cli_cmd(CMD_SET_IOMANAGER_DEBUG,&cmd_set_ioManager_debug);
  reg_cli_cmd(CMD_SHOW_ETH_PACKET,&cmd_show_eth_packet);  
  reg_cli_cmd(CMD_ADD_ETH_PORT,&cmd_add_eth_packet_port);    
  reg_cli_cmd(CMD_DEL_ETH_PORT,&cmd_del_eth_packet_port);    
  reg_cli_cmd(CMD_RESET_PKT_CNT,&cmd_reset_eth_packet_port_cnt);    
  reg_cli_cmd(CMD_BIT_MODE,&cmd_set_bit_mode);    
  reg_cli_cmd(CMD_SET_ERROR_BIT,&cmd_set_error_bit);    
  reg_cli_cmd(CMD_SHOW_SEND_ETH_PACKET,&cmd_show_send_eth_packet); 
  reg_cli_cmd(CMD_RESET_SEND_PKT_CNT,&cmd_reset_eth_send_packet_port_cnt);
  reg_cli_cmd(CMD_SHOW_PACKET_RATE,&cmd_show_packet_rate);
  reg_cli_cmd(CMD_RESET_PACKET_RATE,&cmd_reset_packet_rate);
  reg_cli_cmd(CMD_SET_RF_FREQ,&cmd_set_rf_freq);      
  reg_cli_cmd(CMD_NWK_TOP_MODE,&cmd_nwk_set_mode);      
  reg_cli_cmd(CMD_NWK_2HOP_MODE,&cmd_nwk_2hop_mode);        
  reg_cli_cmd(CMD_PL_LOOPBACK,&cmd_pl_loopback);      
	reg_cli_cmd(CMD_MAC_DETER,&cmd_mac_deter_ctrl);
	
	reg_cli_cmd(CMD_CHECK_MAP_ALIVE,&cmd_en_chk_dl);
	reg_cli_cmd(CMD_CHECK_LQ_ALIVE,&cmd_en_chk_ah);
	reg_cli_cmd(CMD_CHECK_LQ_RPT_ALVIE,&cmd_en_chk_ul);	
	reg_cli_cmd(CMD_FORCE_HOP_CMD,&cmd_force_hop);	

  reg_cli_cmd(CMD_SET_PORT_BLOCK,&cmd_set_port_block);	
  reg_cli_cmd(CMD_SET_ALL_PORT_BLOCK,&cmd_set_all_port_block);	
  reg_cli_cmd(CMD_SHOW_PORT_INFO,&cmd_show_port_info);	
  reg_cli_cmd(CMD_REMOTE_PORT_CTRL_USC1,&cmd_remote_port_ctrl_usc1);	

  reg_cli_cmd(CMD_REMOTE_PORT_CTRL_USC2,&cmd_remote_port_ctrl_usc2);	
  reg_cli_cmd(CMD_REMOTE_PORT_CTRL_ALL,&cmd_remote_all_port_ctrl);	
  reg_cli_cmd(CMD_ANT_ANGLE_TEST_CTRL,&cmd_set_ant_ctrl);

  reg_cli_cmd(CMD_AES_CTRL,&cmd_aes_ctrl);
  reg_cli_cmd(CMD_AES_CAPTURE,&cmd_aes_capture);
  reg_cli_cmd(CMD_AES_MS_CTRL,&cmd_aes_ms_ctrl);

  reg_cli_cmd(CMD_MS1_PORT_ADD_PORT,&cmd_m1_add_ctrl_port);
	reg_cli_cmd(CMD_MS2_PORT_ADD_PORT,&cmd_m2_add_ctrl_port);

	reg_cli_cmd(CMD_MS1_PORT_DEL_PORT,&cmd_m1_del_ctrl_port);
  reg_cli_cmd(CMD_MS2_PORT_DEL_PORT,&cmd_m2_del_ctrl_port);
  reg_cli_cmd(CMD_MS_TPC_P_OFFSET,&cmd_ms_tpc_offset_p);
  reg_cli_cmd(CMD_MS_TPC_M_OFFSET,&cmd_ms_tpc_offset_m);
	reg_cli_cmd(CMD_EXT_DEV_RESET, &cmd_ext_dev_reset);

	

}

void cli_cmd_deinit( void ){

  //if(pcli_cmds != 0)
    //free(pcli_cmds);
   ; 
}
