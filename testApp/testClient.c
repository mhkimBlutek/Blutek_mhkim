#include "testClient.h"

extern void cli_cmd_init( void );

typedef struct _MSNETCONTROL
{
	char				FreqChannel;
	char				txPower;
	char				dlMCSL;
	char				ulMCSL;
	char				ahMCSL;
}__attribute__((packed)) MSNETCONTROL;


enum{
  RF1_ON = 0,
  RF1_OFF,
  RF2_ON,
  RF2_OFF,
  RF_ALL_ON,
  RF_ALL_OFF
};

extern char main_node_mode;
extern char enable_hmac_fa2_flag; //Only USE BS(RCC)
extern char set_cli_hmac_mode;

char g_node_id=0;
char g_node_mode=0;

#define INVENTORY_INFO_PATH "/sys_d/info/inven.txt"

//XADC TEMP >>>
#define XADC_REG_ADDR_ZYNQ_TEMP         0x00
#define XADC_REG_ADDR_K7_TEMP           0x00
#define XADC_TEMP_REF_1                 503.975
#define XADC_TEMP_REF_2                 4096
#define XADC_TEMP_REF_3                 273.15
//XADC TEMP <<<

//XADC POWER >>>
#define XADC_REG_ADDR_ZYNQ_PWR_VCCINT   0x01
#define XADC_REG_ADDR_ZYNQ_PWR_VCCAUX   0x02
#define XADC_REG_ADDR_ZYNQ_PWR_VCCPINT  0x0D
#define XADC_REG_ADDR_ZYNQ_PWR_VCCPAUX  0x0E
#define XADC_REG_ADDR_ZYNQ_PWR_VCCO_DDR 0x0F

#define XADC_REG_ADDR_K7_PWR_VCCINT     0x01
#define XADC_REG_ADDR_K7_PWR_VCCAUX     0x02

#define XADC_REG_ADDR_PWR_REF_1         4096
#define XADC_REG_ADDR_PWR_REF_2         3
//XADC POWER <<<
extern void modem_ctrl_load_config( unsigned char device, const char *file_name );



void modem_phy_write_dl( unsigned char path, unsigned char *dl_map, unsigned char dl_map_length ) {

  char temp_data[MODEMD_MAX_CMD_BUFF];

  memset( temp_data, 0x00, MODEMD_MAX_CMD_BUFF );

  temp_data[2] = HMAC_CMDMODEM_MAP_DL;
  temp_data[3] = dl_map_length;
  memcpy( &temp_data[4], dl_map, dl_map_length );

  //write_to_hmac( path, temp_data, dl_map_length + 4 );
}
extern void modem_ctrl_exec_config( unsigned char device );

void modem_phy_write_ul( unsigned char path, unsigned char *ul_map, unsigned char ul_map_length ) {

  char temp_data[MODEMD_MAX_CMD_BUFF];

  memset( temp_data, 0x00, MODEMD_MAX_CMD_BUFF );

  temp_data[2] = HMAC_CMDMODEM_MAP_UL;
  temp_data[3] = ul_map_length;
  memcpy( &temp_data[4], ul_map, ul_map_length );

  //write_to_hmac( path, temp_data, ul_map_length + 4 );
}


void modem_phy_write_ah( unsigned char path, unsigned char *ah_map, unsigned char ah_map_length ) {

  char temp_data[MODEMD_MAX_CMD_BUFF];

  memset( temp_data, 0x00, MODEMD_MAX_CMD_BUFF );

  temp_data[2] = HMAC_CMDMODEM_MAP_AH;
  temp_data[3] = ah_map_length;
  memcpy( &temp_data[4], ah_map, ah_map_length );

//   write_to_hmac( path, temp_data, ah_map_length + 4 );
}
 
void modem_phy_write_ms_id( unsigned char path, unsigned char cmd, unsigned char ms_id, unsigned char ms_mode) {

  unsigned char temp_data[MODEMD_MAX_CMD_BUFF];

  memset( temp_data, 0x00, MODEMD_MAX_CMD_BUFF );

  temp_data[2] = cmd;
  temp_data[3] = ms_id;
  temp_data[4] = ms_mode;

//   write_to_hmac( path, (char*)temp_data, 5 );
}

void modem_phy_write_node_cnt( unsigned char path, unsigned char ms_cnt ) {

  unsigned char temp_data[MODEMD_MAX_CMD_BUFF];

  memset( temp_data, 0x00, MODEMD_MAX_CMD_BUFF );

  temp_data[2] = HMAC_CMDNODE_CNT;
  temp_data[3] = ms_cnt;

//   write_to_hmac( path, (char*)temp_data, 4 );
}

void enable_modem(unsigned char path){

  char temp_data[MODEMD_MAX_CMD_BUFF];
  memset( temp_data, 0x00, MODEMD_MAX_CMD_BUFF );
  
  temp_data[2] = HMAC_CMDMODEM_START;
  temp_data[3] = PIPE_FLAG_TRUE;

//   write_to_hmac( path, temp_data, MODEMD_MAX_CMD_BUFF );
}

void disable_modem(unsigned char path){

  char temp_data[MODEMD_MAX_CMD_BUFF];

  memset( temp_data, 0x00, MODEMD_MAX_CMD_BUFF );
  temp_data[2] = HMAC_CMDMODEM_START;
  temp_data[3] = PIPE_FLAG_FALSE;

//   write_to_hmac( path, temp_data, MODEMD_MAX_CMD_BUFF );
}
#define SET_NODE_ID_CONFIG_FILE "/home/set_node_id.txt"
int modem_node_id_load_config(unsigned int *node_type, unsigned int *node_id, unsigned int *node_mode){

	FILE *fp;
  char line[MCSPI_BUFFER_SIZE];
  char *tok = NULL;
  signed char result=SUCCESS;
	
  fp = fopen( SET_NODE_ID_CONFIG_FILE, "r" );
  if( fp == NULL ) {
    printf( "%s open failed. Set NODE_ID Config File.\n", SET_NODE_ID_CONFIG_FILE );
    result = FAIL;
  } 
  else
  {
	  // Read line unit
	  while( fgets(line, sizeof(line), fp) != NULL ) {
	  
	    // ���Ͽ��� ������ �б�
	    tok = (char *)strtok( line, ":" );
	    if( tok != NULL )
	      if(sscanf( tok, "%d", node_type ) < 0){
				printf("sscanf error\n");
			}

	    tok = (char *)strtok( NULL, ":" );
	    if( tok != NULL )
	      if(sscanf( tok, "%d", node_id ) < 0){
				printf("sscanf error\n");
		   }

	    if(*node_type == 1) {// MS
	      tok = (char *)strtok( NULL, ":" );
	      if( tok != NULL )
	        if(sscanf( tok, "%d", node_mode ) < 0){
				printf("sscanf error\n");
			  }
			}
	  }

	  fclose(fp);  
  }
  return (int)result;
  
}

void auto_set_node_id(){
  
  unsigned char ah_map[PHY_LENGTH_AH_MAP] = {};
  unsigned char dl_map[PHY_LENGTH_DL_MAP];
  unsigned char ul_map[PHY_LENGTH_UL_MAP];
  unsigned char i;
  unsigned int node_type = 0;
  unsigned int node_id = 0;
  unsigned int node_mode = 0;
  unsigned char path=TO_FA1_MAC;
  
  if(modem_node_id_load_config(&node_type, &node_id, &node_mode) >= 0)
	{

  	dl_map[0] = 0;
		dl_map[1] = 0;
	
		for( i=0; i<PHY_LENGTH_REGION_UL; i++)
			ul_map[i] = 0xFF;
	
		// BS operation
		if(node_type == 0) {
			path = TO_ALL_MAC;
			modem_phy_write_node_cnt(path, (unsigned char)node_id);
			usleep(500);
			modem_phy_write_ms_id(path, HMAC_CMDMODEM_MS_ID_WRITE, (unsigned char)node_id, (unsigned char)node_mode );
		}
		if(node_type == 1) {
			path = TO_FA1_MAC;
			modem_phy_write_ms_id(path, HMAC_CMDMODEM_MS_ID_WRITE, (unsigned char)node_id, (unsigned char)node_mode );
		}		
		if(node_type == 2){
			path = TO_FA1_MAC;
			modem_phy_write_ms_id(path, HMAC_CMDMODEM_RS_ID_WRITE, (unsigned char)node_id, (unsigned char)node_mode );
		}

		if(node_type == 0){
			for( i=0; i<PHY_LENGTH_AH_MAP; i++)
				ah_map[i] = i;
		}else{
			for( i=0; i<PHY_LENGTH_AH_MAP; i++)
				ah_map[i] = 0x00;
		}
				
		usleep( 700 );
		modem_phy_write_ah( path, ah_map, PHY_LENGTH_AH_MAP );
		usleep( 700 );
		modem_phy_write_dl( path, dl_map, 2 );
		usleep( 700 );
		modem_phy_write_ul( path, ul_map, PHY_LENGTH_REGION_UL );
	
		printf("[AUTO SET NODE ID] type : %d, id : %d, ms_mode : %d\n",node_type,node_id, node_mode);
	
		g_node_id 	= node_id;
		g_node_mode = node_mode;

		enable_modem(path);
 
	}
  
}


int modem_inventory_info_loading(char (*pdate)[64]){

	FILE *fp;
  char line[MCSPI_BUFFER_SIZE];
  char fic_data[MCSPI_BUFFER_SIZE];
  int  fic_cnt=0;
  int  inven_cnt=0;
  char *tok = NULL;
  int i;
	signed char result=SUCCESS;
	
  fp = fopen( INVENTORY_INFO_PATH, "r" );
  if( fp == NULL ) {
    printf( "%s open failed. Inventory information File.\n", INVENTORY_INFO_PATH );
		result = FAIL;

  } 
	else
	{
		// Read line unit
		while( fgets(line, sizeof(line), fp) != NULL ) {
		
			//���� ����
			for(i=0; i < MCSPI_BUFFER_SIZE ; i++){		
				if(line[i] != ' '){
						fic_data[fic_cnt] = line[i];
						fic_cnt++;
					} 		 
			}
			
			fic_cnt=0;
			
			// ���Ͽ��� ������ �б�
			tok = (char *)strtok( fic_data, ":" );		
			tok = (char *)strtok( NULL, ":" );
			if( tok != NULL ){
				strcpy( pdate[inven_cnt],tok);			
				inven_cnt++;
			}
		}
		fclose(fp);  
	}
  return (int)result;  
}

void read_zynq_xadc_reg(u16 addr,u16 *value){

  int fd =0;
  struct CPLD_CMD_t command;  

  // ����̽� ����
  fd = open( INIT_DEV_NAME, O_RDWR );
  if( fd < 0 ) {
    printf( "%s open device failed. \n", INIT_DEV_NAME );    
    
  }
	else
	{
		command.command_id = INIT_DEV_ZYNQ_XADC;
		command.address = (addr * 0x08);	

		if(read( fd, (char *)&command, sizeof(struct CPLD_CMD_t) ) < 0){
			command.address =0;
		}
	    
		*value = command.value; 
		close( fd );
	}
}

void read_k7_xadc_reg(u16 addr, u16 *value){

  int fd =0;
  struct CPLD_CMD_t command;  

  // ����̽� ����
  fd = open( INIT_DEV_NAME, O_RDWR );
  if( fd < 0 ) {
    printf( "%s open device failed. \n", INIT_DEV_NAME );    
  }
	else
	{
	  command.command_id = INIT_DEV_K7_XADC;
	  command.address = (addr * 0x08);  
	 if( read( fd, (char *)&command, sizeof(struct CPLD_CMD_t) ) < 0){
	 	command.address = 0;
	 }
	  *value = command.value; 
	  close( fd );
	}
}


void ms_rf_ctrl_func(u8 cmd){

  struct CPLD_CMD_t command;
  int fd = 0;
  int result=TRUE;
  // ����̽� ����
  fd = open( INIT_DEV_NAME, O_RDWR );
  if( fd < 0 ) {
    printf( "%s open device failed. \n", INIT_DEV_NAME );    
  }
	else
  {
		command.command_id = INIT_DEV_CONTRL;
		command.address = 0x0000; 	 
		
		if(read( fd, (char*)&command, sizeof(struct CPLD_CMD_t) ) < 0){
			result = FALSE;
		}
		
		
		switch(cmd){
			case RF1_ON:
				command.value &= 0xFE;
			break;
			case RF1_OFF:
				command.value |= 0x01;	
			break;
			case RF2_ON:
				command.value &= 0xFD;	
			break;
			case RF2_OFF:
				command.value |= 0x02;	
			break;
			case RF_ALL_ON: 
				command.value = 0x00;  
			break;
			case RF_ALL_OFF:
				command.value = 0x03;  
			break;
			default:
				 printf("rf ctrl error\n");
			break;
		}
		
		if(write(fd, (char*)&command, sizeof(struct CPLD_CMD_t) ) < 0){
			result = FALSE;
		}
		
		close(fd);

  }
  if(result == FALSE){
		command.value=0;
  }

}

void modem_create( void ) {

  printf( "MODEMD initialization\n" );
 
}

void init_check_devExist(){


  char dev_name[256];

  if(main_node_mode == MAIN_NODE_MODE_BS){
    strcpy(dev_name,MCSPI_DEV_NAME_BS); 
  }
  else{
    strcpy(dev_name,MCSPI_DEV_NAME_MS);
  }
  
//   if( check_devExist(dev_name, MCSPI_DEV_MAJOR, MCSPI_DEV_MINOR) < 0 ) {

//     printf( "%s created\n", dev_name );
//   }


//   if( check_devExist(INIT_DEV_NAME, INIT_DEV_MAJOR, INIT_DEV_MINOR) < 0 ) {

//     printf( "%s created\n", INIT_DEV_NAME );
//   }


//   if( check_devExist(CPLD_DEV_NAME, CPLD_DEV_MAJOR, CPLD_DEV_MINOR) < 0 ) {
//     printf( "%s created\n", CPLD_DEV_NAME );
//   }

//   if( check_devExist(FPGA_DEV_NAME, FPGA_DEV_MAJOR, FPGA_DEV_MINOR) < 0 ) {

//     printf( "%s created\n", FPGA_DEV_NAME );
//   }
  
//   if( check_devExist(SPI_DEV_NAME, SPI_DEV_MAJOR, SPI_DEV_MINOR) < 0 ) {

//     printf( "%s created\n", SPI_DEV_NAME );
//   }
  
//   if( check_devExist(NETWORK_DEV_NAME_FA1, NETWORK_DEV_MAJOR_FA1, NETWORK_DEV_MINOR_FA1) < 0 ) {

//     printf( "%s created\n", NETWORK_DEV_NAME_FA1 );
//   }
  
//   if( check_devExist(NETWORK_DEV_NAME_FA2, NETWORK_DEV_MAJOR_FA2, NETWORK_DEV_MINOR_FA2) < 0 ) {

//     printf( "%s created\n", NETWORK_DEV_NAME_FA2 );
//   }
}

#define DAC_INIT_RETRY 3
// void dac_init_proc(){

//   unsigned char read_value=0;
//   unsigned char dac_init_flag=0;
//   int i;

//   modem_ctrl_mcspi_write( MCSPI_TARGET_DAC1, 0x10, 0x48 );   
  
//   for(i=0; i < DAC_INIT_RETRY ; i++){
    
//     modem_ctrl_mcspi_write( MCSPI_TARGET_DAC1, 0x17, 0x05 );   
//     modem_ctrl_mcspi_write( MCSPI_TARGET_DAC1, 0x18, 0x02 );    

//     read_value = modem_mcspi_read_raw(MCSPI_TARGET_DAC1, 0x18);
//     printf("dac write value : ADDR: 0x18, VALUE: 0x02\n");
//     printf("dac read value : ADDR: 0x18, 2 bit value %d\n", (read_value & 0x04) >> 2);
    
//     if((read_value & 0x04) != 0x04){
//       printf("retry dac init..\n");
//       continue;    
//     }       
    
//     modem_ctrl_mcspi_write( MCSPI_TARGET_DAC1, 0x18, 0x00 );
//     read_value = modem_mcspi_read_raw(MCSPI_TARGET_DAC1, 0x18);    

//     printf("dac write value : ADDR: 0x18, VALUE: 0x00\n");
//     printf("adc read value : ADDR: 0x18, 2 bit value %d\n", (read_value & 0x04) >> 2);
    
//     if((read_value & 0x04) == 0x04){
//       printf("retry dac init..\n"); 
//       continue;    
//     } 
    
//     read_value = modem_mcspi_read_raw(MCSPI_TARGET_DAC1, 0x19);    
//     printf("dac read value : ADDR: 0x19, 0x%x\n",read_value);
    
//     if(!(read_value == 0x0F || read_value == 0x07)){
//       printf("retry dac init..\n");   
//       continue;    
//     }else{       
//       dac_init_flag  = 1;
//       break;
//     } 
//   }


//   if(dac_init_flag == 1)printf("dac init success.\n");
//   else printf("dac init fail.\n");
  

//   if(main_node_mode == MAIN_NODE_MODE_BS){   

//   dac_init_flag = 0;  

//   modem_ctrl_mcspi_write( MCSPI_TARGET_DAC1, 0x10, 0x48 );   
  
//   for(i=0; i < DAC_INIT_RETRY ; i++){
    
//     modem_ctrl_mcspi_write( MCSPI_TARGET_DAC2, 0x17, 0x05 );   
//     modem_ctrl_mcspi_write( MCSPI_TARGET_DAC2, 0x18, 0x02 );    

//     read_value = modem_mcspi_read_raw(MCSPI_TARGET_DAC2, 0x18);
//     printf("dac2 write value : ADDR: 0x18, VALUE: 0x02\n");
//     printf("dac2 read value : ADDR: 0x18, 2 bit value %d\n", (read_value & 0x04) >> 2);
    
//     if((read_value & 0x04) != 0x04){
//       printf("retry dac init..\n");
//       continue;    
//     }       
    
//     modem_ctrl_mcspi_write( MCSPI_TARGET_DAC2, 0x18, 0x00 );
//     read_value = modem_mcspi_read_raw(MCSPI_TARGET_DAC2, 0x18);    

//     printf("dac2 write value : ADDR: 0x18, VALUE: 0x00\n");
//     printf("adc2 read value : ADDR: 0x18, 2 bit value %d\n", (read_value & 0x04) >> 2);
    
//     if((read_value & 0x04) == 0x04){
//       printf("retry dac init..\n"); 
//       continue;    
//     } 
    
//     read_value = modem_mcspi_read_raw(MCSPI_TARGET_DAC2, 0x19);    
//     printf("dac2 read value : ADDR: 0x19, 0x%x\n",read_value);
    
//     if(!(read_value == 0x0F || read_value == 0x07)){
//       printf("retry dac2 init..\n");   
//       continue;    
//     }else{       
//       dac_init_flag  = 1;
//       break;
//     }  
//   }

// #if 0
// 	modem_ctrl_mcspi_write( MCSPI_TARGET_DAC1, 0x40, 0x01 );
// 	modem_ctrl_mcspi_write( MCSPI_TARGET_DAC1, 0x45, 0x01 );
	
//   modem_ctrl_mcspi_write( MCSPI_TARGET_DAC1, 0x41, 0xA0 );		
// 	modem_ctrl_mcspi_write( MCSPI_TARGET_DAC1, 0x44, 0xA0 );		

// 	modem_ctrl_mcspi_write( MCSPI_TARGET_DAC2, 0x40, 0x01 );
// 	modem_ctrl_mcspi_write( MCSPI_TARGET_DAC2, 0x45, 0x01 );
	
//   modem_ctrl_mcspi_write( MCSPI_TARGET_DAC2, 0x41, 0xA0 );		
// 	modem_ctrl_mcspi_write( MCSPI_TARGET_DAC2, 0x44, 0xA0 );	
	
// #endif

//   if(dac_init_flag == 1)printf("dac2 init success.\n");
//   else printf("dac2 init fail.\n");
//   }
  
// }
//
//  Allocate new MODEMD structure and set default value.
//


 void ver_print( void ){

  printf("\n-------------------------------------------------------------\n");
  printf("                       MODEMD COMPAILE INFO\n");  
  printf("-------------------------------------------------------------\n");  
  printf("DATE : %s, TIME : %s\n",__DATE__,__TIME__);
  printf("-------------------------------------------------------------\n"); 
}


void fpga_ver_print( void ){

  unsigned int a = 0;
  unsigned int read_value = a;  //modem_ctrl_fpga_read_raw( 0x00 );
  printf("\n-------------------------------------------------------------\n");
  printf("                       FPGA VERSION INFO\n");  
  printf("-------------------------------------------------------------\n");  

  if(main_node_mode == MAIN_NODE_MODE_BS){
    printf("TOOL VER : %d.%d\n",(read_value & 0xF000) >> 12,(read_value & 0x0F00) >> 8);  
    printf("PROG VER : %d.%d\n",(read_value & 0x00F0) >> 4,(read_value & 0x000F));
    
  }else{
    printf("TOOL VER : %d.%d\n",(read_value & 0x00F0) >> 4,(read_value & 0x000F));
    printf("PROG VER : %d.%d\n",(read_value & 0xF000) >> 12,(read_value & 0x0F00) >> 8);  
  }
  
  printf("-------------------------------------------------------------\n");   

} 
void modem_init_rf_on( void ){
  ms_rf_ctrl_func(RF1_ON);
  sleep(1);
  ms_rf_ctrl_func(RF2_ON);
}


// void modem_ext_reset( void ){


//   printf("ext dev reset...\n");
// 	modem_ctrl_cpld_write( 0x0040,0xffef	);
// 	sleep(1);
// 	modem_ctrl_cpld_write( 0x0040,0xffff	);

// 	modem_ctrl_load_config( MCSPI_TARGET_PLL1, PLL1_CONFIG_FILE );	 
// 	modem_ctrl_load_config( MCSPI_TARGET_PLL2, PLL2_CONFIG_FILE );	 
// 	modem_ctrl_exec_config( MCSPI_TARGET_PLL1 );
// 	modem_ctrl_exec_config( MCSPI_TARGET_PLL2 );	 

// 	printf("ext dev reset compelete.\n");

// }


void modem_init( void ) {

  cli_cmd_init();
//   cli_ipc_init();

  init_check_devExist(); //디바이스 장치 생성

 

 // modem_hmac_init_fa1(); // 스레드 동기화 시켜주고, 디바이스 이름 가져와서 파일이나 디바이스를 응용 프로그램의 주소 공간 메모리에 대응시키고, 스레드 까준다.
  if(main_node_mode == MAIN_NODE_MODE_BS) { // MMUSV
    printf("@@@@@ modem_init(): FA2 enable !!!! \n");
    enable_hmac_fa2_flag = 1;
    // modem_hmac_init_fa2(); 
  }
  
//   eth_init();  //이더넷 통신 
//   modem_iomgr_sock_init(); //
//   if(main_node_mode == MAIN_NODE_MODE_MS){
//     modem_init_rf_on();
//   }
    
//   modem_ctrl_init();

//   //hdshin 131119
//   dac_init_proc();
//   ver_print();   
//   fpga_ver_print();

//   init_ip_frag_mgmt();


}


void modem_deinit( void ) {

  printf( "terminate modemd\n" );
 
//   eth_deinit();
//   modem_hmac_deinit_fa1();
//   if(enable_hmac_fa2_flag){
//     modem_hmac_deinit_fa2();
//   }
//   printf("hmac deinit\n");
//  // modem_ctrl_deinit();
//   modem_iomgr_sock_deinit();  

}

void modem_get_node_info(char *id, char *mode){

  *id   = g_node_id;
  *mode = g_node_mode;
}


void cmd_show_inventory( u32 **argv )
{

  char inventory_info_data[5][64];
  modem_inventory_info_loading(inventory_info_data);
  printf("\n--------------------------------------\n");
  printf("        INVENTORY INFORMATION         \n");
  printf("--------------------------------------\n");
  printf("Manufacturer         : %s\n",inventory_info_data[0]);
  printf("Line Item            : %s\n",inventory_info_data[1]);  
  printf("Item Number          : %s\n",inventory_info_data[2]);  
  printf("Date of Manufacture  : %s\n",inventory_info_data[3]);  
  printf("Last Update          : %s\n",inventory_info_data[4]);  

}



void cmd_show_temperature( u32 **argv )
{
  
  u8 code_temp_ref1;
  u8 code_temp_ref2;
  u16 code_temp = 0;
  u16 value = 0;
  float dac_temp = 0.0;
  float zynq_temp = 0;
  float k7_temp = 0;
  
  //---------------------------------------------------------------//
  //                        DAC TEMPERATURE
  //---------------------------------------------------------------//
//   code_temp_ref1 = modem_mcspi_read_raw(MCSPI_TARGET_DAC1, 0x49);    
//   code_temp_ref2 = modem_mcspi_read_raw(MCSPI_TARGET_DAC1, 0x4A);    

//   code_temp = (0x00FF & code_temp_ref1);
//   code_temp |= 0xFF00 & (code_temp_ref2 << 8); 


//   dac_temp = DAC_T_REF + (7.7*(code_temp - DAC_CODE_REF))/1000 + 1;
  //---------------------------------------------------------------//
  //                       XADC TEMPERATURE (ZYNQ)
  //---------------------------------------------------------------//
  read_zynq_xadc_reg(XADC_REG_ADDR_ZYNQ_TEMP,&value);

  value = (value >> 4); //���� 12 bit �̿�

  zynq_temp = ((float)(value * XADC_TEMP_REF_1)/(float)XADC_TEMP_REF_2) - XADC_TEMP_REF_3;
  
  //---------------------------------------------------------------//
  //                       XADC TEMPERATURE (K7)
  //---------------------------------------------------------------//
  read_k7_xadc_reg(XADC_REG_ADDR_ZYNQ_TEMP,&value);

  value = (value >> 4); //���� 12 bit �̿�
  k7_temp = ((float)(value * XADC_TEMP_REF_1)/(float)XADC_TEMP_REF_2) - XADC_TEMP_REF_3;

  
  printf("\n--------------------------------------\n");
  printf("          DEVICE TEMPERATURE          \n");
  printf("--------------------------------------\n");
  printf("ZYNQ_XADC   : %0.1f��C\n",zynq_temp);
  printf("K7_XADC     : %0.1f��C\n",k7_temp);  
 // printf("DAC         : %0.1f��C\n",dac_temp);

 
}

void cmd_show_power( u32 **argv ){
  u16   value = 0;
  float pwr_result = 0.0;
  
  printf("\n--------------------------------------\n");
  printf("          XADC POWER (ZYNQ)         \n");
  printf("--------------------------------------\n");
  
  read_zynq_xadc_reg(XADC_REG_ADDR_ZYNQ_PWR_VCCINT,&value);

  value = (value >> 4);
  pwr_result = ((float)value/(float)XADC_REG_ADDR_PWR_REF_1) * XADC_REG_ADDR_PWR_REF_2;
  
  printf("Z_VCCINT   : %0.2f\n",pwr_result);

  read_zynq_xadc_reg(XADC_REG_ADDR_ZYNQ_PWR_VCCAUX,&value);
  value = (value >> 4);
  pwr_result = ((float)value/(float)XADC_REG_ADDR_PWR_REF_1) * XADC_REG_ADDR_PWR_REF_2;
  
  printf("Z_VCCAUX   : %0.2f\n",pwr_result);

  read_zynq_xadc_reg(XADC_REG_ADDR_ZYNQ_PWR_VCCPINT,&value);
  value = (value >> 4);
  pwr_result = ((float)value/(float)XADC_REG_ADDR_PWR_REF_1) * XADC_REG_ADDR_PWR_REF_2;
  
  printf("Z_VCCPINT  : %0.2f\n",pwr_result);
  
  read_zynq_xadc_reg(XADC_REG_ADDR_ZYNQ_PWR_VCCPAUX,&value);
  value = (value >> 4);
  pwr_result = ((float)value/(float)XADC_REG_ADDR_PWR_REF_1) * XADC_REG_ADDR_PWR_REF_2;
  
  printf("Z_VCCPAUX  : %0.2f\n",pwr_result);  

  read_zynq_xadc_reg(XADC_REG_ADDR_ZYNQ_PWR_VCCO_DDR,&value);
  value = (value >> 4);
  pwr_result = ((float)value/(float)XADC_REG_ADDR_PWR_REF_1) * XADC_REG_ADDR_PWR_REF_2;
  
  printf("Z_VCCO_DDR : %0.2f\n",pwr_result);  

  read_k7_xadc_reg(XADC_REG_ADDR_K7_PWR_VCCINT,&value);
  value = (value >> 4);
  pwr_result = ((float)value/(float)XADC_REG_ADDR_PWR_REF_1) * XADC_REG_ADDR_PWR_REF_2;

  printf("\n--------------------------------------\n");
  printf("          XADC POWER (K7)         \n");
  printf("--------------------------------------\n");  
   
  printf("K_VCCINT   : %0.2f\n",pwr_result); 

  read_k7_xadc_reg(XADC_REG_ADDR_K7_PWR_VCCAUX,&value);
  value = (value >> 4);
  pwr_result = ((float)value/(float)XADC_REG_ADDR_PWR_REF_1) * XADC_REG_ADDR_PWR_REF_2;
  
  printf("K_VCCAUX   : %0.2f\n",pwr_result); 
  
}

void cmd_enable_modem( u32 **argv )
{
  unsigned char path = TO_FA1_MAC;
//   if(set_cli_hmac_mode == HMAC_MODE_FA2)
//     path = TO_FA2_MAC;
    
  printf("Start modem functionality\n");
  enable_modem(path);
}


void cmd_stop_modem( u32 **argv )
{
  unsigned char path = TO_FA1_MAC;
//   if(set_cli_hmac_mode == HMAC_MODE_FA2)
//     path = TO_FA2_MAC;

  printf("Stop modem functionality\n");
  disable_modem(path);
}



void cmd_rf_ctrl( u32 **argv ) 
{
  
  if(strcmp((u8 *)argv[0],"on") == 0){
      ms_rf_ctrl_func(RF1_ON);
      sleep(1);
      ms_rf_ctrl_func(RF2_ON); 
    printf("ms rf all on\n");
  }

  if(strcmp((u8 *)argv[0],"off") == 0){
    ms_rf_ctrl_func(RF_ALL_OFF);
    printf("ms rf all off\n");    
  } 

  if(strcmp((u8 *)argv[0],"1_on") == 0){
    ms_rf_ctrl_func(RF1_ON);
    printf("ms rf 1 on\n");    
  } 
  
  if(strcmp((u8 *)argv[0],"1_off") == 0){
    ms_rf_ctrl_func(RF1_OFF);
    printf("ms rf 1 off\n");    
  }

  if(strcmp((u8 *)argv[0],"2_on") == 0){
    ms_rf_ctrl_func(RF2_ON);
    printf("ms rf 2 on\n");    
  }  
  
  if(strcmp((u8 *)argv[0],"2_off") == 0){
    ms_rf_ctrl_func(RF2_OFF);
    printf("ms rf 2 off\n");    
  }    

}


void read_zynq_xadc_dump(){

  struct FPGA_CMD_t command;
  int fd;
  int i=0; 
  int add_cnt =0;
	int ret = 0;

  fd = open( INIT_DEV_NAME, O_RDWR );
  if( fd < 0 ) {
    printf( "%s open failed\n", INIT_DEV_NAME );

  }
	else{
  
	  command.command_id  = INIT_DEV_ZYNQ_XADC;
	  command.address     = 0;
	  command.value       = 0;

	  for( /*i=0*/ ; i < (127 * 8) ; i += 8 ) {

	    command.address  = i; 
	    
	    ret = read( fd, (char *)&command, sizeof(struct FPGA_CMD_t) );
			if(ret == -1)
				ret = 0;

	    if( (i % 64 ) == 0 ) { 
	      printf("\n%03x: ", (add_cnt + 0x200)); 
	    }
	    add_cnt += 0x4;
	    printf("  %04x ", command.value ); 
	  } 

	  printf("\n\n");
	}

}

void read_k7_xadc_dump(){

  struct FPGA_CMD_t command;
  int fd;
  int i=0; 
  int add_cnt =0;
	int ret = 0;
	
  fd = open( INIT_DEV_NAME, O_RDWR );
  if( fd < 0 ) {
    printf( "%s open failed\n", INIT_DEV_NAME );

  }
	else{
  
	  command.command_id  = INIT_DEV_K7_XADC;
	  command.address     = 0;
	  command.value       = 0;

	   for( /*i=0*/ ; i < (127 * 8) ; i += 8 ) {

	    command.address  = i; 
	    
	   ret = read( fd, (char *)&command, sizeof(struct FPGA_CMD_t) );
		if(ret == -1)
				ret = 0;
		
	    if( (i % 64 ) == 0 ) { 
	      printf("\n%03x: ", (add_cnt + 0x200)); 
	    }
	    add_cnt += 0x4;
	    printf("  %04x ", command.value ); 
	  } 

	  printf("\n\n");
	}

}


void cmd_show_xadc_zynq_dump( u32 **argv)
{ 
  read_zynq_xadc_dump();
}
 
void cmd_show_xadc_k7_dump( u32 **argv)
{
  read_k7_xadc_dump();
}


// ADD ==================================================================================
void cmd_read_pll1( u32 **argv)        
{
  unsigned int address = 0;
  printf("PLL1 read address 0x%x\n",address);

}

void cmd_set_pll1( u32 **argv)        
{
  unsigned int address = 0, value = 0;
  printf("PLL1 set address 0x%x 0x%x\n",address,value);
}

void cmd_read_pll1_dump( u32 **argv)        
{
  modem_ctrl_mcspi_dump( MCSPI_TARGET_PLL1 );
}


