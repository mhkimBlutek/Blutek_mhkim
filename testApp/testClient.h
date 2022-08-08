

//===== ADD ====//
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>

typedef unsigned int   u32;
typedef unsigned short u16;
typedef unsigned char  u8;


#define MCSPI_BUFFER_SIZE               128
#define PHY_LENGTH_DL_MAP               13 //7
#define PHY_LENGTH_UL_MAP               91 //42
#define PHY_LENGTH_AH_MAP               8 //6
#define PHY_LENGTH_REGION_UL            7
#define INIT_DEV_NAME         "/dev/init_dev"
#define MCSPI_DEV_NAME_BS               "/dev/mcspi1.0.bs"   // MCSPI file name
#define MCSPI_DEV_NAME_MS               "/dev/mcspi1.0.ms"   // MCSPI file name

#define MCSPI_DEV_MAJOR                 250                  // MCSPI major number
#define MCSPI_DEV_MINOR                 0                    // MCSPI minor number


#define INIT_DEV_NAME         "/dev/init_dev"
#ifdef KERNEL_3_8
#define INIT_DEV_MAJOR        260
#else
#define INIT_DEV_MAJOR        251
#endif
#define INIT_DEV_MINOR        0

struct FPGA_CMD_t {
  char          command_id;                     // Command ID
  unsigned int  address;                        // Address
  unsigned int  value;                          // Read/write value
};

// CPLD command structure
struct CPLD_CMD_t {
  char          command_id;                     // Command ID
  unsigned int  address;                        // Address
  unsigned int  value;                          // Read/Write value
};

#define MCSPI_TARGET_ADC1               0x00
#define MCSPI_TARGET_ADC2               0x01
#define MCSPI_TARGET_DAC1               0x02
#define MCSPI_TARGET_DAC2               0x03
#define MCSPI_TARGET_PLL1               0x04
#define MCSPI_TARGET_PLL2               0x05
#define GPMC_TARGET_FPGA_RX             0x06
#define GPMC_TARGET_FPGA_TX             0x07
#define GPMC_TARGET_CPLD                0x08

#define ETH_ALLOW_TCP_PORT              0x09
#define ETH_ALLOW_UDP_PORT              0x10





//===== ADD ====//









/* MODEM port number. */
#define MODEM_VTY_PORT                  2602
#define MODEM_VTYSH_PATH                "/tmp/.modemd"

/* Default configuration file name. */
#define MODEMD_DEFAULT_CONFIG           "modemd.conf"



#define TO_FA1_MAC    0x0
#define TO_FA2_MAC    0x1
#define TO_ALL_MAC    0x2

#define TRUE  1
#define FALSE 2 
#define FORWARD 3

#define SUCCESS 0
#define FAIL -1

/* function define */
#define REMOTE_CTRL_PORT_FUNC //hdshin 170710 


#define REPORT_USV_INFO //hdshin 170926

/* Pipe protocol type */
#define HMAC_CMDMODEM_INIT            0x01
#define HMAC_CMDMODEM_START           0x02
#define HMAC_CMDMODEM_CONFIG          0x03
#define HMAC_CMDMODEM_MAP_DL          0x04
#define HMAC_CMDMODEM_MAP_UL          0x05
#define HMAC_CMDMODEM_MAP_AH          0x06
#define HMAC_CMDMODEM_MAP_READ        0x07
#define HMAC_CMDMODEM_MS_ID_READ      0x08
#define HMAC_CMDMODEM_MS_ID_WRITE     0x09
#define HMAC_CMDMODEM_INFO            0x0A
#define HMAC_CMDMODEM_BUFFER          0x0B
#define HMAC_CMDMODEM_IPC_BUFFER      0x0C
#define HMAC_CMDMODEM_READ_VERSION    0x0D
#define HMAC_CMDMODEM_READ_SYNC       0x0E
#define HMAC_CMDMODEM_CHILD_WRITE     0x0F
#define HMAC_CMDMODEM_CHILD_READ      0x10
#define HMAC_CMDMODEM_AGC_WRITE       0x11
#define HMAC_CMDMODEM_TXP_WRITE       0x12
#define HMAC_CMDMODEM_RX_MODE         0x13
#define HMAC_CMDMODEM_MAP_CLEAR       0x14
#define HMAC_CMDMODEM_RF_CTRL_READ    0x15
#define HMAC_CMDMODEM_AGC_MODE        0x16
#define HMAC_CMDMODEM_MAP_SET_MIMO    0x17
#define HMAC_CMDMODEM_MAP_SET_MCS     0x18
#define HMAC_CMDMODEM_TPC_MODE        0x19
#define HMAC_CMDMODEM_TPC_PARAM       0x1A
#define HMAC_CMDMODEM_MIMO_MODE       0x1B
#define HMAC_CMDMODEM_CRC             0x1C
#define HMAC_CMDUART_READ             0x20
#define HMAC_CMDUART_WRITE            0x21
#define HMAC_CMDUART_SEND             0x22
#define HMAC_CMDUART_MODE             0x23
#define HMAC_CMDUART_INTERVAL         0x24
#define HMAC_CMDTEST_SEND_DATA        0x30
#define HMAC_CMDTEST_TX               0x31
#define HMAC_CMDMCBSP_READ            0x40
#define HMAC_CMDMCBSP_WRITE           0x41
#define HMAC_CMDMCBSP_REG_DUMP        0x42
#define HMAC_CMDMCBSP_THRESHOLD       0x43
#define HMAC_CMDMCBSP_REMAIN_BYTE     0x44
#define HMAC_CMDMAC_FRAME_SEQ         0x50
#define HMAC_CMDICD_NET_CTRL          0x60
#define HMAC_CMDICD_NET_INFO          0x61
#define HMAC_CMDSIGNALING_SEND        0x70
#define HMAC_CMDMODEM_CRC_INFO        0x80
#define HMAC_CMDMODEM_SET_ANT         0x81
#define HMAC_CMDMODEM_UL_ONOFF        0x82
#define HMAC_CMDMODEM_SET_RANGING     0x83
#define HMAC_CMDSIMUL_MODE_SWITCH     0x84

#define HMAC_CMDDSP_RX_CHKSUM_ON      0x85
#define HMAC_CMDDSP_RX_CHKSUM_OFF     0x86
#define HMAC_CMDDSP_CRC_CHKSUM_ON     0x87
#define HMAC_CMDDSP_CRC_CHKSUM_OFF    0x88

#define HMAC_CMDMODEM_MAP_SET_MCS_UL_RX	0x90
#define HMAC_CMDALC_RF_GAIN_CTRL      0x91
#define HMAC_CMDNODE_CNT              0x92
#define HMAC_CMDAGC_MODE              0x93
#define HMAC_CMDNETWORK_DEBUG         0x94
#define HMAC_HMAC_SCH_MSG             0x95
#define HMAC_CMD_MS_TPC_OFFSET        0xC2


#define CMD_TYPE_ETE_DELAY_TEST       0x95
#define CMD_TYPE_PER_TEST             0x96
#define CMD_TYPE_QOS_TEST             0x97
#define CMD_TYPE_ALL_TX_TEST          0x98

#define HMAC_SET_FA_MODE              0x99 

#define HMAC_CMDSEND_FRAME            0xA0

#define HAMC_CMD_RAMPING_TXP_WRITE    0xA1
#define HMAC_CMD_RAMPING_AGC_WRITE    0xA2
#define HMAC_CMD_MODEM_MS_STATE       0xA3
#define HMAC_CMDMODEM_RS_ID_WRITE     0xA4

#define HMAC_DISABLE_FA2_MODE         0xA5

#define HMAC_CMDEXIT                  0xFE
#define HMAC_CMDACK                   0xFF

#define HMAC_CMDDEBUG_SYNC_INFO       0xB0
#define HMAC_CMD_DETER_MODE           0xB1

#define HMAC_CMD_MS_NET_STATUS       0xB2


#define HMAC_CMD_MAP_ALVIE           0xB3
#define HMAC_CMD_LQ_ALIVE            0xB4
#define HMAC_CMD_LQ_RPT_ALIVE        0xB5

#define HMAC_CMD_PORT_INFO           0xB6

#define HMAC_CMD_REMOTE_PORT_CTRL    0xB7

#define HMAC_CMD_USC_INFO            0xB8
#define HMAC_CMD_AES_CTRL            0xB9
#define HMAC_CMD_AES_CAP             0xC0
#define HMAC_CMD_AES_MS_CTRL         0xC1
#define HMAC_CMD_EXT_INFO_FA1        0xD0
#define HMAC_CMD_GPS_INFO            0xD1
#define HMAC_CMD_EXT_INFO_FA2        0xD2
#define HMAC_CMD_SW_RESET            0xD3
#define HMAC_CMD_CTRL_PORT           0xD4
#define HMAC_CMD_PHY_TIMING          0xD5

#define PIPE_FLAG_FALSE                 0
#define PIPE_FLAG_TRUE                  1

#define MODEMD_MAX_CMD_BUFF             30
#define MODEMD_IPC_MSG_SIZE             1500

#define MODEM_CONFIG_ZONE_STATUS        0
#define MODEM_CONFIG_DLCCH_MCS          1
#define MODEM_CONFIG_DLCCH_MIMO         2
#define MODEM_CONFIG_ULDCH_MCS          3
#define MODEM_CONFIG_ULDCH_MIMO         4
#define MODEM_CONFIG_AHCCH_MCS          5
#define MODEM_CONFIG_AHCCH_MIMO         6

#define MODEMD_CONFIG_MIMO_NONE         0
#define MODEMD_CONFIG_MIMO_SISO         1
#define MODEMD_CONFIG_MIMO_SM           2
#define MODEMD_CONFIG_MIMO_STBC         3

#define MODEMD_CONFIG_MIMO_MODE_4x4     0
#define MODEMD_CONFIG_MIMO_MODE_4x2     1


#define MAIN_NODE_MODE_BS               0
#define MAIN_NODE_MODE_MS               1

#define MS_NODE_TYPE_MASTER             0
#define MS_NODE_TYPE_SLAVE              1



// RS ������ �� SLAVE_ETH_PACKET_PROC �ּ� ó�� �ؾ� ��.
#define SLAVE_ETH_PACKET_PROC
#define FA2_ALL_FORWARD

//INIT DEV COMMND
enum{
  INIT_DEV_PL_VER = 0,
  INIT_DEV_ZYNQ_XADC,
  INIT_DEV_K7_XADC,
  INIT_DEV_CONTRL,
  INIT_DEV_PL_LOOPBACK,
};

//DAC TEMP
#define DAC_T_REF 15
#define DAC_CODE_REF 0xC950

struct FPGA_CONFIG_t {

  unsigned char   enable_flag;
  unsigned char   ms_id;

  unsigned short  dlcch_mcs;
  unsigned short  uldch_mcs;
  unsigned short  ahcch_mcs;

  unsigned short  dlcch_mimo;
  unsigned short  uldch_mimo;
  unsigned short  ahcch_mimo;

  unsigned short  mimo_mode;
};

typedef struct _MODEM_BS_BIT_INFO{

  u16
    freq_bit :1,
    dcdc_bit :1,        
    modem_tx_bit :1,        
    modem_rx_bit :1,       
    cpu_bit :1,    
    dac1_bit :1,    
    dac2_bit :1,        
    pll_bit :1,        
    acdcbit :1,     
    pad :7;

}__attribute__((packed))MODEM_BS_BIT_INFO;

typedef struct _MODEM_MS_BIT_INFO{
  u16 
    freq_bit :1,
    dcdc_bit :1,        
    modem_bit :1,      
    cpu_bit  :1,       
    dac1_bit :1,      
    dac2_bit :1,        
    pll_bit :1,     
    pad :9;  

}__attribute__((packed))MODEM_MS_BIT_INFO;



#define FPGA_TX_VER_ADDR 0x0000
#define FGPA_RX_VER_ADDR 0x0400
