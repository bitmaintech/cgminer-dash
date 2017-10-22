#ifndef __DRIVER_BTM_L3_H__

#define __DRIVER_BTM_L3_H__

#include <poll.h>


#define GPIO_DEVICE_TEMPLATE    "/sys/class/gpio/gpio%d/value"
#define TTY_DEVICE_TEMPLATE     "/dev/ttyO%d"
#define PWM_CTRL_TEMPLATE       "echo %u > /sys/class/pwm/pwm1/duty_ns"
#define GPIO_SPEED_TEMPLATE     GPIO_DEVICE_TEMPLATE
#define LED_CTRL_TEMPLATE       "echo %d > /sys/class/gpio/gpio%d/value"
#define BEEP_CTRL_TEMPLATE      LED_CTRL_TEMPLATE
#define IIC_DEVIVEE             "/dev/i2c-0"

#define FPGA_SOURCE

//Command Description
#define CMD_ALL             (0x01 << 4)
#define SET_ADDR            0x0
#define SET_CONFIG          0x1
#define GET_STATUS          0x2
#define CHAIN_INACTIVE      0x3

#define CMD_LENTH           0x4
#define CONFIG_LENTH        0x8
//Register description
#define CHIP_ADDR           0x00
#define HASHRATE            0x04
#define PLL_PARAMETER       0x08
#define SNO                 0x0C
#define HCN                 0x10
#define TICKET_MASK         0x14
#define MISC_CONTROL        0x18
#define GENERAL_IIC         0x1C
#define SECURITY_IIC        0x20
#define SIG_INPUT           0x24
#define SIG_NONCE_0         0x28
#define SIG_NONCE_1         0x2c
#define SIG_ID              0x30
#define SEC_CTRL_STATUS     0x34
#define MEMORY_STSATUS      0x38
#define CORE_CMD_IN         0x3c
#define CORE_RESP_OUT       0x40
#define EXT_TEMP_SENSOR     0x44

//core command in value
#define CORE_CMD_ALL        (0x1 << 7)
#define CORE_INDEX(X)       (X & 0xf)
#define CORE_DATA(X)        (X & 0xff)
#define CLK_EN(X)           (X & 0x3)
#define LCM(X)              ((X & 0x3) << 1)
#define PM_START            (0x1 << 2)
#define PM_SEL(X)           (X & 0x3)           // 1:select vdd;0:select vss


//cmd type
#define CLOCK_EN_CTRL       0x00
#define CALC_MODE_SET       0x01
#define PRO_MONI_CTRL       0x02
#define TEMP_DIODE_SEL      0x03
#define VOLT_MONI_SEL       0x04

//Register bits value
#define CMD_TYPE            (0x2 << 5)
//#define   HASHRATE_CTRL1(X)   (X & 0x07)
#define HASHRATE_CTRL1(X)   (X & 0x07)
//#define HASHRATE_CTRL2(X) ((X & 0x07) << 4)
#define HASHRATE_CTRL2(X)   ((X & 0x07) << 4)
#define PAT                 (0x0 << 7)
#define LDO18CTRL(X)        ((X & 0x7) << 4)
#define GAP_ERROR           (0x1 << 2)
#define WORK_CRC_ERROR      (0x1 << 1)
#define CMD_CRC_ERROR       0x1
#define VM_SEL(X)           (X & 0x7)
#define INV_CLKO            (0x1 << 5)
#define GATEBCLK            0x1 << 7
#define RFS                 (0x1 << 6)
#define BT8D                0x1A
#define MMEN                (0x1 << 7)
#define TFS(X)              ((X & 0x03) << 5)
#define LDO09_PD            (0x1 << 7)
#define LDO09_CTRL(X)       ((X & 0x07) << 2)
#define RN_CTRL(X)          (X & 0x1)
#define IIC_BUSY            (0x1 << 7)
#define IIC_RW_FAIL         (0x1 << 6)
#define AUTOREADTEMP        (0x1 << 1)
#define REGADDR_VALID       0x1
#define DEVICE_ADDR(X)      ((X & 0x7F) << 1)
#define IIC_WRITE           0x1                 //0: READ  1: WRITE
#define ADDR_RF             (0x1 << 7)
#define EPROM_ADDR          0x50
#define SIG_CNT(X)          (X & 0x0F) << 4)
#define ECC_CLKEN           (0x1 << 3)
#define SIG_PASS            (0x1 << 1)
#define DISA_CHIP           0x1
#define ADDR_RF             (0x1 << 7)
#define NONCE_BIT           (0x1 << 7)
#define SIG_BIT             (0x1 << 6)
#define PARITY_BIT          (0x1 << 5)


union REG_DATA
{

    struct MISC_CTRL_DATA
    {
        uint8_t hashratrectrl1  : 3;
        uint8_t reserved        : 1;
        uint8_t hashratrectrl2  : 3;
        uint8_t reserved1       : 1;

        uint8_t cmd_crc_err     : 1;
        uint8_t work_crc_err    : 1;
        uint8_t gap_crc_err     : 1;
        uint8_t reserved2       : 1;
        uint8_t ldo18ctrl       : 3;
        uint8_t reserved3       : 1;

        uint8_t bt8d            : 5;
        uint8_t inv_clko        : 1;
        uint8_t rfs             : 1;
        uint8_t reserved4       : 1;

        uint8_t reserved5       : 2;
        uint8_t ldo09ctrl       : 3;
        uint8_t tfs             : 2;
        uint8_t ldo09_pd        : 1;
    } misc_ctrl_data;

    struct GENERAL_IIC_DATA
    {
        uint8_t regaddrvalid        :1;
        uint8_t autoreadtemp        :1;
        uint8_t reserved            :4;
        uint8_t rw_fail             :1;
        uint8_t busy                :1;

        uint8_t rw_ctrl             :1;  // 0:read  1:write
        uint8_t deviceaddr          :7;

        uint8_t regaddr;
        uint8_t data;
    } general_iic_data;

    struct SECURITY_IIC_DATA
    {
        uint8_t reserved            :6;
        uint8_t rw_fail             :1;
        uint8_t busy                :1;

        uint8_t rw_ctrl             :1;  // 0:read  1:write
        uint8_t reserved1           :7;

        uint8_t regaddr;
        uint8_t data;
    } security_iic_data;

    struct SCS_DATA
    {
        uint8_t reserved;

        uint8_t rn_ctrl     :1;
        uint8_t reserved1   :7;

        uint8_t eprom_addr  :7;
        uint8_t addr_rf     :1;

        uint8_t disa_chip   :1;
        uint8_t sig_pass    :1;
        uint8_t wp          :1;
        uint8_t ecc_clken   :1;
        uint8_t sig_cnt     :4;
    } scs_data;

    struct CORE_CMD_DATA
    {
        uint8_t ana_sel     :5;
        uint8_t reserved    :2;
        uint8_t all         :1;

        uint8_t core_index  :4;
        uint8_t reserved1   :4;

        uint8_t cmd_type;
        uint8_t cmd_data;
    } core_cmd_data;

    struct TM_DATA
    {
        uint8_t reg_data[4];
    } tm_data;

    struct HCN_DATA
    {
        uint8_t reg_data[4];
    } hcn_data;

    struct PLL_DATA
    {
        uint8_t reg_data[4];
    } pll_data;

    struct SNO_DATA
    {
        uint8_t reg_data[4];
    } sno_data;

} __attribute__((packed, aligned(4)));

#define IIC_SLEEP                   200

// TMP451 register
#define LOCAL_TEMP_VALUE            0x0
#define EXT_TEMP_VALUE_HIGH_BYTE    0x1
#define STATUS                      0x2
#define CONFIGURATION               0x3
#define EXT_TEMP_VALUE_LOW_BYTE     0x10
#define EXT_TEMP_OFFSET_HIGH_BYTE   0x11
#define EXT_TEMP_OFFSET_LOW_BYTE    0x12

// macro define about miner
#define BITMAIN_MAX_CHAIN_NUM           4
#define BM1485_CORE_NUM                 12


#define BITMAIN_MAX_FAN_NUM             2
#define BITMAIN_DEFAULT_ASIC_NUM        64
#define MAX_RETURNED_NONCE_NUM          30
#define INIT_CONFIG_TYPE                0x51
#define SEND_WORK_TYPE                  0x52
#define MAX_NONCE_NUMBER_IN_FIFO        CHAIN_ASIC_NUM * (BITMAIN_MAX_CHAIN_NUM)*3
#define BITMAIN_DEFAULT_BAUD            115200
#define DEVICE_DIFF                     8

#define CHECK_SYSTEM_TIME_GAP           10000           // 10s
//fan
#define PWM_PERIOD_NS                   100000
#define MIN_FAN_NUM                     1
#define MAX_FAN_SPEED                   4100
#define MIN_PWM_PERCENT                 0
#define MAX_PWM_PERCENT                 100
#define TEMP_INTERVAL                   2
#define MAX_TEMP                        85
#define MAX_FAN_TEMP                    75
#define MIN_FAN_TEMP                    35

#define L3_P
//#define L3
#ifdef L3
#define CHAIN_ASIC_NUM                  36
#define HAVE_TEMP                       0xe7
#define HAVE_TEMP_2                     0xa8
#define HAVE_TEMP_3                     0x69
#define BITMAIN_REAL_TEMP_CHIP_NUM 1
#else
#define CHAIN_ASIC_NUM                  72
#define HAVE_TEMP                       0xc
#define HAVE_TEMP_2                     0xc9
#define HAVE_TEMP_3                     0x69
#define BITMAIN_REAL_TEMP_CHIP_NUM 2
#endif
#define BITMAIN_MAX_TEMP_CHIP_NUM 3 // Each Chain





#define PWM_ADJUST_FACTOR               ((100 - MIN_PWM_PERCENT)/(MAX_FAN_TEMP - MIN_FAN_TEMP))
#define PWM_SCALE                       50
#define PWM_ADJ_SCALE                   9/10

//PIC
#define PIC_FLASH_POINTER_START_ADDRESS_H       0x03
#define PIC_FLASH_POINTER_START_ADDRESS_L       0x00
#define PIC_FLASH_POINTER_END_ADDRESS_H         0x0f
#define PIC_FLASH_POINTER_END_ADDRESS_L         0x7f
#define PIC_FREQ_START_ADDRESS_H                0x0f
#define PIC_FREQ_START_ADDRESS_L                0xA0
#define PIC_FLASH_POINTER_FREQ_START_ADDRESS_H  0x0F
#define PIC_FLASH_POINTER_FREQ_START_ADDRESS_L  0xA0
#define PIC_FLASH_POINTER_FREQ_END_ADDRESS_H    0x0f
#define PIC_FLASH_POINTER_FREQ_END_ADDRESS_L    0xDF
#define FREQ_MAGIC                              0x7D
#define PIC_FLASH_LENGTH                    (((unsigned int)PIC_FLASH_POINTER_END_ADDRESS_H<<8 + PIC_FLASH_POINTER_END_ADDRESS_L) - ((unsigned int)PIC_FLASH_POINTER_START_ADDRESS_H<<8 + PIC_FLASH_POINTER_START_ADDRESS_L) + 1)
#define PIC_FLASH_SECTOR_LENGTH             32
#define PIC_SOFTWARE_VERSION_LENGTH         1
#define PIC_VOLTAGE_TIME_LENGTH             6

#define PIC_COMMAND_1                       0x55
#define PIC_COMMAND_2                       0xaa
#define SET_PIC_FLASH_POINTER               0x01
#define SEND_DATA_TO_PIC                    0x02    // just send data into pic's cache
#define READ_DATA_FROM_PIC_FLASH            0x03
#define ERASE_PIC_FLASH                     0x04    // erase 32 bytes one time
#define WRITE_DATA_INTO_FLASH               0x05    // tell pic write data into flash from cache
#define JUMP_FROM_LOADER_TO_APP             0x06
#define RESET_PIC                           0x07
#define GET_PIC_FLASH_POINTER               0x08
#define SET_VOLTAGE                         0x10
#define SET_VOLTAGE_TIME                    0x11
#define SET_HASH_BOARD_ID                   0x12
#define READ_HASH_BOARD_ID                  0x13
#define SET_HOST_MAC_ADDRESS                0x14
#define ENABLE_VOLTAGE                      0x15
#define SEND_HEART_BEAT                     0x16
#define READ_PIC_SOFTWARE_VERSION           0x17
#define GET_VOLTAGE                         0x18
#define READ_VOLTAGE_SETTING_TIME           0x19
#define READ_WHICH_MAC                      0x20
#define READ_MAC                            0x21
#define WR_TEMP_OFFSET_VALUE                0x22
#define RD_TEMP_OFFSET_VALUE                0x23

#define HEART_BEAT_TIME_GAP                 10      // 10s

static unsigned char Pic_command_1[1] = {PIC_COMMAND_1};
static unsigned char Pic_command_2[1] = {PIC_COMMAND_2};
static unsigned char Pic_set_flash_pointer[1] = {SET_PIC_FLASH_POINTER};
static unsigned char Pic_read_flash_pointer[1] = {GET_PIC_FLASH_POINTER};
static unsigned char Pic_send_data_to_pic[1] = {SEND_DATA_TO_PIC};
static unsigned char Pic_write_data_into_flash[1] = {WRITE_DATA_INTO_FLASH};
static unsigned char Pic_erase_pic_flash[1] = {ERASE_PIC_FLASH};
static unsigned char Pic_read_data_from_pic_flash[1] = {READ_DATA_FROM_PIC_FLASH};
static unsigned char Pic_read_pic_software_version[1] = {READ_PIC_SOFTWARE_VERSION};
static unsigned char Pic_set_host_mac_address[1] = {SET_HOST_MAC_ADDRESS};
static unsigned char Pic_read_which_mac[1] = {READ_WHICH_MAC};
static unsigned char Pic_read_mac[1] = {READ_MAC};
static unsigned char Pic_reset[1] = {RESET_PIC};
static unsigned char Pic_jump_from_loader_to_app[1] = {JUMP_FROM_LOADER_TO_APP};
static unsigned char Pic_set_voltage[1] = {SET_VOLTAGE};
static unsigned char Pic_get_voltage[1] = {GET_VOLTAGE};
static unsigned char Pic_set_temperature_offset_value[1] = {WR_TEMP_OFFSET_VALUE};
static unsigned char Pic_get_temperature_offset_value[1] = {RD_TEMP_OFFSET_VALUE};
static unsigned char Pic_heart_beat[1] = {SEND_HEART_BEAT};
static unsigned char Pic_set_voltage_setting_time[1] = {SET_VOLTAGE_TIME};
static unsigned char Pic_read_voltage_setting_time[1] = {READ_VOLTAGE_SETTING_TIME};
static unsigned char Pic_set_hash_board_id_number[1] = {SET_HASH_BOARD_ID};
static unsigned char Pic_read_hash_board_id_number[1] = {READ_HASH_BOARD_ID};
static unsigned char Pic_enable[1] = {ENABLE_VOLTAGE};

//#define PIC_PROGRAM "/usr/bin/pic.txt"
#define PIC_PROGRAM "/sbin/pic.txt"

#define MAX_CHAR_NUM                        4028


struct init_config
{
    uint8_t     token_type;
    uint8_t     version;
    uint16_t    length;
    uint32_t    baud;
    uint8_t     reset                   :1;
    uint8_t     fan_eft                 :1;
    uint8_t     timeout_eft             :1;
    uint8_t     frequency_eft           :1;
    uint8_t     voltage_eft             :1;
    uint8_t     chain_check_time_eft    :1;
    uint8_t     chip_config_eft         :1;
    uint8_t     hw_error_eft            :1;
    uint8_t     beeper_ctrl             :1;
    uint8_t     temp_ctrl               :1;
    uint8_t     chain_freq_eft          :1;
    uint8_t     auto_read_temp          :1;
    uint8_t     reserved1               :4;
    uint8_t     reserved2[2];
    uint8_t     chain_num;
    uint8_t     asic_num;
    uint8_t     fan_pwm_percent;
    uint8_t     temperature;
    uint16_t    frequency;
    uint8_t     voltage[2];
    uint8_t     chain_check_time_integer;
    uint8_t     chain_check_time_fractions;
    uint8_t     timeout_data_integer;
    uint8_t     timeout_data_fractions;
    uint32_t    reg_data;
    uint8_t     chip_address;
    uint8_t     reg_address;
    uint16_t    chain_min_freq;
    uint16_t    chain_max_freq;
    uint16_t    crc;
} __attribute__((packed, aligned(4)));

#define BITMAIN_MAX_QUEUE_NUM 128
struct bitmain_L3_info
{
    cglock_t    update_lock;
    uint8_t     data_type;
    uint8_t     version;
    uint16_t    length;
    uint8_t     chip_value_eft  :1;
    uint8_t     reserved1       :7;
    uint8_t     chain_num;
    uint16_t    reserved2;
    uint8_t     fan_num;
    uint8_t     temp_num;
    uint8_t     reserved3[2];
    uint32_t    fan_exist;
    uint32_t    temp_exist;
    uint16_t    diff;
    uint16_t    reserved4;
    uint32_t    reg_value;
    uint32_t    chain_asic_exist[BITMAIN_MAX_CHAIN_NUM][CHAIN_ASIC_NUM/32];
    uint32_t    chain_asic_status[BITMAIN_MAX_CHAIN_NUM][CHAIN_ASIC_NUM/32];
    uint8_t     chain_asic_num[BITMAIN_MAX_CHAIN_NUM];
    uint8_t     temp[BITMAIN_MAX_CHAIN_NUM];
    uint8_t     chain_index;
    uint8_t     chain_status[BITMAIN_MAX_CHAIN_NUM];
    uint32_t    dev_fd[BITMAIN_MAX_CHAIN_NUM];
    uint8_t     fan_speed_value[BITMAIN_MAX_FAN_NUM];
    uint16_t    freq[BITMAIN_MAX_CHAIN_NUM];
    uint32_t    i2c_fd;
    struct work *work_queue[BITMAIN_MAX_QUEUE_NUM];
    struct thr_info *thr;
    pthread_t read_nonce_thr;
    pthread_t uart_tx_t[BITMAIN_MAX_CHAIN_NUM];
    pthread_t uart_rx_t[BITMAIN_MAX_CHAIN_NUM];
    pthread_mutex_t lock;

    struct init_config L3_config;
    uint16_t    crc;
} __attribute__((packed, aligned(4)));


#define SCRYPTDATA_SIZE 76
struct nonce_ctx
{
    uint8_t nonce[4];
    uint8_t diff;   //Bit[7:6] reserved  Bit[5:0] diff
    uint8_t wc;     //Bit[7]: Reserved. Bit[6:0]: work count
    uint8_t crc5;   // Bit[7] fixed as 1. Bit[6]:sig  1: signature; 0:non signature. Bit[5] sig parity. Bit[4:0] crc5
    uint8_t chainid;
};

struct dev_info
{

    uint8_t     dev_fd;
    uint32_t    chainid;
};

struct all_parameters
{

    uint32_t    pwm_value;
    uint32_t    duty_ns;
    uint32_t    dev_fd[BITMAIN_MAX_CHAIN_NUM];
    uint8_t     chain_exist[BITMAIN_MAX_CHAIN_NUM];
    uint8_t     chain_asic_in_full[BITMAIN_MAX_CHAIN_NUM];
    uint32_t    timeout;
    uint32_t    fan_exist_map;
    uint32_t    temp_sensor_map;
    uint32_t    nonce_error;
    uint32_t    chain_asic_exist[BITMAIN_MAX_CHAIN_NUM][8];
    uint32_t    chain_asic_status[BITMAIN_MAX_CHAIN_NUM][8];
    int16_t     chain_asic_temp[BITMAIN_MAX_CHAIN_NUM][8][4];
    int8_t      chain_asic_iic[CHAIN_ASIC_NUM];
    uint32_t    chain_hw[BITMAIN_MAX_CHAIN_NUM];
    uint32_t    chain_asic_nonce[BITMAIN_MAX_CHAIN_NUM][CHAIN_ASIC_NUM];
    char        chain_asic_status_string[BITMAIN_MAX_CHAIN_NUM][CHAIN_ASIC_NUM + 16];

    uint32_t    total_nonce_num;

    uint32_t    fan_fd[BITMAIN_MAX_FAN_NUM];
    uint8_t     fan_exist[BITMAIN_MAX_FAN_NUM];
    uint32_t    fan_event_count[BITMAIN_MAX_FAN_NUM];
    uint32_t    fan_speed_value[BITMAIN_MAX_FAN_NUM];
    uint32_t    temp[BITMAIN_MAX_CHAIN_NUM];
    uint8_t     chain_asic_num[BITMAIN_MAX_CHAIN_NUM];
    uint8_t     check_bit;
    uint8_t     pwm_percent;
    uint8_t     chain_num;
    uint8_t     fan_num;
    uint8_t     temp_num;
    uint32_t    fan_speed_top1;
    uint32_t    temp_top1;
    uint32_t    temp_top_i[BITMAIN_MAX_CHAIN_NUM];
    uint32_t    temp_top1_last;
    uint8_t     corenum;
    uint8_t     addrInterval;
    uint8_t     max_asic_num_in_one_chain;
    uint8_t     baud;
    uint8_t     diff;
    uint8_t     fan_eft;
    uint8_t     fan_pwm;

    uint16_t    frequency;
    char        frequency_t[10];
    uint16_t    freq[BITMAIN_MAX_CHAIN_NUM];
    uint32_t    i2c_fd;
    struct pollfd pfd[BITMAIN_MAX_FAN_NUM];

} __attribute__((packed, aligned(4)));


struct nonce_buf
{
    uint32_t p_wr;
    uint32_t p_rd;
    uint32_t nonce_num;
    uint8_t reserved[2];
    struct nonce_ctx nonce_buffer[MAX_NONCE_NUMBER_IN_FIFO];
    uint16_t crc16;
} __attribute__((packed, aligned(4)));

struct reg_ctx
{
    uint8_t reg_value[4];
    uint8_t chipaddr;
    uint8_t regaddr;
    uint8_t crc5;   //Bit[7:5]:0 Bit[4:0] crc5
    uint8_t chainid;
};

struct reg_buf
{
    uint32_t p_wr;
    uint32_t p_rd;
    uint32_t reg_value_num;
    uint8_t reserved[2];
    struct reg_ctx reg_buffer[MAX_NONCE_NUMBER_IN_FIFO];
    uint16_t crc16;
} __attribute__((packed, aligned(4)));

struct work_ltc
{
    //uint8_t chainid;

    uint8_t type;       //Bit[7:5]: Type,fixed as 0x01. Bit[4:1]:Reserved   Bit[0]:start nonce valid
    uint8_t length;     //data length£¬from Byte0 to the end,whitout crc bytes.
    uint8_t wc_base;    // Bit[7]: Reserved.   Bit[6:0]: Work count base
    uint8_t reserved;
    //uint32_t sno;     // StartNonce
    uint8_t Sdata[SCRYPTDATA_SIZE];  // ScryptData
    uint16_t crc16;
};

struct work_buf
{
    uint8_t token_type;
    uint8_t reserved[2];
    uint8_t p_wr;
    uint8_t p_rd;
    uint8_t work_num;
    struct work_ltc workdata[BITMAIN_MAX_CHAIN_NUM];
    uint16_t crc16;
} __attribute__((packed, aligned(4)));

struct pll_freq
{
    int const freq;
    unsigned int vilpll;
};

#define Swap32(l) (((l) >> 24) | (((l) & 0x00ff0000) >> 8) | (((l) & 0x0000ff00) << 8) | ((l) << 24))

static struct pll_freq freq_pll_1485[] =
{
    {100, 0x00400242},
    {106, 0x00440242},
    {112, 0x00480242},
    {118, 0x00420271},
    {125, 0x00460271},
    {131, 0x003f0261},
    {137, 0x00420232},
    {142, 0x00440261},
    {148, 0x00470261},
    {154, 0x004a0261},
    {160, 0x004d0232},
    {166, 0x005d0271},
    {172, 0x00450251},
    {178, 0x00470251},
    {184, 0x00670271},
    {190, 0x005b0232},
    {196, 0x005e0261},
    {200, 0x00400241},
    {206, 0x00420241},
    {212, 0x00440241},
    {217, 0x00570251},
    {223, 0x006b0232},
    {229, 0x006e0261},
    {235, 0x00710232},
    {242, 0x00610251},
    {248, 0x00770232},
    {254, 0x007a0232},
    {260, 0x007d0232},
    {267, 0x006b0251},
    {273, 0x006d0251},
    {279, 0x00430231},
    {285, 0x00720251},
    {294, 0x002f0221},
    {300, 0x00600241},
    {306, 0x00310221},
    {312, 0x00640241},
    {319, 0x00660241},
    {325, 0x004e0231},
    {331, 0x006a0241},
    {338, 0x00510231},
    {344, 0x006e0241},
    {350, 0x00540231},
    {353, 0x00710241},
    {356, 0x00720241},
    {359, 0x00730241},
    {362, 0x00570231},
    {366, 0x00750241},
    {369, 0x00760241},
    {375, 0x005a0231},
    {378, 0x00790241},
    {381, 0x003d0221},
    {384, 0x007b0241},
    {387, 0x005d0231},
    {391, 0x007d0241},
    {394, 0x003f0221},
    {397, 0x007f0241},
    {400, 0x00600231},
    {406, 0x00410221},
    {412, 0x00420221},
    {419, 0x00430221},
    {425, 0x00440221},
    {431, 0x00450221},
    {437, 0x00460221},
    {438, 0x00460221},
    {444, 0x00470221},
    {450, 0x00480221},
    {456, 0x00490221},
    {462, 0x004a0221},
    {469, 0x004b0221},
    {475, 0x004c0221},
    {481, 0x004d0221},
    {487, 0x004e0221},
    {494, 0x004f0221},
    {500, 0x00500221},
    {506, 0x00510221},
    {512, 0x00520221},
    {519, 0x00530221},
    {525, 0x00540221},
    {531, 0x00550221},
    {537, 0x00560221},
    {544, 0x00570221},
    {550, 0x00580221},
    {556, 0x00590221},
    {562, 0x005a0221},
    {569, 0x005b0221},
    {575, 0x005c0221},
    {581, 0x005d0221},
    {587, 0x005e0221},
    {588, 0x005e0221},
    {594, 0x005f0221},
    {600, 0x00600221},
    {606, 0x00610221},
    {612, 0x00620221},
    {619, 0x00630221},
    {625, 0x00640221},
    {631, 0x00650221},
    {637, 0x00660221},
    {638, 0x00660221},
    {644, 0x00670221},
    {650, 0x00680221}
};

extern bool opt_bitmain_fan_ctrl;
extern int opt_bitmain_fan_pwm;
extern int opt_bitmain_L3_freq;
extern int opt_bitmain_L3_voltage;
extern int8_t opt_bitmain_L3_core_temp;


#endif
