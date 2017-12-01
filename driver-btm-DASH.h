#ifndef __DRIVER_BTM_DASH_H__

#define __DRIVER_BTM_DASH_H__

#include <poll.h>
#include <termios.h>
#include "miner.h"



/******************** about D1 miner *********************/

// how many Hash board that the A8 control board will connect
#define BITMAIN_MAX_CHAIN_NUM           4

// default baud
#define BITMAIN_DEFAULT_BAUD            115200

// default ticket mask

#define DEVICE_DIFF_SET                 0x1B
#define DEVICE_DIFF_SET_MASK            ((1 << (32 - DEVICE_DIFF_SET)) - 1)

#define DEVICE_DIFF_STANDARD            0x16
#define DEVICE_DIFF_STANDARD_MASK       0x003fffffull
// something about fan
#define BITMAIN_MAX_FAN_NUM             2
#define PWM_PERIOD_NS                   100000
#define MIN_FAN_NUM                     1
#define FAN_WANN_SPEED                  6600
#define FAN1_MAX_SPEED                  6000
#define FAN2_MAX_SPEED                  4300
#define FAN_SPEED_OK_PERCENT            (0.85)
#define MIN_PWM_PERCENT                 20
#define MAX_PWM_PERCENT                 100
#define TEMP_INTERVAL                   2
#define MAX_TEMP                        85
#define MAX_FAN_TEMP                    75
#define MIN_FAN_TEMP                    35
#define PWM_ADJUST_FACTOR               ((100 - MIN_PWM_PERCENT)/(MAX_FAN_TEMP - MIN_FAN_TEMP))
#define FANINT                          1
#define PROCFILENAME                    "/proc/interrupts"


/****************** about D1 miner end *******************/

//#define JZ4775

#ifdef JZ4775
#define FAN0                    "110:"   // front fan
#define FAN1                    "113:"   // back fan
#define GPIO_DEVICE_TEMPLATE    "/sys/class/gpio/gpio%d/value"
#define TTY_DEVICE_TEMPLATE     "/dev/ttyS%d"
#define PWM_CTRL_TEMPLATE       "echo %u > /sys/class/pwm/pwmchip0/pwm1/duty_ns"
#define GPIO_SPEED_TEMPLATE     GPIO_DEVICE_TEMPLATE
#define LED_CTRL_TEMPLATE       "echo %d > /sys/class/gpio/gpio%d/value"
#define BEEP_CTRL_TEMPLATE      LED_CTRL_TEMPLATE
#define IIC_DEVIVEE             "/dev/i2c-0"
#define SET_ASIC_RST_0          "echo 0 > /sys/class/gpio/gpio%d/value"
#define SET_ASIC_RST_1          "echo 1 > /sys/class/gpio/gpio%d/value"

#else
/******************** about A8 platform ********************/
#define FAN0                    "256:"   // front fan
#define FAN1                    "254:"   // back fan
#define GPIO_DEVICE_TEMPLATE    "/sys/class/gpio/gpio%d/value"
#define TTY_DEVICE_TEMPLATE     "/dev/ttyO%d"
#define PWM_CTRL_TEMPLATE       "echo %u > /sys/class/pwm/pwm1/duty_ns"
#define GPIO_SPEED_TEMPLATE     GPIO_DEVICE_TEMPLATE
#define LED_CTRL_TEMPLATE       "echo %d > /sys/class/gpio/gpio%d/value"
#define BEEP_CTRL_TEMPLATE      LED_CTRL_TEMPLATE
#define IIC_DEVIVEE             "/dev/i2c-0"
#define SET_ASIC_RST_0          "echo 0 > /sys/class/gpio/gpio%d/value"
#define SET_ASIC_RST_1          "echo 1 > /sys/class/gpio/gpio%d/value"
/****************** about A8 platform end *******************/
#endif

/******************** about D1 Hash board ********************/

// how many BM1760 on 1 D1 Hash board
#define D1_MINER_ASIC_NUM_EACH_CHAIN            60
#define ASIC_NUM_EACH_CHAIN                     D1_MINER_ASIC_NUM_EACH_CHAIN
//#define CHAIN_ASIC_NUM                        60
#define D1_MINER_CHIP_ADDR_INTERVAL             (0x100 / ASIC_NUM_EACH_CHAIN)
// Each Chain support max sensor number
#define BITMAIN_MAX_SUPPORT_TEMP_CHIP_NUM       3
// the real number sensor on each hash board
#define D1_MINER_REAL_TEMP_CHIP_NUM             1
#define BITMAIN_REAL_TEMP_CHIP_NUM              D1_MINER_REAL_TEMP_CHIP_NUM
// about temperature sensor
//#define D1
#define TEMP_CHIP_0_LOCATION                    4   // the 1st temperature sensor connect to the 4th ASIC(count from 0)
#define TEMP_CHIP_1_LOCATION                    0   // 0 means no temperature sensor
#define TEMP_CHIP_2_LOCATION                    0   // 0 means no temperature sensor

//ASIC_TEMP_T TempBuffer[BITMAIN_MAX_CHAIN_NUM][BITMAIN_MAX_SUPPORT_TEMP_CHIP_NUM] = {{{0},{0}}};

/****************** about D1 Hash board end ******************/


/******************** about BM1760 ASIC ********************/

// how many cores in BM1760
#define BM1760_CORE_NUM                             8

// BM1760 ASIC input/output data header
#define INPUT_HEADER_1                              0x55
#define INPUT_HEADER_2                              0xAA
#define OUTPUT_HEADER_1                             0xAA
#define OUTPUT_HEADER_2                             0x55

// BM1760 ASIC input data length
#define WORK_INPUT_TYPE_WITH_SNO                    0x30
#define WORK_INPUT_TYPE_WITHOUT_SNO                 0x20
#define WORK_DATA_INPUT_LENGTH                      80
#define WORK_INPUT_LENGTH_WITHOUT_CRC               82
#define WORK_INPUT_LENGTH_WITH_CRC                  84

// BM1760 ASIC output data length
#define ASIC_INPUT_HEADER_LENGTH                    0x2         // CPU send data to ASIC
#define ASIC_OUTPUT_HEADER_LENGTH                   0x2         // ASIC send data to CPU
#define ASIC_RETURN_DATA_LENGTH_WITHOUT_HEADER      0x7
#define ASIC_RETURN_DATA_LENGTH                     0x9

// Command description
#define CMD_TYPE                                    (0x2 << 5)
#define CMD_ALL                                     (0x01 << 4)
#define SET_ADDR                                    0x0
#define SET_CONFIG                                  0x1
#define GET_STATUS                                  0x2
#define CHAIN_INACTIVE                              0x3
#define CMD_LENTH                                   0x5
#define CONFIG_LENTH                                0x9

// Register description
#define CHIP_ADDR                                   0x0
#define HASH_RATE                                   0x8
#define PLL_PARAMETER                               0xc
#define START_NONCE_OFFSET                          0x10
#define TICK_MASK                                   0x14
#define MISC_CONTROL                                0x1c
#define GENERAL_I2C_COMMAND                         0x20
#define CHIP_OFFSET                                 0x28
#define CORE_OFFSET                                 0x2C
#define CORE_ENABLE                                 0x30
#define CHIP_STATUS                                 0x34
#define TIME_OUT                                    0x38
#define PMONITOR_CTRL                               0x3C
#define ANALOG_MUX_CONTROL                          0x40
#define PROCESS_MONITOR_RETURE_DATA                 0xAA

// Register bits value

// misc control
#define MISC_CONTROL_DEFAULT_VALUE                  0x07003A01
#define RFS                                         (0x1 << 14)
#define TFS(X)                                      ((X & 0x03) << 5)
#define BT8D(x)                                     (x << 8)

// analog mux control
#define DIODE_MUX_SEL_DEFAULT_VALUE                 4
#define VDD_MUX_SEL_DEFAULT_VALUE                   0

// general i2c command
#define REGADDRVALID                                (1 << 24)
#define DEVICEADDR(addr)                            (addr << 17)
//#define DEVICEADDR                                    (0x4c << 17)
#define RW                                          (1 << 16)   // default: write; 1:write; 0:read
#define REGADDR(addr)                               (addr << 8)
#define DATA(data)                                  (data << 0)

// hash rate
#define HASH_RATE_LEFT_SHIFT_BITS                   10


/****************** about BM1760 ASIC end ******************/


/******************** about PIC16F1704 ********************/

// about command between CPU and PIC16F1704
#define PIC_COMMAND_1                               0x55
#define PIC_COMMAND_2                               0xaa
#define SET_PIC_FLASH_POINTER                       0x01
#define SEND_DATA_TO_PIC                            0x02    // just send data into pic's cache
#define READ_DATA_FROM_PIC_FLASH                    0x03
#define ERASE_PIC_FLASH                             0x04    // erase 32 bytes one time
#define WRITE_DATA_INTO_FLASH                       0x05    // tell pic write data into flash from cache
#define JUMP_FROM_LOADER_TO_APP                     0x06
#define RESET_PIC                                   0x07
#define GET_PIC_FLASH_POINTER                       0x08
#define SET_VOLTAGE                                 0x10
#define SET_HASH_BOARD_ID                           0x12
#define READ_HASH_BOARD_ID                          0x13
#define ENABLE_VOLTAGE                              0x15
#define SEND_HEART_BEAT                             0x16
#define READ_PIC_SOFTWARE_VERSION                   0x17
#define GET_VOLTAGE                                 0x18
#define WR_TEMP_OFFSET_VALUE                        0x22
#define RD_TEMP_OFFSET_VALUE                        0x23
#define SAVE_FREQ                                   0x24
#define READ_OUT_FREQ                               0x25

// data address in pic
#define PIC_FLASH_POINTER_START_ADDRESS_H_NEW       0x06
#define PIC_FLASH_POINTER_START_ADDRESS_L_NEW       0x00
#define PIC_FLASH_POINTER_END_ADDRESS_H             0x0f
#define PIC_FLASH_POINTER_END_ADDRESS_L             0x7f
#define PIC_FLASH_LENGTH                            (((unsigned int)PIC_FLASH_POINTER_END_ADDRESS_H<<8 + PIC_FLASH_POINTER_END_ADDRESS_L) - ((unsigned int)PIC_FLASH_POINTER_START_ADDRESS_H_NEW<<8 + PIC_FLASH_POINTER_START_ADDRESS_L_NEW) + 1)
// pic flash erase range. 4 lines, 8 short in each line, 1 short = 2 bytes(0x3FFF).
#define PIC_FLASH_SECTOR_LENGTH                     32
// pic heart beat time gap
#define HEART_BEAT_TIME_GAP                         10      // 10s
// pic update file
#define PIC16F1704_PROGRAM_NEW                      "/sbin/pic.txt"
#define MAX_CHAR_NUM                                1024
#define PIC_VERSION                                 0x81

/****************** about PIC16F1704 end ******************/


/**************** about temperature sensor ****************/

// TMP451 register
#define TMP451_IIC_SALVE_ADDR                       0x4c
#define LOCAL_TEMP_VALUE                            0x0
#define EXT_TEMP_VALUE_HIGH_BYTE                    0x1
#define STATUS                                      0x2
#define CONFIGURATION                               0x3
#define EXT_TEMP_VALUE_LOW_BYTE                     0x10
#define EXT_TEMP_OFFSET_HIGH_BYTE                   0x11
#define EXT_TEMP_OFFSET_LOW_BYTE                    0x12
#define MANUFACTURER_ID                             0xFE
#define MANUFACTURER_ID_ECT218                      0x1A


/************** about temperature sensor end **************/


/******************** other MACRO ********************/

#define MAX_RETURNED_NONCE_NUM                      30
#define MAX_NONCE_NUMBER_IN_FIFO                    (ASIC_NUM_EACH_CHAIN * BITMAIN_MAX_CHAIN_NUM * 30)
#define IIC_SLEEP                                   200
// how many bytes uart received will trigger return before timeout
#define C_CC_VMIN                                   ASIC_RETURN_DATA_LENGTH

// the max work number in the queue where stores the latest works sent to Hash board
#define BITMAIN_MAX_QUEUE_NUM                       128

// swap unsigned int data
#define Swap32(l)                                   (((l) >> 24) | (((l) & 0x00ff0000) >> 8) | (((l) & 0x0000ff00) << 8) | ((l) << 24))
#define Swap16(l)                                   (l >> 8) | ((l & 0xff) << 8)
#define hex_print(p)                                applog(LOG_DEBUG, "%s", p)
#define BYTES_PER_LINE                              0x10
#define INIT_CONFIG_TYPE                            0x51
#define WAIT_REG_VALUE_COUNTER                      4       // every time we check register value, we should wait WAIT_REG_VALUE_COUNTER times to make sure recieve the return value
#define READ_LOOP                                   3       // read temperature sensor times
#define READ_TEMPERATURE_TIME_GAP                   2       // 2s
#define READ_HASH_RATE_TIME_GAP                     5       // 5s

/****************** other MACRO end ******************/


/******************** struct definition ********************/

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
    uint32_t    misc_control_reg_value;
    uint8_t     diode_mux_sel;
    uint8_t     vdd_mux_sel;
    uint8_t     reserved3[2];
    uint16_t    crc;
} __attribute__((packed, aligned(4)));

struct bitmain_DASH_info_with_index
{
    struct bitmain_DASH_info *info;
    uint8_t chain_index;
}__attribute__((packed, aligned(4)));

struct bitmain_DASH_info
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
    uint32_t    chain_asic_exist[BITMAIN_MAX_CHAIN_NUM][ASIC_NUM_EACH_CHAIN/32];
    uint32_t    chain_asic_status[BITMAIN_MAX_CHAIN_NUM][ASIC_NUM_EACH_CHAIN/32];
    uint8_t     chain_asic_num[BITMAIN_MAX_CHAIN_NUM];
    uint8_t     temp[BITMAIN_MAX_CHAIN_NUM];
    uint8_t     chain_index;
    uint8_t     chain_status[BITMAIN_MAX_CHAIN_NUM];    // 1: chain exist; 0: chain not exist
    uint32_t    dev_fd[BITMAIN_MAX_CHAIN_NUM];
    uint8_t     fan_speed_value[BITMAIN_MAX_FAN_NUM];
    uint16_t    freq[BITMAIN_MAX_CHAIN_NUM];
    uint32_t    i2c_fd;
    struct work *work_queue[BITMAIN_MAX_QUEUE_NUM];     // store the latest works that sent to Hash boards
    struct thr_info *thr;
    struct thr_info uart_tx_t[BITMAIN_MAX_CHAIN_NUM];
    struct thr_info uart_rx_t[BITMAIN_MAX_CHAIN_NUM];
    pthread_mutex_t lock;

    struct init_config DASH_config;
    uint16_t    crc;
} __attribute__((packed, aligned(4)));

struct all_parameters
{

    uint32_t    pwm_value;
    uint32_t    duty_ns;
    uint32_t    dev_fd[BITMAIN_MAX_CHAIN_NUM];
    uint8_t     chain_exist[BITMAIN_MAX_CHAIN_NUM];     // 1: chain exist; 0: chain not exist
    uint8_t     chain_asic_in_full[BITMAIN_MAX_CHAIN_NUM];
    uint32_t    timeout;
    uint32_t    fan_exist_map;
    uint32_t    temp_sensor_map;
    uint32_t    nonce_error;
    uint32_t    chain_asic_exist[BITMAIN_MAX_CHAIN_NUM][8];
    uint32_t    chain_asic_status[BITMAIN_MAX_CHAIN_NUM][8];
    int16_t     chain_asic_temp[BITMAIN_MAX_CHAIN_NUM][8][4];
    char        whether_read_out_temp[BITMAIN_MAX_CHAIN_NUM][BITMAIN_MAX_SUPPORT_TEMP_CHIP_NUM];    // -1: not read out; 1: read out. only local temp
    int8_t      chain_asic_iic[ASIC_NUM_EACH_CHAIN];
    uint32_t    chain_hw[BITMAIN_MAX_CHAIN_NUM];
    uint32_t    chain_asic_nonce[BITMAIN_MAX_CHAIN_NUM][ASIC_NUM_EACH_CHAIN];
    char        chain_asic_status_string[BITMAIN_MAX_CHAIN_NUM][ASIC_NUM_EACH_CHAIN + 16];

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

typedef enum
{
    BUF_START,
    BUF_READY,
    BUF_READING,
    BUF_WRITING,
    BUF_IDLE
} BUF_STATE_E;


typedef struct
{
    BUF_STATE_E state;
    unsigned char data;
} ASIC_TEMP_T;


struct dev_info
{
    uint32_t     dev_fd;
    uint32_t    chainid;
};


struct nonce_ctx
{
    uint32_t nonce;
    uint8_t diff;
    uint8_t wc;         // Bit[7]: Reserved. Bit[6:0]: work count
    uint8_t crc5;       // Bit[7] fixed as 1. Bit[6:5]:reserved. Bit[4:0] crc5
    uint8_t chainid;
} __attribute__((packed, aligned(4)));


struct nonce_buf
{
    uint32_t p_wr;
    uint32_t p_rd;
    uint32_t nonce_num;
    struct nonce_ctx nonce_buffer[MAX_NONCE_NUMBER_IN_FIFO];
} __attribute__((packed, aligned(4)));


struct reg_ctx
{
    uint32_t reg_value;
    uint8_t chipaddr;
    uint8_t regaddr;
    uint8_t crc5;       //Bit[7:5]:0, Bit[4:0] crc5
    uint8_t chainid;
} __attribute__((packed, aligned(4)));


struct reg_buf
{
    uint32_t p_wr;
    uint32_t p_rd;
    uint32_t reg_value_num;
    struct reg_ctx reg_buffer[MAX_NONCE_NUMBER_IN_FIFO];
} __attribute__((packed, aligned(4)));


struct work_dash
{
    uint8_t type;                           // Bit[7:5]: Type,fixed as 0x01. Bit[4]:sno valid   Bit[3:0]:reserved
    uint8_t wc;                             // bit[7]: reserved, bit[6:0]: work count base
    uint8_t work[WORK_DATA_INPUT_LENGTH];   // 0-75 bytes are dash work; 76-79 are start nonce field
    uint16_t crc16;                         // crc, but not include the header 0x55, 0xaa
};

/* it does not seem to be used
struct work_buf
{
    uint8_t p_wr;
    uint8_t p_rd;
    uint8_t work_num;
    struct work_dash workdata[BITMAIN_MAX_CHAIN_NUM];
} __attribute__((packed, aligned(4)));
*/

struct pll_freq
{
    int const freq;
    unsigned int vilpll;
};

/****************** struct definition end ******************/


/******************** global variable ********************/

static char nibble[] =
{
    '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
};


static struct pll_freq freq_pll_map[] =
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
    {650, 0x00680221},
    {656, 0x00690221},
	{662, 0x006a0221},
	{668, 0x006b0221},
	{675, 0x006c0221},
	{681, 0x006d0221},
	{687, 0x006e0221},
	{693, 0x006f0221},
	{700, 0x00700221},
	{706, 0x00710221},
	{712, 0x00720221},
	{718, 0x00730221},
	{725, 0x00740221},
	{731, 0x00750221},
	{737, 0x00760221},
	{743, 0x00770221},
	{750, 0x00780221},
	{756, 0x00790221},
	{762, 0x007a0221},
	{768, 0x007b0221},
	{775, 0x007c0221},
	{781, 0x007d0221},
	{787, 0x007e0221},
	{793, 0x007f0221},
	{800, 0x00800221},
	{825, 0x00420211}
   
};

/****************** global variable end ******************/


/******************** referencing functions from other files ********************/

extern void rev(unsigned char *s, size_t l);
extern void cg_logwork(struct work *work, unsigned char *nonce_bin, bool ok);

/****************** referencing functions from other files end ******************/

void check_asic_reg(unsigned int which_chain, unsigned char mode, unsigned char chip_addr, unsigned char reg_addr);
void *get_asic_response(void* arg);
void *DASH_fill_work(void *usrdata);
void enable_read_temperature_from_asic(unsigned int misc_control_reg_value);
void select_core_to_check_temperature(unsigned char diode_mux_sel, unsigned char vdd_mux_sel);
void calibration_sensor_offset(void);
void set_temperature_offset_value(void);
void suffix_string_DASH(uint64_t val, char *buf, size_t bufsiz, int sigdigits,bool display);
void clear_register_value_buf(void);
void check_sensor_ID(void);
void set_PWM(unsigned char pwm_percent);


#endif
