#include "config.h"
#include <assert.h>

#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/file.h>
#include <dirent.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <assert.h>
#include <unistd.h>
#include <math.h>

#ifndef WIN32
#include <sys/select.h>
//#include <termios.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <net/if.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <errno.h>

#ifndef O_CLOEXEC
#define O_CLOEXEC 0
#endif
#else
#include "compat.h"
#include <windows.h>
#include <io.h>
#endif

#include "elist.h"
//#include "usbutils.h"
#include "driver-btm-DASH.h"
#include "util.h"



/****************** checked ***************************/
#ifndef JZ4775
int const plug[BITMAIN_MAX_CHAIN_NUM] = {51,48,47,44};
int const tty[BITMAIN_MAX_CHAIN_NUM] = {1,2,4,5};
int const beep = 20;
int const red_led = 45;
int const green_led = 23;
int const fan_speed[BITMAIN_MAX_FAN_NUM] = {112,110};
int const g_gpio_data[BITMAIN_MAX_CHAIN_NUM] = {5, 4, 27, 22};
#else
int const plug[BITMAIN_MAX_CHAIN_NUM] = {184,185,187,186};
int const tty[BITMAIN_MAX_CHAIN_NUM] = {0,1,2,3};
int const beep = 81;
int const red_led = 126;
int const green_led = 127;
int const fan_speed[BITMAIN_MAX_FAN_NUM] = {38,41};
int const g_gpio_data[BITMAIN_MAX_CHAIN_NUM] = {192, 193, 194, 195};
#endif
unsigned char bt8d = 0x1a;
int const i2c_slave_addr[BITMAIN_MAX_CHAIN_NUM] = {0xa0,0xa2,0xa4,0xa6};

pthread_mutex_t reinit_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t i2c_mutex = PTHREAD_MUTEX_INITIALIZER;  // used when cpu operates i2c interface
pthread_mutex_t iic_mutex = PTHREAD_MUTEX_INITIALIZER;  // used when cpu communicate with pic
pthread_mutex_t reg_read_mutex = PTHREAD_MUTEX_INITIALIZER;     // used when read ASIC register periodic in pthread
pthread_mutex_t work_queue_mutex = PTHREAD_MUTEX_INITIALIZER;

unsigned int gChipOffset = 0;   // record register CHIP_OFFSET value
unsigned int gCoreOffset = 0;   // record register CORE_OFFSET value
unsigned char TempChipAddr[BITMAIN_MAX_SUPPORT_TEMP_CHIP_NUM] = {0, 0, 0};  // record temperature sensor address

int opt_bitmain_fan_pwm = 30;       // if not control fan speed according to temperature, use this parameter to control fan speed
bool opt_bitmain_fan_ctrl = false;  // false: control fan speed according to temperature; true: use opt_bitmain_fan_pwm to control fan speed
int opt_bitmain_DASH_freq = 100;
int opt_bitmain_DASH_voltage = 176;
int8_t opt_bitmain_DASH_core_temp = 2;
int last_temperature = 0, temp_highest = 0;

struct all_parameters dev;
struct thr_info *pic_heart_beat;
struct thr_info *scan_reg_id;
struct thr_info *read_temp_id;
struct thr_info *check_miner_status_id;                  // thread id for check system
struct thr_info *read_hash_rate;
struct thr_info *check_fan_id;
struct dev_info dev_info[BITMAIN_MAX_CHAIN_NUM];
bool update_asic_num = false;
bool is_rt = true;
enum I2C_TYPE { LOCAL, REMOTE, OFFSET, ID};

signed char gTempOffsetValue[BITMAIN_MAX_CHAIN_NUM][BITMAIN_MAX_SUPPORT_TEMP_CHIP_NUM] = {{0}};
signed char sensor_id[BITMAIN_MAX_CHAIN_NUM][BITMAIN_MAX_SUPPORT_TEMP_CHIP_NUM] = {{0}};
signed char send_back_gTempOffsetValue[BITMAIN_MAX_CHAIN_NUM][BITMAIN_MAX_SUPPORT_TEMP_CHIP_NUM] = {{0}};

unsigned char g_CHIP_ADDR_reg_value_num[BITMAIN_MAX_CHAIN_NUM] = {0};                   // record receive how many CHIP_ADDR register value each chain
unsigned int g_CHIP_ADDR_reg_value[BITMAIN_MAX_CHAIN_NUM][128] = {{0}};             // record receive CHIP_ADDR register value each chain
unsigned char g_HASH_RATE_reg_value_num[BITMAIN_MAX_CHAIN_NUM] = {0};                   // record receive how many HASH_RATE register value each chain
unsigned int g_HASH_RATE_reg_value[BITMAIN_MAX_CHAIN_NUM][128] = {{0}};             // record receive HASH_RATE register value each chain
bool g_HASH_RATE_reg_value_from_which_asic[BITMAIN_MAX_CHAIN_NUM][128] = {{0}}; // record receive HASH_RATE register value from which asic
unsigned char g_CHIP_STATUS_reg_value_num[BITMAIN_MAX_CHAIN_NUM] = {0};             // record receive how many CHIP_STATUS register value each chain
unsigned int g_CHIP_STATUS_reg_value[BITMAIN_MAX_CHAIN_NUM][128] = {{0}};           // record receive CHIP_STATUS register value each chain

bool g_chip_temp_return[BITMAIN_MAX_CHAIN_NUM][BITMAIN_REAL_TEMP_CHIP_NUM][4] = {{0}};

uint64_t rate[BITMAIN_MAX_CHAIN_NUM] = {0};
unsigned char rate_error[BITMAIN_MAX_CHAIN_NUM] = {0};  // record not receive all ASIC's HASH_RATE register value time
char displayed_rate[BITMAIN_MAX_CHAIN_NUM][16];
unsigned char pic_version[BITMAIN_MAX_CHAIN_NUM] = {0};
bool gLost_internet_10_min = false;     // lost internet for 10 minutes
bool gGot_Temperature_value = false;    // wether read out new temperature value
bool gMinerStatus_Low_Hashrate = false;         // hash rate is too low
bool gMinerStatus_High_Temp = false;            // the temperature is higher than MAX_TEMP
bool gMinerStatus_Not_read_all_sensor = false;  // do not read out all sensor's temperature
bool gMinerStatus_Lost_connection_to_pool = false;  // can't receive job from pool
bool gFan_Error = false;                        // lost fan or fan speed to low
unsigned char gMinerStatus_High_Temp_Counter = 0;           // the temperature is higher than MAX_TEMP counter

/****************** checked end ***************************/

struct thr_info *read_nonce_reg_id;                 // thread id for read nonce and register
uint64_t h = 0;
uint64_t h_each_chain[BITMAIN_MAX_CHAIN_NUM] = {0};
double each_chain_h_avg[BITMAIN_MAX_CHAIN_NUM] = {0};
double geach_chain_h_all = 0;
unsigned char hash_board_id[BITMAIN_MAX_CHAIN_NUM][12];
unsigned char voltage[BITMAIN_MAX_CHAIN_NUM] = {0,0,0,0};
bool need_recheck = false;
pthread_mutex_t reg_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t nonce_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t tty_write_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t temp_buf_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool start_send[BITMAIN_MAX_CHAIN_NUM] = {false,false,false,false};
static bool start_recv[BITMAIN_MAX_CHAIN_NUM] = {false,false,false,false};
static bool reiniting[BITMAIN_MAX_CHAIN_NUM] = {false,false,false,false};


bool once_error = false;
bool status_error = false;
bool check_rate = false;
bool gBegin_get_nonce = false;
bool send_heart = true;
bool new_block[BITMAIN_MAX_CHAIN_NUM] = {false, false, false, false};
uint64_t hashboard_average_hash_rate[BITMAIN_MAX_CHAIN_NUM] = {0};
uint64_t hashboard_real_time_hash_rate[BITMAIN_MAX_CHAIN_NUM] = {0};
struct nonce_buf nonce_fifo;
struct reg_buf reg_fifo;
struct timeval tv_send_job = {0, 0};
// this is reset pins

/******************** global functions ********************/

static void hexdump(const uint8_t *p, unsigned int len)
{
    unsigned int i, addr;
    unsigned int wordlen = sizeof(unsigned int);
    unsigned char v, line[BYTES_PER_LINE * 5];

    for (addr = 0; addr < len; addr += BYTES_PER_LINE)
    {
        /* clear line */
        for (i = 0; i < sizeof(line); i++)
        {
            if (i == wordlen * 2 + 52 ||
                i == wordlen * 2 + 69)
            {
                line[i] = '|';
                continue;
            }

            if (i == wordlen * 2 + 70)
            {
                line[i] = '\0';
                continue;
            }

            line[i] = ' ';
        }

        /* print address */
        for (i = 0; i < wordlen * 2; i++)
        {
            v = addr >> ((wordlen * 2 - i - 1) * 4);
            line[i] = nibble[v & 0xf];
        }

        /* dump content */
        for (i = 0; i < BYTES_PER_LINE; i++)
        {
            int pos = (wordlen * 2) + 3 + (i / 8);

            if (addr + i >= len)
                break;

            v = p[addr + i];
            line[pos + (i * 3) + 0] = nibble[v >> 4];
            line[pos + (i * 3) + 1] = nibble[v & 0xf];

            /* character printable? */
            line[(wordlen * 2) + 53 + i] =
                (v >= ' ' && v <= '~') ? v : '.';
        }

        hex_print(line);
    }
}
unsigned char CRC5(unsigned char *ptr, unsigned char len)
{
    unsigned char i, j, k;
    unsigned char crc = 0x1f;

    unsigned char crcin[5] = {1, 1, 1, 1, 1};
    unsigned char crcout[5] = {1, 1, 1, 1, 1};
    unsigned char din = 0;

    j = 0x80;
    k = 0;
    for (i = 0; i < len; i++)
    {
        if (*ptr & j)
        {
            din = 1;
        }
        else
        {
            din = 0;
        }
        crcout[0] = crcin[4] ^ din;
        crcout[1] = crcin[0];
        crcout[2] = crcin[1] ^ crcin[4] ^ din;
        crcout[3] = crcin[2];
        crcout[4] = crcin[3];

        j = j >> 1;
        k++;
        if (k == 8)
        {
            j = 0x80;
            k = 0;
            ptr++;
        }
        memcpy(crcin, crcout, 5);
    }
    crc = 0;
    if(crcin[4])
    {
        crc |= 0x10;
    }
    if(crcin[3])
    {
        crc |= 0x08;
    }
    if(crcin[2])
    {
        crc |= 0x04;
    }
    if(crcin[1])
    {
        crc |= 0x02;
    }
    if(crcin[0])
    {
        crc |= 0x01;
    }
    return crc;
}

/** CRC table for the CRC ITU-T V.41 0x0x1021 (x^16 + x^12 + x^5 + 1) */
const uint16_t crc_itu_t_table[256] =
{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};


static inline uint16_t crc_itu_t_byte(uint16_t crc, const uint8_t data)
{
    return (crc << 8) ^ crc_itu_t_table[((crc >> 8) ^ data) & 0xff];
}


unsigned short CRC16(unsigned short crc, unsigned char *buffer, int len)
{
    while (len--)
        crc = crc_itu_t_byte(crc, *buffer++);
    return crc;
}


uint16_t crc_itu_t(uint16_t crc, const uint8_t *buffer, int len)
{
    while (len--)
        crc = crc_itu_t_byte(crc, *buffer++);
    return crc;
}

/****************** global functions end ******************/

#define ABOUT_PIC

/******************** about PIC16F1704 ********************/

// 0: old api; 1: new api
/*
unsigned char use_new_or_old_PIC16F1704_api(void)
{
    unsigned char version = 0;

    pic_reset();
    pic_jump_from_loader_to_app();
    pic_read_pic_software_version(&version);
    pic_reset();

    printf("\n--- %s: version = 0x%02x\n", __FUNCTION__, version);

    if(version < 0x80)
    {
        printf("\n--- %s: use old PIC16F1704 api\n", __FUNCTION__);
        return 0;
    }
    else
    {
        printf("\n--- %s: use new PIC16F1704 api\n", __FUNCTION__);
        return 1;
    }
}
*/

void *pic_heart_beat_func_new(void * arg)
{
    int which_chain = 0;

    applog(LOG_NOTICE, "%s", __FUNCTION__);

    while(1)
    {
        for(which_chain=0; which_chain<BITMAIN_MAX_CHAIN_NUM; which_chain++)
        {
            if(dev.chain_exist[which_chain] && send_heart)
            {
                pthread_mutex_lock(&iic_mutex);
                if(unlikely(ioctl(dev.i2c_fd, I2C_SLAVE, i2c_slave_addr[which_chain] >> 1 ) < 0))
                    applog(LOG_ERR," %d ioctl error in %s", which_chain, __FUNCTION__);
                heart_beat_PIC16F1704_new();
                pthread_mutex_unlock(&iic_mutex);
                cgsleep_ms(10);
            }
        }
        sleep(HEART_BEAT_TIME_GAP);
    }
}


int set_PIC16F1704_flash_pointer_new(unsigned char flash_addr_h, unsigned char flash_addr_l)
{
    unsigned char length = 0x06, crc_data[2] = {0xff}, read_back_data[2] = {0xff};
    unsigned short crc = 0;
    unsigned char i,send_data[8] = {0};

    //applog(LOG_NOTICE, "%s", __FUNCTION__);

    crc = length + SET_PIC_FLASH_POINTER + flash_addr_h + flash_addr_l;
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = SET_PIC_FLASH_POINTER;
    send_data[4] = flash_addr_h;
    send_data[5] = flash_addr_l;
    send_data[6] = crc_data[0];
    send_data[7] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<8; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(100*1000);
    for(i=0; i<2; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    if((read_back_data[0] != SET_PIC_FLASH_POINTER) || (read_back_data[1] != 1))
    {
        applog(LOG_ERR,"%s failed! read_back_data[0] = 0x%02x, read_back_data[1] = 0x%02x\n\n", __FUNCTION__, read_back_data[0], read_back_data[1]);
        return 0;   // error
    }
    else
    {
        applog(LOG_NOTICE,"%s ok", __FUNCTION__);
        return 1;   // ok
    }
}


int send_data_to_PIC16F1704_new(unsigned char *buf)
{
    unsigned char length = 0x14, crc_data[2] = {0xff}, read_back_data[2] = {0xff};
    unsigned short crc = 0;
    unsigned char i,send_data[22] = {0};

    //applog(LOG_NOTICE,"%s", __FUNCTION__);

    crc = length + SEND_DATA_TO_PIC;
    for(i=0; i<16; i++)
    {
        crc += *(buf + i);
    }
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = SEND_DATA_TO_PIC;
    for(i=0; i<16; i++)
    {
        send_data[i+4]= *(buf + i);
    }
    send_data[20] = crc_data[0];
    send_data[21] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<22; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(100*1000);
    for(i=0; i<2; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    if((read_back_data[0] != SEND_DATA_TO_PIC) || (read_back_data[1] != 1))
    {
        applog(LOG_ERR,"%s failed! read_back_data[0] = 0x%02x, read_back_data[1] = 0x%02x\n\n", __FUNCTION__, read_back_data[0], read_back_data[1]);
        return 0;   // error
    }
    else
    {
        applog(LOG_NOTICE,"%s ok", __FUNCTION__);
        return 1;   // ok
    }
}


int read_PIC16F1704_flash_pointer_new(unsigned char *flash_addr_h, unsigned char *flash_addr_l)
{
    unsigned char length = 0x04, crc_data[2] = {0xff}, read_back_data[6] = {0xff};
    unsigned short crc = 0;
    unsigned char i,send_data[6] = {0};

    //applog(LOG_NOTICE,"%s", __FUNCTION__);

    crc = length + GET_PIC_FLASH_POINTER;
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = GET_PIC_FLASH_POINTER;
    send_data[4] = crc_data[0];
    send_data[5] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<6; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(100*1000);
    for(i=0; i<6; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    if((read_back_data[1] != GET_PIC_FLASH_POINTER) || (read_back_data[0] != 6))
    {
        applog(LOG_ERR,"%s failed! read_back_data[0] = 0x%x, read_back_data[1] = 0x%x, read_back_data[2] = 0x%x, read_back_data[3] = 0x%x, read_back_data[4] = 0x%x, read_back_data[5] = 0x%x\n\n", __FUNCTION__,
               read_back_data[0], read_back_data[1], read_back_data[2], read_back_data[3], read_back_data[4], read_back_data[5]);
        return 0;   // error
    }
    else
    {
        crc = read_back_data[0] + read_back_data[1] + read_back_data[2] + read_back_data[3];
        if(((unsigned char)((crc >> 8) & 0x00ff) != read_back_data[4]) || ((unsigned char)((crc >> 0) & 0x00ff) != read_back_data[5]))
        {
            applog(LOG_ERR,"%s failed!!! read_back_data[0] = 0x%x, read_back_data[1] = 0x%x, read_back_data[2] = 0x%x, read_back_data[3] = 0x%x, read_back_data[4] = 0x%x, read_back_data[5] = 0x%x\n\n", __FUNCTION__,
                   read_back_data[0], read_back_data[1], read_back_data[2], read_back_data[3], read_back_data[4], read_back_data[5]);
            return 0;   // error
        }
        *flash_addr_h = read_back_data[2];
        *flash_addr_l = read_back_data[3];
        applog(LOG_NOTICE,"%s ok! flash_addr_h = 0x%02x, flash_addr_l = 0x%02x", __FUNCTION__, *flash_addr_h, *flash_addr_l);
        return 1;   // ok
    }
}

int read_PIC16F1704_flash_data_new(unsigned char *buf)
{
    unsigned char length = 0x04, crc_data[2] = {0xff}, read_back_data[20] = {0xff};
    unsigned short crc = 0;
    unsigned char i,send_data[6] = {0};

    //applog(LOG_NOTICE,"%s", __FUNCTION__);

    crc = length + READ_DATA_FROM_PIC_FLASH;
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = READ_DATA_FROM_PIC_FLASH;
    send_data[4] = crc_data[0];
    send_data[5] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<6; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(300*1000);
    for(i=0; i<20; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    applog(LOG_NOTICE,"%s: read_back_data[0] = 0x%x, read_back_data[1] = 0x%x, read_back_data[2] = 0x%x, read_back_data[3] = 0x%x, \
		read_back_data[4] = 0x%x, read_back_data[5] = 0x%x, read_back_data[6] = 0x%x, read_back_data[7] = 0x%x, \
		read_back_data[8] = 0x%x, read_back_data[9] = 0x%x, read_back_data[10] = 0x%x, read_back_data[11] = 0x%x, \
		read_back_data[12] = 0x%x, read_back_data[13] = 0x%x, read_back_data[14] = 0x%x, read_back_data[15] = 0x%x, \
		read_back_data[16] = 0x%x, read_back_data[17] = 0x%x, read_back_data[18] = 0x%x, read_back_data[19] = 0x%x\n", __FUNCTION__,\
           read_back_data[0], read_back_data[1], read_back_data[2], read_back_data[3], read_back_data[4], read_back_data[5], \
           read_back_data[6], read_back_data[7], read_back_data[8], read_back_data[9], read_back_data[10], read_back_data[11], \
           read_back_data[12], read_back_data[13], read_back_data[14], read_back_data[15], read_back_data[16], read_back_data[17], \
           read_back_data[18], read_back_data[19]);
    usleep(100*1000);

    if((read_back_data[1] != READ_DATA_FROM_PIC_FLASH) || (read_back_data[0] != 20))
    {
        applog(LOG_ERR,"%s failed!", __FUNCTION__);
        return 0;   // error
    }
    else
    {
        crc = read_back_data[0] + read_back_data[1] + read_back_data[2] + read_back_data[3] + read_back_data[4] + read_back_data[5] +
              read_back_data[6] + read_back_data[7] + read_back_data[8] + read_back_data[9] + read_back_data[10] + read_back_data[11] +
              read_back_data[12] + read_back_data[13] + read_back_data[14] + read_back_data[15] + read_back_data[16] + read_back_data[17];

        if(((unsigned char)((crc >> 8) & 0x00ff) != read_back_data[18]) || ((unsigned char)((crc >> 0) & 0x00ff) != read_back_data[19]))
        {
            applog(LOG_ERR,"%s failed! crc = 0x%04x\n\n", __FUNCTION__, crc);
            return 0;   // error
        }
        for(i=0; i<16; i++)
        {
            *(buf + i) = read_back_data[2 + i];
        }
        applog(LOG_NOTICE,"%s ok", __FUNCTION__);
        return 1;   // ok
    }
}


int erase_PIC16F1704_flash_new(void)
{
    unsigned char length = 0x04, crc_data[2] = {0xff}, read_back_data[2] = {0xff};
    unsigned short crc = 0;
    unsigned char i,send_data[6] = {0};

    //applog(LOG_NOTICE,"%s", __FUNCTION__);

    crc = length + ERASE_PIC_FLASH;
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = ERASE_PIC_FLASH;
    send_data[4] = crc_data[0];
    send_data[5] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<6; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(100*1000);
    for(i=0; i<2; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    usleep(200*1000);

    if((read_back_data[0] != ERASE_PIC_FLASH) || (read_back_data[1] != 1))
    {
        applog(LOG_ERR,"%s failed! read_back_data[0] = 0x%02x, read_back_data[1] = 0x%02x\n\n", __FUNCTION__, read_back_data[0], read_back_data[1]);
        return 0;   // error
    }
    else
    {
        applog(LOG_NOTICE,"%s ok", __FUNCTION__);
        return 1;   // ok
    }
}


int write_data_into_PIC16F1704_flash_new(void)
{
    unsigned char length = 0x04, crc_data[2] = {0xff}, read_back_data[2] = {0xff};
    unsigned short crc = 0;
    unsigned char i,send_data[6] = {0};

    //applog(LOG_NOTICE,"%s", __FUNCTION__);

    crc = length + WRITE_DATA_INTO_FLASH;
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = WRITE_DATA_INTO_FLASH;
    send_data[4] = crc_data[0];
    send_data[5] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<6; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(200*1000);
    for(i=0; i<2; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    usleep(200*1000);

    if((read_back_data[0] != WRITE_DATA_INTO_FLASH) || (read_back_data[1] != 1))
    {
        applog(LOG_ERR,"%s failed! read_back_data[0] = 0x%02x, read_back_data[1] = 0x%02x\n\n", __FUNCTION__, read_back_data[0], read_back_data[1]);
        return 0;   // error
    }
    else
    {
        applog(LOG_NOTICE,"%s ok\n\n", __FUNCTION__);
        return 1;   // ok
    }
}


int jump_from_loader_to_app_PIC16F1704_new(void)
{
    unsigned char length = 0x04, crc_data[2] = {0xff}, read_back_data[2] = {0xff};
    unsigned short crc = 0;
    unsigned char i,send_data[6] = {0};

    //applog(LOG_DEBUG,"%s", __FUNCTION__);

    crc = length + JUMP_FROM_LOADER_TO_APP;
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = JUMP_FROM_LOADER_TO_APP;
    send_data[4] = crc_data[0];
    send_data[5] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<6; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(100*1000);
    for(i=0; i<2; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    usleep(300*1000);

    if((read_back_data[0] != JUMP_FROM_LOADER_TO_APP) || (read_back_data[1] != 1))
    {
        applog(LOG_ERR,"%s failed! read_back_data[0] = 0x%02x, read_back_data[1] = 0x%02x", __FUNCTION__, read_back_data[0], read_back_data[1]);
        return 0;   // error
    }
    else
    {
        applog(LOG_NOTICE,"%s ok", __FUNCTION__);
        return 1;   // ok
    }
}


int reset_PIC16F1704_pic_new(void)
{
    unsigned char length = 0x04, crc_data[2] = {0xff}, read_back_data[2] = {0xff};
    unsigned short crc = 0;
    unsigned char i,send_data[6] = {0};

    //applog(LOG_DEBUG,"%s", __FUNCTION__);

    crc = length + RESET_PIC;
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = RESET_PIC;
    send_data[4] = crc_data[0];
    send_data[5] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<6; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(100*1000);
    for(i=0; i<2; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    usleep(1*1000*1000);

    if((read_back_data[0] != RESET_PIC) || (read_back_data[1] != 1))
    {
        applog(LOG_ERR,"%s failed! read_back_data[0] = 0x%02x, read_back_data[1] = 0x%02x\n\n", __FUNCTION__, read_back_data[0], read_back_data[1]);
        return 0;   // error
    }
    else
    {
        applog(LOG_NOTICE,"%s ok", __FUNCTION__);
        return 1;   // ok
    }
}


int set_PIC16F1704_voltage_new(unsigned char voltage)
{
    unsigned char length = 0x05, crc_data[2] = {0xff}, read_back_data[2] = {0xff};
    unsigned short crc = 0;
    unsigned char i,send_data[7] = {0};

    //applog(LOG_DEBUG,"%s", __FUNCTION__);

    crc = length + SET_VOLTAGE + voltage;
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = SET_VOLTAGE;
    send_data[4] = voltage;
    send_data[5] = crc_data[0];
    send_data[6] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<7; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(200*1000);
    for(i=0; i<2; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    usleep(200*1000);

    if((read_back_data[0] != SET_VOLTAGE) || (read_back_data[1] != 1))
    {
        applog(LOG_ERR,"%s failed! read_back_data[0] = 0x%02x, read_back_data[1] = 0x%02x\n\n", __FUNCTION__, read_back_data[0], read_back_data[1]);
        return 0;   // error
    }
    else
    {
        applog(LOG_NOTICE,"%s ok, voltage = 0x%02x", __FUNCTION__, voltage);
        return 1;   // ok
    }
}


int write_hash_ID_PIC16F1704_new(unsigned char *buf)
{
    unsigned char length = 0x10, crc_data[2] = {0xff}, read_back_data[2] = {0xff};
    unsigned short crc = 0;
    unsigned char i,send_data[18] = {0};

    //applog(LOG_DEBUG,"%s", __FUNCTION__);

    crc = length + SET_HASH_BOARD_ID;
    for(i=0; i<12; i++)
    {
        crc += *(buf + i);
    }
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = SET_HASH_BOARD_ID;
    for(i=0; i<12; i++)
    {
        send_data[i+4]= *(buf + i);
    }
    send_data[16] = crc_data[0];
    send_data[17] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<18; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(200*1000);
    for(i=0; i<2; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    usleep(200*1000);

    if((read_back_data[0] != SET_HASH_BOARD_ID) || (read_back_data[1] != 1))
    {
        applog(LOG_ERR,"%s failed! read_back_data[0] = 0x%02x, read_back_data[1] = 0x%02x\n\n", __FUNCTION__, read_back_data[0], read_back_data[1]);
        return 0;   // error
    }
    else
    {
        applog(LOG_NOTICE,"%s ok", __FUNCTION__);
        return 1;   // ok
    }
}


int read_hash_id_PIC16F1704_new(unsigned char *buf)
{
    unsigned char length = 0x04, crc_data[2] = {0xff}, read_back_data[16] = {0xff};
    unsigned short crc = 0;
    unsigned char i,send_data[6] = {0};

    //applog(LOG_DEBUG,"%s", __FUNCTION__);

    crc = length + READ_HASH_BOARD_ID;
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = READ_HASH_BOARD_ID;
    send_data[4] = crc_data[0];
    send_data[5] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<6; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(200*1000);
    for(i=0; i<16; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    applog(LOG_NOTICE,"%s: read_back_data[0] = 0x%x, read_back_data[1] = 0x%x, read_back_data[2] = 0x%x, read_back_data[3] = 0x%x,\
		read_back_data[4] = 0x%x, read_back_data[5] = 0x%x, read_back_data[6] = 0x%x, read_back_data[7] = 0x%x,\
		read_back_data[8] = 0x%x, read_back_data[9] = 0x%x, read_back_data[10] = 0x%x, read_back_data[11] = 0x%x,\
		read_back_data[12] = 0x%x, read_back_data[13] = 0x%x, read_back_data[14] = 0x%x, read_back_data[15] = 0x%x\n", __FUNCTION__,\
           read_back_data[0], read_back_data[1], read_back_data[2], read_back_data[3], read_back_data[4], read_back_data[5],\
           read_back_data[6], read_back_data[7], read_back_data[8], read_back_data[9], read_back_data[10], read_back_data[11],\
           read_back_data[12], read_back_data[13], read_back_data[14], read_back_data[15]);

    if((read_back_data[1] != READ_HASH_BOARD_ID) || (read_back_data[0] != 16))
    {
        applog(LOG_ERR,"%s failed!\n\n", __FUNCTION__);
        return 0;   // error
    }
    else
    {
        crc = read_back_data[0] + read_back_data[1] + read_back_data[2] + read_back_data[3] + read_back_data[4] + read_back_data[5] +
              read_back_data[6] + read_back_data[7] + read_back_data[8] + read_back_data[9] + read_back_data[10] + read_back_data[11] +
              read_back_data[12] + read_back_data[13];

        if(((unsigned char)((crc >> 8) & 0x00ff) != read_back_data[14]) || ((unsigned char)((crc >> 0) & 0x00ff) != read_back_data[15]))
        {
            applog(LOG_ERR,"%s failed! crc = 0x%04x", __FUNCTION__, crc);
            return 0;   // error
        }
        for(i=0; i<12; i++)
        {
            *(buf + i) = read_back_data[2 + i];
        }
        applog(LOG_NOTICE,"%s ok", __FUNCTION__);
        return 1;   // ok
    }
}


int enable_PIC16F1704_dc_dc_new(void)
{
    unsigned char length = 0x05, crc_data[2] = {0xff}, read_back_data[2] = {0xff};
    unsigned short crc = 0;
    unsigned char i,send_data[7] = {0};

    //applog(LOG_DEBUG,"%s", __FUNCTION__);

    crc = length + ENABLE_VOLTAGE + 1;
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = ENABLE_VOLTAGE;
    send_data[4] = 1;
    send_data[5] = crc_data[0];
    send_data[6] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<7; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(100*1000);
    for(i=0; i<2; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    if((read_back_data[0] != ENABLE_VOLTAGE) || (read_back_data[1] != 1))
    {
        applog(LOG_ERR,"%s failed! read_back_data[0] = 0x%02x, read_back_data[1] = 0x%02x", __FUNCTION__, read_back_data[0], read_back_data[1]);
        return 0;   // error
    }
    else
    {
        applog(LOG_NOTICE,"%s ok", __FUNCTION__);
        return 1;   // ok
    }
}


int disable_PIC16F1704_dc_dc_new(void)
{
    unsigned char length = 0x05, crc_data[2] = {0xff}, read_back_data[2] = {0xff};
    unsigned short crc = 0;
    unsigned char i,send_data[7] = {0};

    //applog(LOG_DEBUG,"%s", __FUNCTION__);

    crc = length + ENABLE_VOLTAGE + 0;
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = ENABLE_VOLTAGE;
    send_data[4] = 0;
    send_data[5] = crc_data[0];
    send_data[6] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<7; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(100*1000);
    for(i=0; i<2; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    if((read_back_data[0] != ENABLE_VOLTAGE) || (read_back_data[1] != 1))
    {
        applog(LOG_ERR,"%s failed! read_back_data[0] = 0x%02x, read_back_data[1] = 0x%02x", __FUNCTION__, read_back_data[0], read_back_data[1]);
        return 0;   // error
    }
    else
    {
        applog(LOG_DEBUG,"%s ok", __FUNCTION__);
        return 1;   // ok
    }
}



int heart_beat_PIC16F1704_new(void)
{
    unsigned char length = 0x04, crc_data[2] = {0xff}, read_back_data[6] = {0xff};
    unsigned short crc = 0;
    unsigned char i,send_data[6] = {0};

    //applog(LOG_DEBUG,"%s", __FUNCTION__);

    crc = length + SEND_HEART_BEAT;
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = SEND_HEART_BEAT;
    send_data[4] = crc_data[0];
    send_data[5] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<6; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(100*1000);
    for(i=0; i<6; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    if((read_back_data[1] != SEND_HEART_BEAT) || (read_back_data[2] != 1))
    {
        applog(LOG_ERR,"%s failed!", __FUNCTION__);
        applog(LOG_ERR,"%s: read_back_data[0] = 0x%x, read_back_data[1] = 0x%x, read_back_data[2] = 0x%x, read_back_data[3] = 0x%x, read_back_data[4] = 0x%x, read_back_data[5] = 0x%x\n",
               __FUNCTION__, read_back_data[0], read_back_data[1], read_back_data[2], read_back_data[3], read_back_data[4], read_back_data[5]);
        return 0;   // error
    }
    else
    {
        applog(LOG_DEBUG,"%s ok", __FUNCTION__);
        return 1;   // ok
    }
}


int get_PIC16F1704_software_version_new(unsigned char *version)
{
    unsigned char length = 0x04, crc_data[2] = {0xff}, read_back_data[5] = {0xff};
    unsigned short crc = 0;
    unsigned char i,send_data[6] = {0};

    //applog(LOG_DEBUG,"%s", __FUNCTION__);

    crc = length + READ_PIC_SOFTWARE_VERSION;
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = READ_PIC_SOFTWARE_VERSION;
    send_data[4] = crc_data[0];
    send_data[5] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<6; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(400*1000);
    for(i=0; i<5; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    //usleep(200*1000);

    applog(LOG_DEBUG,"%s: read_back_data[0] = 0x%x, read_back_data[1] = 0x%x, read_back_data[2] = 0x%x, read_back_data[3] = 0x%x, read_back_data[4] = 0x%x",
           __FUNCTION__, read_back_data[0], read_back_data[1], read_back_data[2], read_back_data[3], read_back_data[4]);

    if((read_back_data[1] != READ_PIC_SOFTWARE_VERSION) || (read_back_data[0] != 5))
    {
        applog(LOG_ERR,"%s failed!", __FUNCTION__);
        applog(LOG_ERR,"%s: read_back_data[0] = 0x%x, read_back_data[1] = 0x%x, read_back_data[2] = 0x%x, read_back_data[3] = 0x%x, read_back_data[4] = 0x%x",
               __FUNCTION__, read_back_data[0], read_back_data[1], read_back_data[2], read_back_data[3], read_back_data[4]);
        return 0;   // error
    }
    else
    {
        crc = read_back_data[0] + read_back_data[1] + read_back_data[2];
        if(((unsigned char)((crc >> 8) & 0x00ff) != read_back_data[3]) || ((unsigned char)((crc >> 0) & 0x00ff) != read_back_data[4]))
        {
            applog(LOG_ERR,"%s failed! crc = 0x%04x", __FUNCTION__, crc);
            return 0;   // error
        }
        *version = read_back_data[2];
        applog(LOG_NOTICE,"%s ok, version = 0x%02x", __FUNCTION__, *version);
        return 1;   // ok
    }
}


int get_PIC16F1704_voltage_new(unsigned char *voltage)
{
    unsigned char length = 0x04, crc_data[2] = {0xff}, read_back_data[5] = {0xff};
    unsigned short crc = 0;
    unsigned char i,send_data[6] = {0};

    //applog(LOG_DEBUG,"%s", __FUNCTION__);

    crc = length + GET_VOLTAGE;
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = GET_VOLTAGE;
    send_data[4] = crc_data[0];
    send_data[5] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<6; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(200*1000);
    for(i=0; i<5; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    applog(LOG_NOTICE,"%s: read_back_data[0] = 0x%x, read_back_data[1] = 0x%x, read_back_data[2] = 0x%x, read_back_data[3] = 0x%x, read_back_data[4] = 0x%x\n",
           __FUNCTION__, read_back_data[0], read_back_data[1], read_back_data[2], read_back_data[3], read_back_data[4]);

    if((read_back_data[1] != GET_VOLTAGE) || (read_back_data[0] != 5))
    {
        applog(LOG_ERR,"%s failed!", __FUNCTION__);
        return 0;   // error
    }
    else
    {
        crc = read_back_data[0] + read_back_data[1] + read_back_data[2];
        if(((unsigned char)((crc >> 8) & 0x00ff) != read_back_data[3]) || ((unsigned char)((crc >> 0) & 0x00ff) != read_back_data[4]))
        {
            applog(LOG_ERR,"%s failed! crc = 0x%04x", __FUNCTION__, crc);
            return 0;   // error
        }
        *voltage = read_back_data[2];
        applog(LOG_NOTICE,"%s ok, voltage = 0x%02x", __FUNCTION__, *voltage);
        return 1;   // ok
    }
}


int write_temperature_offset_PIC16F1704_new(unsigned char *buf)
{
    unsigned char length = 0x0c, crc_data[2] = {0xff}, read_back_data[2] = {0xff};
    unsigned short crc = 0;
    unsigned char i,send_data[14] = {0};

    //applog(LOG_DEBUG,"%s", __FUNCTION__);

    crc = length + WR_TEMP_OFFSET_VALUE;
    for(i=0; i<8; i++)
    {
        crc += *(buf + i);
    }
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = WR_TEMP_OFFSET_VALUE;
    for(i=0; i<8; i++)
    {
        send_data[i+4]= *(buf + i);
    }
    send_data[12] = crc_data[0];
    send_data[13] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<14; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(200*1000);
    for(i=0; i<2; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    usleep(200*1000);

    if((read_back_data[0] != WR_TEMP_OFFSET_VALUE) || (read_back_data[1] != 1))
    {
        applog(LOG_ERR,"%s failed! read_back_data[0] = 0x%02x, read_back_data[1] = 0x%02x", __FUNCTION__, read_back_data[0], read_back_data[1]);
        return 0;   // error
    }
    else
    {
        applog(LOG_NOTICE,"%s ok", __FUNCTION__);
        return 1;   // ok
    }
}


int read_temperature_offset_PIC16F1704_new(unsigned char *buf)
{
    unsigned char length = 0x04, crc_data[2] = {0xff}, read_back_data[0x0c] = {0xff};
    unsigned short crc = 0;
    unsigned char i = 0;
    unsigned char send_data[6] = {0};

    //applog(LOG_DEBUG,"%s", __FUNCTION__);

    crc = length + RD_TEMP_OFFSET_VALUE;
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = RD_TEMP_OFFSET_VALUE;
    send_data[4] = crc_data[0];
    send_data[5] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<6; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(200*1000);
    for(i=0; i<12; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    applog(LOG_NOTICE,"%s: read_back_data[0] = 0x%x, read_back_data[1] = 0x%x, read_back_data[2] = 0x%x, read_back_data[3] = 0x%x, \
		read_back_data[4] = 0x%x, read_back_data[5] = 0x%x, read_back_data[6] = 0x%x, read_back_data[7] = 0x%x, \
		read_back_data[8] = 0x%x, read_back_data[9] = 0x%x, read_back_data[10] = 0x%x, read_back_data[11] = 0x%x\n", __FUNCTION__,\
           read_back_data[0], read_back_data[1], read_back_data[2], read_back_data[3], read_back_data[4], read_back_data[5], \
           read_back_data[6], read_back_data[7], read_back_data[8], read_back_data[9], read_back_data[10], read_back_data[11]);

    if((read_back_data[1] != RD_TEMP_OFFSET_VALUE) || (read_back_data[0] != 0x0c))
    {
        applog(LOG_ERR,"%s failed!", __FUNCTION__);
        return 0;   // error
    }
    else
    {
        crc = read_back_data[0] + read_back_data[1] + read_back_data[2] + read_back_data[3] + read_back_data[4] + read_back_data[5] +
              read_back_data[6] + read_back_data[7] + read_back_data[8] + read_back_data[9];

        if(((unsigned char)((crc >> 8) & 0x00ff) != read_back_data[10]) || ((unsigned char)((crc >> 0) & 0x00ff) != read_back_data[11]))
        {
            applog(LOG_ERR,"%s failed! crc = 0x%04x", __FUNCTION__, crc);
            return 0;   // error
        }
        for(i=0; i<8; i++)
        {
            *(buf + i) = read_back_data[2 + i];
        }
        applog(LOG_NOTICE,"%s ok", __FUNCTION__);
        return 1;   // ok
    }
}


unsigned char erase_PIC16F1704_app_flash_new(void)
{
    unsigned char ret=0xff;
    unsigned int i=0, erase_loop = 0;
    unsigned char start_addr_h = PIC_FLASH_POINTER_START_ADDRESS_H_NEW, start_addr_l = PIC_FLASH_POINTER_START_ADDRESS_L_NEW;
    unsigned char end_addr_h = PIC_FLASH_POINTER_END_ADDRESS_H, end_addr_l = PIC_FLASH_POINTER_END_ADDRESS_L;
    unsigned int pic_flash_length=0;

    //applog(LOG_NOTICE,"%s", __FUNCTION__);

    set_PIC16F1704_flash_pointer_new(PIC_FLASH_POINTER_START_ADDRESS_H_NEW, PIC_FLASH_POINTER_START_ADDRESS_L_NEW);

    pic_flash_length = (((unsigned int)end_addr_h << 8) + end_addr_l) - (((unsigned int)start_addr_h << 8) + start_addr_l) + 1;
    erase_loop = pic_flash_length/PIC_FLASH_SECTOR_LENGTH;
    applog(LOG_NOTICE,"%s: erase_loop = %d\n", __FUNCTION__, erase_loop);

    for(i=0; i<erase_loop; i++)
    {
        erase_PIC16F1704_flash_new();
    }
}


int PIC1704_update_pic_app_program_new(void)
{
    unsigned char program_data[5000] = {0};
    FILE * pic_program_file;
    unsigned int filesize = 0,i=0,j;
    unsigned char data_read[5]= {0,0,0,0,'\0'}, buf[16]= {0};
    unsigned int data_int = 0;
    struct stat statbuff;
    unsigned char start_addr_h = PIC_FLASH_POINTER_START_ADDRESS_H_NEW, start_addr_l = PIC_FLASH_POINTER_START_ADDRESS_L_NEW;
    unsigned char end_addr_h = PIC_FLASH_POINTER_END_ADDRESS_H, end_addr_l = PIC_FLASH_POINTER_END_ADDRESS_L;
    unsigned int pic_flash_length=0;
    int ret=0;

    applog(LOG_NOTICE,"%s", __FUNCTION__);

    // read upgrade file first, if it is wrong, don't erase pic, but just return;
    pic_program_file = fopen(PIC16F1704_PROGRAM_NEW, "r");
    if(!pic_program_file)
    {
        applog(LOG_ERR,"%s: open pic16f1704_app_new.txt failed\n", __FUNCTION__);
        return;
    }
    fseek(pic_program_file,0,SEEK_SET);
    memset(program_data, 0x0, 5000);

    pic_flash_length = (((unsigned int)end_addr_h << 8) + end_addr_l) - (((unsigned int)start_addr_h << 8) + start_addr_l) + 1;
    applog(LOG_NOTICE,"%s: pic_flash_length = %d\n", __FUNCTION__, pic_flash_length);

    for(i=0; i<pic_flash_length; i++)
    {
        fgets(data_read, MAX_CHAR_NUM - 1, pic_program_file);
        //printf("data_read[0]=%c, data_read[1]=%c, data_read[2]=%c, data_read[3]=%c\n", data_read[0], data_read[1], data_read[2], data_read[3]);
        data_int = strtoul(data_read, NULL, 16);
        //printf("data_int = 0x%04x\n", data_int);
        program_data[2*i + 0] = (unsigned char)((data_int >> 8) & 0x000000ff);
        program_data[2*i + 1] = (unsigned char)(data_int & 0x000000ff);
        //printf("program_data[%d]=0x%02x, program_data[%d]=0x%02x\n\n", 2*i + 0, program_data[2*i + 0], 2*i + 1, program_data[2*i + 1]);
    }

    fclose(pic_program_file);

    // after read upgrade file correct, erase pic
    ret = reset_PIC16F1704_pic_new();
    if(ret == 0)
    {
        applog(LOG_ERR,"%s: reset pic error!\n\n", __FUNCTION__);
        return 0;
    }

    ret = erase_PIC16F1704_app_flash_new();
    if(ret == 0)
    {
        applog(LOG_ERR,"%s: erase flash error!\n\n", __FUNCTION__);
        return 0;
    }

    ret = set_PIC16F1704_flash_pointer_new(PIC_FLASH_POINTER_START_ADDRESS_H_NEW, PIC_FLASH_POINTER_START_ADDRESS_L_NEW);
    if(ret == 0)
    {
        applog(LOG_ERR,"%s: set flash pointer error!\n\n", __FUNCTION__);
        return 0;
    }

    for(i=0; i<pic_flash_length/PIC_FLASH_SECTOR_LENGTH*4; i++)
    {
        memcpy(buf, program_data+i*16, 16);
        /**/
        applog(LOG_NOTICE,"send pic program time: %d",i);
        for(j=0; j<16; j++)
        {
            applog(LOG_DEBUG,"buf[%d] = 0x%02x", j, *(buf+j));
        }

        send_data_to_PIC16F1704_new(buf);
        write_data_into_PIC16F1704_flash_new();
    }

    ret = reset_PIC16F1704_pic_new();
    if(ret == 0)
    {
        applog(LOG_ERR,"%s: reset pic error!\n\n", __FUNCTION__);
        return 0;
    }

    return 1;
}


void every_chain_reset_PIC16F1704_pic_new(void)
{
    unsigned char which_chain;

    applog(LOG_NOTICE, "%s", __FUNCTION__);

    for(which_chain=0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        if(dev.chain_exist[which_chain] == 1)
        {
            pthread_mutex_lock(&iic_mutex);
            if(unlikely(ioctl(dev.i2c_fd,I2C_SLAVE,i2c_slave_addr[which_chain] >> 1 ) < 0))
                applog(LOG_ERR, "ioctl error @ line %d",__LINE__);
            reset_PIC16F1704_pic_new();
            pthread_mutex_unlock(&iic_mutex);
            cgsleep_ms(100);
        }
    }
    cgsleep_ms(500);
}

void every_chain_jump_from_loader_to_app_PIC16F1704_new(void)
{
    unsigned char which_chain;

    applog(LOG_NOTICE, "%s", __FUNCTION__);

    for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        if(dev.chain_exist[which_chain] == 1)
        {
            pthread_mutex_lock(&iic_mutex);
            if(unlikely(ioctl(dev.i2c_fd,I2C_SLAVE,i2c_slave_addr[which_chain] >> 1 ) < 0))
                applog(LOG_ERR, "ioctl error @ line %d",__LINE__);
            jump_from_loader_to_app_PIC16F1704_new();
            pthread_mutex_unlock(&iic_mutex);
        }
        cgsleep_ms(100);
    }
    cgsleep_ms(500);
}

void every_chain_disable_PIC16F1704_dc_dc_new(void)
{
    unsigned char which_chain;
    applog(LOG_NOTICE, "%s", __FUNCTION__);

    for(which_chain=0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        if(dev.chain_exist[which_chain] == 1)
        {
            pthread_mutex_lock(&iic_mutex);
            if(unlikely(ioctl(dev.i2c_fd,I2C_SLAVE,i2c_slave_addr[which_chain] >> 1 ) < 0))
                applog(LOG_ERR,"ioctl error @ line %d",__LINE__);
            disable_PIC16F1704_dc_dc_new();
            pthread_mutex_unlock(&iic_mutex);
        }
    }
    cgsleep_ms(500);
}

void every_chain_enable_PIC16F1704_dc_dc_new(void)
{
    unsigned char which_chain;

    applog(LOG_NOTICE, "%s", __FUNCTION__);

    for(which_chain=0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        if(dev.chain_exist[which_chain] == 1)
        {
            pthread_mutex_lock(&iic_mutex);
            if(unlikely(ioctl(dev.i2c_fd, I2C_SLAVE, i2c_slave_addr[which_chain] >> 1) < 0))
                applog(LOG_ERR,"ioctl error @ line %d",__LINE__);
            enable_PIC16F1704_dc_dc_new();
            pthread_mutex_unlock(&iic_mutex);
            cgsleep_ms(100);
        }
    }
    cgsleep_ms(500);
}


int send_heart_beat_to_every_chain(void)
{
    pic_heart_beat = calloc(1,sizeof(struct thr_info));
    if(thr_info_create(pic_heart_beat, NULL, pic_heart_beat_func_new, pic_heart_beat))
    {
        applog(LOG_ERR, "%s: create thread error for pic_heart_beat_func", __FUNCTION__);
        return -3;
    }
    pthread_detach(pic_heart_beat->pth);
}


void check_whether_need_update_pic_program(void)
{
    unsigned char which_chain, pic_update_counter = 0;
    bool need_update_pic = false;
    int ret = 0;

    applog(LOG_NOTICE, "%s", __FUNCTION__);

    for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        pic_update_counter = 0;
        need_update_pic = false;

        if(dev.chain_exist[which_chain] == 1)
        {
            pthread_mutex_lock(&iic_mutex);
            if(unlikely(ioctl(dev.i2c_fd,I2C_SLAVE,i2c_slave_addr[which_chain] >> 1 ) < 0))
                applog(LOG_ERR, "ioctl error @ line %d",__LINE__);
            reset_PIC16F1704_pic_new();
            cgsleep_ms(100);
            jump_from_loader_to_app_PIC16F1704_new();
            cgsleep_ms(100);
            ret = get_PIC16F1704_software_version_new(&pic_version[which_chain]);
            pthread_mutex_unlock(&iic_mutex);

            if((pic_version[which_chain] < PIC_VERSION) && (ret == 1))
                //if(ret == 1)
            {
                need_update_pic = true;
            }
        }

        while(need_update_pic && (pic_update_counter < 3))
        {
            need_update_pic = false;

            pthread_mutex_lock(&iic_mutex);
            if(unlikely(ioctl(dev.i2c_fd,I2C_SLAVE,i2c_slave_addr[which_chain] >> 1 ) < 0))
                applog(LOG_ERR, "ioctl error @ line %d",__LINE__);
            PIC1704_update_pic_app_program_new();
            cgsleep_ms(100);
            jump_from_loader_to_app_PIC16F1704_new();
            cgsleep_ms(200);
            ret = get_PIC16F1704_software_version_new(&pic_version[which_chain]);
            pthread_mutex_unlock(&iic_mutex);

            pic_update_counter++;
            applog(LOG_NOTICE, "%s: Chain%d pic update for %d times",__FUNCTION__, which_chain, pic_update_counter);

            if((pic_version[which_chain] != PIC_VERSION) || (ret == 0))
            {
                need_update_pic = true;
            }
        }
    }
}

void check_every_chain_asic_number(bool whether_update_asic_num)
{
    unsigned char which_chain;

    applog(LOG_NOTICE, "%s", __FUNCTION__);

    for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        if(dev.chain_exist[which_chain])
        {
            update_asic_num = whether_update_asic_num;
            check_asic_reg(which_chain, 1, 0, CHIP_ADDR);
        }
        cgsleep_ms(100);
    }
}


int save_freq_PIC16F1704_new(unsigned short freq)
{
    unsigned char length = 0x06, crc_data[2] = {0xff}, read_back_data[2] = {0xff};
    unsigned short crc = 0;
    unsigned char i,send_data[8] = {0};

    //applog(LOG_DEBUG, "\n--- %s\n", __FUNCTION__);

    crc = length + SAVE_FREQ + ((freq >> 8) & 0xff) + (freq & 0xff);
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = SAVE_FREQ;
    send_data[4] = (freq >> 8) & 0xff;
    send_data[5] = freq & 0xff;
    send_data[6] = crc_data[0];
    send_data[7] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<8; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(300*1000);
    for(i=0; i<2; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    usleep(200*1000);

    if((read_back_data[0] != SAVE_FREQ) || (read_back_data[1] != 1))
    {
        applog(LOG_ERR, "\n--- %s failed! read_back_data[0] = 0x%02x, read_back_data[1] = 0x%02x\n\n", __FUNCTION__, read_back_data[0], read_back_data[1]);
        return 0;   // error
    }
    else
    {
        applog(LOG_DEBUG, "\n--- %s ok\n\n", __FUNCTION__);
        return 1;   // ok
    }
}

int get_PIC16F1704_freq_new(unsigned short *freq)
{
    unsigned char length = 0x04, crc_data[2] = {0xff}, read_back_data[6] = {0xff};
    unsigned short crc = 0;
    unsigned char i,send_data[6] = {0};

    //applog(LOG_NOTICE, "\n--- %s\n", __FUNCTION__);

    crc = length + READ_OUT_FREQ;
    crc_data[0] = (unsigned char)((crc >> 8) & 0x00ff);
    crc_data[1] = (unsigned char)((crc >> 0) & 0x00ff);
    //printf("--- %s: crc_data[0] = 0x%x, crc_data[1] = 0x%x\n", __FUNCTION__, crc_data[0], crc_data[1]);

    send_data[0] = PIC_COMMAND_1;
    send_data[1] = PIC_COMMAND_2;
    send_data[2] = length;
    send_data[3] = READ_OUT_FREQ;
    send_data[4] = crc_data[0];
    send_data[5] = crc_data[1];

    pthread_mutex_lock(&i2c_mutex);
    for(i=0; i<6; i++)
    {
        write(dev.i2c_fd, send_data+i, 1);
    }
    usleep(300*1000);
    for(i=0; i<6; i++)
    {
        read(dev.i2c_fd, read_back_data+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    usleep(200*1000);

    applog(LOG_DEBUG, "--- %s: read_back_data[0] = 0x%x, read_back_data[1] = 0x%x, read_back_data[2] = 0x%x, read_back_data[3] = 0x%x, read_back_data[4] = 0x%x, read_back_data[5] = 0x%x\n",
           __FUNCTION__, read_back_data[0], read_back_data[1], read_back_data[2], read_back_data[3], read_back_data[4], read_back_data[5]);

    if((read_back_data[1] != READ_OUT_FREQ) || (read_back_data[0] != 6))
    {
        applog(LOG_ERR, "\n--- %s failed!\n\n", __FUNCTION__);
        return 0;   // error
    }
    else
    {
        crc = read_back_data[0] + read_back_data[1] + read_back_data[2] + read_back_data[3];
        if(((unsigned char)((crc >> 8) & 0x00ff) != read_back_data[4]) || ((unsigned char)((crc >> 0) & 0x00ff) != read_back_data[5]))
        {
            applog(LOG_ERR, "\n--- %s failed! crc = 0x%04x\n\n", __FUNCTION__, crc);
            return 0;   // error
        }
        *freq = (read_back_data[2] << 8) | read_back_data[3];
        applog(LOG_DEBUG, "\n--- %s ok, freq = %d\n\n", __FUNCTION__, *freq);
        return 1;   // ok
    }

}

void every_chain_set_voltage_PIC16F1704_new(unsigned short voltage)
{
    unsigned char which_chain;

    double temp_voltage = 1609.927422-182.739369*(voltage)/100;
    uint8_t pic_voltage1 = (uint8_t)temp_voltage;
    applog(LOG_NOTICE,"set voltage = %.6f  real:%u mv\n", temp_voltage, voltage);

    for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        if(dev.chain_exist[which_chain] == 1)
        {
            pthread_mutex_lock(&iic_mutex);
            if(unlikely(ioctl(dev.i2c_fd,I2C_SLAVE,i2c_slave_addr[which_chain] >> 1 ) < 0))
                applog(LOG_ERR, "ioctl error @ line %d",__LINE__);
            set_PIC16F1704_voltage_new(pic_voltage1);
            pthread_mutex_unlock(&iic_mutex);
        }
        cgsleep_ms(100);
    }
    cgsleep_ms(500);
}

void every_chain_get_voltage_PIC16F1704_new(unsigned short voltage)
{
    unsigned char which_chain;

    uint8_t pic_voltage1;

    for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        if(dev.chain_exist[which_chain] == 1)
        {
            pthread_mutex_lock(&iic_mutex);
            if(unlikely(ioctl(dev.i2c_fd,I2C_SLAVE,i2c_slave_addr[which_chain] >> 1 ) < 0))
                applog(LOG_ERR, "ioctl error @ line %d",__LINE__);
            get_PIC16F1704_voltage_new(&pic_voltage1);
            applog(LOG_NOTICE,"Chain %u voltage %u",which_chain,pic_voltage1);
            pthread_mutex_unlock(&iic_mutex);
        }
        cgsleep_ms(100);
    }
    cgsleep_ms(500);
}


void every_chain_save_freq_PIC16F1704_new(unsigned short freq)
{
    unsigned char which_chain;

    applog(LOG_NOTICE, "%s %u", "set freq:", freq);

    for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        if(dev.chain_exist[which_chain] == 1)
        {
            pthread_mutex_lock(&iic_mutex);
            if(unlikely(ioctl(dev.i2c_fd,I2C_SLAVE,i2c_slave_addr[which_chain] >> 1 ) < 0))
                applog(LOG_ERR, "ioctl error @ line %d",__LINE__);
            save_freq_PIC16F1704_new(freq);
            pthread_mutex_unlock(&iic_mutex);
        }
        cgsleep_ms(100);
    }
    cgsleep_ms(500);
}

void every_chain_get_PIC16F1704_freq_new(unsigned short *freq)
{
    unsigned char which_chain;
    unsigned short freq_read_back[BITMAIN_MAX_CHAIN_NUM] = {0}, min_freq = 650;

    applog(LOG_NOTICE, "%s", __FUNCTION__);

    for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        if(dev.chain_exist[which_chain] == 1)
        {
            pthread_mutex_lock(&iic_mutex);
            if(unlikely(ioctl(dev.i2c_fd,I2C_SLAVE,i2c_slave_addr[which_chain] >> 1 ) < 0))
                applog(LOG_ERR, "ioctl error @ line %d",__LINE__);
            get_PIC16F1704_freq_new(&freq_read_back[which_chain]);
            if(freq_read_back[which_chain] < min_freq)
            {
                min_freq = freq_read_back[which_chain];
            }
            pthread_mutex_unlock(&iic_mutex);
        }
        cgsleep_ms(100);
    }

    *freq = min_freq;

    cgsleep_ms(500);
}



void pic_test_new(void)
{
    unsigned char ret = 0,i;
    unsigned char version = 0, flash_addr_h = 0, flash_addr_l = 0, voltage = 0;
    unsigned char buf_send[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    unsigned char buf_receive[16] = {0};
    unsigned char hash_id_send[12] = {10,11,12,13,14,15,16,17,18,19,20,21};
    unsigned char hash_id_receive[12] = {0};
    unsigned char temp_offset_send[8] = {20,21,22,23,24,25,26,27};
    unsigned char temp_offset_receive[8] = {0};

    //ret = use_new_or_old_PIC16F1704_api();

    reset_PIC16F1704_pic_new();

    /*
    set_PIC16F1704_flash_pointer_new(0x03, 0x00);
    read_PIC16F1704_flash_pointer_new(&flash_addr_h, &flash_addr_l);

    set_PIC16F1704_flash_pointer_new(0x07, 0x00);
    read_PIC16F1704_flash_pointer_new(&flash_addr_h, &flash_addr_l);
    */

    /*
    set_PIC16F1704_flash_pointer_new(0x0f, 0x80);
    erase_PIC16F1704_flash_new();
    set_PIC16F1704_flash_pointer_new(0x0f, 0x80);
    send_data_to_PIC16F1704_new(buf_send);
    write_data_into_PIC16F1704_flash_new();
    set_PIC16F1704_flash_pointer_new(0x0f, 0x80);
    read_PIC16F1704_flash_data_new(buf_receive);
    for(i=0;i<16;i++)
    {
        printf("buf_receive[%02d] = %d\n", i, buf_receive[i]);
    }
    */

    ret = PIC1704_update_pic_app_program_new();


    /**/
    jump_from_loader_to_app_PIC16F1704_new();


    /**/
    get_PIC16F1704_software_version_new(&version);

    write_hash_ID_PIC16F1704_new(hash_id_send);
    read_hash_id_PIC16F1704_new(hash_id_receive);

    write_temperature_offset_PIC16F1704_new(temp_offset_send);
    read_temperature_offset_PIC16F1704_new(temp_offset_receive);

    set_PIC16F1704_voltage_new(0x78);
    get_PIC16F1704_voltage_new(&voltage);

    enable_PIC16F1704_dc_dc_new();


    /**/
    enable_PIC16F1704_dc_dc_new();

    for(i=0; i<3; i++)
    {
        heart_beat_PIC16F1704_new();
        usleep(2*1000*1000);
    }

    for(i=0; i<35; i++)
    {
        printf("i = %d\n", i);
        usleep(2*1000*1000);
    }

    for(i=0; i<3; i++)
    {
        heart_beat_PIC16F1704_new();
        usleep(2*1000*1000);
    }

}


/****************** about PIC16F1704 end ******************/


#define ABOUT_BM1760_ASIC
/****************** about BM1760 ASIC ******************/

void get_status(unsigned int fd, unsigned char mode, unsigned char chip_addr, unsigned char reg_addr)
{
    unsigned char buf[CMD_LENTH] = {0,0,0,0,0};

    if(mode)
    {
        buf[0] = CMD_TYPE | CMD_ALL | GET_STATUS;
    }
    else
    {
        buf[0] = CMD_TYPE | GET_STATUS;
    }

    buf[1] = CMD_LENTH;

    if(mode)
    {
        buf[2] = 0;
    }
    else
    {
        buf[2] = chip_addr;
    }

    buf[3] = reg_addr;
    buf[4] = CRC5(buf, 8*(CMD_LENTH - 1));

    //applog(LOG_DEBUG, "%s: reg_addr: 0x%02x : 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x" ,__FUNCTION__, reg_addr, buf[0], buf[1], buf[2], buf[3], buf[4]);

    DASH_write(fd, buf, CMD_LENTH);
}


void set_config(unsigned int fd, unsigned char mode, unsigned char asic_addr, unsigned char reg_addr, unsigned int reg_data)
{
    unsigned char buf[CONFIG_LENTH] = {0,0,0,0,0,0,0,0,0};

    if(mode)
    {
        buf[0] = CMD_TYPE | CMD_ALL | SET_CONFIG;
    }
    else
    {
        buf[0] = CMD_TYPE | SET_CONFIG;
    }

    buf[1] = CONFIG_LENTH;

    if(mode)
    {
        buf[2] = 0;
    }
    else
    {
        buf[2] = asic_addr;
    }

    buf[3] = reg_addr;
    buf[4] = (unsigned char)((reg_data >> 24) & 0xff);
    buf[5] = (unsigned char)((reg_data >> 16) & 0xff);
    buf[6] = (unsigned char)((reg_data >> 8) & 0xff);
    buf[7] = (unsigned char)((reg_data >> 0) & 0xff);
    buf[8] = CRC5(buf, 8*(CONFIG_LENTH - 1));

    /*
    applog(LOG_DEBUG, "%s: reg_addr: 0x%02x : 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x" ,__FUNCTION__, reg_addr,buf[0], buf[1], buf[2],
           buf[3],buf[4], buf[5], buf[6], buf[7],buf[8]);
    */
    DASH_write(fd, buf, CONFIG_LENTH);
}


void check_asic_reg(unsigned int which_chain, unsigned char mode, unsigned char chip_addr, unsigned char reg_addr)
{

    unsigned char not_receive_reg_value_counter=0;
    unsigned char i=0;
    unsigned int temp_counter = 0;

    switch(reg_addr)
    {
        case CHIP_ADDR:
            if(update_asic_num)
            {
                g_CHIP_ADDR_reg_value_num[which_chain] = 0;

                for(i = 0; i < ASIC_NUM_EACH_CHAIN; i++)
                {
                    g_CHIP_ADDR_reg_value[which_chain][i] = 0;
                }
            }
            break;

        case GENERAL_I2C_COMMAND:
            break;

        case HASH_RATE:
            g_HASH_RATE_reg_value_num[which_chain] = 0;

            for(i = 0; i< ASIC_NUM_EACH_CHAIN; i++)
            {
                g_HASH_RATE_reg_value[which_chain][i] = 0;
                g_HASH_RATE_reg_value_from_which_asic[which_chain][i] = 0;
            }
            break;

        case CHIP_STATUS:
            g_CHIP_STATUS_reg_value_num[which_chain] = 0;

            for(i = 0; i < ASIC_NUM_EACH_CHAIN; i++)
            {
                g_CHIP_STATUS_reg_value[which_chain][i] = 0xffffffff;
            }
            break;

        case TICK_MASK:
            break;

        default:
            applog(LOG_DEBUG, "!!! %s: the input register is not correct, please check it. the input register is 0x%02x", reg_addr);
            break;
    }

    get_status(dev.dev_fd[which_chain], mode, chip_addr, reg_addr);

    switch(reg_addr)
    {
        case CHIP_ADDR:
            while(not_receive_reg_value_counter < 3)    //if there is no register value for 3 times, we can think all asic return their register value
            {
                cgsleep_ms(50);
                if(temp_counter != g_CHIP_ADDR_reg_value_num[which_chain])
                {
                    not_receive_reg_value_counter = 0;
                    temp_counter = g_CHIP_ADDR_reg_value_num[which_chain];
                }
                else
                {
                    not_receive_reg_value_counter++;
                    applog(LOG_DEBUG, "%s: not receive CHIP_ADDR register value for %d time", __FUNCTION__, not_receive_reg_value_counter);
                }
            }
            applog(LOG_WARNING, "%s: Chain%d has %d ASICs", __FUNCTION__, which_chain, g_CHIP_ADDR_reg_value_num[which_chain]);
            if(update_asic_num)
            {
                update_asic_num = false;
                dev.chain_asic_num[which_chain] = g_CHIP_ADDR_reg_value_num[which_chain];
                if(dev.chain_asic_num[which_chain] > dev.max_asic_num_in_one_chain)
                {
                    dev.max_asic_num_in_one_chain = dev.chain_asic_num[which_chain];
                }
            }
            break;
        case GENERAL_I2C_COMMAND:
            cgsleep_ms(50);
            break;
        case HASH_RATE:
            while(not_receive_reg_value_counter < 3)    //if there is no register value for 3 times, we can think all asic return their register value
            {
                cgsleep_ms(50);
                if(temp_counter != g_HASH_RATE_reg_value_num[which_chain])
                {
                    not_receive_reg_value_counter = 0;
                    temp_counter = g_HASH_RATE_reg_value_num[which_chain];
                }
                else
                {
                    not_receive_reg_value_counter++;
                    //applog(LOG_DEBUG, "%s: not receive HASH_RATE register value for %d time", __FUNCTION__, not_receive_reg_value_counter);
                }
            }
            break;
        case CHIP_STATUS:
            while(not_receive_reg_value_counter < 3)    //if there is no register value for 3 times, we can think all asic return their register value
            {
                cgsleep_ms(50);
                if(temp_counter != g_CHIP_STATUS_reg_value_num[which_chain])
                {
                    not_receive_reg_value_counter = 0;
                    temp_counter = g_CHIP_STATUS_reg_value_num[which_chain];
                }
                else
                {
                    not_receive_reg_value_counter++;
                    //applog(LOG_DEBUG, "%s: not receive CHIP_STATUS register value for %d time", __FUNCTION__, not_receive_reg_value_counter);
                }
            }
            break;
        case TICK_MASK:
            break;
        default:
            applog(LOG_ERR, "!!! %s: the input register is not correct, please check it. the input register is 0x%02x\n", reg_addr);
            break;
    }
}


void chain_inactive(int const which_chain)
{
    unsigned char cmd_buf[5] = {0};
    int fd = dev.dev_fd[which_chain];

    cmd_buf[0] = CMD_TYPE | CMD_ALL | CHAIN_INACTIVE;
    cmd_buf[1] = CMD_LENTH;
    cmd_buf[2] = 0;
    cmd_buf[3] = 0;
    cmd_buf[4] = CRC5(cmd_buf, 8*(CMD_LENTH - 1) );

    DASH_write(fd, cmd_buf, CMD_LENTH);
}


inline void set_address(int const which_chain, unsigned char chip_addr)
{
    unsigned char cmd_buf[5] = {0};
    int fd = dev.dev_fd[which_chain];

    cmd_buf[0] = CMD_TYPE | SET_ADDR;
    cmd_buf[1] = CMD_LENTH;
    cmd_buf[2] = chip_addr;
    cmd_buf[3] = 0;
    cmd_buf[4] = CRC5(cmd_buf, 8*(CMD_LENTH - 1) );

    DASH_write(fd, cmd_buf, CMD_LENTH);
}


void software_set_address_chain(unsigned int which_chain)
{
    unsigned int which_asic, which_sensor;
    applog(LOG_NOTICE,"%s: chain %d has %d ASIC, and addrInterval is %d", __FUNCTION__, which_chain, dev.chain_asic_num[which_chain], dev.addrInterval);
    // set ASIC's address, and CHIP_OFFSET / CORE_OFFSET register
    if(dev.chain_exist[which_chain] == 1 /*&& dev.chain_asic_num[i] == CHAIN_ASIC_NUM*/)
    {
        chain_inactive(which_chain);
        cgsleep_ms(30);

        applog(LOG_NOTICE, "Now Set [%d] Chain Address", which_chain);
        for(which_asic = 0; which_asic < ASIC_NUM_EACH_CHAIN; which_asic++)
        {
            set_address(which_chain, which_asic * dev.addrInterval);
            cgsleep_ms(10);
        }

        set_config(dev.dev_fd[which_chain], 1, 0, CHIP_OFFSET, gChipOffset);
        cgsleep_ms(10);
        set_config(dev.dev_fd[which_chain], 1, 0, CORE_OFFSET, gCoreOffset);
        cgsleep_ms(10);
    }
}


void software_set_address()
{
    unsigned int which_chain;
    for(which_chain=0; which_chain<BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        software_set_address_chain(which_chain);
    }
    cgsleep_ms(10);
}

static void get_plldata(unsigned int freq, unsigned int *vil_data)
{
    unsigned int i;
    char vildivider[32] = {0};

    for(i = 0; i < sizeof(freq_pll_map)/sizeof(freq_pll_map[0]); i++)
    {
        if(freq_pll_map[i].freq == freq)
            break;
    }

    if(i == sizeof(freq_pll_map)/sizeof(freq_pll_map[0]))
    {
        applog(LOG_WARNING,"error freq,set default freq(200M) instead");
        i = 17;
    }

    sprintf(vildivider, "%04x", freq_pll_map[i].vilpll);
    *vil_data = freq_pll_map[i].vilpll;

    applog(LOG_DEBUG, "%s: vil_data = 0x%08x", __FUNCTION__, *vil_data);
}


void set_frequency_chain(unsigned char which_chain,unsigned int frequency)
{
    unsigned int freq_data = 0;
    get_plldata(frequency, &freq_data);

    if(dev.chain_exist[which_chain] == 1)
    {
        set_config(dev.dev_fd[which_chain], 1, 0, PLL_PARAMETER, freq_data);
        dev.freq[which_chain] = frequency;
        cgsleep_us(10000);
    }

    cgsleep_ms(10);
}


void set_frequency(unsigned int frequency)
{
    unsigned char which_chain;
    applog(LOG_NOTICE, "%s: frequency = %d", __FUNCTION__, frequency);
    for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        set_frequency_chain(which_chain, frequency);
    }
    cgsleep_ms(10);
}


void set_ticket_mask_chain(unsigned char which_chain, unsigned int ticket_mask)
{
    if(dev.chain_exist[which_chain] == 1)
    {
        set_config(dev.dev_fd[which_chain], 1, 0, TICK_MASK, ticket_mask);
        cgsleep_ms(5);
    }
}

void set_ticket_mask(unsigned int ticket_mask)
{
    unsigned char which_chain;

    applog(LOG_NOTICE, "%s ticket_mask = 0x%08x",__FUNCTION__, ticket_mask);

    for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        set_ticket_mask_chain(which_chain, ticket_mask);
    }
}


int is_nonce_or_reg_value(unsigned char data)
{
    if((data & 0x80) == 0x80)
    {
        return 1;   // it is a nonce
    }
    else
    {
        return 0;   // it is a register value
    }
}

void open_core_chain(unsigned int which_chain)

{
    unsigned int reg_value = 0x0;
    unsigned which_core = 0;
    struct work_dash null_work;

    applog(LOG_NOTICE, "%s", __FUNCTION__);

    memset(&null_work, 0, sizeof(struct work_dash));
    null_work.type = WORK_INPUT_TYPE_WITHOUT_SNO;
    null_work.wc = 0x7f;
    null_work.crc16 = Swap16(CRC16(0xffff, (unsigned char *)(&null_work), WORK_INPUT_LENGTH_WITHOUT_CRC));

    if(dev.chain_exist[which_chain] == 1)
    {
        reg_value = 0;

        for(which_core=0; which_core < dev.corenum; which_core++)
        {
            reg_value = (reg_value << 1) | 0x00000001;
            set_config(dev.dev_fd[which_chain], 1, 0, CORE_ENABLE, reg_value);
            cgsleep_ms(5);
            DASH_write(dev.dev_fd[which_chain], &null_work, WORK_INPUT_LENGTH_WITH_CRC);
            cgsleep_ms(10);
            //applog(LOG_DEBUG, "%s: chain%d core%d reg_value=0x%08x", __FUNCTION__, which_chain, which_core, reg_value);
        }
    }
    applog(LOG_DEBUG, "%s end", __FUNCTION__);
}

void open_core(void)
{
    unsigned int which_chain = 0;
    for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        open_core_chain(which_chain);
    }
}

void init_asic_display_status()
{
    unsigned char which_chain, which_asic, offset;

    for(which_chain=0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        if(dev.chain_exist[which_chain])
        {
            offset = 0;
            for(which_asic = 0; which_asic < dev.chain_asic_num[which_chain]; which_asic++)
            {
                if(which_asic % 8 == 0)
                {
                    if ( ( which_asic + offset ) > ( ASIC_NUM_EACH_CHAIN + 16 ))
                    {
                        applog(LOG_ERR, "offset[%d] ERR", (which_asic + offset));
                    }
                    dev.chain_asic_status_string[which_chain][which_asic + offset] = ' ';
                    offset++;
                }

                if ( ( which_asic + offset ) > ( ASIC_NUM_EACH_CHAIN + 16 ))
                {
                    applog(LOG_ERR, "offset[%d] ERR", (which_asic + offset));
                }
                dev.chain_asic_status_string[which_chain][which_asic + offset] = 'o';
                dev.chain_asic_nonce[which_chain][which_asic] = 0;
            }

            if ( ( which_asic + offset ) > ( ASIC_NUM_EACH_CHAIN + 16 ))
            {
                applog(LOG_ERR, "offset[%d] ERR", (which_asic + offset));
            }
            dev.chain_asic_status_string[which_chain][which_asic + offset] = '\0';
        }
    }
}


void calculate_hash_rate(void)
{
    unsigned int i;
    unsigned int which_chain, which_asic;
    char displayed_avg_rate[BITMAIN_MAX_CHAIN_NUM][16];
    bool find_asic = false, avg_hash_rate = false, rt_hash_rate = false;
    uint64_t tmp_rate = 0, tmp_rt_rate_all_chain = 0;

    // check if we received all ASIC's HASH_RATE register value
    for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        if(dev.chain_exist[which_chain])
        {
            if(g_HASH_RATE_reg_value_num[which_chain] == 0)
            {
                rate_error[which_chain]++;
                if((rate_error[which_chain] > 3) || status_error)
                {
                    rate[which_chain] = 0;
                    suffix_string_DASH(rate[which_chain], (char * )displayed_rate[which_chain], sizeof(displayed_rate[which_chain]), 6, true);
                }
                continue;
            }

            if(g_HASH_RATE_reg_value_num[which_chain] < ASIC_NUM_EACH_CHAIN)
            {
                for(which_asic=0; which_asic < ASIC_NUM_EACH_CHAIN; which_asic++)
                {
                    if(!g_HASH_RATE_reg_value_from_which_asic[which_chain][i] && !status_error)
                    {
                        applog(LOG_DEBUG, "%s: Chain%d ASIC%d didn't send back HASH_RATE register value", __FUNCTION__, which_chain, which_asic);
                    }
                }

                rate_error[which_chain]++;
                if((rate_error[which_chain] > 3) || status_error)
                {
                    rate[which_chain] = 0;
                    suffix_string_DASH(rate[which_chain], (char * )displayed_rate[which_chain], sizeof(displayed_rate[which_chain]), 6, true);
                }

            }
            else
            {
                if(is_rt)
                {
                    tmp_rate = 0;
                    for(which_asic=0; which_asic < ASIC_NUM_EACH_CHAIN; which_asic++)
                    {
                        tmp_rate += g_HASH_RATE_reg_value[which_chain][which_asic] & 0x7fffffff;
                        applog(LOG_DEBUG,"%s: RT g_HASH_RATE_reg_value[%d][%d] = 0x%08x", __FUNCTION__, which_chain, which_asic, g_HASH_RATE_reg_value[which_chain][which_asic] & 0x7fffffff);
                    }
                    applog(LOG_DEBUG, "%s: chain%d RT hash rate is %0.2fGHz/s", __FUNCTION__, which_chain, (double)tmp_rate/1000000);

                    rate_error[which_chain] = 0;
                    rate[which_chain] = tmp_rate * 1000 * 1000;
                    suffix_string_DASH(rate[which_chain], (char * )displayed_rate[which_chain], sizeof(displayed_rate[which_chain]), 6, false);
                }
                else
                {
                    tmp_rate = 0;
                    for(which_asic=0; which_asic < ASIC_NUM_EACH_CHAIN; which_asic++)
                    {
                        tmp_rate += g_HASH_RATE_reg_value[which_chain][which_asic] & 0x7fffffff;
                        applog(LOG_DEBUG,"%s: avg g_HASH_RATE_reg_value[%d][%d] = 0x%08x", __FUNCTION__, which_chain, which_asic, g_HASH_RATE_reg_value[which_chain][which_asic] & 0x7fffffff);
                    }
                    applog(LOG_DEBUG, "%s: chain%d avg hash rate is %0.2fGHz/s", __FUNCTION__, which_chain, (double)tmp_rate/1000000);
                    rate_error[which_chain] = 0;
                }
            }
            tmp_rt_rate_all_chain += rate[which_chain];
        }
    }
    suffix_string_DASH(tmp_rt_rate_all_chain, (char * )displayed_hash_rate, sizeof(displayed_hash_rate), 6, false);
}


int calculate_asic_number(unsigned int actual_asic_number)
{
    int i = 0;
    if(actual_asic_number == 1)
    {
        i = 1;
    }
    else if(actual_asic_number == 2)
    {
        i = 2;
    }
    else if((actual_asic_number > 2) && (actual_asic_number <= 4))
    {
        i = 4;
    }
    else if((actual_asic_number > 4) && (actual_asic_number <= 8))
    {
        i = 8;
    }
    else if((actual_asic_number > 8) && (actual_asic_number <= 16))
    {
        i = 16;
    }
    else if((actual_asic_number > 16) && (actual_asic_number <= 32))
    {
        i = 32;
    }
    else if((actual_asic_number > 32) && (actual_asic_number <= 64))
    {
        i = 64;
    }
    else if((actual_asic_number > 64) && (actual_asic_number <= 128))
    {
        i = 128;
    }
    else
    {
        applog(LOG_DEBUG,"actual_asic_number = %d, but it is error\n", actual_asic_number);
        return -1;
    }
    return i;
}


void calculate_address_interval(void)
{
    int temp_asic_number = 0;

    // according to the max asic number of all the hash boards to calculate address interval
    //temp_asic_number = dev.max_asic_num_in_one_chain;
    temp_asic_number = calculate_asic_number(dev.max_asic_num_in_one_chain);
    //temp_asic_number = calculate_asic_number(60);
    if(temp_asic_number <= 0)
    {
        dev.addrInterval = 7;
        applog(LOG_ERR, "%s: temp_asic_number = %d, set addrInterval = '%d'", __FUNCTION__, temp_asic_number, dev.addrInterval);
        return;
    }

    dev.addrInterval = 0x100 / temp_asic_number;
    dev.addrInterval = 1;
    applog(LOG_NOTICE, "%s: temp_asic_number = %d, addrInterval = %d", __FUNCTION__, temp_asic_number, dev.addrInterval);


    // according to the address interval, calculate out the temperature sensor's address
    TempChipAddr[0] = TEMP_CHIP_0_LOCATION * dev.addrInterval;
    TempChipAddr[1] = TEMP_CHIP_1_LOCATION * dev.addrInterval;
    TempChipAddr[2] = TEMP_CHIP_2_LOCATION * dev.addrInterval;

    // according to the address interval, calculate out the chip/core offset
    //gChipOffset = (0x01 << 1) * ((0x01 << 31) / temp_asic_number / dev.addrInterval); // 2^32 / temp_asic_number / dev.addrInterval
    //gChipOffset = 0x1000000;  // 2^32 / 0x100 (0x100 = temp_asic_number * dev.addrInterval)
    gChipOffset = 0x04000000;
    gCoreOffset = gChipOffset / BM1760_CORE_NUM;
    applog(LOG_NOTICE,"%s: CHIP_OFFSET = 0x%08x, CORE_OFFSET = 0x%08x", __FUNCTION__, gChipOffset, gCoreOffset);
}
/**************** about BM1760 ASIC end ****************/

#define ABOUT_CONTROL_BOARD_HASH_BOARD
/******************** about control board & Hash Board ********************/
speed_t tiospeed_t(int baud)
{
    switch (baud)
    {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 921600:
            return B921600;
        case 3000000:
            return B3000000;
        default:
            return B0;
    }
}


int open_uart(struct bitmain_DASH_info *info, uint8_t which_chain)
{
    char dev_fname[PATH_MAX] = "";
    sprintf(dev_fname, TTY_DEVICE_TEMPLATE, tty[which_chain]);
    info->dev_fd[which_chain] = open(dev_fname, O_RDWR|O_NOCTTY);
    if(info->dev_fd[which_chain] < 0)
    {
        applog(LOG_ERR, "%s : open %s failed", __FUNCTION__, dev_fname);
    }
    return info->dev_fd[which_chain];
}

int uart_init(struct bitmain_DASH_info *info,uint8_t which_chain, int baud)
{
    int i,ret;
    speed_t speed;
    struct termios options;

    tcgetattr(info->dev_fd[which_chain], &options);
    speed = tiospeed_t(baud);
    if (speed == B0)
    {
        applog(LOG_WARNING, "Unrecognized baud rate: %d,set default baud", baud);
        speed = B115200;
    }
    cfsetispeed(&options,speed);
    cfsetospeed(&options,speed);

    options.c_cflag &= ~(CSIZE | PARENB);
    options.c_cflag |= CS8;
    options.c_cflag |= CREAD;
    options.c_cflag |= CLOCAL;

    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK |
                         ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = C_CC_VMIN;
    tcsetattr(info->dev_fd[which_chain], TCSANOW, &options);

    tcflush(info->dev_fd[which_chain], TCIOFLUSH);

}

void tty_init_chain(uint8_t which_chain, struct bitmain_DASH_info *info, int baud)
{
    int ret;

    applog(LOG_NOTICE, "%s",__FUNCTION__);

    if(info->chain_status[which_chain] == 1)
    {
        open_uart(info, which_chain);
        uart_init(info,which_chain,115200);

        if(baud != 115200)
        {
            switch(baud)
            {
                case 3000000:
                    bt8d = 0;
                    break;

                case 1500000:
                    bt8d = 1;
                    break;

                case 921600:
                    bt8d = 2;
                    break;

                case 460800:
                    bt8d = 6;
                    break;

                case 115200:
                    bt8d = 26;
                    break;

                default:
                    bt8d = 26;
                    printf("%s: the baud is not recommand value, so use 115200 baud\n", __FUNCTION__);
                    break;
            }
            unsigned int gMisc_Control_reg_value = 0x07003a01;
            gMisc_Control_reg_value = gMisc_Control_reg_value & 0xFFFFE0FF | BT8D(bt8d);
            set_config(info->dev_fd[which_chain],1, 0, MISC_CONTROL, gMisc_Control_reg_value);
            //check_asic_reg(which_chain,1,0,unsigned char MISC_CONTROL);

            close(info->dev_fd[which_chain]);

            open_uart(info, which_chain);
            uart_init(info,which_chain,baud);
        }

        dev_info[which_chain].chainid = which_chain;
        dev_info[which_chain].dev_fd = info->dev_fd[which_chain];
        applog(LOG_NOTICE, "%s chainid = %d",__FUNCTION__,dev_info[which_chain].chainid);

        start_recv[which_chain] = true;
        ret = thr_info_create(&info->uart_rx_t[which_chain], NULL, get_asic_response, (void *)&dev_info[which_chain]);
        if(unlikely(ret != 0))
        {
            applog(LOG_ERR,"create rx read thread for chain %d failed", which_chain);
        }
        else
        {
            applog(LOG_ERR,"create rx read thread for chain %d ok", which_chain);
        }

        cgsleep_ms(50);
        struct bitmain_DASH_info_with_index info_with_index;
        info_with_index.info = info;
        info_with_index.chain_index = which_chain;
        ret = thr_info_create(&info->uart_tx_t[which_chain], NULL, DASH_fill_work, (void *)(&info_with_index));
        cgsleep_ms(200);
        if(unlikely(ret != 0))
        {
            applog(LOG_ERR,"create tx read thread for chain %d failed",which_chain);
        }
        else
        {
            applog(LOG_ERR,"create tx read thread for chain %d ok",which_chain);
        }
    }

    applog(LOG_NOTICE,"open device over");


    dev.dev_fd[which_chain] = info->dev_fd[which_chain];

    cgsleep_ms(10);
}


void tty_init(struct bitmain_DASH_info *info, int baud)
{
    uint8_t which_chain = 0,ret;

    applog(LOG_NOTICE, "%s",__FUNCTION__);

    for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        tty_init_chain(which_chain, info, baud);
    }
    cgsleep_ms(10);
}

int DASH_write(int fd, const void *buf, size_t bufLen)
{
    size_t ret = 0;
    unsigned char header[ASIC_INPUT_HEADER_LENGTH] = {INPUT_HEADER_1, INPUT_HEADER_2};

    if (unlikely(fd == -1))
        ret = -1;

    flock(fd,LOCK_EX);
    cgsleep_ms(1);
    ret = write(fd, header, ASIC_INPUT_HEADER_LENGTH);
    ret += write(fd, buf, bufLen);

    if(unlikely(ret != (bufLen + ASIC_INPUT_HEADER_LENGTH)))
    {
        applog(LOG_ERR,"write error!!");
    }
    flock(fd,LOCK_UN);
    cgsleep_us(500);
    return ret;
}


int DASH_read(int uart_fd, unsigned char *buf, size_t MAX_READ_BYTES)
{
    int fs_sel, nbytes=0, ret = 0;
    ssize_t len=0;

    if (unlikely(uart_fd == -1))
    {
        applog(LOG_ERR, "%s: fd = %d", __FUNCTION__, uart_fd);
        return 0;
    }

    if(ioctl(uart_fd, FIONREAD, &nbytes) == 0)
    {
        if(nbytes < MAX_READ_BYTES)
        {
            len = read(uart_fd, buf, nbytes);
        }
        else
        {
            len = read(uart_fd, buf, MAX_READ_BYTES);
        }
    }
    else
    {
        applog(LOG_ERR, "%s: ioctl error", __FUNCTION__);
        len = 0;
    }
    return len;
}


void i2c_init(struct bitmain_DASH_info *info)
{
    int which_chain;

    if(unlikely((info->i2c_fd = open(IIC_DEVIVEE, O_RDWR | O_NONBLOCK)) < 0))
    {
        perror(IIC_DEVIVEE);
        return;
    }

    for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        if(info->chain_status[which_chain])
        {
            if(unlikely(ioctl(info->i2c_fd, I2C_SLAVE, i2c_slave_addr[which_chain] >> 1 ) < 0))
            {
                perror("set i2c slave addr error");
            }
        }
    }

    dev.i2c_fd = info->i2c_fd;
    applog(LOG_NOTICE,"i2c init ok");
    cgsleep_ms(10);
}

void check_chain(struct bitmain_DASH_info *info)
{
    int i, fd, ret;
    char dev_fname[PATH_MAX], command[2];

    for(i = 0; i < sizeof(plug)/sizeof(int); i++)
    {
        sprintf(dev_fname, GPIO_DEVICE_TEMPLATE, plug[i]);

        if((fd = open(dev_fname, O_RDONLY)) < 0)
        {
            applog(LOG_ERR,"%s :open %s failed",__FUNCTION__,dev_fname);
            continue;
        }

        if(lseek(fd, 0, SEEK_SET) < 0)
        {
            perror(dev_fname);
            applog(LOG_ERR,"%s :lseek %s failed",__FUNCTION__,dev_fname);
            continue;
        }

        ret = read(fd, command, 2);

        if(ret > 0 && command[0] == '1')
        {
            info->chain_num ++;
            info->chain_status[i] = 1;
            dev.chain_exist[i] = 1;
            dev.chain_num++;
            applog(LOG_NOTICE,"detected at %s  chain %d",dev_fname,i);
        }
        else
        {
            info->chain_status[i] = 0;
            dev.chain_exist[i] = 0;
        }
    }

    applog(LOG_NOTICE,"detect total chain num %d",info->chain_num);

    if(info->chain_num == 0)
    {
        set_PWM(0);

        while(1)
        {
            applog(LOG_ERR,"No Hash Board\n");
            sleep(3);
        }
    }
    cgsleep_ms(10);
}

static void reset_all_hash_board(void)
{
    char rstBuf[128] = "";
    unsigned char which_chain = 0;

    applog(LOG_NOTICE, "%s", __FUNCTION__);

    for (which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++ )
    {
        sprintf(rstBuf, SET_ASIC_RST_0, g_gpio_data[which_chain]);
        system(rstBuf);
    }

    cgsleep_ms(500);

    for (which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        sprintf(rstBuf, SET_ASIC_RST_1, g_gpio_data[which_chain]);
        system(rstBuf);
    }

    cgsleep_ms(200);
}

static void reset_hash_board(unsigned char which_chain)
{
    char rstBuf[128] = "";

    applog(LOG_DEBUG,"%s: which_chain = %d", __FUNCTION__, which_chain);

    sprintf(rstBuf, SET_ASIC_RST_0, g_gpio_data[which_chain]);
    system(rstBuf);

    cgsleep_ms(500);

    sprintf(rstBuf, SET_ASIC_RST_1, g_gpio_data[which_chain]);
    system(rstBuf);
}

void set_led(bool stop)
{
    static bool blink = true;
    char cmd[100];
    blink = !blink;

    if(stop)
    {
        sprintf(cmd,LED_CTRL_TEMPLATE,0,green_led);
        system(cmd);
        sprintf(cmd,LED_CTRL_TEMPLATE,(blink)?1:0,red_led);
        system(cmd);
    }
    else
    {
        sprintf(cmd,LED_CTRL_TEMPLATE,0,red_led);
        system(cmd);
        sprintf(cmd,LED_CTRL_TEMPLATE,(blink)?1:0,green_led);
        system(cmd);
    }
}

void set_beep(bool flag)
{
    char cmd[128] = {0};

    sprintf(cmd,BEEP_CTRL_TEMPLATE,flag?1:0,beep);
    system(cmd);
}

static unsigned int getNum(const char* buffer)
{
    char* pos = strstr(buffer, ":");

    while(*(++pos) == ' ');
    char* startPos = pos;

    while(*(++pos) != ' ');
    *pos = '\0';

    return (atoi(startPos));
}

void check_fan_speed(void)
{

    uint32_t fan0SpeedHist = 0;
    uint32_t fan1SpeedHist = 0;
    uint32_t fan0SpeedCur = 0;
    uint32_t fan1SpeedCur = 0;
    uint32_t fan0Speed = 0, fan0Speed_ok = 0;
    uint32_t fan1Speed = 0, fan1Speed_ok = 0;
    uint32_t fan0_exist = 0;
    uint32_t fan1_exist = 0;
    unsigned char ok_counter = 0;
    char buffer[256] = "";
    char* pos = NULL;
    FILE* fanpfd = fopen(PROCFILENAME, "r");

    if(fanpfd == NULL)
    {
        while(1)
        {
            applog(LOG_ERR, "open /proc/interrupts error");
            sleep(3);
        }
    }

    fseek(fanpfd, 0, SEEK_SET);

    while(fgets(buffer, 256, fanpfd))
    {
        if ( ((pos = strstr(buffer, FAN0)) != 0) && (strstr(buffer, "gpiolib") != 0 ) )
        {
            applog(LOG_DEBUG, "find fan1.");
            fan0SpeedHist = fan0SpeedCur = getNum(buffer);
        }

        if (((pos = strstr(buffer, FAN1)) != 0) && (strstr(buffer, "gpiolib") != 0 ))
        {
            applog(LOG_DEBUG, "find fan2.");
            fan1SpeedHist = fan1SpeedCur = getNum(buffer);
        }
    }

    cgsleep_ms(500);

    while(1)
    {
        applog(LOG_DEBUG, "test_loop = %d", ok_counter);

        fseek(fanpfd, 0, SEEK_SET);

        while(fgets(buffer, 256, fanpfd))
        {
            if ( ((pos = strstr(buffer, FAN0)) != 0) && (strstr(buffer, "gpiolib") != 0 ) )
            {
                applog(LOG_DEBUG, "find fan1");
                fan0SpeedCur = getNum(buffer);
                if (fan0SpeedHist > fan0SpeedCur)
                {
                    fan0Speed = (0xffffffff - fan0SpeedHist + fan0SpeedCur);
                }
                else
                {
                    fan0Speed = ( fan0SpeedCur - fan0SpeedHist );
                }
                fan0Speed = fan0Speed * 60 / 2 * 2; // 60: 1 minute; 2: 1 interrupt has 2 edges; FANINT: check fan speed every FANINT senconds
                fan0SpeedHist = fan0SpeedCur;
                applog(LOG_DEBUG, "fan1Speed = %d", fan0Speed);
                if( fan0Speed > FAN1_MAX_SPEED * FAN_SPEED_OK_PERCENT)
                {
                    fan0Speed_ok++;
                }
            }

            if (((pos = strstr(buffer, FAN1)) != 0) && (strstr(buffer, "gpiolib") != 0 ))
            {
                applog(LOG_DEBUG, "find fan2");
                fan1SpeedCur = getNum(buffer);
                if (fan1SpeedHist > fan1SpeedCur)
                {
                    fan1Speed = (0xffffffff - fan1SpeedHist + fan1SpeedCur);
                }
                else
                {
                    fan1Speed = ( fan1SpeedCur - fan1SpeedHist );
                }
                fan1SpeedHist = fan1SpeedCur;
                fan1Speed = fan1Speed * 60 / 2 * 2; // 60: 1 minute; 2: 1 interrupt has 2 edges; FANINT: check fan speed every FANINT senconds
                fan1SpeedHist = fan1SpeedCur;
                applog(LOG_DEBUG, "fan2Speed = %d", fan1Speed);
                if( fan1Speed > FAN2_MAX_SPEED * FAN_SPEED_OK_PERCENT)
                {
                    fan1Speed_ok++;
                }
            }
        }

        if((fan0Speed_ok >= 3) && (fan1Speed_ok >= 3))
        {
            applog(LOG_WARNING, "%s OK", __FUNCTION__);
            return;
        }
        else
        {
            if(fan0Speed_ok < 3)
            {
                applog(LOG_ERR, "FAN1 too slow ...");   // FAN1 is fan0
            }

            if(fan1Speed_ok < 3)
            {
                applog(LOG_ERR, "FAN2 too slow ...");   // FAN2 is fan1
            }
        }

        cgsleep_ms(500);
    }
}

void set_PWM(unsigned char pwm_percent)
{
    int temp_pwm_percent = 0;
    char buf[128];

    temp_pwm_percent = pwm_percent;
    if(temp_pwm_percent < MIN_PWM_PERCENT)
    {
        temp_pwm_percent = MIN_PWM_PERCENT;
    }

    if(temp_pwm_percent > MAX_PWM_PERCENT || gMinerStatus_Not_read_all_sensor)
    {
        temp_pwm_percent = MAX_PWM_PERCENT;
    }

    dev.duty_ns = PWM_PERIOD_NS * temp_pwm_percent /100;
    dev.pwm_percent = temp_pwm_percent;

    //applog(LOG_DEBUG,"set pwm duty_ns %d",dev.duty_ns);
    sprintf(buf,PWM_CTRL_TEMPLATE,dev.duty_ns);
    system(buf);
}

void set_PWM_according_to_temperature(void)
{
    int pwm_percent = 0, temp_change = 0;

    temp_highest = dev.temp_top1;
    if(temp_highest >= MAX_FAN_TEMP)
    {
        //applog(LOG_DEBUG,"%s: Temperature is higher than %d 'C", __FUNCTION__, temp_highest);
    }

    if(dev.fan_eft)
    {
        if((dev.fan_pwm >= 0) && (dev.fan_pwm <= 100))
        {
            set_PWM(dev.fan_pwm);
            return;
        }
    }

    temp_change = temp_highest - last_temperature;

    if(temp_highest >= MAX_FAN_TEMP || temp_highest == 0)
    {
        set_PWM(MAX_PWM_PERCENT);
        dev.fan_pwm = MAX_PWM_PERCENT;
        //applog(LOG_DEBUG,"%s: Set PWM percent : MAX_PWM_PERCENT", __FUNCTION__);
        return;
    }

    if(temp_highest <= MIN_FAN_TEMP)
    {
        set_PWM(MIN_PWM_PERCENT);
        dev.fan_pwm = MIN_PWM_PERCENT;
        //applog(LOG_DEBUG,"%s: Set PWM percent : MIN_PWM_PERCENT", __FUNCTION__);
        return;
    }

    if(temp_change >= TEMP_INTERVAL || temp_change <= -TEMP_INTERVAL)
    {
        pwm_percent = MIN_PWM_PERCENT + (temp_highest -MIN_FAN_TEMP) * PWM_ADJUST_FACTOR;
        if(pwm_percent < 0)
        {
            pwm_percent = 0;
        }
        dev.fan_pwm = pwm_percent;
        applog(LOG_DEBUG,"%s: Set PWM percent : %d", __FUNCTION__, pwm_percent);
        set_PWM(pwm_percent);
        last_temperature = temp_highest;
    }
}

void recheck_asic_num()
{
    every_chain_disable_PIC16F1704_dc_dc_new();
    sleep(1);
    every_chain_enable_PIC16F1704_dc_dc_new();
    reset_all_hash_board();
    clear_register_value_buf();
    cgsleep_ms(100);
    check_every_chain_asic_number(true);
    applog(LOG_NOTICE,"%s DONE!", __FUNCTION__);
}
int bitmain_DASH_init(struct bitmain_DASH_info *info)
{
    struct init_config config = info->DASH_config;
    struct init_config config_parameter;
    uint16_t crc = 0, freq = 0;
    unsigned char which_chain = 0;
    int i = 0,check_asic_times = 0, ret = 0;
    bool check_asic_fail = false;

    applog(LOG_WARNING, "%s", __FUNCTION__);

    memcpy(&config_parameter, &config, sizeof(struct init_config));

    sprintf(g_miner_version, "1.0.0.9");
    dev.addrInterval = 1;
    // start fans

    set_PWM(100);
    check_fan_speed();

    // check parameters
    if(config_parameter.token_type != INIT_CONFIG_TYPE)
    {
        applog(LOG_ERR,"%s: config_parameter.token_type != 0x%x, it is 0x%x", __FUNCTION__, INIT_CONFIG_TYPE, config_parameter.token_type);
        return -1;
    }

    crc = crc_itu_t(0xff, (uint8_t*)(&config_parameter), sizeof(struct init_config) - sizeof(uint16_t));
    if(crc != config_parameter.crc)
    {
        applog(LOG_ERR,"%s: config_parameter.crc = 0x%x, but we calculate it as 0x%x", __FUNCTION__, config_parameter.crc, crc);
        return -2;
    }

    // init the work queue which stores the latest work that sent to hash boards
    for(i=0; i < BITMAIN_MAX_QUEUE_NUM; i++)
    {
        info->work_queue[i] = NULL;
    }

    /*
    dev = calloc(sizeof(struct all_parameters), sizeof(char));
    if(unlikely(!dev))
    {
        applog(LOG_ERR,"calloc dev failed");
    }
    */

    // check chain
    check_chain(info);

    // init i2c
    i2c_init(info);
    check_whether_need_update_pic_program();
    every_chain_reset_PIC16F1704_pic_new();
    every_chain_jump_from_loader_to_app_PIC16F1704_new();

    ret = send_heart_beat_to_every_chain();
    if(ret == -3)
    {
        return ret;
    }
    every_chain_enable_PIC16F1704_dc_dc_new();
    reset_all_hash_board();
    tty_init(info, config_parameter.baud);
    cgsleep_ms(100);
    ret = creat_bitmain_scanreg_pthread();
    cgsleep_ms(100);
    clear_register_value_buf();

    if(ret == -4)
    {
        return ret;
    }
    // check ASIC number for every chain
    check_every_chain_asic_number(true);
    cgsleep_ms(300);
    for(check_asic_times = 0; check_asic_times < 3; check_asic_times++)
    {
        need_recheck = false;
        for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
        {
            if(dev.chain_exist[i] == 1)
            {
                if(dev.chain_asic_num[i] != ASIC_NUM_EACH_CHAIN)
                {
                    need_recheck = true;
                    break;
                }
            }
        }
        if(need_recheck)
            recheck_asic_num();
        else
            break;
    }
    calculate_address_interval();
    //cgsleep_ms(1000);
    //update_asic_num = false;

    // set core ticket mask
    set_ticket_mask(DEVICE_DIFF_SET);   // if we set this register value as 0x16, we will not need to set

    //set core number
    dev.corenum = BM1760_CORE_NUM;

    // set frequency
    if(config_parameter.frequency_eft)
    {
        dev.frequency = config_parameter.frequency;
        if(dev.frequency == 0)
        {
            every_chain_get_PIC16F1704_freq_new(&freq);
            dev.frequency = freq;
            set_frequency(dev.frequency);
        }
        else
        {
            set_frequency(dev.frequency);
        }
        sprintf(dev.frequency_t,"%u",dev.frequency);
    }

    // set every chain's ASIC address
    software_set_address();
    //check_every_chain_asic_number(false);

    // about temperature sensor
    enable_read_temperature_from_asic(config_parameter.misc_control_reg_value);
    select_core_to_check_temperature(DIODE_MUX_SEL_DEFAULT_VALUE, VDD_MUX_SEL_DEFAULT_VALUE);
    //check_sensor_ID();
    calibration_sensor_offset();
    set_temperature_offset_value();

    // check who control fan
    dev.fan_eft = config_parameter.fan_eft;
    dev.fan_pwm = config_parameter.fan_pwm_percent;
    applog(LOG_DEBUG,"%s: fan_eft : %d  fan_pwm : %d", __FUNCTION__, dev.fan_eft, dev.fan_pwm);
    if(config_parameter.fan_eft)
    {
        if((config_parameter.fan_pwm_percent >= 0) && (config_parameter.fan_pwm_percent <= 100))
        {
            set_PWM(config_parameter.fan_pwm_percent);
        }
        else
        {
            set_PWM_according_to_temperature();
        }
    }
    else
    {
        set_PWM_according_to_temperature();
    }

    // calculate real timeout
    if(config_parameter.timeout_eft)
    {
        if(config_parameter.timeout_data_integer == 0 && config_parameter.timeout_data_fractions == 0)   //driver calculate out timeout value
        {
            applog(LOG_NOTICE, "frequency = '%d'", dev.frequency);
            // timeout = 2^32 / (256 / AddrInterval) / Freq / core number
            dev.timeout = 0xffffffff / (0x100 / (dev.addrInterval * 4)) / dev.frequency / dev.corenum * 0.90 * 40;
            applog(LOG_NOTICE,"dev.timeout = %d us", dev.timeout);
        }
        else
        {
            dev.timeout = config_parameter.timeout_data_integer * 1000 + config_parameter.timeout_data_fractions;
        }
    }


    open_core();

    // create some pthread
    ret = create_bitmain_check_fan_pthread();
    if(ret == -8)
    {
        return ret;
    }
    sleep(FANINT);
    ret = create_bitmain_read_temp_pthread();
    if(ret == -7)
    {
        return ret;
    }

    ret = create_bitmain_check_miner_status_pthread(info);
    if(ret == -5)
    {
        return ret;
    }

    ret = create_bitmain_get_hash_rate_pthread();
    if(ret == -6)
    {
        return ret;
    }
    // init ASIC status which will be display on the web
    init_asic_display_status();
    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
        start_send[i] = true;

    return 0;
}


void* bitmain_DASH_reinit_chain(void * usrdata )
{
    pthread_detach(pthread_self());
    pthread_mutex_lock(&reinit_mutex);
    struct bitmain_DASH_info_with_index *info_with_chain = (struct bitmain_DASH_info_with_index *)usrdata;
    struct bitmain_DASH_info *info = info_with_chain->info;
    uint8_t chain = info_with_chain->chain_index;

    pthread_mutex_lock(&iic_mutex);
    if(unlikely(ioctl(dev.i2c_fd,I2C_SLAVE,i2c_slave_addr[chain] >> 1 ) < 0))
        applog(LOG_ERR,"ioctl error @ line %d",__LINE__);
    disable_PIC16F1704_dc_dc_new();
    pthread_mutex_unlock(&iic_mutex);

    start_send[chain] = false;
    start_recv[chain] = false;
    reiniting[chain] = true;
    cgsleep_ms(10);
    thr_info_join(&info->uart_tx_t[chain]);
    thr_info_join(&info->uart_rx_t[chain]);
    sleep(1);

    pthread_mutex_lock(&iic_mutex);
    if(unlikely(ioctl(dev.i2c_fd,I2C_SLAVE,i2c_slave_addr[chain] >> 1 ) < 0))
        applog(LOG_ERR,"ioctl error @ line %d",__LINE__);
    enable_PIC16F1704_dc_dc_new();
    pthread_mutex_unlock(&iic_mutex);

    sleep(1);
    reset_hash_board(chain);
    flock(dev.dev_fd[chain],LOCK_EX);
    close(dev.dev_fd[chain]);
    flock(dev.dev_fd[chain],LOCK_UN);
    reiniting[chain] = false;
    tty_init_chain(chain,info, BITMAIN_DEFAULT_BAUD);
    software_set_address_chain(chain);
    set_ticket_mask_chain(chain,DEVICE_DIFF_SET);
    enable_read_temperature_from_asic_chain(chain,MISC_CONTROL_DEFAULT_VALUE);
    select_core_to_check_temperature_chain(chain,DIODE_MUX_SEL_DEFAULT_VALUE, VDD_MUX_SEL_DEFAULT_VALUE);
    set_temperature_offset_value_chain(chain);
    set_frequency_chain(chain, dev.frequency);
    open_core_chain(chain);
    start_send[chain] = true;
    pthread_mutex_unlock(&reinit_mutex);
}


int bitmain_DASH_reinit(struct bitmain_DASH_info *info)
{
    struct init_config config = info->DASH_config;
    struct init_config config_parameter;
    uint16_t crc = 0;
    unsigned char which_chain = 0;
    int i = 0,check_asic_times = 0, ret = 0;
    bool check_asic_fail = false;

    applog(LOG_WARNING, "%s", __FUNCTION__);

    // start fans
    set_PWM(100);
    check_fan_speed();

    // init the work queue which stores the latest work that sent to hash boards
    for(i=0; i < BITMAIN_MAX_QUEUE_NUM; i++)
    {
        info->work_queue[i] = NULL;
    }

    every_chain_enable_PIC16F1704_dc_dc_new();
    cgsleep_ms(100);

    reset_all_hash_board();

    clear_register_value_buf();

    cgsleep_ms(100);

    // set frequency
    if(config_parameter.frequency_eft)
    {
        dev.frequency = config_parameter.frequency;
        set_frequency(dev.frequency);
        sprintf(dev.frequency_t,"%u",dev.frequency);
    }

    // set every chain's ASIC address
    software_set_address();
    //check_every_chain_asic_number(false);

    // about temperature sensor
    enable_read_temperature_from_asic(config_parameter.misc_control_reg_value);
    select_core_to_check_temperature(DIODE_MUX_SEL_DEFAULT_VALUE, VDD_MUX_SEL_DEFAULT_VALUE);
    calibration_sensor_offset();
    set_temperature_offset_value();

    //set baud
    //set_misc_ctrl();      // dash use 115200 baud, so no need to set baud

    //open core
    open_core();

    // init ASIC status which will be display on the web
    init_asic_display_status();

    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
        start_send[i] = true;

    return 0;
}

/****************** about control board & Hash Board end ******************/

#define TEMPERATURE_SENSOR
/****************** about temperature sensor ******************/


void enable_read_temperature_from_asic_chain(unsigned int which_chain, unsigned int misc_control_reg_value)
{
    unsigned int which_sensor=0, reg_value = 0;

    reg_value = ((misc_control_reg_value | RFS | TFS(3)) & 0xFFFFE0FF) | BT8D(bt8d);

    applog(LOG_NOTICE, "%s: reg_value = 0x%08x", __FUNCTION__, reg_value);


    if ( dev.chain_exist[which_chain] == 1)
    {
        for (which_sensor = 0 ; which_sensor < BITMAIN_REAL_TEMP_CHIP_NUM; which_sensor++)
        {
            set_config(dev.dev_fd[which_chain], 0, TempChipAddr[which_sensor], MISC_CONTROL, reg_value);
            cgsleep_ms(2);
        }
    }

}

void enable_read_temperature_from_asic(unsigned int misc_control_reg_value)
{
    unsigned int which_chain=0;

    for (which_chain=0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        enable_read_temperature_from_asic_chain(which_chain, misc_control_reg_value);
    }
}

void select_core_to_check_temperature_chain(unsigned int which_chain, unsigned char diode_mux_sel, unsigned char vdd_mux_sel)
{
    unsigned int which_sensor=0;
    unsigned int regdata = ((diode_mux_sel & 0x0f) << 16) | (vdd_mux_sel & 0x1f);

    applog(LOG_NOTICE, "%s: diode_mux_sel = %d, vdd_mux_sel = %d", __FUNCTION__, diode_mux_sel, vdd_mux_sel);


    if ( dev.chain_exist[which_chain] == 1)
    {
        for (which_sensor = 0 ; which_sensor < BITMAIN_REAL_TEMP_CHIP_NUM; which_sensor++)
        {
            set_config(dev.dev_fd[which_chain], 0, TempChipAddr[which_sensor], ANALOG_MUX_CONTROL, regdata);
            cgsleep_ms(2);
        }
    }

}

void select_core_to_check_temperature(unsigned char diode_mux_sel, unsigned char vdd_mux_sel)
{
    unsigned int which_chain=0;

    applog(LOG_NOTICE, "%s: diode_mux_sel = %d, vdd_mux_sel = %d", __FUNCTION__, diode_mux_sel, vdd_mux_sel);

    for (which_chain=0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        select_core_to_check_temperature_chain(which_chain, diode_mux_sel, vdd_mux_sel);
    }
}

void read_i2c_reg(unsigned char which_chain, unsigned char which_sensor, unsigned char reg_type,unsigned int reg_data)
{
    int read_temperature_time;
    cgsleep_ms(50);
    set_config(dev.dev_fd[which_chain], 0, TempChipAddr[which_sensor], GENERAL_I2C_COMMAND, reg_data);
    g_chip_temp_return[which_chain][which_sensor][reg_type] = false;
    read_temperature_time = 0;
    do
    {
        cgsleep_ms(50);
        check_asic_reg(which_chain, 0, TempChipAddr[which_sensor], GENERAL_I2C_COMMAND);
        read_temperature_time++;
    }
    while((!g_chip_temp_return[which_chain][which_sensor][reg_type]) && (read_temperature_time < READ_LOOP));

}
void calibration_sensor_offset_chain(unsigned char which_chain)
{
    unsigned char which_sensor, cali_times = 0;
    applog(LOG_NOTICE, "%d %s", which_chain, __FUNCTION__);

    if ( dev.chain_exist[which_chain] == 1 )
    {
        for (which_sensor = 0 ; which_sensor < BITMAIN_REAL_TEMP_CHIP_NUM; which_sensor++ )
        {
        	cali_times = 0;
            do
            {
                read_i2c_reg(which_chain,which_sensor,REMOTE,REGADDRVALID | DEVICEADDR(TMP451_IIC_SALVE_ADDR) | REGADDR(EXT_TEMP_VALUE_HIGH_BYTE) | DATA(0) & (~RW));
                read_i2c_reg(which_chain,which_sensor,LOCAL,REGADDRVALID | DEVICEADDR(TMP451_IIC_SALVE_ADDR) | REGADDR(LOCAL_TEMP_VALUE) | DATA(0) & (~RW));
                // record temperature offset value
                applog(LOG_NOTICE,"chain:%d local:%d remote:%d",which_chain, dev.chain_asic_temp[which_chain][which_sensor][LOCAL], dev.chain_asic_temp[which_chain][which_sensor][REMOTE]);
                gTempOffsetValue[which_chain][which_sensor] += dev.chain_asic_temp[which_chain][which_sensor][LOCAL] - dev.chain_asic_temp[which_chain][which_sensor][REMOTE];
                set_temperature_offset_value_chain_sensor(which_chain, which_sensor);
			}
            while((abs(dev.chain_asic_temp[which_chain][which_sensor][LOCAL] - dev.chain_asic_temp[which_chain][which_sensor][REMOTE]) > 2) && (cali_times++ < 10));
        }
    }
}

void calibration_sensor_offset(void)
{
    unsigned char which_chain;
    //applog(LOG_NOTICE, "%s", __FUNCTION__);
    for ( which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++ )
    {
        if ( dev.chain_exist[which_chain] == 1 )
        {
            calibration_sensor_offset_chain(which_chain);
        }
    }
}

void set_temperature_offset_value_chain_sensor(unsigned char which_chain, unsigned char which_sensor)
{
    unsigned char read_temperature_time;
    signed char local_temp=0, remote_temp=0, offset=0;
    unsigned int reg_data = 0, ret = 0;
    bool not_read_out_temperature = false;

    applog(LOG_WARNING, "Chain %d %s", which_chain, __FUNCTION__);


    // write the offset value into GENERAL_I2C_COMMAND register to send to temperature sensor
    reg_data = 0x000000ff & DATA(gTempOffsetValue[which_chain][which_sensor]);
    reg_data |= REGADDRVALID | DEVICEADDR(TMP451_IIC_SALVE_ADDR) | REGADDR(EXT_TEMP_OFFSET_HIGH_BYTE) | RW;
    read_i2c_reg(which_chain,which_sensor,OFFSET,reg_data);

    reg_data = REGADDRVALID | DEVICEADDR(TMP451_IIC_SALVE_ADDR) | REGADDR(EXT_TEMP_OFFSET_HIGH_BYTE) | DATA(0) & (~RW);
    read_i2c_reg(which_chain,which_sensor,OFFSET,reg_data);

    if(send_back_gTempOffsetValue[which_chain][which_sensor] != gTempOffsetValue[which_chain][which_sensor])
    {
        applog(LOG_ERR, "%s: Chain%d Sensor%d temp offset value set error. It should be %02d, but read back is %02d\n",
               __FUNCTION__, which_chain, which_sensor, gTempOffsetValue[which_chain][which_sensor], send_back_gTempOffsetValue[which_chain][which_sensor]);
    }
    else
    {
        applog(LOG_NOTICE, "%s: Chain%d Sensor%d temp offset : %02d, ",__FUNCTION__, which_chain, which_sensor, send_back_gTempOffsetValue[which_chain][which_sensor]);
    }
}


void set_temperature_offset_value_chain(unsigned char which_chain)
{
    unsigned char which_sensor, read_temperature_time;
    signed char local_temp=0, remote_temp=0, offset=0;
    unsigned int reg_data = 0, ret = 0;
    bool not_read_out_temperature = false;

    applog(LOG_WARNING, "Chain %d %s", which_chain, __FUNCTION__);

    if ( dev.chain_exist[which_chain] == 1 )
    {
        for (which_sensor = 0 ; which_sensor < BITMAIN_REAL_TEMP_CHIP_NUM; which_sensor++ )
        {
            // write the offset value into GENERAL_I2C_COMMAND register to send to temperature sensor
            set_temperature_offset_value_chain_sensor(which_chain, which_sensor);
        }
    }
}


void set_temperature_offset_value(void)
{
    unsigned char which_chain;

    for ( which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++ )
    {
        if ( dev.chain_exist[which_chain] == 1 )
        {
            set_temperature_offset_value_chain(which_chain);
        }
    }
}

void check_sensor_ID(void)
{
    unsigned char which_chain, which_sensor, read_temperature_time;
    signed char manufacturer_id=0;
    unsigned int ret = 0;
    bool not_read_out_temperature = false;
    applog(LOG_DEBUG, "%s", __FUNCTION__);
    for ( which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++ )
    {
        if ( dev.chain_exist[which_chain] == 1 )
        {
            for (which_sensor = 0 ; which_sensor < BITMAIN_REAL_TEMP_CHIP_NUM; which_sensor++ )
            {
                read_i2c_reg(which_chain,which_sensor,ID, REGADDRVALID | DEVICEADDR(TMP451_IIC_SALVE_ADDR) | REGADDR(MANUFACTURER_ID) | DATA(0) & (~RW));
                manufacturer_id = sensor_id[which_chain][which_sensor];
                applog(LOG_ERR, "%s: Chain%d Sensor%d Manufacturer ID = 0x%02x", __FUNCTION__, which_chain, which_sensor, manufacturer_id);
            }
        }
    }
}


/**************** about temperature sensor end ****************/


#define ABOUT_OTHER_FUNCTIONS
/****************** about other functions ******************/

void suffix_string_DASH(uint64_t val, char *buf, size_t bufsiz, int sigdigits,bool display)
{
    const double  dkilo = 1000.0;
    const uint64_t kilo = 1000ull;
    const uint64_t mega = 1000000ull;
    const uint64_t giga = 1000000000ull;
    char suffix[2] = "";
    bool decimal = true;
    double dval;
    /*
        if (val >= exa)
        {
            val /= peta;
            dval = (double)val / dkilo;
            strcpy(suffix, "E");
        }
        else if (val >= peta)
        {
            val /= tera;
            dval = (double)val / dkilo;
            strcpy(suffix, "P");
        }
        else if (val >= tera)
        {
            val /= giga;
            dval = (double)val / dkilo;
            strcpy(suffix, "T");
        }
        else */
    if (val >= giga)
    {
        val /= mega;
        dval = (double)val / dkilo;
        strcpy(suffix, "G");
    }
    else if (val >= mega)
    {
        val /= kilo;
        dval = (double)val / dkilo;
        strcpy(suffix, "M");
    }
    else if (val >= kilo)
    {
        dval = (double)val / dkilo;
        strcpy(suffix, "K");
    }
    else
    {
        dval = val;
        decimal = false;
    }

    if (!sigdigits)
    {
        if (decimal)
            snprintf(buf, bufsiz, "%.3g%s", dval, suffix);
        else
            snprintf(buf, bufsiz, "%d%s", (unsigned int)dval, suffix);
    }
    else
    {
        /* Always show sigdigits + 1, padded on right with zeroes
         * followed by suffix */
        int ndigits = sigdigits - 1 - (dval > 0.0 ? floor(log10(dval)) : 0);
        if(display)
            snprintf(buf, bufsiz, "%*.*f%s", sigdigits + 1, ndigits, dval, suffix);
        else
            snprintf(buf, bufsiz, "%*.*f", sigdigits + 1, ndigits, dval);

    }
}

void clear_register_value_buf(void)
{
    pthread_mutex_lock(&reg_mutex);
    reg_fifo.p_wr = 0;
    reg_fifo.p_rd = 0;
    reg_fifo.reg_value_num = 0;
    pthread_mutex_unlock(&reg_mutex);
}

void clear_nonce_fifo()
{
    int i = 0;
    pthread_mutex_lock(&nonce_mutex);
    nonce_fifo.p_wr = 0;
    nonce_fifo.p_rd = 0;
    nonce_fifo.nonce_num = 0;
    for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
        tcflush(dev.dev_fd[i], TCIOFLUSH);
    pthread_mutex_unlock(&nonce_mutex);
}

static inline void my_be32enc(void *pp, uint32_t x)
{
    uint8_t *p = (uint8_t *)pp;
    p[3] = x & 0xff;
    p[2] = (x >> 8) & 0xff;
    p[1] = (x >> 16) & 0xff;
    p[0] = (x >> 24) & 0xff;
}

/**************** about other functions end ****************/


#define ABOUT_CGMINER_PTHREAD
/******************** about cgminer pthread ********************/

void *bitmain_scanhash(void *arg)
{
    struct thr_info *thr = (struct thr_info*)arg;
    struct cgpu_info *bitmain_DASH = thr->cgpu;
    struct bitmain_DASH_info *info = bitmain_DASH->device_data;
    struct timeval current;
    uint8_t nonce_bin[4],crc_check,which_asic_nonce;
    uint32_t nonce, i,k, *work_nonce=NULL;
    int submitfull = 0;
    bool submitnonceok = true;
    unsigned char work_id = 0, chain_id = 0, nonce_diff = 0, nonce_crc5 = 0;
    unsigned char pworkdata[128]= {0},  hash1[32]= {0};
    unsigned int endiandata[32]= {0};
    unsigned char *ob_hex=NULL;

    unsigned char buf[80] =
    {
        0x00,0x00,0x00,0x20,0xD8,0xC4,0x6A,0xAE,
        0x69,0x2C,0x0D,0xA5,0x5F,0xEA,0xB7,0x74,
        0x15,0x67,0xD6,0x4E,0x42,0x8E,0xBB,0x90,
        0x4B,0x68,0x49,0x5D,0xD5,0x05,0x00,0x00,
        0x00,0x00,0x00,0x00,0x2E,0x29,0x56,0x66,
        0x91,0x76,0xDD,0xE1,0x8D,0x08,0x9E,0x89,
        0x8B,0x76,0xF8,0x35,0xB4,0xC1,0xB4,0xF6,
        0x28,0xDB,0xA4,0x2F,0x35,0x2F,0xA9,0xA8,
        0xF1,0xAE,0x3E,0xD9,0x70,0x23,0x5A,0x59,
        0xCD,0x4A,0x53,0x1A,0x00,0x30,0xD9,0x1A
    };

    struct work *work = NULL;
    cgtime(&current);
    h = 0;
    pthread_mutex_lock(&nonce_mutex);
    cg_rlock(&info->update_lock);

    while(nonce_fifo.nonce_num)
    {
        nonce_fifo.nonce_num--;

        crc_check = CRC5((uint8_t *)&(nonce_fifo.nonce_buffer[nonce_fifo.p_rd]), ASIC_RETURN_DATA_LENGTH_WITHOUT_HEADER*8-5);
        nonce = (nonce_fifo.nonce_buffer[nonce_fifo.p_rd].nonce);
        work_id = nonce_fifo.nonce_buffer[nonce_fifo.p_rd].wc;
        chain_id = nonce_fifo.nonce_buffer[nonce_fifo.p_rd].chainid;
        nonce_diff = nonce_fifo.nonce_buffer[nonce_fifo.p_rd].diff;
        nonce_crc5 = nonce_fifo.nonce_buffer[nonce_fifo.p_rd].crc5 & 0x1f;

        if(crc_check != nonce_crc5)
        {
            //applog(LOG_ERR,"%s: crc5 error,should be 0x%02x,but check as 0x%02x", __FUNCTION__, nonce_crc5, crc_check);
            applog(LOG_ERR,"%s: get nonce: 0x%08x, diff: 0x%02x, wc: 0x%02x, crc_received: 0x%02x, crc_checked: 0x%02x,chainid: 0x%02x", __FUNCTION__, nonce, nonce_diff, work_id, nonce_fifo.nonce_buffer[nonce_fifo.p_rd].crc5, crc_check, chain_id);

            //if signature enabled,check SIG_INFO register
            goto crc_error;
        }
        pthread_mutex_lock(&work_queue_mutex);
        work = info->work_queue[work_id];
        pthread_mutex_unlock(&work_queue_mutex);
        if(work)
        {
            //applog(LOG_ERR,"%s: work_id = 0x%02x", __FUNCTION__, work_id);

            submitfull = 0;
            //hexdump((uint8_t *)&work->data, 128);

            work_nonce = (uint32_t *)(work->data + 64 + 12);
            *work_nonce = Swap32(nonce);
            applog(LOG_DEBUG,"%s: work_nonce = 0x%08x", __FUNCTION__, nonce);

            memcpy(pworkdata, work->data, 80);

            /*
            for(i=0;i<WORK_DATA_INPUT_LENGTH; i++)
            {
                applog(LOG_NOTICE, "%s: work->data[%d] = 0x%02x", __FUNCTION__, i, work->data[i]);
            }
            */

            for (k=0; k < 20; k++)
            {
                endiandata[k] = ((uint32_t*)pworkdata)[k];
                endiandata[k] = Swap32(endiandata[k]);
                //applog(LOG_DEBUG,"%s: endiandata[%d] = 0x%08x", __FUNCTION__, k, endiandata[k]);
            }

#if 0
            for (k=0; k < 20; k++)
            {
                my_be32enc(&endiandata[k], ((uint32_t*)buf)[k]);
                endiandata[k] = Swap32(endiandata[k]);
                applog(LOG_ERR,"%s: endiandata[%d] = 0x%08x", __FUNCTION__, k, endiandata[k]);
            }
#endif

            Xhash(hash1, endiandata);
            memcpy(work->hash, hash1, 32);

            if(*((uint32_t *)(&work->hash) + 7) <= DEVICE_DIFF_SET_MASK)
            {
                update_work_stats(thr, work);
                submitnonceok = true;
                if (fulltest(hash1, work->target))
                {
                    submit_nonce_direct(thr,work,Swap32(nonce));
                }
            }
            else
            {
                applog(LOG_DEBUG,"got a hw from Chain%d Asic%d", chain_id, ((Swap32(nonce) & 0xFC000000) >> 26) / dev.addrInterval);
                inc_hw_errors(thr);
                submitnonceok = false;
                if ( chain_id >= BITMAIN_MAX_CHAIN_NUM )
                {
                    applog( LOG_ERR, "%s: Chain_ID [%d] Error!", __FUNCTION__, chain_id);
                }
                else
                {
                    dev.chain_hw[chain_id] ++;
                }
            }

            if(submitnonceok)
            {
                h_each_chain[chain_id]++;
                h += 0x1UL << (DEVICE_DIFF_SET - DEVICE_DIFF_STANDARD);
                which_asic_nonce = ((Swap32(nonce) & 0xFC000000) >> 26) / dev.addrInterval;
                applog(LOG_DEBUG,"%s: chain %d which_asic_nonce %d ", __FUNCTION__, chain_id, which_asic_nonce);

                if (( chain_id > BITMAIN_MAX_CHAIN_NUM ) || (!dev.chain_exist[chain_id]))
                {
                    applog(LOG_ERR, "ChainID Cause Error! ChainID:[%d] %d", chain_id,dev.chain_exist[chain_id]);
                    goto crc_error;
                }

                if ( which_asic_nonce >= D1_MINER_ASIC_NUM_EACH_CHAIN )
                {
                    applog(LOG_ERR, "Which Nonce Cause Err![%d]", which_asic_nonce);
                    goto crc_error;
                }

                dev.chain_asic_nonce[chain_id][which_asic_nonce]++;
            }
            cg_logwork(work, nonce_bin, submitnonceok);

        }
        else
        {
            applog(LOG_ERR, "%s %d: work %02x not find error", bitmain_DASH->drv->name, bitmain_DASH->device_id, work_id);
        }
    crc_error:
        if(nonce_fifo.p_rd < MAX_NONCE_NUMBER_IN_FIFO)
        {
            nonce_fifo.p_rd++;
        }
        else
        {
            nonce_fifo.p_rd = 0;
        }
    }


    cg_runlock(&info->update_lock);
    pthread_mutex_unlock(&nonce_mutex);
    cgsleep_ms(1);

    if(h != 0)
    {
        applog(LOG_DEBUG,"%s: hashes %"PRIu64"...", __FUNCTION__,h * DEVICE_DIFF_STANDARD_MASK);
    }

    h = h * DEVICE_DIFF_STANDARD_MASK;
    return 0;
}

void *check_fan_thr(void *arg)
{

    uint32_t fan0SpeedHist = 0;
    uint32_t fan1SpeedHist = 0;
    uint32_t fan0SpeedCur = 0;
    uint32_t fan1SpeedCur = 0;
    uint32_t fan0Speed = 0;
    uint32_t fan1Speed = 0;
    uint32_t fan0_exist = 0;
    uint32_t fan1_exist = 0;
    char buffer[256] = "";
    char* pos = NULL;
    FILE* fanpfd = fopen(PROCFILENAME, "r");

    if(fanpfd == NULL)
    {
        while(1)
        {
            applog(LOG_ERR, "%s: open /proc/interrupt error", __FUNCTION__);
            sleep(3);
        }
    }

    fseek(fanpfd, 0, SEEK_SET);

    while(fgets(buffer, 256, fanpfd))
    {
        if ( ((pos = strstr(buffer, FAN0)) != 0) && (strstr(buffer, "gpiolib") != 0 ) )
        {
            applog(LOG_DEBUG, "find fan1.");
            fan0SpeedHist = fan0SpeedCur = getNum(buffer);
        }

        if (((pos = strstr(buffer, FAN1)) != 0) && (strstr(buffer, "gpiolib") != 0 ))
        {
            applog(LOG_DEBUG, "find fan2.");
            fan1SpeedHist = fan1SpeedCur = getNum(buffer);
        }
    }

    while( 1 )
    {
        fseek(fanpfd, 0, SEEK_SET);

        while( fgets(buffer, 256, fanpfd) )
        {
            if ( ((pos = strstr(buffer, FAN0)) != 0) && (strstr(buffer, "gpiolib") != 0 ) )
            {
                fan0SpeedCur = getNum(buffer);
                if (fan0SpeedHist > fan0SpeedCur)
                {
                    fan0Speed = (0xffffffff - fan0SpeedHist + fan0SpeedCur);
                }
                else
                {
                    fan0Speed = ( fan0SpeedCur - fan0SpeedHist );
                }
                fan0Speed = fan0Speed * 60 / 2 / FANINT;    // 60: 1 minute; 2: 1 interrupt has 2 edges; FANINT: check fan speed every FANINT senconds
                if ( fan0Speed )
                {
                    fan0_exist = 1;
                    dev.fan_exist[0] = 1;
                    if( fan0Speed > FAN_WANN_SPEED)
                    {
                        fan0Speed = FAN_WANN_SPEED;
                    }
                }
                else
                {
                    fan0_exist = 0;
                    dev.fan_exist[0] = 0;
                }
                dev.fan_speed_value[0] = fan0Speed;
                fan0SpeedHist = fan0SpeedCur;
                if (dev.fan_speed_top1 <  fan0Speed)
                {
                    dev.fan_speed_top1 = fan0Speed;
                }
            }

            if (((pos = strstr(buffer, FAN1)) != 0) && (strstr(buffer, "gpiolib") != 0 ))
            {
                fan1SpeedCur = getNum(buffer);
                if (fan1SpeedHist > fan1SpeedCur)
                {
                    fan1Speed = (0xffffffff - fan1SpeedHist + fan1SpeedCur);
                }
                else
                {
                    fan1Speed = ( fan1SpeedCur - fan1SpeedHist );
                }
                fan1SpeedHist = fan1SpeedCur;
                fan1Speed = fan1Speed * 60 / 2 / FANINT;    // 60: 1 minute; 2: 1 interrupt has 2 edges; FANINT: check fan speed every FANINT senconds
                if ( fan1Speed )
                {
                    fan1_exist = 1;
                    dev.fan_exist[1] = 1;
                    if( fan1Speed > FAN_WANN_SPEED)
                    {
                        fan1Speed = FAN_WANN_SPEED;
                    }
                }
                else
                {
                    fan1_exist = 0;
                    dev.fan_exist[1] = 0;
                }

                dev.fan_speed_value[1] = fan1Speed;
                if (dev.fan_speed_top1 <  fan1Speed)
                {
                    dev.fan_speed_top1 = fan1Speed;
                }
            }
        }

        dev.fan_num = fan1_exist + fan0_exist;

        sleep(FANINT);
    }
}

int create_bitmain_check_fan_pthread(void)
{
    check_fan_id = calloc(1,sizeof(struct thr_info));
    if(thr_info_create(check_fan_id, NULL, check_fan_thr, NULL))
    {
        applog(LOG_DEBUG,"%s: create thread for check miner_status", __FUNCTION__);
        return -8;
    }
    pthread_detach(check_fan_id->pth);
    cgsleep_ms(500);
}

int fan_error_num = 0;
inline int check_fan_ok()
{
    int ret = 0;
    if(dev.fan_num < MIN_FAN_NUM)
    {
        ret = 1;
        goto err;
    }
    if(dev.fan_speed_value[0] < (FAN1_MAX_SPEED * dev.fan_pwm / 130))
    {
        ret = 2;
        goto err;
    }
    if(dev.fan_speed_value[1] < (FAN2_MAX_SPEED * dev.fan_pwm / 130))
    {
        ret = 3;
        goto err;
    }
    if((dev.pwm_percent == 100) && (dev.fan_speed_value[0] < (FAN1_MAX_SPEED * 90 / 100) || dev.fan_speed_value[1] < (FAN2_MAX_SPEED * 90 / 100)))
    {
        ret = 4;
        goto err;
    }
err:
    if(ret != 0)
    {
        fan_error_num++;
        if(fan_error_num > (FANINT * 10))
            return ret;
    }
    else
    {
        fan_error_num = 0;
        return 0;
    }
}

void *check_miner_status(void *arg)
{
    struct bitmain_DASH_info *info = (struct bitmain_DASH_info*)arg;
    struct timeval tv_start = {0, 0}, diff = {0, 0}, tv_end,tv_send;
    double ghs = 0;
    int i = 0, j = 0;
    cgtime(&tv_end);
    cgtime(&tv_send);
    copy_time(&tv_start, &tv_end);
    copy_time(&tv_send_job,&tv_send);
    bool stop = false;
    unsigned int asic_num = 0, error_asic = 0, avg_num = 0;
    unsigned int which_chain, which_asic, which_sensor;
    unsigned int offset = 0;
    int fan_ret = 0;
    struct bitmain_DASH_info_with_index info_with_index[BITMAIN_MAX_CHAIN_NUM];
    pthread_t reinit_id[BITMAIN_MAX_CHAIN_NUM];

    while(1)
    {
        diff.tv_sec = 0;
        diff.tv_usec = 0;
        cgtime(&tv_end);
        cgtime(&tv_send);
        timersub(&tv_end, &tv_start, &diff);

        // 10 minutes
        if (diff.tv_sec > 120)
        {
            asic_num = 0, error_asic = 0, avg_num = 0;

            for(which_chain=0; which_chain<BITMAIN_MAX_CHAIN_NUM; which_chain++)
            {
                if(dev.chain_exist[which_chain])
                {
                    asic_num += dev.chain_asic_num[which_chain];
                    for(which_asic=0; which_asic<dev.chain_asic_num[which_chain]; which_asic++)
                    {
                        avg_num += dev.chain_asic_nonce[which_chain][which_asic];
                        applog(LOG_DEBUG,"%s: chain %d asic %d asic_nonce_num %d", __FUNCTION__, which_chain,which_asic,dev.chain_asic_nonce[which_chain][which_asic]);
                    }
                }
            }
            if (asic_num != 0)
            {
                applog(LOG_DEBUG,"%s: avg_num %d asic_num %d", __FUNCTION__, avg_num,asic_num);
                avg_num = avg_num / asic_num / 8;
            }
            else
            {
                avg_num = 1;
            }

            for(which_chain=0; which_chain<BITMAIN_MAX_CHAIN_NUM; which_chain++)
            {
                if(dev.chain_exist[which_chain])
                {
                    offset = 0;

                    for(which_asic=0; which_asic<dev.chain_asic_num[which_chain]; which_asic++)
                    {
                        if(which_asic % 8 == 0)
                        {
                            if ( ( which_asic + offset ) > (ASIC_NUM_EACH_CHAIN + 16) )
                                applog(LOG_ERR, "asic num err![%d]", (which_asic + offset));
                            dev.chain_asic_status_string[which_chain][which_asic+offset] = ' ';
                            offset++;
                        }

                        if(dev.chain_asic_nonce[which_chain][which_asic]>avg_num)
                        {
                            if ( ( which_asic +offset ) > (ASIC_NUM_EACH_CHAIN + 16) )
                                applog(LOG_ERR, "asic num err![%d]", (which_asic + offset));
                            dev.chain_asic_status_string[which_chain][which_asic+offset] = 'o';
                        }
                        else
                        {
                            if ( ( which_asic + offset ) > (ASIC_NUM_EACH_CHAIN + 16) )
                                applog(LOG_ERR, "asic num err![%d]", (which_asic + offset));
                            dev.chain_asic_status_string[which_chain][which_asic+offset] = 'x';
                            error_asic++;
                        }

                        if ( ( which_asic ) > (ASIC_NUM_EACH_CHAIN + 16) )
                            applog(LOG_ERR, "asic num err![%d]", (which_asic));
                        dev.chain_asic_nonce[which_chain][which_asic] = 0;
                    }

                    if ( ( which_asic + offset ) > (ASIC_NUM_EACH_CHAIN + 16) )
                        applog(LOG_ERR, "asic num err![%d]", (which_asic + offset));
                    dev.chain_asic_status_string[which_chain][which_asic+offset] = '\0';
                    if(((each_chain_h_avg[which_chain]/1000000/10) < (double)(( ASIC_NUM_EACH_CHAIN * dev.frequency * dev.corenum * 0.50) / 40 )) && (!status_error))
                    {
                        applog(LOG_NOTICE,"chain %d reinit %0.2f",which_chain,each_chain_h_avg[which_chain]/1000000/10);
                        info_with_index[which_chain].info = info;
                        info_with_index[which_chain].chain_index = which_chain;
                        pthread_create(&reinit_id[which_chain], NULL, bitmain_DASH_reinit_chain, (void *)(&info_with_index[which_chain]));
                        gMinerStatus_Low_Hashrate = true;
                    }
                    else
                    {
                        gMinerStatus_Low_Hashrate = false;
                    }
                }
            }
            copy_time(&tv_start, &tv_end);
        }

        // check temperature
        if(dev.temp_top1 > MAX_TEMP)
        {
            gMinerStatus_High_Temp_Counter++;
            if(gMinerStatus_High_Temp_Counter > 2)
            {
                gMinerStatus_High_Temp = true;
                applog_e(status_error,LOG_ERR,"%s: the temperature is too high, close PIC and need reboot!!!", __FUNCTION__);
            }
            else
            {
                applog_e(status_error,LOG_ERR, "Temperature is higher than 85'C for %d time", gMinerStatus_High_Temp_Counter);
            }
        }
        else
        {
            gMinerStatus_High_Temp_Counter = 0;
            gMinerStatus_High_Temp = false;
        }

        // check sensor

        // check internet connection
        timersub(&tv_send, &tv_send_job, &diff);
        if(diff.tv_sec > 120)
        {
            gMinerStatus_Lost_connection_to_pool = true;
            applog_e(status_error,LOG_ERR, "%s: We have lost internet for %d seconds, so don't send work to hashboard anymore", __FUNCTION__, diff.tv_sec);
        }
        else
        {
            gMinerStatus_Lost_connection_to_pool = false;
        }

        // check fan
        fan_ret = check_fan_ok();
        if(fan_ret != 0)
        {
            gFan_Error = true;
            switch (fan_ret)
            {
                case 1:
                    applog_e(status_error,LOG_ERR, "Fan Err! Disable PIC! Fan num is %d",dev.fan_num);
                    break;
                case 2:
                    applog_e(status_error,LOG_ERR, "Fan Err! Disable PIC! Fan1 speed is too low %d pwm %d ",dev.fan_speed_value[0],dev.pwm_percent);
                    break;
                case 3:
                    applog_e(status_error,LOG_ERR, "Fan Err! Disable PIC! Fan2 speed is too low %d pwm %d ",dev.fan_speed_value[1],dev.pwm_percent);
                    break;
                case 4:
                    applog_e(status_error,LOG_ERR, "Fan Err! Disable PIC! Fan1:%d Fan2:%d pwm %d",dev.fan_speed_value[0],dev.fan_speed_value[1],dev.pwm_percent);
                    break;
            }
        }
        else
        {
            gFan_Error = false;
        }

        if(gMinerStatus_Low_Hashrate || gMinerStatus_Lost_connection_to_pool|| gMinerStatus_High_Temp || gFan_Error || gMinerStatus_Not_read_all_sensor)
        {
            stop = true;
            if((!once_error) && (gMinerStatus_High_Temp || gFan_Error ))
            {
                status_error = true;
                once_error = true;
                for(which_chain=0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
                {
                    if(dev.chain_exist[which_chain] == 1)
                    {
                        pthread_mutex_lock(&iic_mutex);
                        if(unlikely(ioctl(dev.i2c_fd,I2C_SLAVE,i2c_slave_addr[which_chain] >> 1 ) < 0))
                            applog(LOG_ERR,"ioctl error @ line %d",__LINE__);
                        disable_PIC16F1704_dc_dc_new();
                        pthread_mutex_unlock(&iic_mutex);
                    }
                }
            }
        }
        else
        {
            stop = false;
            status_error = false;
            if (once_error)
            {
                stop = true;
                status_error = true;
            }
        }

        set_led(stop);
        /**/
        // set fan pwm
        if(stop)
        {
            set_PWM(MAX_PWM_PERCENT);
        }
        else
        {
            set_PWM_according_to_temperature();
        }

        cgsleep_ms(1000);
    }
}


int create_bitmain_check_miner_status_pthread(struct bitmain_DASH_info *info)
{
    check_miner_status_id = calloc(1,sizeof(struct thr_info));
    if(thr_info_create(check_miner_status_id, NULL, check_miner_status, info))
    {
        applog(LOG_DEBUG,"%s: create thread for check miner_status", __FUNCTION__);
        return -5;
    }
    pthread_detach(check_miner_status_id->pth);
    cgsleep_ms(500);
}

void *get_hash_rate()
{
    uint32_t which_chain = 0, i = 0;
    struct timeval old_h, new_h, diff;
    double each_chain_h[BITMAIN_MAX_CHAIN_NUM][10] = {{0}};
    double each_chain_h_all = 0;
    int index[BITMAIN_MAX_CHAIN_NUM] = {0};
    cgtime(&old_h);
    cgtime(&new_h);

    while(1)
    {
        cgtime(&new_h);
        timersub(&new_h, &old_h, &diff);
        each_chain_h_all = 0;
        for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
        {
            if(dev.chain_exist[which_chain])
            {
                each_chain_h[which_chain][index[which_chain]] = (double)(h_each_chain[which_chain] * ((1 << 0x1B) -1));
                h_each_chain[which_chain] = 0;
                each_chain_h[which_chain][index[which_chain]] = each_chain_h[which_chain][index[which_chain]] / (diff.tv_sec + ((double)(diff.tv_usec + 1) / 1000000));
                each_chain_h_avg[which_chain] = 0;
                for( i = 0; i < 10; i++)
                {
                    each_chain_h_avg[which_chain]+= each_chain_h[which_chain][i];
                }

                snprintf(displayed_rate[which_chain],sizeof(displayed_rate[which_chain]),"%.2f",each_chain_h_avg[which_chain]/1000000/10);
                each_chain_h_all += each_chain_h_avg[which_chain]/1000000/10;

                index[which_chain]++;
                if (index[which_chain] >= 10)
                {
                    index[which_chain] = 0;
                }
            }
        }
        snprintf(displayed_hash_rate,sizeof(displayed_hash_rate),"%.2f",each_chain_h_all);
        geach_chain_h_all = each_chain_h_all;
        copy_time(&old_h,&new_h);

        sleep(READ_HASH_RATE_TIME_GAP);
    }
}


/*
void *get_hash_rate()
{
    uint32_t which_chain = 0, i = 0;

    while(1)
    {
        //pthread_mutex_lock(&reg_read_mutex);
        for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
        {
            if(dev.chain_exist[which_chain])
            {
                check_asic_reg(which_chain, 1, 0, HASH_RATE);
            }
            cgsleep_ms(10);
        }
        //check_rate = true;
        //pthread_mutex_unlock(&reg_read_mutex);

        calculate_hash_rate();


for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
{
    if(dev.chain_exist[which_chain])
    {
        check_asic_reg(which_chain, 1, 0, CHIP_STATUS);
    }
    cgsleep_ms(10);
}

for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
{
    for(i=0; i<ASIC_NUM_EACH_CHAIN; i++)
    {
        if(g_CHIP_STATUS_reg_value[which_chain][i] != 0)
        {
            if(g_CHIP_STATUS_reg_value[which_chain][i] == 0xffffffff)
            {
                applog(LOG_DEBUG, "%s: Chain%d ASIC%02d  didn't receive CHIP_STATUS value", __FUNCTION__, which_chain, i);
            }
            else
            {
                applog(LOG_DEBUG, "%s: Chain%d ASIC%02d  g_CHIP_STATUS_reg_value = 0x%08x", __FUNCTION__, which_chain, i, g_CHIP_STATUS_reg_value[which_chain][i]);
            }
        }
    }
}

for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
{
    if(dev.chain_exist[which_chain] == 1)
    {
        set_config(dev.dev_fd[which_chain], 1, 0, CHIP_STATUS, 0x80000000);
        cgsleep_ms(5);
    }
}

sleep(READ_HASH_RATE_TIME_GAP);
}
}
*/

int create_bitmain_get_hash_rate_pthread(void)
{
    read_hash_rate = calloc(1,sizeof(struct thr_info));
    if(thr_info_create(read_hash_rate, NULL, get_hash_rate, read_hash_rate))
    {
        applog(LOG_DEBUG,"%s: create thread for get hashrate from asic failed", __FUNCTION__);
        return -6;
    }
    pthread_detach(read_hash_rate->pth);
    cgsleep_ms(500);
}

void process_i2creg(unsigned int chain_id, unsigned char chip_addr,unsigned int reg_data)
{
    int which_sensor = 0;
    if ((reg_data & 0xc0000000) == 0)
    {
        for (which_sensor = 0 ; which_sensor < BITMAIN_REAL_TEMP_CHIP_NUM; which_sensor++ )
        {
            if(TempChipAddr[which_sensor] == chip_addr)
            {
                if((reg_data & 0xff00) == 0x0)
                {
                    dev.chain_asic_temp[chain_id][which_sensor][LOCAL] = reg_data & 0xff;
                    g_chip_temp_return[chain_id][which_sensor][LOCAL] = true;
                }
                else if ((reg_data & 0xff00) == 0x100)
                {
                    dev.chain_asic_temp[chain_id][which_sensor][REMOTE] = reg_data & 0xff;
                    g_chip_temp_return[chain_id][which_sensor][REMOTE] = true;
                }
                else if ((reg_data & 0xff00) == 0x1100)
                {
                    send_back_gTempOffsetValue[chain_id][which_sensor] = reg_data & 0xff;
                    g_chip_temp_return[chain_id][which_sensor][OFFSET] = true;
                }
                else if ((reg_data & 0xff00) == 0xFE00)
                {
                    sensor_id[chain_id][which_sensor] = reg_data & 0xff;
                    g_chip_temp_return[chain_id][which_sensor][ID] = true;
                }
            }
        }
    }
}

void process_hashrate_reg(unsigned int chain_id, unsigned char chip_addr,unsigned int reg_data)
{
    int which_chip = chip_addr/dev.addrInterval;
    if((reg_data & 0x80000000) == 0)
        is_rt = true;
    else
        is_rt = false;

    g_HASH_RATE_reg_value[chain_id][which_chip] = reg_data;
    g_HASH_RATE_reg_value_from_which_asic[chain_id][which_chip] = true;
    g_HASH_RATE_reg_value_num[chain_id]++;
}

void *bitmain_scanreg(void* args)
{
    unsigned int i,j, not_reg_data_time = 0;
    unsigned int reg_value_num = 0, reg_data = 0, received_reg_num = 0;
    unsigned char crc_check, chip_addr, reg_addr,chain_id,which_chip;
    int read_num = 0;
    uint64_t tmp_rate = 0, temp_average_rate = 0, temp_real_time_rate = 0, Hashboard_temp_average_rate = 0, Hashboard_temp_real_time_rate = 0;
    bool zero_asic_num = true;
    uint64_t temp_hash_rate = 0;


rerun_all:
    clear_register_value_buf();
    zero_asic_num = true;
    read_num = 0;
    tmp_rate = 0;
    received_reg_num = 0;
    temp_average_rate = 0;
    temp_real_time_rate = 0;
    Hashboard_temp_average_rate = 0;
    Hashboard_temp_real_time_rate = 0;


    while(1)
    {
        cgsleep_ms(3);

        pthread_mutex_lock(&reg_mutex);
        reg_value_num = reg_fifo.reg_value_num;
        pthread_mutex_unlock(&reg_mutex);

        /*
        if(reg_value_num > 0)
        {
            applog(LOG_DEBUG,"%s: reg_value_num = %d", __FUNCTION__, reg_value_num);
        }
        */

        if(reg_value_num >= MAX_NONCE_NUMBER_IN_FIFO || reg_fifo.p_rd >= MAX_NONCE_NUMBER_IN_FIFO)
        {
            applog(LOG_WARNING,"reg fifo is full !!");
            goto rerun_all;
        }

        if(reg_value_num > 0)
        {
            for(j = 0; j < reg_value_num; j++)
            {
                pthread_mutex_lock(&reg_mutex);
                reg_data = Swap32(reg_fifo.reg_buffer[reg_fifo.p_rd].reg_value);
                chip_addr = reg_fifo.reg_buffer[reg_fifo.p_rd].chipaddr;
                reg_addr = reg_fifo.reg_buffer[reg_fifo.p_rd].regaddr;
                chain_id = reg_fifo.reg_buffer[reg_fifo.p_rd].chainid;

                crc_check = CRC5((uint8_t *)&(reg_fifo.reg_buffer[reg_fifo.p_rd]), ASIC_RETURN_DATA_LENGTH_WITHOUT_HEADER*8-5);
                if(crc_check != (reg_fifo.reg_buffer[reg_fifo.p_rd].crc5 & 0x1f))
                {
                    applog(LOG_ERR,"%s: reg_value = 0x%08x crc5 error,should be %02x,but check as %02x",__FUNCTION__,reg_data, reg_fifo.reg_buffer[reg_fifo.p_rd].crc5 & 0x1f,crc_check);

                    reg_fifo.p_rd++;
                    reg_fifo.reg_value_num--;
                    if(reg_fifo.p_rd >= MAX_NONCE_NUMBER_IN_FIFO)
                    {
                        reg_fifo.p_rd = 0;
                    }

                    pthread_mutex_unlock(&reg_mutex);
                    continue;
                }

                reg_fifo.p_rd++;
                reg_fifo.reg_value_num--;
                if(reg_fifo.p_rd >= MAX_NONCE_NUMBER_IN_FIFO)
                {
                    reg_fifo.p_rd = 0;
                }
                pthread_mutex_unlock(&reg_mutex);
                which_chip = chip_addr/dev.addrInterval;
                if(which_chip >= ASIC_NUM_EACH_CHAIN)
                    continue;

                if(reg_addr == CHIP_ADDR)
                {
                    if(update_asic_num)
                    {
                        g_CHIP_ADDR_reg_value[chain_id][g_CHIP_ADDR_reg_value_num[chain_id]] = reg_data;
                        g_CHIP_ADDR_reg_value_num[chain_id]++;
                    }
                }

                if(reg_addr == PLL_PARAMETER)
                {
                    applog(LOG_ERR,"%s: the Chip%d PLL_PARAMETER reg_value = 0x%08x @chain%d", __FUNCTION__, which_chip, reg_data, chain_id);
                }

                if ( reg_addr == MISC_CONTROL )
                {
                    applog(LOG_ERR,"%s: the Chip%d MISC_CONTROL reg_value = 0x%08x @chain%d", __FUNCTION__, which_chip, reg_data, chain_id);
                }

                if ( reg_addr == TICK_MASK)
                {
                    applog(LOG_ERR,"%s: the Chip%d TICK_MASK reg_value = 0x%08x @chain%d", __FUNCTION__, chip_addr/dev.addrInterval, reg_data, chain_id);
                }

                if ( reg_addr == GENERAL_I2C_COMMAND)
                {
                    process_i2creg(chain_id,chip_addr,reg_data);
                    applog(LOG_DEBUG,"%s: the Chip%d GENERAL_I2C_COMMAND reg_value = 0x%08x @chain%d", __FUNCTION__, chip_addr/dev.addrInterval, reg_data, chain_id);
                }

                if(reg_addr == HASH_RATE)
                {
                    process_hashrate_reg(chain_id,chip_addr,reg_data);
                }

                if(reg_addr == CHIP_STATUS)
                {
                    g_CHIP_STATUS_reg_value[chain_id][which_chip] = reg_data;
                    g_CHIP_STATUS_reg_value_num[chain_id]++;
                }
            }
        }
    }
}


int creat_bitmain_scanreg_pthread(void)
{
    scan_reg_id = calloc(1,sizeof(struct thr_info));
    if(thr_info_create(scan_reg_id, NULL, bitmain_scanreg, scan_reg_id))
    {
        applog(LOG_ERR,"%s: create thread error for bitmain_scanreg", __FUNCTION__);
        return -4;
    }
    pthread_detach(scan_reg_id->pth);
    cgsleep_ms(100);
}

void *read_temp_func()
{
    unsigned char which_chain, which_sensor, read_temperature_time;
    signed char local_temp=0, remote_temp=0, temp_offset_value=0;
    unsigned int ret = 0, i = 0, tmpTemp = 0;
    bool read_temp_result = false;
    bool not_read_out_temperature = false;

    applog(LOG_DEBUG, "%s", __FUNCTION__);

    while(1)
    {
        read_temp_result = false;
        for ( which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++ )
        {
            if ( dev.chain_exist[which_chain] == 1 )
            {
                for (which_sensor = 0 ; which_sensor < BITMAIN_REAL_TEMP_CHIP_NUM; which_sensor++ )
                {
                    read_i2c_reg(which_chain,which_sensor,REMOTE,REGADDRVALID | DEVICEADDR(TMP451_IIC_SALVE_ADDR) | REGADDR(EXT_TEMP_VALUE_HIGH_BYTE) | DATA(0) & (~RW));
                    read_i2c_reg(which_chain,which_sensor,LOCAL,REGADDRVALID | DEVICEADDR(TMP451_IIC_SALVE_ADDR) | REGADDR(LOCAL_TEMP_VALUE) | DATA(0) & (~RW));
                }
            }
        }

        tmpTemp = 0;

        for ( which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++ )
        {
            if ( dev.chain_exist[which_chain] == 1 )
            {
                for (which_sensor = 0 ; which_sensor < BITMAIN_REAL_TEMP_CHIP_NUM; which_sensor++ )
                {
                    if(g_chip_temp_return[which_chain][which_sensor][LOCAL] || g_chip_temp_return[which_chain][which_sensor][REMOTE])
                        read_temp_result = true;
                    if ( dev.chain_asic_temp[which_chain][which_sensor][0] > tmpTemp)
                    {
                        tmpTemp = dev.chain_asic_temp[which_chain][which_sensor][0];
                        if(tmpTemp > MAX_TEMP)
                        {
                            applog_e(status_error,LOG_ERR,"%s: Chain%d sensor%d local temp is %d `C, higher than MAX_TEMP", __FUNCTION__, which_chain, which_sensor, tmpTemp);
                        }
                    }
                }
            }
        }
        dev.temp_top1 = tmpTemp;

        if(!read_temp_result)
        {
            gMinerStatus_Not_read_all_sensor = true;
            applog_e(status_error,LOG_ERR,"%s: can't read all sensor's temperature, close PIC and need reboot!!!", __FUNCTION__);
        }
        else
        {
            gMinerStatus_Not_read_all_sensor = false;
        }
        sleep(READ_TEMPERATURE_TIME_GAP);
    }
}

int create_bitmain_read_temp_pthread(void)
{
    read_temp_id = calloc(1,sizeof(struct thr_info));
    if(thr_info_create(read_temp_id, NULL, read_temp_func, read_temp_id))
    {
        applog(LOG_DEBUG,"%s: create thread for read temp", __FUNCTION__);
        return -7;
    }
    pthread_detach(read_temp_id->pth);
    cgsleep_ms(500);
}

void *DASH_fill_work(void *usrdata)
{
    pthread_detach(pthread_self());
    applog(LOG_DEBUG, "Start To Fill Work!");
    struct bitmain_DASH_info_with_index *info_with_chain = (struct bitmian_DASH_info_with_index *)usrdata;
    struct bitmain_DASH_info *info = info_with_chain->info;
    uint8_t chainid = info_with_chain->chain_index;
    struct thr_info * thr = info->thr;
    struct timeval send_start, last_send, send_elapsed;
    struct work_dash workdata;
    unsigned int workbuf[WORK_DATA_INPUT_LENGTH/4];
    struct work *work = NULL;
    unsigned char workid = 0;
    unsigned int i = 0;
    unsigned char test_work_data[80] =
    {
        0x00, 0x00, 0x00, 0x20, 0xf2, 0xef, 0x22, 0xeb,
        0x7b, 0x28, 0xae, 0x6a, 0x03, 0x99, 0xb9, 0x73,
        0xf2, 0x3a, 0x4c, 0x25, 0x26, 0x00, 0x8a, 0x20,
        0xe3, 0x11, 0xdb, 0xbc, 0x80, 0x09, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0xdb, 0x61, 0x29, 0x8c,
        0xd3, 0x1c, 0x00, 0x77, 0x19, 0xb9, 0xf2, 0xb7,
        0x3f, 0xf1, 0x34, 0x8c, 0xf9, 0x2c, 0x7f, 0xab,
        0x1e, 0x23, 0x24, 0x51, 0x03, 0xf6, 0x11, 0x6c,
        0x32, 0xb1, 0x63, 0x93, 0x6e, 0x93, 0x5b, 0x59,
        0xf4, 0x48, 0x50, 0x1a, 0x55, 0x63, 0xea, 0x6a
    };

    applog(LOG_DEBUG, "Start To Fill Work!ChainIndex:[%d]", chainid);

    while(1 && !reiniting[chainid])
    {

        if(!start_send[chainid])
        {
            cgsleep_ms(10);
            continue;
        }

        cgtime(&send_start);
        timersub(&send_start, &last_send, &send_elapsed);
        //applog(LOG_DEBUG, "send_elapsed.tv_sec = %d, send_elapsed.tv_usec = %d, dev.timeout = %d", send_elapsed.tv_sec, send_elapsed.tv_usec, dev.timeout);

        if(new_block[chainid] || send_elapsed.tv_sec*1000000 + send_elapsed.tv_usec  >= dev.timeout )
        {
            cgtime(&last_send);
        more_work:
            pthread_mutex_lock(&work_queue_mutex);
            work = get_work(thr, thr->id);
            pthread_mutex_unlock(&work_queue_mutex);
            if (unlikely(!work))
            {
                applog(LOG_ERR, "Work Error![%d]", workid);
                goto more_work;
            }
            applog(LOG_DEBUG, "new_block[chainid] = %d", new_block[chainid]);
            applog(LOG_DEBUG, "ChainIndex:[%d], workid = %d", chainid, (work->id & 0x7f));
            workid = work->id & 0x7f;
            new_block[chainid] = false;
            memset((void *)(&workdata), 0, sizeof(workdata));

            for(i=0; i<WORK_DATA_INPUT_LENGTH/4; i++)
            {
                workdata.work[i*4 + 3] = work->data[i*4 + 0];
                workdata.work[i*4 + 2] = work->data[i*4 + 1];
                workdata.work[i*4 + 1] = work->data[i*4 + 2];
                workdata.work[i*4 + 0] = work->data[i*4 + 3];
            }

            /*
            for(i=0;i<WORK_DATA_INPUT_LENGTH; i++)
            {
                applog(LOG_NOTICE, "%s: workdata.work[%d] = 0x%02x", __FUNCTION__, i, workdata.work[i]);
            }
            */
            workdata.type = WORK_INPUT_TYPE_WITHOUT_SNO;
            workdata.wc = workid;
            workdata.crc16 = crc_itu_t(0xffff,(uint8_t *) &workdata, WORK_INPUT_LENGTH_WITHOUT_CRC);

            // endian change
            workdata.crc16 = (workdata.crc16 >> 8) | ((workdata.crc16 & 0xff) << 8);
            memcpy((unsigned char *)&workdata + WORK_INPUT_LENGTH_WITHOUT_CRC, &workdata.crc16, 2);

            pthread_mutex_lock(&work_queue_mutex);
            if(info->work_queue[workid])
            {
                free_work(info->work_queue[workid]);
                info->work_queue[workid] = NULL;
            }

            if ( workid >= BITMAIN_MAX_QUEUE_NUM ) applog(LOG_ERR, "WorkID Error![%d]", workid);
            info->work_queue[workid] = copy_work(work);
            pthread_mutex_unlock(&work_queue_mutex);

            applog(LOG_DEBUG, "ChainID[%d] Wirte Work. workid=%d", chainid, workid);
            DASH_write(info->dev_fd[chainid], (uint8_t *)&workdata, WORK_INPUT_LENGTH_WITH_CRC);
            // cg_runlock(&info->update_lock);
            gBegin_get_nonce = true;
            //hexdump((uint8_t *)&workdata, WORK_INPUT_LENGTH_WITH_CRC);

            cgtime(&tv_send_job);

        }

        cgsleep_ms(10);
        if(work != NULL)
        {
            free_work(work);
        }
    }
}

inline void add_point(int * point,int MAX_SIZE)
{
    (*point)++;
    if((*point) >= MAX_SIZE)
        (*point) = 0;
}

inline int use_point_sub_1(int point,int MAX_SIZE)
{
    return (point == 0) ? MAX_SIZE -1 : point -1;
}


inline int use_point_add_1(int point,int MAX_SIZE)
{
    return (point >= MAX_SIZE -1) ? 0 : (point + 1) ;
}

void *get_asic_response(void* arg)
{
    pthread_detach(pthread_self());

    uint32_t  nonce_number, read_loop;
    unsigned char nonce_bin[7],chainid;
    uint32_t fd;

    struct dev_info *dev_i = (struct dev_info*)arg;
    chainid = dev_i->chainid;
    fd = dev_i->dev_fd;

    unsigned char receive_buf[ASIC_RETURN_DATA_LENGTH*10] = {0};    // used to receive data
    unsigned char tmp[ASIC_RETURN_DATA_LENGTH] = {0};          // store 9 bytes data that from data_buf
    unsigned char data_buf[ASIC_RETURN_DATA_LENGTH * 512] = {0};
    int data_buf_rp = 0, data_buf_wp = 0;
    ssize_t len=0;
    int i = 0;
    int max = ASIC_RETURN_DATA_LENGTH * 512;

    tcflush(fd, TCIOFLUSH);
    applog(LOG_NOTICE, "Start A New Asic Response.Chain Id:[%d]", chainid);
    applog(LOG_DEBUG, "%s %d",__FUNCTION__,chainid);

    while(start_recv[chainid])
    {
        cgsleep_ms(1);
        len = DASH_read(fd, receive_buf, ASIC_RETURN_DATA_LENGTH*10);
        for(i = 0; i < len; i++)
        {
            data_buf[data_buf_wp] = receive_buf[i];
            add_point(&data_buf_wp,max);
        }

        if(data_buf_rp != data_buf_wp)
        {
            len = (data_buf_wp > data_buf_rp) ? (data_buf_wp - data_buf_rp):( ASIC_RETURN_DATA_LENGTH * 512 - data_buf_rp + data_buf_wp);
        }
        else
        {
            continue;
        }
        while(len >= ASIC_RETURN_DATA_LENGTH)
        {
            if(data_buf[data_buf_rp] == OUTPUT_HEADER_1 && data_buf[use_point_add_1(data_buf_rp,max)] == OUTPUT_HEADER_2)
            {
                for(i = 0; i < ASIC_RETURN_DATA_LENGTH; i++)
                {
                    tmp[i] = data_buf[data_buf_rp];
                    add_point(&data_buf_rp,max);
                }
            }
            else
            {
                for(i=0 ; i < ASIC_RETURN_DATA_LENGTH - 1; i++ )
                {
                    if(data_buf[data_buf_rp] != OUTPUT_HEADER_1 || data_buf[use_point_add_1(data_buf_rp,max)] != OUTPUT_HEADER_2)
                    {
                        add_point(&data_buf_rp,max);
                        applog(LOG_DEBUG,"%s: %d Headers are not corret! Header0 = 0x%02x, Header1 = 0x%02x, Header2 = 0x%02x rp = %d\n", __FUNCTION__,chainid,data_buf[use_point_sub_1(data_buf_rp,max)],data_buf[data_buf_rp], data_buf[use_point_add_1(data_buf_rp,max)],data_buf_rp);
                        continue;
                    }
                    else
                        break;
                }
                len = len -i;
                continue;
            }

            len = len - ASIC_RETURN_DATA_LENGTH;

            if(is_nonce_or_reg_value(tmp[ASIC_RETURN_DATA_LENGTH -1])) // get a nonce
            {
                if(gBegin_get_nonce)
                {
                    pthread_mutex_lock(&nonce_mutex);

                    memcpy((unsigned char *)(&nonce_fifo.nonce_buffer[nonce_fifo.p_wr].nonce), &tmp[2], 4);    // we do not swap32 here, but swap it when we analyse nonce
                    nonce_fifo.nonce_buffer[nonce_fifo.p_wr].diff           = tmp[6];
                    nonce_fifo.nonce_buffer[nonce_fifo.p_wr].wc             = tmp[7] & 0x7f;
                    nonce_fifo.nonce_buffer[nonce_fifo.p_wr].crc5           = tmp[8];
                    nonce_fifo.nonce_buffer[nonce_fifo.p_wr].chainid        = chainid;

                    if(nonce_fifo.p_wr < MAX_NONCE_NUMBER_IN_FIFO)
                    {
                        nonce_fifo.p_wr++;
                    }
                    else
                    {
                        nonce_fifo.p_wr = 0;
                    }

                    if(nonce_fifo.nonce_num < MAX_NONCE_NUMBER_IN_FIFO)
                    {
                        nonce_fifo.nonce_num++;
                    }
                    else
                    {
                        nonce_fifo.nonce_num = MAX_NONCE_NUMBER_IN_FIFO;
                        applog(LOG_WARNING, "%s: nonce fifo full!!!", __FUNCTION__);
                    }
                    pthread_mutex_unlock(&nonce_mutex);

                    //applog(LOG_DEBUG,"get nonce num %d",nonce_fifo.nonce_num);
                }
            }
            else    // get a register value
            {
                if(reg_fifo.reg_value_num >= MAX_NONCE_NUMBER_IN_FIFO || reg_fifo.p_wr >= MAX_NONCE_NUMBER_IN_FIFO)
                {
                    applog(LOG_DEBUG, "Will Clean!");
                    clear_register_value_buf();
                    continue;
                }
                pthread_mutex_lock(&reg_mutex);

                memcpy((unsigned char *)(&reg_fifo.reg_buffer[reg_fifo.p_wr].reg_value), &tmp[2], 4);      // we do not swap32 here, but swap it when we analyse register value
                reg_fifo.reg_buffer[reg_fifo.p_wr].chipaddr         = tmp[6];
                reg_fifo.reg_buffer[reg_fifo.p_wr].regaddr          = tmp[7];
                reg_fifo.reg_buffer[reg_fifo.p_wr].crc5             = tmp[8] & 0x1f;
                reg_fifo.reg_buffer[reg_fifo.p_wr].chainid          = chainid;

                if(reg_fifo.p_wr < MAX_NONCE_NUMBER_IN_FIFO )
                {
                    //applog(LOG_DEBUG,"%s: p_wr = %d reg_value_num = %d", __FUNCTION__,reg_fifo.p_wr,reg_fifo.reg_value_num);
                    reg_fifo.p_wr++;
                }
                else
                {
                    reg_fifo.p_wr = 0;
                }

                if(reg_fifo.reg_value_num < MAX_NONCE_NUMBER_IN_FIFO)
                {
                    reg_fifo.reg_value_num++;
                }
                else
                {
                    reg_fifo.reg_value_num = MAX_NONCE_NUMBER_IN_FIFO;
                    applog(LOG_WARNING, "%s: reg fifo full!!!", __FUNCTION__);
                }
                pthread_mutex_unlock(&reg_mutex);
            }
        }
    }
}

/****************** about cgminer pthread end ******************/



#define ABOUT_CGMINER_DRIVER
/******************** about cgminer driver ********************/

static void bitmain_DASH_detect(__maybe_unused bool hotplug)
{
    struct cgpu_info *cgpu = calloc(1, sizeof(*cgpu));
    struct device_drv *drv = &bitmainD1_drv;
    assert(cgpu);
    cgpu->drv = drv;
    cgpu->deven = DEV_ENABLED;
    cgpu->threads = 1;
    cgpu->device_data = calloc(sizeof(struct bitmain_DASH_info), 1);
    if (unlikely(!(cgpu->device_data)))
        quit(1, "Failed to calloc cgpu_info data");

    assert(add_cgpu(cgpu));
    applog(LOG_DEBUG,"%s detect new device",__FUNCTION__);
}


static bool bitmain_DASH_prepare(struct thr_info *thr)
{
    struct cgpu_info *bitmain_DASH = thr->cgpu;
    struct bitmain_DASH_info *info = bitmain_DASH->device_data;

    info->thr = thr;
    mutex_init(&info->lock);
    cglock_init(&info->update_lock);

    struct init_config DASH_config =
    {
        .token_type                 = 0x51,
        .version                    = 0,
        .length                     = 24,
        .baud                       = BITMAIN_DEFAULT_BAUD,
        .reset                      = 1,
        .fan_eft                    = opt_bitmain_fan_ctrl,
        .timeout_eft                = 1,
        .frequency_eft              = 1,
        .voltage_eft                = 1,
        .chain_check_time_eft       = 1,
        .chip_config_eft            = 1,
        .hw_error_eft               = 1,
        .beeper_ctrl                = 1,
        .temp_ctrl                  = 1,
        .chain_freq_eft             = 1,
        .auto_read_temp             = 0,
        .reserved1                  = 0,
        .reserved2                  = {0},
        .chain_num                  = 6,
        .asic_num                   = ASIC_NUM_EACH_CHAIN,
        .fan_pwm_percent            = opt_bitmain_fan_pwm,
        .temperature                = 80,
        .frequency                  = opt_bitmain_DASH_freq,
        .voltage                    = {0x07,0x25},
        .chain_check_time_integer   = 10,
        .chain_check_time_fractions = 10,
        .timeout_data_integer       = 0,
        .timeout_data_fractions     = 0,
        .reg_data                   = 0,
        .chip_address               = 0x04,
        .reg_address                = 0,
        .chain_min_freq             = 400,
        .chain_max_freq             = 600,
        .misc_control_reg_value     = MISC_CONTROL_DEFAULT_VALUE,
        .diode_mux_sel              = DIODE_MUX_SEL_DEFAULT_VALUE,
        .vdd_mux_sel                = VDD_MUX_SEL_DEFAULT_VALUE,
    };

    DASH_config.crc = crc_itu_t(0xff,(uint8_t *)(&DASH_config), sizeof(DASH_config)-2);
    info->DASH_config = DASH_config;

    bitmain_DASH_init(info);

    return true;
}


static int64_t bitmain_DASH_scanhash(struct thr_info *thr)
{
    h = 0;
    pthread_t send_id;
    pthread_create(&send_id, NULL, bitmain_scanhash, (void*)thr);
    pthread_join(send_id, NULL);

    return h;
}

static void bitmain_DASH_update(struct cgpu_info *bitmain)
{
    int i = 0;
    applog(LOG_DEBUG, "Updated Work!");
    for ( ; i < BITMAIN_MAX_CHAIN_NUM; ++i )
    {
        new_block[i] = true;
    }
}

static double hwp = 0;

static struct api_data *bitmain_api_stats(struct cgpu_info *cgpu)
{
    struct api_data *root = NULL;
    int i = 0;
    uint64_t hash_rate_all = 0;
    bool copy_data = false;

    root = api_add_uint8(root, "miner_count", &(dev.chain_num), copy_data);
    root = api_add_string(root, "frequency", dev.frequency_t, copy_data);
    root = api_add_uint8(root, "fan_num", &(dev.fan_num), copy_data);

    // dev.fan_speed_value[0] is FAN1
    // dev.fan_speed_value[1] is FAN2
    for(i = 0; i < BITMAIN_MAX_FAN_NUM; i++)
    {
        char fan_name[12];
        sprintf(fan_name,"fan%d",i+1);
        root = api_add_uint32(root, fan_name, &(dev.fan_speed_value[i]), copy_data);
    }

    root = api_add_uint8(root, "temp_num", &(dev.chain_num), copy_data);

    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        char temp_name[12];
        sprintf(temp_name,"temp%d",i+1);
        root = api_add_int16(root, temp_name, &(dev.chain_asic_temp[i][0][0]), copy_data);
    }


    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        char temp2_name[12];
        sprintf(temp2_name,"temp2_%d",i+1);
        root = api_add_int16(root, temp2_name, &(dev.chain_asic_temp[i][0][1]), copy_data);
    }

    /*
    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        char temp_name[12];
        sprintf(temp_name,"temp3%d",i+1);
        root = api_add_int16(root, temp_name, &(dev.chain_asic_temp[i][1][0]), copy_data);
    }


    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        char temp2_name[12];
        sprintf(temp2_name,"temp4_%d",i+1);
        root = api_add_int16(root, temp2_name, &(dev.chain_asic_temp[i][1][1]), copy_data);
    }
    */

    root = api_add_uint32(root, "temp_max", &(dev.temp_top1), copy_data);

    total_diff1 = total_diff_accepted + total_diff_rejected + total_diff_stale;
    hwp = (hw_errors + total_diff1) ?
          (double)(hw_errors) / (pow(2, (32 - DEVICE_DIFF_SET))) / (double)(hw_errors + total_diff1) : 0;

    root = api_add_percent(root, "Device Hardware%", &hwp, true);
    root = api_add_int(root, "no_matching_work", &hw_errors, true);

    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        char chain_name[12];
        sprintf(chain_name,"chain_acn%d",i+1);
        root = api_add_uint8(root, chain_name, &(dev.chain_asic_num[i]), copy_data);

    }

    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        char chain_asic_name[12];
        sprintf(chain_asic_name,"chain_acs%d",i+1);
        root = api_add_string(root, chain_asic_name, dev.chain_asic_status_string[i], copy_data);
    }

    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        char chain_hw[16];
        sprintf(chain_hw,"chain_hw%d",i+1);
        root = api_add_uint32(root, chain_hw, &(dev.chain_hw[i]), copy_data);
    }

    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        char chain_rate[16];
        sprintf(chain_rate,"chain_rate%d",i+1);
        root = api_add_string(root, chain_rate, displayed_rate[i], copy_data);
    }

    for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(dev.chain_exist[i] == 1)
        {
            hash_rate_all += rate[i];
        }
    }

    //suffix_string_DASH(hash_rate_all, (char * )displayed_hash_rate, sizeof(displayed_hash_rate), 6,false);

    return root;
}


static void bitmain_DASH_reinit_device(struct cgpu_info *bitmain)
{
    if(!status_error)
        system("/etc/init.d/cgminer.sh restart > /dev/null 2>&1 &");
}

static void get_bitmain_statline_before(char *buf, size_t bufsiz, struct cgpu_info *bitmain_DASH)
{
    struct bitmain_DASH_info *info = bitmain_DASH->device_data;
}

static void bitmain_DASH_shutdown(struct thr_info *thr)
{
    thr_info_cancel(check_miner_status_id);
    thr_info_cancel(check_fan_id);
    thr_info_cancel(read_hash_rate);
    thr_info_cancel(read_temp_id);
}

struct device_drv bitmainD1_drv =
{
    .drv_id = DRIVER_bitmainD1,
    .dname = "Bitmain_D1",
    .name = "D1",
    .drv_detect = bitmain_DASH_detect,
    .thread_prepare = bitmain_DASH_prepare,
    .hash_work = &hash_driver_work,
    .scanwork = bitmain_DASH_scanhash,
    .update_work = bitmain_DASH_update,
    .get_api_stats = bitmain_api_stats,
    .reinit_device = bitmain_DASH_reinit_device,
    .get_statline_before = get_bitmain_statline_before,
    .thread_shutdown = bitmain_DASH_shutdown,
};

/****************** about cgminer driver end ******************/

