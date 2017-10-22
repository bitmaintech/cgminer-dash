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
#include <termios.h>
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
#include "miner.h"
#include "usbutils.h"
#include "hexdump.c"
#include "util.h"
#include "driver-btm-L3.h"

#define SET_ASIC_RST_0 "echo 0 > /sys/class/gpio/gpio%d/value"
#define SET_ASIC_RST_1 "echo 1 > /sys/class/gpio/gpio%d/value"

struct thr_info *read_nonce_reg_id;                 // thread id for read nonce and register
struct thr_info *check_miner_status_id;                  // thread id for check system
struct thr_info *read_temp_id;
struct thr_info *read_hash_rate;
struct thr_info *pic_heart_beat;
struct thr_info *scan_reg_id;


uint64_t h = 0;
int const plug[BITMAIN_MAX_CHAIN_NUM] = {51,48,47,44};
int const tty[BITMAIN_MAX_CHAIN_NUM] = {1,2,4,5};
int const beep = 20;
int const red_led = 45;
int const green_led = 23;
int const fan_speed[BITMAIN_MAX_FAN_NUM] = {112,110};
int const i2c_slave_addr[BITMAIN_MAX_CHAIN_NUM] = {0xa0,0xa2,0xa4,0xa6};
struct dev_info dev_info[BITMAIN_MAX_CHAIN_NUM];


int opt_bitmain_L3_freq = 100;
int opt_bitmain_L3_voltage = 176;
int opt_bitmain_fan_pwm = 30;
int last_temperature = 0, temp_highest = 0;
int8_t opt_bitmain_L3_core_temp = 2;


bool update_asic_num = false;
bool opt_bitmain_fan_ctrl = false;
bool need_recheck[BITMAIN_MAX_CHAIN_NUM] = {false, false, false, false};

extern void rev(unsigned char *s, size_t l);
extern void cg_logwork(struct work *work, unsigned char *nonce_bin, bool ok);

pthread_mutex_t i2c_mutex = PTHREAD_MUTEX_INITIALIZER;


pthread_mutex_t reg_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t nonce_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t reg_read_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t iic_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t tty_write_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t temp_buf_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool start_send = false;
bool once_error = false;
bool status_error = false;
bool check_rate = false;
bool gBegin_get_nonce = false;
bool send_heart = true;
bool new_block[BITMAIN_MAX_CHAIN_NUM] = {false, false, false, false};

uint64_t rate[BITMAIN_MAX_CHAIN_NUM] = {0};
int rate_error[BITMAIN_MAX_CHAIN_NUM] = {0};
char displayed_rate[BITMAIN_MAX_CHAIN_NUM][16];
unsigned char pic_version[BITMAIN_MAX_CHAIN_NUM] = {0};


#define FANINT 5
#define FAN0 "84"
#define FAN1 "85"
#define PROCFILENAME "/proc/interrupts"


struct nonce_buf nonce_fifo;
struct reg_buf reg_fifo;
struct all_parameters dev;
struct timeval tv_send_job = {0, 0};

static int g_gpio_data[BITMAIN_MAX_CHAIN_NUM] = {5, 4, 27, 22};

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

static unsigned char TempChipAddr[BITMAIN_MAX_TEMP_CHIP_NUM] = {HAVE_TEMP, HAVE_TEMP_2, HAVE_TEMP_3};

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

static ASIC_TEMP_T TempBuffer[BITMAIN_MAX_CHAIN_NUM][BITMAIN_MAX_TEMP_CHIP_NUM] = {{{0},{0}}};
uint16_t crc_itu_t(uint16_t crc, const uint8_t *buffer, int len)
{
    while (len--)
        crc = crc_itu_t_byte(crc, *buffer++);
    return crc;
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


int L3_write(int fd, const void *buf, size_t bufLen)
{
    size_t ret;
    if (unlikely(fd == -1))
        ret = -1;
    // mutex_lock(&tty_write_mutex);
    flock(fd,LOCK_EX);
    ret = write(fd, buf, bufLen);
    //  mutex_unlock(&tty_write_mutex);
    if(unlikely(ret != bufLen))
    {
        applog(LOG_ERR,"write error!!");
    }
    flock(fd,LOCK_UN);
    cgsleep_us(500);
    return ret;
}


int L3_read(int fd, unsigned char *buf, size_t bufLen)
{
    size_t ret;
    if (unlikely(fd == -1))
        ret = -1;
    ret = read(fd, buf, bufLen);
    if(unlikely(ret != bufLen))
        applog(LOG_ERR,"read error!!");
    return ret;

}

static void get_bitmain_statline_before(char *buf, size_t bufsiz, struct cgpu_info *bitmain_L3)
{

}


static void reset_asic()
{
    char rstBuf[128] = "";
    int i = 0;
    for (i = 0; i < BITMAIN_MAX_CHAIN_NUM; ++i )
    {
        sprintf(rstBuf, SET_ASIC_RST_0, g_gpio_data[i]);
        system(rstBuf);
    }

    cgsleep_ms(500);

    for (i = 0; i < BITMAIN_MAX_CHAIN_NUM; ++i )
    {
        sprintf(rstBuf, SET_ASIC_RST_1, g_gpio_data[i]);
        system(rstBuf);
    }
}


static void suffix_string_L3(uint64_t val, char *buf, size_t bufsiz, int sigdigits,bool display)
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
        else */if (val >= giga)
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

void clear_register_value_buf()
{
    pthread_mutex_lock(&reg_mutex);
    reg_fifo.p_wr = 0;
    reg_fifo.p_rd = 0;
    reg_fifo.reg_value_num = 0;
    pthread_mutex_unlock(&reg_mutex);
}


void *bitmain_scanhash(void *arg)
{
    struct thr_info *thr = (struct thr_info*)arg;
    struct cgpu_info *bitmain_L3 = thr->cgpu;
    struct bitmain_L3_info *info = bitmain_L3->device_data;
    struct timeval current;
    bool loged = false;
    uint8_t nonce_bin[4],crc_check,which_asic_nonce;
    uint32_t nonce;
    int submitfull = 0;
    bool submitnonceok = true;

    struct work *work = NULL;
    cgtime(&current);
    h = 0;
    pthread_mutex_lock(&nonce_mutex);
    cg_rlock(&info->update_lock);

    while(nonce_fifo.nonce_num)
    {
        nonce_fifo.nonce_num--;
        crc_check = CRC5((uint8_t *)&(nonce_fifo.nonce_buffer[nonce_fifo.p_rd]), 7*8-5);
        if(crc_check != (nonce_fifo.nonce_buffer[nonce_fifo.p_rd].crc5 & 0x1f))
        {
            applog(LOG_ERR,"crc5 error,should be %02x,but check as %02x",nonce_fifo.nonce_buffer[nonce_fifo.p_rd].crc5 & 0x1f,crc_check);
            applog(LOG_NOTICE,"get nonce %02x%02x%02x%02x wc %02x diff %02x crc5 %02x chainid %02x",nonce_fifo.nonce_buffer[nonce_fifo.p_rd].nonce[0], \
                   nonce_fifo.nonce_buffer[nonce_fifo.p_rd].nonce[1],nonce_fifo.nonce_buffer[nonce_fifo.p_rd].nonce[2], \
                   nonce_fifo.nonce_buffer[nonce_fifo.p_rd].nonce[3],nonce_fifo.nonce_buffer[nonce_fifo.p_rd].diff, \
                   nonce_fifo.nonce_buffer[nonce_fifo.p_rd].wc, nonce_fifo.nonce_buffer[nonce_fifo.p_rd].crc5,    \
                   nonce_fifo.nonce_buffer[nonce_fifo.p_rd].chainid);

            //if signature enabled,check SIG_INFO register
            goto crc_error;
        }
        memcpy(nonce_bin,nonce_fifo.nonce_buffer[nonce_fifo.p_rd].nonce,4);
        uint8_t work_id = nonce_fifo.nonce_buffer[nonce_fifo.p_rd].wc;
        uint8_t chain_id = nonce_fifo.nonce_buffer[nonce_fifo.p_rd].chainid;



        memcpy((uint8_t *)&nonce,nonce_bin,4);
        nonce = htobe32(nonce);

        work = info->work_queue[work_id];
        if(work)
        {
            submitfull = 0;
            if(submit_nonce_1(thr, work, nonce, &submitfull))
            {
                submitnonceok = true;
                submit_nonce_2(work);
            }
            else
            {
                if(submitfull)
                {
                    submitnonceok = true;
                }
                else
                {
                    submitnonceok = false;
                    if ( chain_id > BITMAIN_MAX_CHAIN_NUM ) applog( LOG_ERR, "Chain_ID [%d] Error!", chain_id);
                    dev.chain_hw[chain_id] ++;
                }
            }
            if(submitnonceok)
            {
                which_asic_nonce = (((nonce >> (20)) & 0xff) / dev.addrInterval);
                applog(LOG_DEBUG,"%s: chain %d which_asic_nonce %d ", __FUNCTION__, chain_id, which_asic_nonce);
                if ((chain_id > BITMAIN_MAX_CHAIN_NUM) || (!dev.chain_exist[chain_id]))
                {
                    if(!loged)
                    {
                        applog(LOG_ERR, "ChainID Cause Error! ChainID:[%d]", chain_id);
                        loged = true;
                    }
                    goto crc_error;
                }
                if ( which_asic_nonce >= CHAIN_ASIC_NUM )
                {
                    applog(LOG_ERR, "Which Nonce Cause Err![%d]", which_asic_nonce);
                    goto crc_error;
                }
                h += 0x1UL << DEVICE_DIFF;
                dev.chain_asic_nonce[chain_id][which_asic_nonce]++;

            }
            cg_logwork(work, nonce_bin, submitnonceok);

        }
        else
        {
            applog(LOG_ERR, "%s%d: work %02x not find error", bitmain_L3->drv->name, bitmain_L3->device_id, work_id);
        }
    crc_error:
        if(nonce_fifo.p_rd < MAX_NONCE_NUMBER_IN_FIFO - 1)
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
        applog(LOG_DEBUG,"%s: hashes %"PRIu64"...", __FUNCTION__,h * 0x0000ffffull);
    }

    h = h * 0x0000ffffull;
    return 0;
}

static int64_t bitmain_L3_scanhash(struct thr_info *thr)
{
    h = 0;
    pthread_t send_id;
    pthread_create(&send_id, NULL, bitmain_scanhash, (void*)thr);
    pthread_join(send_id, NULL);

    return h;
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

void set_PWM(unsigned char pwm_percent)
{
    int temp_pwm_percent = 0;
    char buf[128];

    temp_pwm_percent = pwm_percent;
    if(temp_pwm_percent < MIN_PWM_PERCENT)
    {
        temp_pwm_percent = MIN_PWM_PERCENT;
    }

    if(temp_pwm_percent > MAX_PWM_PERCENT)
    {
        temp_pwm_percent = MAX_PWM_PERCENT;
    }
    dev.duty_ns = PWM_PERIOD_NS * temp_pwm_percent /100;
    dev.pwm_percent = temp_pwm_percent;
    applog(LOG_DEBUG,"set pwm duty_ns %d",dev.duty_ns);
    sprintf(buf,PWM_CTRL_TEMPLATE,dev.duty_ns);
    system(buf);
}


void set_PWM_according_to_temperature()
{
    int  pwm_percent = 0, temp_change = 0;
    temp_highest = dev.temp_top1;
    if(temp_highest >= MAX_FAN_TEMP)
    {
        applog(LOG_DEBUG,"%s: Temperature is higher than %d 'C", __FUNCTION__, temp_highest);
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
        applog(LOG_DEBUG,"%s: Set PWM percent : MAX_PWM_PERCENT", __FUNCTION__);
        return;
    }

    if(temp_highest <= MIN_FAN_TEMP)
    {
        set_PWM(MIN_PWM_PERCENT);
        dev.fan_pwm = MIN_PWM_PERCENT;
        applog(LOG_DEBUG,"%s: Set PWM percent : MIN_PWM_PERCENT", __FUNCTION__);
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

static inline void send_pic_command()
{
    uint8_t cmd[2];
    cmd[0] = PIC_COMMAND_1;
    cmd[1] = PIC_COMMAND_2;
    write(dev.i2c_fd,&cmd[0],1);
    cgsleep_us(200);// need ?
    write(dev.i2c_fd,&cmd[1],1);

    cgsleep_us(200);// need ?
}

void pic_dac_ctrl(uint8_t value)
{
    uint8_t cmd[2];
    cmd[0] = ENABLE_VOLTAGE;
    cmd[1] = value;
    send_pic_command();
    write(dev.i2c_fd,&cmd[0],1);
    cgsleep_us(200);

    write(dev.i2c_fd,&cmd[1],1);
    cgsleep_us(200);
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
    FILE* fanpfd = fopen(PROCFILENAME, "r");

    while( 1 )
    {
        fseek(fanpfd, 0, SEEK_SET);
        char buffer[256] = "";
        char* pos = NULL;
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
                fan0Speed = fan0Speed* 60 / 2 / FANINT;
                if ( fan0Speed )
                {
                    fan0_exist = 1;
                    if( fan0Speed > MAX_FAN_SPEED)
                        fan0Speed = MAX_FAN_SPEED;
                }
                else
                {
                    fan0_exist = 0;
                }
                dev.fan_speed_value[0] = fan0Speed;
                fan0SpeedHist = fan0SpeedCur;
                if (dev.fan_speed_top1 <  fan0Speed)
                {
                    dev.fan_speed_top1 = fan0Speed;
                }

            }
            else if (((pos = strstr(buffer, FAN1)) != 0) && (strstr(buffer, "gpiolib") != 0 ))
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
                fan1Speed = fan1Speed* 60 / 2 / FANINT;
                if ( fan1Speed )
                {
                    fan1_exist = 1;
                    if( fan1Speed > MAX_FAN_SPEED)
                        fan1Speed = MAX_FAN_SPEED;
                }
                else
                {
                    fan1_exist = 0;
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

void *check_miner_status(void *arg)
{
    //asic status,fan,led
    struct timeval tv_start = {0, 0}, tv_end,tv_send;
    double ghs = 0;
    int i = 0, j = 0;
    bool loged = false;
    cgtime(&tv_end);

    copy_time(&tv_start, &tv_end);
    copy_time(&tv_send_job,&tv_send);
    bool stop = false;
    int asic_num = 0, error_asic = 0, avg_num = 0;


    while(1)
    {
        struct timeval diff;
        cgtime(&tv_end);
        cgtime(&tv_send);
        timersub(&tv_end, &tv_start, &diff);

        if (diff.tv_sec > 600)
        {
            asic_num = 0, error_asic = 0, avg_num = 0;
            for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
            {
                if(dev.chain_exist[i])
                {
                    asic_num += dev.chain_asic_num[i];
                    for(j=0; j<dev.chain_asic_num[i]; j++)
                    {
                        avg_num += dev.chain_asic_nonce[i][j];
                        applog(LOG_DEBUG,"%s: chain %d asic %d asic_nonce_num %d", __FUNCTION__, i,j,dev.chain_asic_nonce[i][j]);
                    }
                }
            }
            if (asic_num != 0)
            {
                applog(LOG_DEBUG,"%s: avg_num %d asic_num %d", __FUNCTION__, avg_num,asic_num);
                avg_num = avg_num / asic_num / 8;
                avg_num = 0;
            }
            else
            {
                avg_num = 1;
            }
            for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
            {
                if(dev.chain_exist[i])
                {
                    int offset = 0;

                    for(j=0; j<dev.chain_asic_num[i]; j++)
                    {
                        if(j%8 == 0)
                        {
                            if ( ( j + offset ) > (CHAIN_ASIC_NUM + 16) )
                                applog(LOG_ERR, "asic num err![%d]", (j + offset));
                            dev.chain_asic_status_string[i][j+offset] = ' ';
                            offset++;
                        }

                        if(dev.chain_asic_nonce[i][j]>avg_num)
                        {
                            if ( ( j +offset ) > (CHAIN_ASIC_NUM + 16) )
                                applog(LOG_ERR, "asic num err![%d]", (j + offset));
                            dev.chain_asic_status_string[i][j+offset] = 'o';
                        }
                        else
                        {
                            if ( ( j + offset ) > (CHAIN_ASIC_NUM + 16) )
                                applog(LOG_ERR, "asic num err![%d]", (j + offset));
                            dev.chain_asic_status_string[i][j+offset] = 'x';
                            error_asic++;
                        }

                        if ( ( j ) > (CHAIN_ASIC_NUM + 16) )
                            applog(LOG_ERR, "asic num err![%d]", (j));
                        dev.chain_asic_nonce[i][j] = 0;
                    }

                    if ( ( j + offset ) > (CHAIN_ASIC_NUM + 16) )
                        applog(LOG_ERR, "asic num err![%d]", (j + offset));
                    dev.chain_asic_status_string[i][j+offset] = '\0';
                }
            }

            ghs = total_mhashes_done / 1 / total_secs;
            if((ghs < (double)((dev.chain_num * CHAIN_ASIC_NUM * dev.frequency * dev.corenum * 0.95) / 2500 * 0.8)) && (!status_error))
                system("echo \"Rate too low, reboot!!\" >> /usr/bin/already_reboot");
            copy_time(&tv_start, &tv_end);
        }

        //check_fan();
        set_PWM_according_to_temperature();
        timersub(&tv_send, &tv_send_job, &diff);
        if(diff.tv_sec > 120 || dev.temp_top1 > MAX_TEMP)
            //|| dev.fan_num < MIN_FAN_NUM || dev.fan_speed_top1 < (MAX_FAN_SPEED * dev.fan_pwm / 150))
        {
            stop = true;
            if(dev.temp_top1 > MAX_TEMP)
                //|| dev.fan_num < MIN_FAN_NUM || dev.fan_speed_top1 < (MAX_FAN_SPEED * dev.fan_pwm / 150))
            {
                // status_error = true;
                status_error = false;
                once_error = true;

                for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
                {
                    if(dev.chain_exist[i] == 1)
                    {
                        pthread_mutex_lock(&iic_mutex);
                        if(unlikely(ioctl(dev.i2c_fd,I2C_SLAVE,i2c_slave_addr[i] >> 1 ) < 0))
                            applog(LOG_ERR,"ioctl error @ line %d",__LINE__);
                        if(!loged)
                        {
                            applog(LOG_ERR, "Temp Err! Please Check Fan! Will Disable PIC!");
                            loged = true;
                        }
                        pic_dac_ctrl(0);
                        pthread_mutex_unlock(&iic_mutex);
                    }
                }
            }
        }
        else
        {
            stop = false;
            if (!once_error)
                status_error = false;
        }
        set_led(stop);

        cgsleep_ms(1000);
    }
}


static inline int get_nonce_num_in_fifo(int const  fd)
{
    int rx_len;
    if(ioctl(fd, FIONREAD, &rx_len) == 0)
    {
        return rx_len/7;
    }
    else
    {
        perror("ioctl error");
        return 0;
    }

}

void check_asic_reg(unsigned char chip_addr, unsigned char reg_addr, unsigned char mode)
{
    unsigned char rdreg_buf[5] = {0},i;
    rdreg_buf[0] = CMD_TYPE | GET_STATUS;
    if(mode)
        rdreg_buf[0] |= CMD_ALL;
    rdreg_buf[1] = CMD_LENTH;
    rdreg_buf[2] = chip_addr;
    rdreg_buf[3] = reg_addr;
    rdreg_buf[4] = 0;
    rdreg_buf[4] = CRC5(rdreg_buf, 32);

    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(dev.chain_exist[i])
        {
            L3_write(dev.dev_fd[i], rdreg_buf, CMD_LENTH + 1);
        }
        cgsleep_ms(10);
    }
}


void *get_hash_rate()
{
    while(42)
    {
        pthread_mutex_lock(&reg_read_mutex);
        check_asic_reg(0,HASHRATE,1);
        check_rate = true;
        pthread_mutex_unlock(&reg_read_mutex);
        cgsleep_ms(1000);
    }
}


void calibration_sensor_offset();


static int GetResponseResult(const unsigned int AsicNum)
{
    int count = 0;

    while( 42 )
    {
        if ( count++ > 3 )
        {
            return -1;
        }
        check_asic_reg(TempChipAddr[AsicNum], GENERAL_IIC, 0);
        cgsleep_ms(100);
        int i = 0;
        int activedChain = 0;
        int successData = 0;
        for ( i = 0; i < BITMAIN_MAX_CHAIN_NUM; ++i )
        {
            if ( dev.chain_asic_in_full[i] == 1 )
            {
                ++activedChain;
                if ( TempBuffer[i][AsicNum].state == BUF_READY )
                {
                    ++successData;
                }
            }
        }
        if ( activedChain == successData )
        {
            return 0;
        }
    }
}


void *read_temp_func()
{
    while( 42 )
    {

        int i;
        for ( i = 0; i < BITMAIN_REAL_TEMP_CHIP_NUM; ++i )
        {
            check_asic_reg(TempChipAddr[i], EXT_TEMP_SENSOR, 0);
        }

        cgsleep_ms(100);
        unsigned char tmpTemp = 0;
        for ( i = 0; i < BITMAIN_MAX_CHAIN_NUM; ++i )
        {
            if ( dev.chain_exist[i] == 1 )
            {
                if ( dev.chain_asic_temp[i][2][0] > tmpTemp )
                    tmpTemp = dev.chain_asic_temp[i][2][0];
            }
        }
        dev.temp_top1 = tmpTemp;
        sleep(10);
    }
}
void *L3_fill_work(void *usrdata)
{
    pthread_detach(pthread_self());
    applog(LOG_DEBUG, "Start To Fill Work!");
    struct bitmain_L3_info *info = (struct bitmain_L3_info *)usrdata;
    uint8_t chainid = info->chain_index;
    struct thr_info * thr = info->thr;
    struct timeval send_start, last_send, send_elapsed;
    struct work_ltc workdata;
    unsigned char workbuf[80];
    struct work *work = NULL;
    unsigned char workid = 0;
    int sendlen;
    applog(LOG_DEBUG, "Start To Fill Work!ChainIndex:[%d]", chainid);

    while(1)
    {

        if(!start_send)
        {
            cgsleep_ms(10);
            continue;
        }

        cgtime(&send_start);
        timersub(&send_start, &last_send, &send_elapsed);
        if(new_block[chainid] || send_elapsed.tv_sec*1000 + send_elapsed.tv_usec/1000  >= (dev.timeout*1000)/*40000*/ )
        {
            cgtime(&last_send);
        more_work:
            work = get_work(thr, thr->id);
            if (unlikely(!work))
            {
                goto more_work;
            }

            workid = work->id & 0x7f;
            new_block[chainid] = false;
            memset((void *)(&workdata), 0, sizeof(workdata));

            memcpy(workbuf, work->data, 80);
            rev(workbuf, (size_t)80);
            memcpy(workdata.Sdata,workbuf+4,SCRYPTDATA_SIZE);
            workdata.type = 0x01<<5;
            workdata.wc_base = workid;
            workdata.length = 1+1+1+1+ SCRYPTDATA_SIZE;
            sendlen = workdata.length + 2;
            workdata.crc16 = crc_itu_t(0xffff,(uint8_t *) &workdata, sendlen - 2);
            // endian change
            workdata.crc16 = (workdata.crc16 >> 8) | ((workdata.crc16 & 0xff) << 8);
            memcpy((unsigned char *)&workdata + sendlen - 2,&workdata.crc16,2);

            if(info->work_queue[workid])
            {
                free_work(info->work_queue[workid]);
                info->work_queue[workid] = NULL;
            }
            if ( workid >= BITMAIN_MAX_QUEUE_NUM ) applog(LOG_ERR, "WorkID Error![%d]", workid);
            info->work_queue[workid] = copy_work(work);
            applog(LOG_DEBUG, "ChainID[%d] Wirte Work", chainid);
            L3_write(info->dev_fd[chainid], (uint8_t *)&workdata, sendlen);
            // cg_runlock(&info->update_lock);
            gBegin_get_nonce = true;
            hexdump((uint8_t *)&workdata, sendlen);

            cgtime(&tv_send_job);

        }


        cgsleep_ms(10);
        if(work != NULL)
        {
            free_work(work);
        }
    }
}


void *get_asic_response(void* arg)
{
    pthread_detach(pthread_self());

    uint32_t  j, nonce_number, read_loop;
    unsigned char nonce_bin[7],chainid;
    uint32_t fd;

    struct dev_info *dev_i = (struct dev_info*)arg;
    chainid = dev_i->chainid;
    fd = dev_i->dev_fd;

    tcflush(fd, TCIOFLUSH);
    applog(LOG_NOTICE, "Start A New Asic Response.Chain Id:[%d]", chainid);
    applog(LOG_DEBUG, "%s %d",__FUNCTION__,chainid);
    while(1)
    {
        read_loop = 0;
        nonce_number = get_nonce_num_in_fifo(fd);
        //applog(LOG_DEBUG,"%s: response number = %d", __FUNCTION__, nonce_number);
        if(nonce_number)
        {
            read_loop = nonce_number;
            for(j = 0; j < read_loop; j++)
            {
                memset(nonce_bin,0,sizeof(nonce_bin));
                L3_read(fd,nonce_bin,sizeof(nonce_bin));

                applog(LOG_DEBUG, "Chain [%d] Read Data", chainid);
                applog(LOG_DEBUG,"get sth %02x%02x%02x%02x%02x%02x%02x",nonce_bin[0],nonce_bin[1],nonce_bin[2],nonce_bin[3],
                       nonce_bin[4],nonce_bin[5],nonce_bin[6]);
                if((nonce_bin[6] & 0x80) == NONCE_BIT)   //nonce
                {
                    if(gBegin_get_nonce)
                    {

                        pthread_mutex_lock(&nonce_mutex);
                        //      memcpy(&nonce,nonce_bin,4);
                        //      nonce = htobe32(nonce);

#if 1
                        memcpy(nonce_fifo.nonce_buffer[nonce_fifo.p_wr].nonce,nonce_bin,4);
                        nonce_fifo.nonce_buffer[nonce_fifo.p_wr].diff           = nonce_bin[4];
                        nonce_fifo.nonce_buffer[nonce_fifo.p_wr].wc             = nonce_bin[5];
                        nonce_fifo.nonce_buffer[nonce_fifo.p_wr].crc5           = nonce_bin[6];
                        nonce_fifo.nonce_buffer[nonce_fifo.p_wr].chainid        = chainid;
                        applog(LOG_DEBUG,"get nonce %02x%02x%02x%02x wc %02x diff %02x crc5 %02x chainid %02x",nonce_fifo.nonce_buffer[nonce_fifo.p_wr].nonce[0], \
                               nonce_fifo.nonce_buffer[nonce_fifo.p_wr].nonce[1],nonce_fifo.nonce_buffer[nonce_fifo.p_wr].nonce[2], \
                               nonce_fifo.nonce_buffer[nonce_fifo.p_wr].nonce[3],nonce_fifo.nonce_buffer[nonce_fifo.p_wr].diff, \
                               nonce_fifo.nonce_buffer[nonce_fifo.p_wr].wc, nonce_fifo.nonce_buffer[nonce_fifo.p_wr].crc5,    \
                               nonce_fifo.nonce_buffer[nonce_fifo.p_wr].chainid);

                        if(nonce_fifo.p_wr < MAX_NONCE_NUMBER_IN_FIFO - 1)
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
                        }
                        applog(LOG_DEBUG,"get nonce num %d",nonce_fifo.nonce_num);


#endif
                        pthread_mutex_unlock(&nonce_mutex);
                        if((nonce_bin[6] & 0x40) == SIG_BIT)
                        {
                            //signature
                        }
                    }
                }
                else     //reg value
                {
                    if(reg_fifo.reg_value_num >= MAX_NONCE_NUMBER_IN_FIFO || reg_fifo.p_wr >= MAX_NONCE_NUMBER_IN_FIFO)
                    {
                        applog(LOG_DEBUG, "Will Clean!");
                        clear_register_value_buf();
                        continue;
                    }
                    pthread_mutex_lock(&reg_mutex);

                    memcpy(reg_fifo.reg_buffer[reg_fifo.p_wr].reg_value,nonce_bin,4);
                    reg_fifo.reg_buffer[reg_fifo.p_wr].chipaddr      = nonce_bin[4];
                    reg_fifo.reg_buffer[reg_fifo.p_wr].regaddr       = nonce_bin[5];
                    reg_fifo.reg_buffer[reg_fifo.p_wr].crc5          = nonce_bin[6];

                    reg_fifo.reg_buffer[reg_fifo.p_wr].chainid = chainid;

                    if(reg_fifo.p_wr < MAX_NONCE_NUMBER_IN_FIFO - 1)
                    {
                        applog(LOG_DEBUG,"%s: p_wr = %d reg_value_num = %d", __FUNCTION__,reg_fifo.p_wr,reg_fifo.reg_value_num);
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
                    }
                    //applog(LOG_NOTICE,"%s: p_wr = %d reg_value_num = %d\n", __FUNCTION__,reg_fifo.p_wr,reg_fifo.reg_value_num);
                    pthread_mutex_unlock(&reg_mutex);
                }
            }
        }
        cgsleep_ms(1);
    }
}


void check_chain(struct bitmain_L3_info *info)
{
    int i,fd,ret;
    char dev_fname[PATH_MAX],command[2];
    for(i = 0; i < sizeof(plug)/sizeof(int); i++)
    {
        sprintf(dev_fname,GPIO_DEVICE_TEMPLATE,plug[i]);
        if((fd = open(dev_fname, O_RDONLY)) < 0)
        {
            applog(LOG_ERR,"%s :open %s failed",__FUNCTION__,dev_fname);
        }


        if(lseek(fd, 0, SEEK_SET) < 0)
            perror(dev_fname);
        ret = read(fd, command, 2);

        if(ret > 0 && command[0] == '1')
        {
            info->chain_num ++;
            info->chain_status[i] = 1;
            dev.chain_exist[i] = 1;
            dev.chain_num++;
            applog(LOG_NOTICE," detected at %s  chain %d",dev_fname,i);
        }
        else
        {
            info->chain_status[i] = 0;
            dev.chain_exist[i] = 0;
        }

    }

    applog(LOG_NOTICE,"detect total chain num %d",info->chain_num);
}

void tty_init(struct bitmain_L3_info *info, int baud)
{
    int i,ret;
    char dev_fname[PATH_MAX] = "";

    struct termios options;
    applog(LOG_NOTICE,"in %s",__FUNCTION__);
    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(info->chain_status[i] == 1)
        {

            sprintf(dev_fname,TTY_DEVICE_TEMPLATE,tty[i]);
            info->dev_fd[i] = open(dev_fname,O_RDWR|O_NOCTTY);
            if(info->dev_fd[i] < 0)
            {
                applog(LOG_ERR, "%s : open %s failed",__FUNCTION__,dev_fname);
            }
            tcgetattr(info->dev_fd[i],&options);
            speed_t speed = tiospeed_t(baud);
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
            options.c_cc[VMIN] = 7;
            tcsetattr(info->dev_fd[i],TCSANOW,&options);

            tcflush(info->dev_fd[i], TCIOFLUSH);

            info->chain_index = dev_info[i].chainid = i;
            dev_info[i].dev_fd = info->dev_fd[i];
            applog(LOG_NOTICE, "%s %d",__FUNCTION__,dev_info[i].chainid);
            ret = pthread_create(&info->uart_rx_t[i],NULL,get_asic_response,(void *)&dev_info[i]);
            if(unlikely(ret != 0))
                applog(LOG_ERR,"create rx read thread for %s failed",dev_fname);

            applog(LOG_DEBUG, "Will Create Pthread @ ChainId[%d]\r", info->chain_index);
            ret = pthread_create(&info->uart_tx_t[i],NULL,L3_fill_work,(void *)info);
            cgsleep_ms(200);
            if(unlikely(ret != 0))
                applog(LOG_ERR,"create tx read thread for %s failed",dev_fname);

        }
    }
    applog(LOG_NOTICE,"open device over");
    int ee = 0;
    for ( ; ee < BITMAIN_MAX_CHAIN_NUM; ++ee )
    {
        dev.dev_fd[ee] = info->dev_fd[ee];
    }
}

void set_config(int fd,unsigned char mode, unsigned char asic_addr, unsigned char reg_addr, union REG_DATA reg_data)
{
    unsigned char cmd_buf[9] = {0};

    cmd_buf[0] = CMD_TYPE | SET_CONFIG;
    if(mode)
        cmd_buf[0] |= CMD_ALL;

    cmd_buf[1] = CONFIG_LENTH;
    cmd_buf[2] = asic_addr;
    cmd_buf[3] = reg_addr;
    if(reg_addr == MISC_CONTROL)
    {
        memcpy(&cmd_buf[4],(unsigned char *)&(reg_data.misc_ctrl_data),4);
    }
    else if(reg_addr == GENERAL_IIC)
    {
        memcpy(&cmd_buf[4],(unsigned char *)&(reg_data.general_iic_data),4);
    }
    else if(reg_addr == SECURITY_IIC)
    {
        memcpy(&cmd_buf[4],(unsigned char *)&(reg_data.security_iic_data),4);
    }
    else if(reg_addr == SEC_CTRL_STATUS)
    {
        memcpy(&cmd_buf[4],(unsigned char *)&(reg_data.scs_data),4);
    }
    else if(reg_addr == CORE_CMD_IN)
    {
        memcpy(&cmd_buf[4],(unsigned char *)&(reg_data.core_cmd_data),4);
    }
    else if(reg_addr == TICKET_MASK)
    {
        memcpy(&cmd_buf[4],(unsigned char *)&(reg_data.tm_data),4);
    }
    else if(reg_addr == HCN)
    {
        memcpy(&cmd_buf[4],(unsigned char *)&(reg_data.hcn_data),4);
    }
    else if(reg_addr == SNO)
    {
        memcpy(&cmd_buf[4],(unsigned char *)&(reg_data.sno_data),4);
    }
    else if(reg_addr == PLL_PARAMETER)
    {
        memcpy(&cmd_buf[4],(unsigned char *)&(reg_data.pll_data),4);
    }


    cmd_buf[8] = CRC5(cmd_buf, 8*8);

    applog(LOG_DEBUG, "Set config reg %02x : %02x%02x%02x%02x%02x%02x%02x%02x%02x" ,reg_addr,cmd_buf[0], cmd_buf[1], cmd_buf[2],
           cmd_buf[3],cmd_buf[4], cmd_buf[5], cmd_buf[6], cmd_buf[7],cmd_buf[8]);
    L3_write(fd, cmd_buf, CONFIG_LENTH + 1);
}

void chain_inactive(int const chain_index)
{
    int fd = dev.dev_fd[chain_index];
    unsigned char cmd_buf[5] = {0};
    cmd_buf[0] = CMD_ALL | CMD_TYPE | CHAIN_INACTIVE;
    cmd_buf[1] = CMD_LENTH ;
    cmd_buf[4] = CRC5(cmd_buf, 4*8 );
    L3_write(fd, cmd_buf, CMD_LENTH + 1);
}

inline void set_address(int const chain_index, unsigned char chip_addr)
{
    int fd = dev.dev_fd[chain_index];

    unsigned char cmd_buf[5] = {0};
    cmd_buf[0] = CMD_TYPE | SET_ADDR;
    cmd_buf[1] = CMD_LENTH;
    cmd_buf[2] = chip_addr;
    cmd_buf[4] = CRC5(cmd_buf, 4*8);
    L3_write(fd, cmd_buf, CMD_LENTH + 1);
}

void software_set_address()
{
    int temp_asic_number = 0;
    unsigned int i, j;
    unsigned char chip_addr = 0;
    unsigned char check_bit = 0;

    temp_asic_number = dev.max_asic_num_in_one_chain;
    if(temp_asic_number <= 0)
    {
        dev.addrInterval = 7;
        return;
    }

    dev.addrInterval = 0x100 / temp_asic_number;
    //dev.addrInterval = 4;
    applog(LOG_NOTICE, "addrInterval = '%d'", dev.addrInterval);
    check_bit = dev.addrInterval - 1;
    while(check_bit)
    {
        check_bit = check_bit >> 1;
        dev.check_bit++;
    }
    applog(LOG_DEBUG,"--- %s interval %d", __FUNCTION__,dev.addrInterval);

    for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
    {
        applog(LOG_NOTICE,"%s: chain %d has %d ASIC, and addrInterval is %d", __FUNCTION__, i, dev.chain_asic_num[i], dev.addrInterval);
        if(dev.chain_exist[i] == 1 /*&& dev.chain_asic_num[i] == CHAIN_ASIC_NUM*/)
        {
            chip_addr = 0;

            chain_inactive(i);
            cgsleep_ms(30);
            applog(LOG_NOTICE, "Now Set [%d] Chain Address", i);
            for(j = 0; j < 0x100/dev.addrInterval; j++)
            {
                set_address(i, chip_addr);
                chip_addr += dev.addrInterval;
                cgsleep_ms(10);
            }
        }
    }
}

void set_core_ctrl(unsigned char cmd_type, unsigned char mode, unsigned char cmd_data)
{
    int i,j,core_index;
    union REG_DATA regdata;

    regdata.core_cmd_data.all = mode;
    regdata.core_cmd_data.core_index = 0;
    regdata.core_cmd_data.cmd_type = cmd_type;
    regdata.core_cmd_data.cmd_data = cmd_data;

    for(j = 0; j < BITMAIN_MAX_CHAIN_NUM; j++)
    {
        if(dev.chain_exist[j] == 1)
        {
            for(i = 0; i < 12; i++)
            {
                core_index = i;
                if(i > 5)
                    core_index = i + 2;
                regdata.core_cmd_data.core_index = core_index;
                set_config(dev.dev_fd[j], 1, 0, CORE_CMD_IN, regdata);
                cgsleep_ms(10);
            }
        }
    }
}

void set_core_temp_ctrl(unsigned char cmd_type, uint8_t core_index, unsigned char mode, unsigned char cmd_data)
{
    int i,j;
    union REG_DATA regdata;

    regdata.core_cmd_data.all = mode;
    regdata.core_cmd_data.core_index = core_index;
    regdata.core_cmd_data.cmd_type = cmd_type;
    regdata.core_cmd_data.cmd_data = cmd_data;
    applog(LOG_NOTICE,"opt_bitmain_L3_core_temp : %x",opt_bitmain_L3_core_temp);

    for(j = 0; j < BITMAIN_MAX_CHAIN_NUM; j++)
    {
        if(dev.chain_exist[j] == 1)
        {
            for ( i = 0; i < BITMAIN_REAL_TEMP_CHIP_NUM; ++i )
            {
                set_config(dev.dev_fd[j], 0, TempChipAddr[i], CORE_CMD_IN, regdata);
                cgsleep_ms(1);
            }
        }
    }
}


void set_auto_read_temp()
{
    union REG_DATA regdata;
    memset(&regdata, 0, sizeof( regdata ));
    regdata.general_iic_data.rw_ctrl = 0x0;
    regdata.general_iic_data.regaddrvalid = 0x1;
    regdata.general_iic_data.deviceaddr = 0x4c;
    regdata.general_iic_data.regaddr = 0x00;
    regdata.general_iic_data.autoreadtemp = 0x01;
    int i , j;
    for ( i = 0; i < BITMAIN_MAX_CHAIN_NUM; ++i )
    {
        if ( dev.chain_exist[i] == 1 )
        {
            for ( j = 0; j < BITMAIN_REAL_TEMP_CHIP_NUM; ++j )
            {
                set_config(dev.dev_fd[i], 0, TempChipAddr[j], GENERAL_IIC, regdata);
                TempBuffer[i][j].state = BUF_IDLE;
                usleep(200);
            }
        }
    }
}

void SetTempRead(unsigned char pos)
{
    union REG_DATA regdata;
    memset(&regdata, 0, sizeof( regdata ));
    regdata.general_iic_data.rw_ctrl = 0x0;
    regdata.general_iic_data.regaddrvalid = 0x1;
    regdata.general_iic_data.deviceaddr = 0x4c;
    regdata.general_iic_data.regaddr = pos;
    regdata.general_iic_data.autoreadtemp = 0x00;
    int i , j;
    for ( i = 0; i < BITMAIN_MAX_CHAIN_NUM; ++i )
    {
        if ( dev.chain_exist[i] == 1 )
        {
            for (j = 0 ; j < BITMAIN_REAL_TEMP_CHIP_NUM; ++j )
            {
                set_config(dev.dev_fd[i], 0, TempChipAddr[j], GENERAL_IIC, regdata);
                TempBuffer[i][j].state = BUF_IDLE;
                usleep(200);
            }
        }
    }
    cgsleep_ms(IIC_SLEEP);
}


void set_misc_ctrl()
{
    union REG_DATA regdata;
    memset(&regdata, 0, sizeof( regdata));
    regdata.misc_ctrl_data.hashratrectrl1 = (1);
    regdata.misc_ctrl_data.hashratrectrl2 = (4);
    regdata.misc_ctrl_data.ldo18ctrl = (4);
    regdata.misc_ctrl_data.inv_clko = (1);
    regdata.misc_ctrl_data.bt8d = 26;
    regdata.misc_ctrl_data.ldo09ctrl = (4);
    regdata.misc_ctrl_data.ldo09_pd = 0;

    int i = 0;
    for ( ; i < BITMAIN_MAX_CHAIN_NUM; ++i )
    {
        if ( dev.chain_exist[i] == 1 )
        {
            set_config(dev.dev_fd[i], 1, 0, MISC_CONTROL, regdata);
            cgsleep_ms(2);
        }
    }

    regdata.misc_ctrl_data.tfs = (3);
    regdata.misc_ctrl_data.rfs = (1);
    i = 0;
    int j = 0;
    for ( ; i < BITMAIN_MAX_CHAIN_NUM; ++i )
    {
        if ( dev.chain_exist[i] == 1)
        {

            for (j = 0 ; j < BITMAIN_REAL_TEMP_CHIP_NUM; ++j)
            {
                set_config(dev.dev_fd[i], 0, TempChipAddr[j], MISC_CONTROL, regdata);
                cgsleep_ms(2);
            }
        }
    }
    cgsleep_ms(100);
    check_asic_reg(0, MISC_CONTROL, 1);

}

static void get_plldata(int type,int freq, uint8_t *vil_data)
{
    uint32_t i;
    char vildivider[32] = {0};

    if(type == 1485)
    {
        for(i = 0; i < sizeof(freq_pll_1485)/sizeof(freq_pll_1485[0]); i++)
        {
            if(freq_pll_1485[i].freq == freq)
                break;
        }
    }

    if(i == sizeof(freq_pll_1485)/sizeof(freq_pll_1485[0]))
    {
        applog(LOG_WARNING,"error freq,set default instead");
        i = 4;
    }


    sprintf(vildivider, "%08x", freq_pll_1485[i].vilpll);
    applog(LOG_DEBUG, "regdata: %s", vildivider);
    if(!hex2bin(vil_data, vildivider, strlen(vildivider)/2))
    {
        quit(1, "Invalid vil plldata for reg data, hex2bin error now: %s",
             vildivider);
    }
}

static void get_plldata_i(int type,int freq, uint8_t *vil_data)
{
    int i;
    char vildivider[32] = {0};

    i = freq;

    sprintf(vildivider, "%08x", freq_pll_1485[i].vilpll);
    applog(LOG_DEBUG, "regdata: %s", vildivider);
    if(!hex2bin(vil_data, vildivider, strlen(vildivider)/2))
    {
        quit(1, "Invalid vil plldata for reg data, hex2bin error now: %s",
             vildivider);
    }
}


void set_frequency(int frequency)
{
    unsigned char i;
    uint8_t reg_data_vil[4] = {0};

    union REG_DATA regdata;

    get_plldata(1485, frequency, reg_data_vil);
    applog(LOG_DEBUG,"%s: frequency = %d", __FUNCTION__, frequency);
    memcpy(regdata.pll_data.reg_data,reg_data_vil,4);

    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(dev.chain_exist[i] == 1)
        {
            set_config(dev.dev_fd[i],1,0,PLL_PARAMETER, regdata);
            dev.freq[i] = frequency;
            cgsleep_us(10000);

        }
    }
}

void set_frequency_i(int frequency)
{
    unsigned char i;
    uint8_t reg_data_vil[4] = {0};

    union REG_DATA regdata;

    get_plldata_i(1485, frequency, reg_data_vil);
    applog(LOG_DEBUG,"%s: frequency = %d", __FUNCTION__, frequency);
    memcpy(regdata.pll_data.reg_data,reg_data_vil,4);

    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(dev.chain_exist[i] == 1)
        {
            set_config(dev.dev_fd[i],1,0,PLL_PARAMETER, regdata);
            dev.freq[i] = frequency;
            cgsleep_us(10000);

        }
    }
}


void set_frequency_with_addr_i(int chain, int frequency, unsigned char addr)
{
    int i;
    uint8_t reg_data_vil[4] = {0};
    i = chain;

    union REG_DATA regdata;

    get_plldata_i(1485, frequency, reg_data_vil);
    applog(LOG_DEBUG,"%s: frequency = %d", __FUNCTION__, frequency);
    memcpy(regdata.pll_data.reg_data,reg_data_vil,4);

    set_config(dev.dev_fd[i],0,addr,PLL_PARAMETER, regdata);
    cgsleep_ms(2);
}


static void SetTempOffset(int8_t offset, int asic_num, int chainid, bool if_auto)
{
    union REG_DATA regdata;
    memset(&regdata, 0, sizeof( regdata ));
    regdata.general_iic_data.rw_ctrl = 0x1;
    regdata.general_iic_data.regaddrvalid = 0x1;
    regdata.general_iic_data.deviceaddr = 0x4c;
    regdata.general_iic_data.regaddr = 0x11;
    regdata.general_iic_data.autoreadtemp = 0x00;
    regdata.general_iic_data.data = offset & 0xff;
    set_config(dev.dev_fd[chainid], 0, TempChipAddr[asic_num], GENERAL_IIC, regdata);
    usleep(2000);
}

static void SetAutoReadTemp(int asic_num, int chainid)
{
    union REG_DATA regdata;
    memset(&regdata, 0, sizeof( regdata ));
    regdata.general_iic_data.rw_ctrl = 0x0;
    regdata.general_iic_data.regaddrvalid = 0x0;
    regdata.general_iic_data.deviceaddr = 0x4c;
    regdata.general_iic_data.regaddr = 0x00;
    regdata.general_iic_data.autoreadtemp = 0x01;
    regdata.general_iic_data.data = 0x00;
    set_config(dev.dev_fd[chainid], 0, TempChipAddr[asic_num], GENERAL_IIC, regdata);
    usleep(2000);
}


void calibration_sensor_offset()
{
    int i , j , rightFlag;
    int8_t localTemp[BITMAIN_MAX_CHAIN_NUM][BITMAIN_REAL_TEMP_CHIP_NUM] = {{0}};
    int8_t remoteTemp[BITMAIN_MAX_CHAIN_NUM][BITMAIN_REAL_TEMP_CHIP_NUM] = {{0}};
    int8_t already_right[BITMAIN_MAX_CHAIN_NUM][BITMAIN_REAL_TEMP_CHIP_NUM] = {{0}};

    set_core_temp_ctrl(TEMP_DIODE_SEL,opt_bitmain_L3_core_temp,0x0,0x0);

    int8_t m_offset[BITMAIN_MAX_CHAIN_NUM][BITMAIN_REAL_TEMP_CHIP_NUM] = {{0}};

    for ( i = 0; i < BITMAIN_MAX_CHAIN_NUM; ++i )
    {
        for ( j = 0; j < BITMAIN_REAL_TEMP_CHIP_NUM; ++j )
        {
            m_offset[i][j] = 0xd8;
        }
    }
    int error_Limit = 0;
    int retryTime = 0;

    int activedChain = 0;

    for ( i = 0; i < BITMAIN_MAX_CHAIN_NUM; ++i )
    {
        if ( dev.chain_asic_in_full[i] == 1)
        {
            ++activedChain;
            for ( j = 0; j < BITMAIN_REAL_TEMP_CHIP_NUM; ++j )
            {
                SetTempOffset(m_offset[i][j], j, i,false);
            }
        }
    }
    cgsleep_ms(IIC_SLEEP);

READONCEMORE:
    rightFlag = 0;
    {
        if (retryTime++ > 10) return ;

        SetTempRead(0x0);

        for ( i = 0; i < BITMAIN_REAL_TEMP_CHIP_NUM; ++i )
        {
            if ( GetResponseResult(i) == 0 )
            {
                for (j = 0 ; j < BITMAIN_MAX_CHAIN_NUM; ++j )
                {
                    if ( dev.chain_asic_in_full[j] == 1)
                    {
                        localTemp[j][i] = TempBuffer[j][i].data;
                        applog(LOG_DEBUG, "Chain %d chip %d LocalTemp 0x%x ", j, i, localTemp[j][i]);
                        TempBuffer[j][i].state = BUF_IDLE;
                    }
                }
            }
            else
            {
                applog(LOG_ERR, "Get [%d]Temp Data Failed!", i);
            }
            cgsleep_ms(2);
        }

        SetTempRead(0x1);

        for ( i = 0; i < BITMAIN_REAL_TEMP_CHIP_NUM; ++i )
        {
            if ( GetResponseResult(i) == 0 )
            {
                for (j = 0 ; j < BITMAIN_MAX_CHAIN_NUM; ++j )
                {
                    if ( dev.chain_asic_in_full[j] == 1)
                    {
                        remoteTemp[j][i] = TempBuffer[j][i].data;
                        TempBuffer[j][i].state = BUF_IDLE;
                        applog(LOG_NOTICE, "Chain %d chip %d RemoteTemp 0x%x", j, i, remoteTemp[j][i]);
                    }
                }
            }
            else
            {
                applog(LOG_DEBUG, "Get [%d]Temp Data Failed!", i);
            }
            cgsleep_ms(2);
        }

        for ( i = 0; i < BITMAIN_MAX_CHAIN_NUM; ++i )
        {
            if ( dev.chain_asic_in_full[i] == 1 )
            {
                for ( j = 0; j < BITMAIN_REAL_TEMP_CHIP_NUM; ++j )
                {
                    error_Limit = abs(remoteTemp[i][j] - localTemp[i][j]);
                    if(remoteTemp[i][j]==0 && already_right[i][j] != true)
                    {
                        applog(LOG_NOTICE, "Remote = 0 Chain %d chip %d local 0x%x remote 0x%x offset 0x%x ", i, j,localTemp[i][j],remoteTemp[i][j], m_offset[i][j]);
                        m_offset[i][j] += 30;   // -70 default is out of temp value.
                        SetTempOffset( m_offset[i][j], j, i ,false);
                    }
                    else if ( error_Limit > 2 )
                    {
                        applog(LOG_NOTICE, "2 Chain %d chip %d local 0x%x remote 0x%x offset 0x%x ", i, j,localTemp[i][j],remoteTemp[i][j], m_offset[i][j]);
                        m_offset[i][j] = m_offset[i][j] + (localTemp[i][j] - remoteTemp[i][j]);
                        SetTempOffset( m_offset[i][j], j, i,false);
                    }
                    else
                    {
                        applog(LOG_NOTICE, "OK Chain %d chip %d local 0x%x remote 0x%x offset 0x%x ", i, j,localTemp[i][j],remoteTemp[i][j], m_offset[i][j]);
                        if(already_right[i][j] != true)
                        {
                            ++rightFlag;
                            already_right[i][j] = true;
                        }
                    }
                }
            }
        }
        cgsleep_ms(IIC_SLEEP);
        SetTempRead(0x11);
        check_asic_reg(TempChipAddr[0], GENERAL_IIC, 0);
    }
    if (rightFlag == activedChain * BITMAIN_REAL_TEMP_CHIP_NUM)
    {
        return ;
    }
    else
        goto READONCEMORE;
}



void set_asic_ticket_mask(unsigned int ticket_mask)
{
    unsigned int tm,i;

    tm = Swap32(ticket_mask);
    applog(LOG_DEBUG,"%s tm=%d",__FUNCTION__,tm);
    union REG_DATA regdata;
    memcpy(regdata.tm_data.reg_data,&tm,4);

    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(dev.chain_exist[i] == 1)
        {
            set_config(dev.dev_fd[i], 1, 0, TICKET_MASK, regdata);
            cgsleep_ms(1);
        }
    }
}



void get_data_from_pic_iic(unsigned char command, unsigned char *buf, unsigned char length)
{
    unsigned char i=0;
    uint8_t cmd[2];
    cmd[0] = command;
    write(dev.i2c_fd,&cmd[0],1);
    cgsleep_ms(100);
    for(i = 0; i < length; i++)
    {
        read(dev.i2c_fd, buf+i, 1);
        cgsleep_ms(100);
    }
}

void get_hash_board_id_number(unsigned char *id)
{
    send_pic_command();
    get_data_from_pic_iic(READ_HASH_BOARD_ID, id, 12);
}
static inline void pic_heart_beat_each_chain()
{
    send_pic_command();
    uint8_t cmd[2];
    cmd[0] = SEND_HEART_BEAT;
    write(dev.i2c_fd,&cmd[0],1);
}

void *pic_heart_beat_func()
{
    int i;
    while(1)
    {
        for(i=0; i<BITMAIN_MAX_CHAIN_NUM; i++)
        {
            if(dev.chain_exist[i] && send_heart)
            {
                pthread_mutex_lock(&iic_mutex);
                if(unlikely(ioctl(dev.i2c_fd,I2C_SLAVE,i2c_slave_addr[i] >> 1 ) < 0))
                    applog(LOG_ERR," %d ioctl error in %s",i,__FUNCTION__);
                pic_heart_beat_each_chain();
                pthread_mutex_unlock(&iic_mutex);
                cgsleep_ms(10);
            }
        }
        sleep(HEART_BEAT_TIME_GAP);
    }
}

unsigned char jump_to_app_from_loader()
{
    uint8_t cmd[2];
    cmd[0] = JUMP_FROM_LOADER_TO_APP;
    send_pic_command();
    write(dev.i2c_fd,&cmd[0],1);
    cgsleep_us(100000);
    return 0;
}

unsigned char reset_iic_pic()
{
    uint8_t cmd[2];
    cmd[0] = RESET_PIC;

    send_pic_command();
    write(dev.i2c_fd,&cmd[0],1);
    cgsleep_us(100000);
    return 0;
}



void pic_send_command(void)
{
    //printf("--- %s\n", __FUNCTION__);
    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_command_1, 1);
    write(dev.i2c_fd, Pic_command_2, 1);
    pthread_mutex_unlock(&i2c_mutex);
}

void pic_set_flash_pointer(unsigned char flash_addr_h, unsigned char flash_addr_l)
{
    pic_send_command();

    //printf("\n--- %s\n", __FUNCTION__);
    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_set_flash_pointer, 1);
    write(dev.i2c_fd, &flash_addr_h, 1);
    write(dev.i2c_fd, &flash_addr_l, 1);
    pthread_mutex_unlock(&i2c_mutex);
    //usleep(500000);
}

void pic_read_flash_pointer(unsigned char *read_back_flash_addr_h, unsigned char *read_back_flash_addr_l)
{
    pic_send_command();

    //printf("\n--- %s\n", __FUNCTION__);
    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_read_flash_pointer, 1);
    read(dev.i2c_fd, read_back_flash_addr_h, 1);
    read(dev.i2c_fd, read_back_flash_addr_l, 1);
    pthread_mutex_unlock(&i2c_mutex);
}

void pic_send_data_to_pic(unsigned char *data)
{
    unsigned char i = 0;

    pic_send_command();

    //printf("\n--- %s\n", __FUNCTION__);
    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_send_data_to_pic, 1);
    for(i=0; i<16; i++)
    {
        write(dev.i2c_fd, data + i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);
}

void pic_write_data_into_flash(void)
{
    pic_send_command();

    //printf("\n--- %s\n", __FUNCTION__);
    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_write_data_into_flash, 1);
    pthread_mutex_unlock(&i2c_mutex);
    usleep(500*1000);
}

void pic_erase_pic_flash(void)
{
    pic_send_command();

    //printf("\n--- %s\n", __FUNCTION__);
    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_erase_pic_flash, 1);
    pthread_mutex_unlock(&i2c_mutex);
    usleep(500*1000);
}

void pic_read_data_from_pic_flash(unsigned char *data)
{
    unsigned char i = 0;

    pic_send_command();

    //printf("\n--- %s\n", __FUNCTION__);
    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_read_data_from_pic_flash, 1);
    for(i=0; i<16; i++)
    {
        read(dev.i2c_fd, data + i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);
}

void pic_reset(void)
{
    pic_send_command();

    printf("\n--- %s\n", __FUNCTION__);
    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_reset, 1);
    pthread_mutex_unlock(&i2c_mutex);
    usleep(600*1000);
}

void pic_jump_from_loader_to_app(void)
{
    pic_send_command();

    //printf("\n--- %s\n", __FUNCTION__);
    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_jump_from_loader_to_app, 1);
    pthread_mutex_unlock(&i2c_mutex);
    usleep(500000);
}

void pic_set_temperature_offset_value(unsigned char *value)
{
    unsigned char i=0;

    pic_send_command();

    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_set_temperature_offset_value, 1);
    for(i=0; i<8; i++)
    {
        write(dev.i2c_fd, value + i, 1);
        usleep(5*1000);
    }
    pthread_mutex_unlock(&i2c_mutex);

    usleep(500*1000);
}

void pic_get_temperature_offset_value(unsigned char *value)
{
    unsigned char i=0;

    pic_send_command();

    //printf("\n--- %s\n", __FUNCTION__);

    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_get_temperature_offset_value, 1);
    for(i=0; i<8; i++)
    {
        read(dev.i2c_fd, value + i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    usleep(500000);
}

void pic_set_voltage_setting_time(unsigned char *time)
{
    unsigned char i = 0;

    pic_send_command();

    //printf("\n--- %s\n", __FUNCTION__);

    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_set_voltage_setting_time, 1);
    for(i=0; i<6; i++)
    {
        write(dev.i2c_fd, time+i, 1);
        usleep(5*1000);
    }
    pthread_mutex_unlock(&i2c_mutex);

    usleep(500000);
}

void pic_read_voltage_setting_time(unsigned char *time)
{
    unsigned char i = 0;

    pic_send_command();

    //printf("\n--- %s\n", __FUNCTION__);

    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_read_voltage_setting_time, 1);
    for(i=0; i<6; i++)
    {
        read(dev.i2c_fd, time+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    usleep(500000);
}


void pic_set_hash_board_id_number(unsigned char *id)
{
    unsigned char i=0;

    pic_send_command();

    //printf("\n--- %s\n", __FUNCTION__);

    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_set_hash_board_id_number, 1);
    for(i=0; i<12; i++)
    {
        write(dev.i2c_fd, id+i, 1);
        usleep(5*1000);
    }
    pthread_mutex_unlock(&i2c_mutex);

    usleep(500000);
}

void pic_read_hash_board_id_number(unsigned char *id)
{
    unsigned char i=0;

    pic_send_command();

    //printf("\n--- %s\n", __FUNCTION__);

    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_read_hash_board_id_number, 1);
    for(i=0; i<12; i++)
    {
        read(dev.i2c_fd, id+i, 1);
    }
    pthread_mutex_unlock(&i2c_mutex);

    usleep(500000);
}

void pic_write_host_MAC_and_time(unsigned char *mac)
{
    unsigned char i=0;

    pic_send_command();

    //printf("\n--- %s\n", __FUNCTION__);

    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_set_host_mac_address, 1);
    for(i=0; i<12; i++)
    {
        write(dev.i2c_fd, mac+i, 1);
        //usleep(5*1000);
    }
    pthread_mutex_unlock(&i2c_mutex);

    usleep(500000);
}

void pic_read_host_MAC_and_time(unsigned char *mac, unsigned char *which_mac)
{
    unsigned char i=0;

    //printf("\n--- %s\n", __FUNCTION__);
    pic_send_command();
    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_read_which_mac, 1);
    write(dev.i2c_fd, which_mac, 1);
    pthread_mutex_unlock(&i2c_mutex);

    pic_send_command();
    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_read_mac, 1);
    for(i=0; i<16; i++)
    {
        read(dev.i2c_fd, mac+i, 1);
        //printf("mac[%02d] = 0x%02x, ", i, *(mac+i));
    }
    //printf("\n");
    pthread_mutex_unlock(&i2c_mutex);

    usleep(500000);
}



void pic_enable(void)
{
    unsigned char value = 1;

    pic_send_command();

    //printf("\n--- %s\n", __FUNCTION__);

    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_enable, 1);
    write(dev.i2c_fd, &value, 1);
    pthread_mutex_unlock(&i2c_mutex);
}

void pic_disable(void)
{
    unsigned char value = 0;

    pic_send_command();

    //printf("\n--- %s\n", __FUNCTION__);

    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_enable, 1);
    write(dev.i2c_fd, &value, 1);
    pthread_mutex_unlock(&i2c_mutex);
}

void pic_read_pic_software_version(unsigned char *version)
{
    pic_send_command();

    //printf("\n--- %s\n", __FUNCTION__);
    pthread_mutex_lock(&i2c_mutex);
    write(dev.i2c_fd, Pic_read_pic_software_version, 1);
    read(dev.i2c_fd, version, 1);
    pthread_mutex_unlock(&i2c_mutex);
}

unsigned char pic_erase_flash_all(void)
{
    unsigned char ret=0xff;
    unsigned int i=0, erase_loop = 0;
    unsigned char start_addr_h = PIC_FLASH_POINTER_START_ADDRESS_H, start_addr_l = PIC_FLASH_POINTER_START_ADDRESS_L;
    unsigned char end_addr_h = PIC_FLASH_POINTER_END_ADDRESS_H, end_addr_l = PIC_FLASH_POINTER_END_ADDRESS_L;
    unsigned int pic_flash_length=0;

    pic_set_flash_pointer(PIC_FLASH_POINTER_START_ADDRESS_H, PIC_FLASH_POINTER_START_ADDRESS_L);

    pic_flash_length = (((unsigned int)end_addr_h << 8) + end_addr_l) - (((unsigned int)start_addr_h << 8) + start_addr_l) + 1;
    erase_loop = pic_flash_length/PIC_FLASH_SECTOR_LENGTH;
    printf("%s: erase_loop = %d\n", __FUNCTION__, erase_loop);

    for(i=0; i<erase_loop; i++)
    {
        pic_erase_pic_flash();
    }
}

void update_pic_program(void)
{
    unsigned char program_data[10*MAX_CHAR_NUM] = {0};
    FILE * pic_program_file;
    unsigned int filesize = 0,i=0,j;
    unsigned char data_read[5]= {0,0,0,0,'\0'}, buf[16]= {0};
    unsigned int data_int = 0;
    struct stat statbuff;
    unsigned char start_addr_h = PIC_FLASH_POINTER_START_ADDRESS_H, start_addr_l = PIC_FLASH_POINTER_START_ADDRESS_L;
    unsigned char end_addr_h = PIC_FLASH_POINTER_END_ADDRESS_H, end_addr_l = PIC_FLASH_POINTER_END_ADDRESS_L;
    unsigned int pic_flash_length=0;

    printf("\n--- update pic program\n");

    // read upgrade file first, if it is wrong, don't erase pic, but just return;
    pic_program_file = fopen(PIC_PROGRAM, "r");
    if(!pic_program_file)
    {
        printf("\n%s: open hash_s8_app.txt failed\n", __FUNCTION__);
        return;
    }
    fseek(pic_program_file,0,SEEK_SET);
    memset(program_data, 0x0, 10*MAX_CHAR_NUM);

    pic_flash_length = (((unsigned int)end_addr_h << 8) + end_addr_l) - (((unsigned int)start_addr_h << 8) + start_addr_l) + 1;
    printf("pic_flash_length = %d\n", pic_flash_length);

    for(i=0; i<pic_flash_length; i++)
    {
        fgets(data_read, MAX_CHAR_NUM - 1 , pic_program_file);
        //printf("data_read[0]=%c, data_read[1]=%c, data_read[2]=%c, data_read[3]=%c\n", data_read[0], data_read[1], data_read[2], data_read[3]);
        data_int = strtoul(data_read, NULL, 16);
        //printf("data_int = 0x%04x\n", data_int);
        program_data[2*i + 0] = (unsigned char)((data_int >> 8) & 0x000000ff);
        program_data[2*i + 1] = (unsigned char)(data_int & 0x000000ff);
        //printf("program_data[%d]=0x%02x, program_data[%d]=0x%02x\n\n", 2*i + 0, program_data[2*i + 0], 2*i + 1, program_data[2*i + 1]);
    }

    fclose(pic_program_file);

    // after read upgrade file correct, erase pic
    pic_reset();
    pic_erase_flash_all();

    // write data into pic
    pic_set_flash_pointer(PIC_FLASH_POINTER_START_ADDRESS_H, PIC_FLASH_POINTER_START_ADDRESS_L);

    for(i=0; i<pic_flash_length/PIC_FLASH_SECTOR_LENGTH*4; i++)
    {
        memcpy(buf, program_data+i*16, 16);
        /*
        printf("send pic program time: %d\n",i);
        for(j=0; j<16; j++)
        {
            printf("buf[%d] = 0x%02x\n", j, *(buf+j));
        }
        printf("\n");
        */
        pic_send_data_to_pic(buf);
        pic_write_data_into_flash();
    }
}

void clear_freq_value_in_pic(void)
{
    printf("--- %s\n", __FUNCTION__);

    pic_reset();
    pic_set_flash_pointer(PIC_FREQ_START_ADDRESS_H, PIC_FREQ_START_ADDRESS_L);
    usleep(100*1000);
    pic_erase_pic_flash();
    pic_erase_pic_flash();
}

unsigned char flash_pic_freq(unsigned char *buf1)
{
    unsigned char ret=0xff;
    unsigned char buf[16] = {0};
    unsigned int i=0, erase_loop = 0;
    unsigned char start_addr_h = PIC_FLASH_POINTER_FREQ_START_ADDRESS_H, start_addr_l = PIC_FLASH_POINTER_FREQ_START_ADDRESS_L;
    unsigned char end_addr_h = PIC_FLASH_POINTER_FREQ_END_ADDRESS_H, end_addr_l = PIC_FLASH_POINTER_FREQ_END_ADDRESS_L;
    unsigned int pic_flash_length=0;

    printf("--- %s\n", __FUNCTION__);

    pic_set_flash_pointer(PIC_FLASH_POINTER_FREQ_START_ADDRESS_H, PIC_FLASH_POINTER_FREQ_START_ADDRESS_L);

    pic_flash_length = (((unsigned int)end_addr_h << 8) + end_addr_l) - (((unsigned int)start_addr_h << 8) + start_addr_l) + 1;
    erase_loop = pic_flash_length/PIC_FLASH_SECTOR_LENGTH;

    for(i=0; i<pic_flash_length/PIC_FLASH_SECTOR_LENGTH*4; i++)
    {
        memcpy(buf, buf1+i*16, 16);
        pic_send_data_to_pic(buf);
        pic_write_data_into_flash();
    }
}


void i2c_init(struct bitmain_L3_info *info)
{
    int i;
    if(unlikely((info->i2c_fd = open(IIC_DEVIVEE,O_RDWR | O_NONBLOCK)) < 0))
    {
        perror(IIC_DEVIVEE);
    }
    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(info->chain_status[i])
        {
            if(unlikely(ioctl(info->i2c_fd,I2C_SLAVE,i2c_slave_addr[i] >> 1 ) < 0))
            {
                perror("set i2c slave addr  error");
            }
        }
    }
    dev.i2c_fd = info->i2c_fd;
    applog(LOG_NOTICE,"i2c init ok");

}

static int ProcessTempData(
    const unsigned char* const buf, const unsigned char chip_addr, const unsigned char chain_id)
{

    unsigned char buffer[4] = "";
    memcpy(buffer, buf, 4);
    unsigned char tmpChar = buffer[0];
    if(buf[2] == 0x11)
        return;
    if ( (tmpChar & 0xc0 ) == 0 )
    {
        switch(chip_addr)
        {
            case (HAVE_TEMP) :
            {
                int count = 0;
                while (TempBuffer[chain_id][0].state != BUF_IDLE )
                {
                    if (count++ > 5)
                        return -1;
                }
                TempBuffer[chain_id][0].data = buffer[3];
                TempBuffer[chain_id][0].state = BUF_READY;
                break;
            }
            case (HAVE_TEMP_2) :
            {
                int count = 0;
                while (TempBuffer[chain_id][1].state != BUF_IDLE )
                {
                    if (count++ > 5)
                        return -1;
                }
                TempBuffer[chain_id][1].data = buffer[3];
                TempBuffer[chain_id][1].state = BUF_READY;
                break;
            }
            case (HAVE_TEMP_3) :
            {
                int count = 0;
                while (TempBuffer[chain_id][2].state != BUF_IDLE )
                {
                    if (count++ > 5)
                        return -1;
                }
                TempBuffer[chain_id][2].data = buffer[3];
                TempBuffer[chain_id][2].state = BUF_READY;
                break;
            }
        }
        return 0;
    }
    return -1;
}

static void SaveTempData(
    unsigned char localTemp, unsigned char remoteTemp, unsigned char chip_addr, unsigned char chain)
{
    switch(chip_addr)
    {
        case (HAVE_TEMP_3) :
        {
            dev.chain_asic_temp[chain][0][0] = localTemp;
            dev.chain_asic_temp[chain][0][1] = remoteTemp;
            break;
        }
        case (HAVE_TEMP_2) :
        {
            dev.chain_asic_temp[chain][1][0] = localTemp;
            dev.chain_asic_temp[chain][1][1] = remoteTemp;
            break;
        }
        case (HAVE_TEMP) :
        {
            dev.chain_asic_temp[chain][2][0] = localTemp;
            dev.chain_asic_temp[chain][2][1] = remoteTemp;

            break;
        }
    }
}





void *bitmain_scanreg(void* args)
{
    unsigned int j, not_reg_data_time = 0;
    unsigned int reg_value_num = 0;
    unsigned char reg_buf[4] = {0},crc_check,chip_addr,reg_addr,chain_id;
    int read_num = 0;
    uint64_t tmp_rate = 0;
    bool zero_asic_num = true;
rerun_all:
    clear_register_value_buf();
    tmp_rate = 0;

    read_num = 0;
    tmp_rate = 0;


    while(1)
    {
        cgsleep_ms(3);

        pthread_mutex_lock(&reg_mutex);

        reg_value_num = reg_fifo.reg_value_num;
        applog(LOG_DEBUG,"%s: reg_value_num = %d", __FUNCTION__, reg_value_num);
        pthread_mutex_unlock(&reg_mutex);
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

                chip_addr = reg_fifo.reg_buffer[reg_fifo.p_rd].chipaddr;
                reg_addr = reg_fifo.reg_buffer[reg_fifo.p_rd].regaddr;
                chain_id = reg_fifo.reg_buffer[reg_fifo.p_rd].chainid;

                crc_check = CRC5((uint8_t *)&(reg_fifo.reg_buffer[reg_fifo.p_rd]), 7*8-5);

                if(crc_check != (reg_fifo.reg_buffer[reg_fifo.p_rd].crc5 & 0x1f))
                {
                    applog(LOG_ERR,"%s,crc5 error,should be %02x,but check as %02x %d %d",__FUNCTION__,reg_fifo.reg_buffer[reg_fifo.p_rd].crc5 & 0x1f,crc_check,j,reg_value_num);
                    applog(LOG_ERR,"%s: reg_value = 0x%02x%02x%02x%02x", __FUNCTION__,reg_buf[0],reg_buf[1],reg_buf[2],reg_buf[3]);

                }

                memcpy(reg_buf,reg_fifo.reg_buffer[reg_fifo.p_rd].reg_value,4);
                pthread_mutex_unlock(&reg_mutex);

                if(reg_addr == CHIP_ADDR)
                {
                    if(update_asic_num)
                    {
                        if (zero_asic_num == true)
                        {
                            dev.chain_asic_num[chain_id] = 0;
                            zero_asic_num = false;
                        }

                        applog(LOG_DEBUG,"%s: reg_value = 0x%02x0x%02x0x%02x0x%02x", __FUNCTION__,reg_buf[0],reg_buf[1],reg_buf[2],reg_buf[3]);
                        if ( ++dev.chain_asic_num[chain_id]  > CHAIN_ASIC_NUM )
                            dev.chain_asic_num[chain_id] = 1;
                    }
                }

                if(reg_addr == CORE_RESP_OUT)
                {
                    chip_addr = reg_fifo.reg_buffer[reg_fifo.p_rd].chipaddr;
                    reg_addr = reg_fifo.reg_buffer[reg_fifo.p_rd].regaddr;
                    chain_id = reg_fifo.reg_buffer[reg_fifo.p_rd].chainid;
                }
                if(reg_addr == PLL_PARAMETER)
                {
                    applog(LOG_DEBUG,"%s: reg_value = 0x%02x0x%02x0x%02x0x%02x", __FUNCTION__,reg_buf[0],reg_buf[1],reg_buf[2],reg_buf[3]);
                }

                if ( reg_addr == MISC_CONTROL )
                {
                    // applog(LOG_DEBUG, "MISC DATA DUMP\r\n");
                    applog(LOG_DEBUG, "Dump MISC Data:[%X][%X][%X][%X]@Chain[%d] -- Chip[%X]", reg_buf[0], reg_buf[1], reg_buf[2], reg_buf[3], chain_id, chip_addr);
                }

                if ( reg_addr == GENERAL_IIC)
                {
                    applog(LOG_NOTICE, "Dump IIC Data:[%X][%X][%X][%X]@ChainID:[%d], Chip[%X]", reg_buf[0], reg_buf[1], reg_buf[2], reg_buf[3], chain_id, chip_addr);
                    ProcessTempData(reg_buf, chip_addr, chain_id);
                }

                if(reg_addr == HASHRATE)
                {
                    int i;
                    uint64_t temp_hash_rate = 0;
                    char rate_buf[10];
                    for(i = 0; i < 4; i++)
                    {
                        sprintf(rate_buf + 2*i,"%02x",reg_buf[i]);
                    }
                    applog(LOG_DEBUG,"%s: hashrate is %s", __FUNCTION__, rate_buf);
                    temp_hash_rate = strtol(rate_buf,NULL,16);
                    temp_hash_rate = (temp_hash_rate << 8);
                    tmp_rate += temp_hash_rate;
                    read_num ++;


                    if(read_num == CHAIN_ASIC_NUM)
                    {
                        rate[chain_id] = tmp_rate;
                        suffix_string_L3(rate[chain_id], (char * )displayed_rate[chain_id], sizeof(displayed_rate[chain_id]), 5,false);
                        rate_error[chain_id] = 0;
                        tmp_rate = 0;
                        read_num = 0;
                        uint64_t hash_rate_all = 0;
                        applog(LOG_DEBUG,"%s: chain %d hashrate is %s", __FUNCTION__, chain_id, displayed_rate[chain_id]);
                        for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
                        {
                            if(dev.chain_exist[i] == 1)
                            {
                                hash_rate_all += rate[i];
                            }
                        }
                        suffix_string_L3(hash_rate_all, (char * )displayed_hash_rate, sizeof(displayed_hash_rate), 5,false);
                    }

                }
                if ( reg_addr == EXT_TEMP_SENSOR )
                {
                    applog(LOG_DEBUG, "TEMP DATA @ Chain[%d]:local[%d] remote[%d] On Chip[%X]", chain_id, reg_buf[1], reg_buf[3], chip_addr);
                    SaveTempData(reg_buf[1], reg_buf[3], chip_addr, chain_id);
                }
                if(reg_addr == CHIP_ADDR)
                {
                    if (dev.chain_asic_num[chain_id] == CHAIN_ASIC_NUM)
                        applog(LOG_DEBUG,"chian %d get asicnum %d",chain_id,CHAIN_ASIC_NUM);

                    if(dev.chain_asic_num[chain_id] > dev.max_asic_num_in_one_chain)
                    {
                        dev.max_asic_num_in_one_chain = dev.chain_asic_num[chain_id];
                    }
                    applog(LOG_DEBUG,"%s: chain J%d has %d ASIC", __FUNCTION__, chain_id+1, dev.chain_asic_num[chain_id]);
                }

                if(status_error || (read_num == 0 && check_rate))
                {
                    rate_error[chain_id]++;
                    if(rate_error[chain_id] > 3 || status_error)
                    {
                        rate[chain_id] = 0;
                        suffix_string_L3(rate[chain_id], (char * )displayed_rate[chain_id], sizeof(displayed_rate[chain_id]), 3,true);
                    }
                }
                reg_fifo.p_rd++;
                reg_fifo.reg_value_num--;
                if(reg_fifo.p_rd >= MAX_NONCE_NUMBER_IN_FIFO)
                {
                    reg_fifo.p_rd = 0;
                }
            }
        }
        else
        {
            if(not_reg_data_time++ > 3)
            {
                clear_register_value_buf();
                not_reg_data_time = 0;
                check_rate = false;
            }
            cgsleep_ms(300);
        }

    }

}

// Test Patten start

#define ASIC_NUM                            36
#define CORE_NUM                            12
#define TEST_TIME                           1
#define TEST_NUM                            4
#define TOTAL_TEST_NUM                      CORE_NUM * TEST_NUM
#define IGNORE_NUM                          2
#define WORK_INPUT_TYPE                     0x21
#define WORK_INPUT_LENGTH_WITHOUT_CRC       84
#define WORK_INPUT_LENGTH_WITH_CRC          86
#define SNO_LENGTH                          0x4
#define SCRYPTDATA_LENGTH                   76
#define ASIC_RETURN_DATA_LENGTH             0x7
#define Swap16(l) (l >> 8) | ((l & 0xff) << 8)

int last_result[BITMAIN_MAX_CHAIN_NUM][256];
int last_freq[BITMAIN_MAX_CHAIN_NUM][256];
int first_freq = 41;
bool first_test = true;


unsigned short CRC16(unsigned short crc, unsigned char *buffer, int len)
{
    while (len--)
        crc = crc_itu_t_byte(crc, *buffer++);
    return crc;
}


//#define ALLOW_KPERCENT_8xPATTEN   // allowed 99.0% 8xPatten , can be treat as SUCCESS
#ifdef ALLOW_KPERCENT_8xPATTEN
#define KPERCENT_8xPATTEN   0.99    //99.0%
#else
#define KPERCENT_8xPATTEN   1

#endif

#define DATA_FILE       "/usr/bin/ltc-test/ltc-asic-%02d/ltc-core-%02d.txt"
union nonce_u
{
    uint32_t nonce;
    struct
    {
        uint32_t content    : 20;
        uint32_t addr       : 8;
        uint32_t core       : 4;
    };
};

struct patten_work
{
    uint32_t id;
    union nonce_u nonce;
    unsigned char pattern[80];
};


struct work_sta
{
    struct patten_work work;
    int back[BITMAIN_MAX_CHAIN_NUM];
};

struct work_sta work_list[CHAIN_ASIC_NUM][TOTAL_TEST_NUM];
int test_result[BITMAIN_MAX_CHAIN_NUM][CHAIN_ASIC_NUM][CORE_NUM];
int total_back_nonce = 0;
bool patten_result = true;
bool start_receive = false;
struct timeval start_send_tv;

unsigned char c2hex(unsigned char value)
{
    unsigned char ret = 0xFF;
    if (value >= 0x30 && value <= 0x39)
        ret = value & 0x0F;
    else if (value == 'a' || value == 'A')
        ret = 0x0A;
    else if (value == 'b' || value == 'B')
        ret = 0x0B;
    else if (value == 'c' || value == 'C')
        ret = 0x0C;
    else if (value == 'd' || value == 'D')
        ret = 0x0D;
    else if (value == 'e' || value == 'E')
        ret = 0x0E;
    else if (value == 'f' || value == 'F')
        ret = 0x0F;
    return ret;
}

unsigned char twoc2hex(unsigned char high, unsigned char low)
{
    unsigned char ret = 0x00;
    high = c2hex(high);
    low = c2hex(low);
    ret = high << 4 & 0xF0;
    ret = ret ^ low;
    return ret;
}

int s2hex(unsigned char * dst, const unsigned char * src, int inlen)
{
    int i = 0, len = 0, p = 0;
    unsigned char high, low;

    if(src == NULL || inlen <= 0 || dst == NULL)
    {
        return -1;
    }
    len = inlen/2;
    p = inlen%2;
    for(i = 0; i < len; i++)
    {
        high = src[i*2];
        low = src[i*2+1];
        dst[i] = twoc2hex(high, low);
    }
    if(p)
    {
        high = src[i*2];
        dst[i] = twoc2hex(high, 0);
    }
    return len+p;
}

static int get_patten_work(int id, int count, FILE * fps, int core)
{
    struct work_sta * new_work;
    char * temp;
    char str[MAX_CHAR_NUM] = {0};
    int subid = 0;
    int i;
    while(fgets(str, MAX_CHAR_NUM - 1 , fps))
    {
        if(NULL == str)
        {
            return subid;
        }
        if(subid >= count)
            break;
        new_work = work_list[id] + subid + core*count;
        for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
            new_work->back[i] = 0;

        temp = strstr(str, "work");
        if(NULL == temp) goto err;
        temp += 4;
        while(*temp == ' ') temp++;
        s2hex(new_work->work.pattern, (unsigned char *)temp, 160);

        temp = strstr(str, "nonce");
        if(NULL == temp) goto err;
        temp += 5;
        while(*temp == ' ') temp++;
        s2hex((uint8_t *)(&new_work->work.nonce.nonce) ,(unsigned char *)temp, 8);
        new_work->work.nonce.nonce = htonl(new_work->work.nonce.nonce);
        new_work->work.id = subid + core*count;
        applog(LOG_DEBUG, "asic %d wc %d nonce %08x", id, new_work->work.id, new_work->work.nonce.nonce);
        subid++;
    }

    return subid;
err:
    free(new_work);
    return subid;
}

void print_result()
{
    applog(LOG_NOTICE,"%s",__FUNCTION__);
    int chain, asic, core;
    int asic_total;
    for(chain = 0; chain < BITMAIN_MAX_CHAIN_NUM; chain++)
    {
        if(dev.chain_exist[chain] == 1)
        {
            printf("Chain %d result\n",chain + 1);
            for (asic = 0; asic < CHAIN_ASIC_NUM; asic++)
            {
                asic_total = 0;
                for (core = 0; core < CORE_NUM; core++)
                {
                    asic_total += test_result[chain][asic][core];
                    //printf("CORE[%02d]=%d\t",core,test_result[chain][asic][core]);
                }
                printf("A%02d=%d %d ",asic,asic_total, freq_pll_1485[last_freq[chain][asic]].freq);
                if(asic_total == TOTAL_TEST_NUM)
                {
                    if( last_result[chain][asic] == 0)
                    {
                        last_freq[chain][asic]+=4;
                        if(last_freq[chain][asic] > 65)
                        {
                            last_freq[chain][asic] = 65;
                            last_result[chain][asic] = 1;
                        }
                        patten_result = false;
                    }
                }
                else
                {
                    if(asic_total < (TOTAL_TEST_NUM - IGNORE_NUM))
                    {
                        last_freq[chain][asic]--;
                        if(last_freq[chain][asic] < 30)
                        {
                            last_freq[chain][asic] = 30;
                        }
                        patten_result = false;
                    }
                    last_result[chain][asic] = 1;
                }
                if((asic + 1) % 8 == 0)
                {
                    printf("\n");
                }
            }
            printf("\n");
        }
    }
}

struct work_input_format
{
    unsigned char type;
    unsigned char length;
    unsigned char wc;
    unsigned char reserved;
    unsigned char sno[4];
    unsigned char scryptdata[76];
    unsigned short crc16;
};

void * send_func(void *usrdata)
{
    struct bitmain_L3_info *info = (struct bitmain_L3_info *)usrdata;
    uint8_t chainid = info->chain_index;
    struct thr_info * thr = info->thr;

    unsigned int work_fifo_ready = 0;
    struct patten_work * work;
    struct work_input_format work_will_be_sent;
    unsigned char *p = (unsigned char *)(&work_will_be_sent);

    int asic = 0, chain = 0, index = 0;
    int i = 0, j = 0, ret = 0;
    int send_work = 0;

    cgtime(&start_send_tv);
    gBegin_get_nonce = true;
    for(asic = 0; asic < dev.max_asic_num_in_one_chain; asic++)
    {
        index = 0;
        for (index = 0; index < TOTAL_TEST_NUM; index++)
        {
            for(chain = 0; chain < BITMAIN_MAX_CHAIN_NUM; chain++)
            {
                if(dev.chain_exist[chain] == 1)
                {
                    cgsleep_ms(10);
                    memset(&work_will_be_sent, 0x0, WORK_INPUT_LENGTH_WITH_CRC);

                    // get work for sending to asic
                    work = &work_list[asic][index].work;

                    work_will_be_sent.type = WORK_INPUT_TYPE;
                    work_will_be_sent.length = WORK_INPUT_LENGTH_WITHOUT_CRC;
                    work_will_be_sent.wc = (unsigned char)(work->id & 0x000000ff);

                    for(i=0; i<SNO_LENGTH; i++)
                    {
                        work_will_be_sent.sno[i] = work->pattern[i];
                    }

                    for(i=SNO_LENGTH; i<SCRYPTDATA_LENGTH + SNO_LENGTH; i++)
                    {
                        work_will_be_sent.scryptdata[i - SNO_LENGTH] = work->pattern[i];
                    }
                    work_will_be_sent.crc16 = Swap16(CRC16(0xffff, (unsigned char *)(&work_will_be_sent), work_will_be_sent.length));

                    ret = L3_write(dev.dev_fd[chain], &work_will_be_sent, WORK_INPUT_LENGTH_WITH_CRC);

                    if(ret != WORK_INPUT_LENGTH_WITH_CRC)
                    {
                        tcflush(dev.dev_fd[chain], TCIOFLUSH);
                        ret = L3_write(dev.dev_fd[chain], &work_will_be_sent, WORK_INPUT_LENGTH_WITH_CRC);
                        if(ret != WORK_INPUT_LENGTH_WITH_CRC)
                        {
                            tcflush(dev.dev_fd[chain], TCIOFLUSH);
                            ret = L3_write(dev.dev_fd[chain], &work_will_be_sent, WORK_INPUT_LENGTH_WITH_CRC);
                            if(ret != WORK_INPUT_LENGTH_WITH_CRC)
                            {
                                perror("Send work error!");
                            }
                        }
                    }
                    send_work++;
                }
            }
        }
    }
    cgsleep_ms(100);
    applog(LOG_NOTICE,"send_work : %d ",send_work);
    return 0;

}

void * receive_func(void *arg)
{
    applog(LOG_NOTICE,"%s", __FUNCTION__);
    struct timeval current, diff;
    int whose_nonce = 0;
    int total_nonce = 0;
    uint8_t nonce_bin[4] = {0};
    while (start_receive)
    {
        pthread_mutex_lock(&nonce_mutex);
        while(nonce_fifo.nonce_num)
        {
            union nonce_u nonce3;

            memcpy(nonce_bin,nonce_fifo.nonce_buffer[nonce_fifo.p_rd].nonce,4);
            uint8_t work_id = nonce_fifo.nonce_buffer[nonce_fifo.p_rd].wc;
            uint8_t chain_id = nonce_fifo.nonce_buffer[nonce_fifo.p_rd].chainid;

            memcpy((uint8_t *)&nonce3.nonce,nonce_bin,4);
            nonce3.nonce = Swap32(nonce3.nonce);

            if(nonce_fifo.p_rd < MAX_NONCE_NUMBER_IN_FIFO - 1)
            {
                nonce_fifo.p_rd++;
            }
            else
            {
                nonce_fifo.p_rd = 0;
            }

            nonce_fifo.nonce_num--;
            whose_nonce = nonce3.addr / dev.addrInterval;
            //applog(LOG_NOTICE, "chian %d wc %d nonce %08x", chain_id, work_id, nonce3.nonce);
            total_nonce++;
            if (work_id > TOTAL_TEST_NUM)
                continue;
            if(work_list[whose_nonce][work_id].work.nonce.nonce == nonce3.nonce)
            {
                if(!(work_list[whose_nonce][work_id].back[chain_id] >= 1))
                {
                    test_result[chain_id][whose_nonce][nonce3.core < 6 ? nonce3.core : nonce3.core - 2] ++;
                    total_back_nonce ++;
                    work_list[whose_nonce][work_id].back[chain_id]++;
                    applog(LOG_DEBUG, "wc %d chian %d asic %d core %d nonce %08x times %d nonce %08x",work_id, chain_id, whose_nonce, nonce3.core < 6 ? nonce3.core : nonce3.core - 2,nonce3.nonce,work_list[whose_nonce][work_id].back[chain_id],work_list[whose_nonce][work_id].work.nonce.nonce);
                }
                else
                {
                    /*repeat nonce*/
                }
            }
        }
        pthread_mutex_unlock(&nonce_mutex);
        cgsleep_ms(1);

    }

    if (total_back_nonce < (int)(((double)(TOTAL_TEST_NUM * CHAIN_ASIC_NUM * dev.chain_num)) * KPERCENT_8xPATTEN) - 10 * IGNORE_NUM )
        patten_result = false;
    gBegin_get_nonce = false;
    applog(LOG_NOTICE,"total_nonce : %d total_value_nonce : %d need %d", total_nonce, total_back_nonce, (int)(((double)(TOTAL_TEST_NUM * CHAIN_ASIC_NUM * dev.chain_num)) * KPERCENT_8xPATTEN)- 10 * IGNORE_NUM );
    applog(LOG_NOTICE,"test result : %s ",patten_result ? "TRUE":"FALSE");
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



void reset_patten_stats()
{
    int asic, index, chain;
    applog(LOG_NOTICE,"%s", __FUNCTION__);
    memset(test_result,0,sizeof(test_result));
    total_back_nonce = 0;
    patten_result = true;
    for(asic = 0; asic < CHAIN_ASIC_NUM; asic++)
        for(index = 0; index < TOTAL_TEST_NUM; index++)
            for(chain = 0; chain < BITMAIN_MAX_CHAIN_NUM; chain++)
                work_list[asic][index].back[chain] = 0;
    //clear_nonce_fifo();
}

void reset_miner(struct bitmain_L3_info *info)
{
    int i;

    if(access("/usr/bin/need_reboot",0) == -1)
    {
        system("touch /usr/bin/need_reboot");
        system("echo 1 >> /usr/bin/already_reboot");
        system("/etc/init.d/cgminer.sh restart");
        return;
    }
    system("echo 2 >> /usr/bin/already_reboot");
    return;
    send_heart = false;
    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(dev.chain_exist[i] == 1)
        {
            pthread_mutex_lock(&iic_mutex);
            if(unlikely(ioctl(info->i2c_fd,I2C_SLAVE,i2c_slave_addr[i] >> 1) < 0))
                applog(LOG_ERR,"ioctl error @ line %d",__LINE__);
            pic_dac_ctrl(0);
            pthread_mutex_unlock(&iic_mutex);
        }
    }
    cgsleep_ms(1000);
    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(dev.chain_exist[i] == 1)
        {
            pthread_mutex_lock(&iic_mutex);
            if(unlikely(ioctl(info->i2c_fd,I2C_SLAVE,i2c_slave_addr[i] >> 1) < 0))
                applog(LOG_ERR,"ioctl error @ line %d",__LINE__);
            pic_dac_ctrl(1);
            pthread_mutex_unlock(&iic_mutex);
        }
    }
    cgsleep_ms(1000);
    reset_asic();
    reset_asic();
    cgsleep_ms(500);
    clear_register_value_buf();
    send_heart = true;
}

bool test_board_core(struct bitmain_L3_info *info)
{
    int i = 0, j = 0, times = 0, tid = 0, chain = 0, asic = 0;
    int patten_num = 0;
    char data_file[128] = "";
    FILE * fp;
    pthread_t send_id, receive_id;
    for(i=0 ; i < CHAIN_ASIC_NUM; i++)
    {
        for(j=0 ; j < CORE_NUM; j++)
        {
            sprintf(data_file, DATA_FILE, i+1, j+1);
            fp = fopen(data_file, "r");
            if(NULL == fp)
            {
                printf("Open test file %s error\n", data_file);
                return -1;
            }
            patten_num = get_patten_work(i, TEST_NUM, fp , j);
            fclose(fp);
        }
    }
    while(1)
    {
        times++;
        reset_patten_stats();
        set_PWM(50);
        cgsleep_ms(10);
        reset_asic();
        cgsleep_ms(10);
        software_set_address();
        cgsleep_ms(10);
        set_misc_ctrl();
        cgsleep_ms(10);
        if(first_test)
        {
            set_frequency_i(first_freq);
            for(chain = 0; chain < BITMAIN_MAX_CHAIN_NUM; chain++)
            {
                for(asic = 0; asic < dev.max_asic_num_in_one_chain; asic++)
                {
                    last_freq[chain][asic] = first_freq;
                }
            }
            first_test = false;
        }
        else
        {
            for(chain = 0; chain < BITMAIN_MAX_CHAIN_NUM; chain++)
            {
                if(dev.chain_exist[chain] == 1)
                {
                    for(asic = 0; asic < dev.max_asic_num_in_one_chain; asic++)
                    {
                        set_frequency_with_addr_i(chain, last_freq[chain][asic], asic * dev.addrInterval);
                        cgsleep_ms(2);
                    }
                }
            }
        }

        set_core_ctrl(CLOCK_EN_CTRL,0,1);
        cgsleep_ms(100);

        set_asic_ticket_mask(0x3f);

        tid = pthread_create(&receive_id, NULL, receive_func, NULL);
        if(tid !=0)
        {
            perror("receive_func create error");
            return false;
        }
        clear_nonce_fifo();
        start_receive = true;
        tid = pthread_create(&send_id, NULL, send_func, info);
        if(tid !=0)
        {
            perror("send_id create error");
            return false;
        }

        pthread_join(send_id, NULL);
        cgsleep_ms(100);
        start_receive = false;
        pthread_join(receive_id, NULL);

        print_result();
        if(patten_result)
            break;
        applog(LOG_NOTICE,"%d" ,times);
    }
    return patten_result;
}

// Test Patten end

int bitmain_L3_init(struct bitmain_L3_info *info)
{
    struct init_config config = info->L3_config;
    uint16_t crc = 0;
    struct init_config config_parameter;
    int i = 0,x = 0,y = 0,check_asic_times = 0;
    bool check_asic_fail = false;

    memcpy(&config_parameter, &config, sizeof(struct init_config));

    sprintf(g_miner_version, "1.0.1.1");

    set_PWM(100);

    if(config_parameter.token_type != INIT_CONFIG_TYPE)
    {
        applog(LOG_DEBUG,"%s: config_parameter.token_type != 0x%x, it is 0x%x", __FUNCTION__, INIT_CONFIG_TYPE, config_parameter.token_type);
        return -1;
    }

    crc = crc_itu_t(0xff,(uint8_t*)(&config_parameter), sizeof(struct init_config) - sizeof(uint16_t));
    if(crc != config_parameter.crc)
    {
        applog(LOG_DEBUG,"%s: config_parameter.crc = 0x%x, but we calculate it as 0x%x", __FUNCTION__, config_parameter.crc, crc);
        return -2;
    }
    for(i = 0; i < BITMAIN_MAX_QUEUE_NUM; i++)
    {
        info->work_queue[i] = NULL;
    }

    check_chain(info);
    cgsleep_ms(10);

    i2c_init(info);
    cgsleep_ms(10);

    bool version_error = false;
update:
    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(dev.chain_exist[i] == 1)
        {
            pthread_mutex_lock(&iic_mutex);
            if(unlikely(ioctl(info->i2c_fd,I2C_SLAVE,i2c_slave_addr[i] >> 1 ) < 0))
                applog(LOG_ERR, "ioctl error @ line %d",__LINE__);
            reset_iic_pic();
            pthread_mutex_unlock(&iic_mutex);
        }
    }

    cgsleep_ms(1000);

    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(dev.chain_exist[i] == 1)
        {
            pthread_mutex_lock(&iic_mutex);
            if(unlikely(ioctl(info->i2c_fd,I2C_SLAVE,i2c_slave_addr[i] >> 1 ) < 0))
                applog(LOG_ERR, "ioctl error @ line %d",__LINE__);
            jump_to_app_from_loader();
            pthread_mutex_unlock(&iic_mutex);
        }
    }
    cgsleep_ms(1000);
//#define UPDATE_PIC
#ifdef UPDATE_PIC
    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(dev.chain_exist[i] == 1)
        {
            pthread_mutex_lock(&iic_mutex);
            if(unlikely(ioctl(info->i2c_fd,I2C_SLAVE,i2c_slave_addr[i] >> 1 ) < 0))
                applog(LOG_ERR, "ioctl error @ line %d",__LINE__);
            pic_read_pic_software_version(&pic_version[i]);
            applog(LOG_NOTICE, "Chain %d PIC Version 0x%x", i, pic_version[i]);
            if(pic_version[i] != 0x3)
            {
                update_pic_program();
                version_error = true;
            }
            pthread_mutex_unlock(&iic_mutex);
        }
    }

    cgsleep_ms(1000);

    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(dev.chain_exist[i] == 1)
        {
            pthread_mutex_lock(&iic_mutex);
            if(unlikely(ioctl(info->i2c_fd,I2C_SLAVE,i2c_slave_addr[i] >> 1 ) < 0))
                applog(LOG_ERR, "ioctl error @ line %d",__LINE__);
            reset_iic_pic();
            pthread_mutex_unlock(&iic_mutex);
        }
    }

    cgsleep_ms(1000);

    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(dev.chain_exist[i] == 1)
        {
            pthread_mutex_lock(&iic_mutex);
            if(unlikely(ioctl(info->i2c_fd,I2C_SLAVE,i2c_slave_addr[i] >> 1 ) < 0))
                applog(LOG_ERR, "ioctl error @ line %d",__LINE__);
            jump_to_app_from_loader();
            pthread_mutex_unlock(&iic_mutex);
        }
    }

    cgsleep_ms(1000);

    if(version_error)
    {
        cgsleep_ms(200);
        version_error = false;
        goto update;
    }
#endif

    pic_heart_beat = calloc(1,sizeof(struct thr_info));
    if(thr_info_create(pic_heart_beat, NULL, pic_heart_beat_func, pic_heart_beat))
    {
        applog(LOG_DEBUG,"%s: create thread error for pic_heart_beat_func", __FUNCTION__);
        return -3;
    }

    pthread_detach(pic_heart_beat->pth);

    for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(dev.chain_exist[i] == 1)
        {
            pthread_mutex_lock(&iic_mutex);
            if(unlikely(ioctl(info->i2c_fd,I2C_SLAVE,i2c_slave_addr[i] >> 1) < 0))
                applog(LOG_ERR,"ioctl error @ line %d",__LINE__);
            //pic_set_voltage(63);
            pic_dac_ctrl(1);
            pthread_mutex_unlock(&iic_mutex);
        }
    }

    cgsleep_ms(5000);
    reset_asic();
    reset_asic();
    cgsleep_ms(500);
    tty_init(info,config_parameter.baud);
    cgsleep_ms(10);

    clear_register_value_buf();

    cgsleep_ms(100);

    scan_reg_id = calloc(1,sizeof(struct thr_info));
    if(thr_info_create(scan_reg_id, NULL, bitmain_scanreg, scan_reg_id))
    {
        applog(LOG_DEBUG,"%s: create thread error for bitmain_scanreg", __FUNCTION__);
        return -3;
    }
    pthread_detach(scan_reg_id->pth);
    cgsleep_ms(100);

    //check ASIC number for every chain
    applog(LOG_NOTICE,"send cmd to get chip address");
check_asic_num:
    update_asic_num = true;
    check_asic_reg(0,CHIP_ADDR,1);
    //cgsleep_ms(100); // W8ng 4 Read Data
    sleep(2);
    update_asic_num = false;

    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        if(dev.chain_exist[i] == 1)
        {
            applog(LOG_NOTICE,"Chain %d ASIC %d !!!", i, dev.chain_asic_num[i]);
            if(dev.chain_asic_num[i] != CHAIN_ASIC_NUM)
            {
                dev.chain_asic_in_full[i] = 0;
                check_asic_fail = true;
                check_asic_times++;
                need_recheck[i] = true;
            }
            else
            {
                dev.chain_asic_in_full[i] = 1;
                need_recheck[i] = false;
            }
        }
    }

    if(check_asic_fail && (check_asic_times < 6))
    {
        applog(LOG_NOTICE,"Need to recheck asic num !!!");
        for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
        {
            dev.chain_asic_num[i] = 0;
        }
        reset_miner(info);
        check_asic_fail = false;
        goto check_asic_num;
    }

    if(access("/usr/bin/need_reboot",0) != -1)
    {
        system("rm /usr/bin/need_reboot");
    }
    //set core number
    dev.corenum = BM1485_CORE_NUM;

    software_set_address();
    cgsleep_ms(10);

    check_asic_reg(0,CHIP_ADDR,1);
    cgsleep_ms(20);

    if(config_parameter.frequency_eft)
    {
        dev.frequency = config_parameter.frequency;
        set_frequency(dev.frequency);
        sprintf(dev.frequency_t,"%u",dev.frequency);
    }

    cgsleep_ms(10);

    //check who control fan
    dev.fan_eft = config_parameter.fan_eft;
    dev.fan_pwm = config_parameter.fan_pwm_percent;
    applog(LOG_DEBUG,"%s: fan_eft : %d  fan_pwm : %d", __FUNCTION__,dev.fan_eft,dev.fan_pwm);
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

    //calculate real timeout
    if(config_parameter.timeout_eft)
    {
        if(config_parameter.timeout_data_integer == 0 && config_parameter.timeout_data_fractions == 0)   //driver calculate out timeout value
        {
            applog(LOG_NOTICE, "frequency = '%d'", dev.frequency);
            dev.timeout = 0xffffffff/16.0/(256/dev.addrInterval)/(dev.frequency*1000000)*0.95*2500;
            applog(LOG_NOTICE,"dev.timeout = %d", dev.timeout);
        }
        else
        {
            dev.timeout = config_parameter.timeout_data_integer * 1000 + config_parameter.timeout_data_fractions;
        }
    }

    //set baud
    set_misc_ctrl();

    calibration_sensor_offset();
    set_auto_read_temp(1);


    //open block
    applog(LOG_NOTICE,"send cmd to open block");
    set_core_ctrl(CLOCK_EN_CTRL,0,1);

    //test_board_core(info);
//#ifndef FPGA_SOURCE

    set_asic_ticket_mask((1<<(DEVICE_DIFF)) - 1);
    cgsleep_ms(10);

    check_miner_status_id = calloc(1,sizeof(struct thr_info));

    if(thr_info_create(check_miner_status_id, NULL, check_fan_thr, NULL))
    {
        applog(LOG_DEBUG,"%s: create thread for check miner_status", __FUNCTION__);
        return -5;
    }


#if 1
    read_temp_id = calloc(1,sizeof(struct thr_info));
    if(thr_info_create(read_temp_id, NULL, read_temp_func, read_temp_id))
    {
        applog(LOG_DEBUG,"%s: create thread for read temp", __FUNCTION__);
        return -7;
    }
    pthread_detach(read_temp_id->pth);

#endif
    sleep(2);


#if 1
    if(thr_info_create(check_miner_status_id, NULL, check_miner_status, check_miner_status_id))
    {
        applog(LOG_DEBUG,"%s: create thread for check miner_status", __FUNCTION__);
        return -5;
    }
    pthread_detach(check_miner_status_id->pth);
#endif
//#endif
    read_hash_rate = calloc(1,sizeof(struct thr_info));
    if(thr_info_create(read_hash_rate, NULL, get_hash_rate, read_hash_rate))
    {
        applog(LOG_DEBUG,"%s: create thread for get hashrate from asic failed", __FUNCTION__);
        return -6;
    }

    pthread_detach(read_hash_rate->pth);



    for(x = 0; x < BITMAIN_MAX_CHAIN_NUM; x++)
    {
        if(dev.chain_exist[x])
        {
            int offset = 0;
            for(y = 0; y < dev.chain_asic_num[x]; y++)
            {
                if(y % 8 == 0)
                {
                    if ( ( y + offset ) > ( CHAIN_ASIC_NUM + 16 )) applog(LOG_ERR, "offset[%d] ERR", (y + offset));
                    dev.chain_asic_status_string[x][y+offset] = ' ';
                    offset++;
                }

                if ( ( y + offset ) > ( CHAIN_ASIC_NUM + 16 )) applog(LOG_ERR, "offset[%d] ERR", (y + offset));
                dev.chain_asic_status_string[x][y+offset] = 'o';
                dev.chain_asic_nonce[x][y] = 0;
            }

            if ( ( y + offset ) > ( CHAIN_ASIC_NUM + 16 )) applog(LOG_ERR, "offset[%d] ERR", (y + offset));
            dev.chain_asic_status_string[x][y+offset] = '\0';
        }
    }
    start_send = true;


    return 0;
}

static bool bitmain_L3_prepare(struct thr_info *thr)
{
    struct cgpu_info *bitmain_L3 = thr->cgpu;
    struct bitmain_L3_info *info = bitmain_L3->device_data;

    info->thr = thr;
    mutex_init(&info->lock);
    cglock_init(&info->update_lock);


    struct init_config L3_config =
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
        .auto_read_temp             = 1,
        .reserved1                  = 0,
        .reserved2                  = {0},
        .chain_num                  = 6,
        .asic_num                   = CHAIN_ASIC_NUM,
        .fan_pwm_percent            = opt_bitmain_fan_pwm,
        .temperature                = 80,
        .frequency                  = opt_bitmain_L3_freq,
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
    };
    L3_config.crc = crc_itu_t(0xff,(uint8_t *)(&L3_config), sizeof(L3_config)-2);
    info->L3_config = L3_config;
    bitmain_L3_init(info);
    return true;
}

static void bitmain_L3_reinit_device(struct cgpu_info *bitmain)
{
    if(!status_error)
        system("/etc/init.d/cgminer.sh restart > /dev/null 2>&1 &");
}

static void bitmain_L3_update(struct cgpu_info *bitmain)
{
    int i = 0;
    applog(LOG_DEBUG, "Updated Work!");
    for ( ; i < BITMAIN_MAX_CHAIN_NUM; ++i )
    {
        new_block[i] = true;
    }
}

static void bitmain_L3_detect(__maybe_unused bool hotplug)
{
    struct cgpu_info *cgpu = calloc(1, sizeof(*cgpu));
    struct device_drv *drv = &bitmainL3_drv;
    assert(cgpu);
    cgpu->drv = drv;
    cgpu->deven = DEV_ENABLED;
    cgpu->threads = 1;
    cgpu->device_data = calloc(sizeof(struct bitmain_L3_info), 1);
    if (unlikely(!(cgpu->device_data)))
        quit(1, "Failed to calloc cgpu_info data");

    assert(add_cgpu(cgpu));
    applog(LOG_DEBUG,"%s detect new device",__FUNCTION__);
}

static void bitmain_L3_shutdown(struct thr_info *thr)
{
    thr_info_cancel(check_miner_status_id);
    thr_info_cancel(read_temp_id);
    thr_info_cancel(read_hash_rate);
    thr_info_cancel(read_temp_id);
}



static struct api_data *
bitmain_api_stats(struct cgpu_info *cgpu)
{
    struct api_data *root = NULL;
    int i = 0;
    uint64_t hash_rate_all = 0;
    bool copy_data = false;

    root = api_add_uint8(root, "miner_count", &(dev.chain_num), copy_data);
    root = api_add_string(root, "frequency", dev.frequency_t, copy_data);
    root = api_add_uint8(root, "fan_num", &(dev.fan_num), copy_data);

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
        root = api_add_int16(root, temp_name, &(dev.chain_asic_temp[i][2][0]), copy_data);
    }


    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        char temp2_name[12];
        sprintf(temp2_name,"temp2_%d",i+1);
        root = api_add_int16(root, temp2_name, &(dev.chain_asic_temp[i][2][1]), copy_data);
    }
#ifdef L3_P
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
#endif
    root = api_add_uint32(root, "temp_max", &(dev.temp_top1), copy_data);
    total_diff1 = total_diff_accepted + total_diff_rejected + total_diff_stale;
    double hwp = (hw_errors + total_diff1) ?
                 (double)(hw_errors) / (double)(hw_errors + total_diff1) : 0;
    root = api_add_percent(root, "Device Hardware%", &hwp, false);
    root = api_add_int(root, "no_matching_work", &hw_errors, false);

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

    suffix_string_L3(hash_rate_all, (char * )displayed_hash_rate, sizeof(displayed_hash_rate), 6,false);

    return root;
}



struct device_drv bitmainL3_drv =
{
    .drv_id = DRIVER_bitmainL3,
    .dname = "Bitmain_L3",
    .name = "L3",
    .drv_detect = bitmain_L3_detect,
    .thread_prepare = bitmain_L3_prepare,
    .hash_work = &hash_driver_work,
    .scanwork = bitmain_L3_scanhash,
    .update_work = bitmain_L3_update,
    .get_api_stats = bitmain_api_stats,
    .reinit_device = bitmain_L3_reinit_device,
    .get_statline_before = get_bitmain_statline_before,
    .thread_shutdown = bitmain_L3_shutdown,
};

