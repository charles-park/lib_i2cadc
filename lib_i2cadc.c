//------------------------------------------------------------------------------
/**
 * @file lib_i2cadc.c
 * @author charles-park (charles.park@hardkernel.com)
 * @brief ADC board(LTC2309) control library for ODROID-JIG.
 * @version 0.2
 * @date 2023-10-11
 *
 * @package apt install minicom
 *
 * @copyright Copyright (c) 2022
 *
 */
//------------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>

#include <unistd.h>
#include <string.h>
#include <time.h>
#include <ctype.h>
#include <fcntl.h>

#include "lib_i2c/lib_i2c.h"
#include "lib_i2cadc.h"

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// I2C ADC LTC2309 (12bits-8CH). (Use 8 Single-Ended, Unipolar Mode)
//
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// LTC2309 DEVICE I2C ADDR
//------------------------------------------------------------------------------
// LTC2309 Chip Pin Status (AD1, AD0)
//
// 0x08 = (  LOW,   LOW), 0x09 = (  LOW, FLOAT), 0x0A = ( LOW, HIGH), 0x0B = (FLOAT,  HIGH),
// 0x18 = (FLOAT, FLOAT), 0x19 = (FLOAT,   LOW), 0x1A = (HIGH,  LOW), 0x1B = ( HIGH, FLOAT),
// 0x14 = ( HIGH,  HIGH)
//
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// LTC2309 Reg Bits
//------------------------------------------------------------------------------
//
//  BIT7  BIT6  BIT5  BIT4  BIT3  BIT2  BIT1  BIT0
//  S/D   O/S   S1    S0    UNI    SLP   X     X
//
//  S/D : Single-ended/Defferential
//  O/S : ODD/Sign
//  S1  : Channel Select 1
//  S0  : Channel Select 0
//  UNI : UNIPOLAR/BIPOLAR
//  SLP : Sleep Mode
//
//  if ) Channel 0 Read
//       Single-ended = 1 | Sign = 0 | S1 = 0 | S0 = 0 | Unipolar = 1 | sleep = 0 | x | x | = 0x88
//  if ) Channel 1 Read
//       Single-ended = 1 | ODD = 1  | S1 = 0 | S0 = 0 | Unipolar = 1 | sleep = 0 | x | x | = 0xC8
//  if ) Channel 2 Read
//       Single-ended = 1 | Sign = 0 | S1 = 0 | S0 = 1 | Unipolar = 1 | sleep = 0 | x | x | = 0xA8
//  if ) Channel 3 Read
//       Single-ended = 1 | ODD = 1  | S1 = 0 | S0 = 1 | Unipolar = 1 | sleep = 0 | x | x | = 0xD8
//
//------------------------------------------------------------------------------------------------------------
#define SWAP_WORD(x)    (((x >> 8) & 0xFF) | ((x << 8) & 0xFF00))

// ADC Reference voltage 5V
#define ADC_REF_VOLTAGE 5

// Reference 5V, ADC weight value = 1200uV
#define ADC_WEIGHT_uV   ((ADC_REF_VOLTAGE * 1000000) / 4096)

enum {
    CHIP_ADC0 = 0,
    CHIP_ADC1,
    CHIP_ADC2,
    CHIP_ADC3,
    CHIP_ADC4,
    CHIP_ADC5,
    NOT_USED,
};

// ADC board의 I2C Device addr. 6개의 ltc2309 device가 있음.
const unsigned char ADC_I2C_ADDR[] = {
    0x08, 0x09, 0x0A, 0x0B, 0x18, 0x19
};

// ADC Channel register address.
const unsigned char ADC_CH_ADDR[] = {
    0x88, 0xC8, 0x98, 0xD8, 0xA8, 0xE8, 0xB8, 0xF8
};

// Header pin info CON1, P1.1 ~ P1.6, P3 port control.
struct pin_info {
    const char *name;
    unsigned char pin_num;
    unsigned char adc_idx;
    unsigned char ch_idx;
};

const struct pin_info HEADER_CON1[] = {
    { "CON1.0" ,   0, NOT_USED , 0},    // Header Pin 0

    { "CON1.1" ,   1, CHIP_ADC0, 0},    // Header Pin 1 Info
    { "CON1.2" ,   2, CHIP_ADC0, 1},    // Header Pin 2 Info
    { "CON1.3" ,   3, CHIP_ADC1, 0},
    { "CON1.4" ,   4, CHIP_ADC0, 2},
    { "CON1.5" ,   5, CHIP_ADC1, 1},
    { "CON1.6" ,   6, NOT_USED , 0},
    { "CON1.7" ,   7, CHIP_ADC1, 2},
    { "CON1.8" ,   8, CHIP_ADC1, 3},
    { "CON1.9" ,   9, NOT_USED , 0},
    { "CON1.10",  10, CHIP_ADC1, 4},

    { "CON1.11",  11, CHIP_ADC1, 5},
    { "CON1.12",  12, CHIP_ADC1, 6},
    { "CON1.13",  13, CHIP_ADC1, 7},
    { "CON1.14",  14, NOT_USED , 0},
    { "CON1.15",  15, CHIP_ADC2, 0},
    { "CON1.16",  16, CHIP_ADC2, 1},
    { "CON1.17",  17, CHIP_ADC0, 3},
    { "CON1.18",  18, CHIP_ADC2, 2},
    { "CON1.19",  19, CHIP_ADC2, 3},
    { "CON1.20",  20, NOT_USED , 0},

    { "CON1.21",  21, CHIP_ADC2, 4},
    { "CON1.22",  22, CHIP_ADC2, 5},
    { "CON1.23",  23, CHIP_ADC2, 6},
    { "CON1.24",  24, CHIP_ADC2, 7},
    { "CON1.25",  25, NOT_USED , 0},
    { "CON1.26",  26, CHIP_ADC3, 0},
    { "CON1.27",  27, CHIP_ADC3, 1},
    { "CON1.28",  28, CHIP_ADC3, 2},
    { "CON1.29",  29, CHIP_ADC3, 3},
    { "CON1.30",  30, NOT_USED , 0},

    { "CON1.31",  31, CHIP_ADC3, 4},
    { "CON1.32",  32, CHIP_ADC3, 5},
    { "CON1.33",  33, CHIP_ADC3, 6},
    { "CON1.34",  34, NOT_USED , 0},
    { "CON1.35",  35, CHIP_ADC3, 7},
    { "CON1.36",  36, CHIP_ADC4, 0},
    { "CON1.37",  37, NOT_USED , 0},
    { "CON1.38",  38, CHIP_ADC0, 4},
    { "CON1.39",  39, NOT_USED , 0},
    { "CON1.40",  40, NOT_USED , 0},
};

const struct pin_info HEADER_P3[] = {
    { "P3.0" ,  0, NOT_USED , 0},   // Header Pin 0
    { "P3.1" ,  1, NOT_USED , 0},   // Header Pin 1 Info
    { "P3.2" ,  2, CHIP_ADC5, 0},   // Header Pin 2 Info
    { "P3.3" ,  3, CHIP_ADC5, 1},
    { "P3.4" ,  4, NOT_USED , 0},
    { "P3.5" ,  5, CHIP_ADC5, 2},
    { "P3.6" ,  6, CHIP_ADC5, 3},
    { "P3.7" ,  7, NOT_USED , 0},
    { "P3.8" ,  8, CHIP_ADC5, 4},
    { "P3.9" ,  9, CHIP_ADC5, 5},
    { "P3.10", 10, NOT_USED , 0},
};

const struct pin_info HEADER_P13[] = {
    { "P13.0", 0, NOT_USED , 0},    // Header Pin 0
    { "P13.1", 1, NOT_USED , 0},    // Header Pin 1 Info
    { "P13.2", 2, CHIP_ADC4, 1},    // Header Pin 2 Info
    { "P13.3", 3, CHIP_ADC0, 5},
    { "P13.4", 4, CHIP_ADC4, 2},
    { "P13.5", 5, CHIP_ADC4, 3},
    { "P13.6", 6, CHIP_ADC4, 4},
    { "P13.7", 7, CHIP_ADC4, 5},
};

const struct pin_info HEADER_P1_1[] = {
    { "P1_1.0", 0, NOT_USED , 0},   // Header Pin 0
    { "P1_1.1", 1, CHIP_ADC0, 7},   // Header Pin 1 Info
    { "P1_1.2", 2, CHIP_ADC0, 6},   // Header Pin 2 Info
    { "P1_1.3", 3, CHIP_ADC0, 5},
    { "P1_1.4", 4, CHIP_ADC0, 4},
    { "P1_1.5", 5, CHIP_ADC0, 3},
    { "P1_1.6", 6, CHIP_ADC0, 2},
    { "P1_1.7", 7, CHIP_ADC0, 1},
    { "P1_1.8", 8, CHIP_ADC0, 0},
};

const struct pin_info HEADER_P1_2[] = {
    { "P1_2.0", 0, NOT_USED , 0},   // Header Pin 0
    { "P1_2.1", 1, CHIP_ADC1, 7},   // Header Pin 1 Info
    { "P1_2.2", 2, CHIP_ADC1, 6},   // Header Pin 2 Info
    { "P1_2.3", 3, CHIP_ADC1, 5},
    { "P1_2.4", 4, CHIP_ADC1, 4},
    { "P1_2.5", 5, CHIP_ADC1, 3},
    { "P1_2.6", 6, CHIP_ADC1, 2},
    { "P1_2.7", 7, CHIP_ADC1, 1},
    { "P1_2.8", 8, CHIP_ADC1, 0},
};

const struct pin_info HEADER_P1_3[] = {
    { "P1_3.0", 0, NOT_USED , 0},   // Header Pin 0
    { "P1_3.1", 1, CHIP_ADC2, 7},   // Header Pin 1 Info
    { "P1_3.2", 2, CHIP_ADC2, 6},   // Header Pin 2 Info
    { "P1_3.3", 3, CHIP_ADC2, 5},
    { "P1_3.4", 4, CHIP_ADC2, 4},
    { "P1_3.5", 5, CHIP_ADC2, 3},
    { "P1_3.6", 6, CHIP_ADC2, 2},
    { "P1_3.7", 7, CHIP_ADC2, 1},
    { "P1_3.8", 8, CHIP_ADC2, 0},
};

const struct pin_info HEADER_P1_4[] = {
    { "P1_4.0", 0, NOT_USED , 0},   // Header Pin 0
    { "P1_4.1", 1, CHIP_ADC3, 7},   // Header Pin 1 Info
    { "P1_4.2", 2, CHIP_ADC3, 6},   // Header Pin 2 Info
    { "P1_4.3", 3, CHIP_ADC3, 5},
    { "P1_4.4", 4, CHIP_ADC3, 4},
    { "P1_4.5", 5, CHIP_ADC3, 3},
    { "P1_4.6", 6, CHIP_ADC3, 2},
    { "P1_4.7", 7, CHIP_ADC3, 1},
    { "P1_4.8", 8, CHIP_ADC3, 0},
};

const struct pin_info HEADER_P1_5[] = {
    { "P1_5.0", 0, NOT_USED , 0},   // Header Pin 0
    { "P1_5.1", 1, CHIP_ADC4, 7},   // Header Pin 1 Info
    { "P1_5.2", 2, CHIP_ADC4, 6},   // Header Pin 2 Info
    { "P1_5.3", 3, CHIP_ADC4, 5},
    { "P1_5.4", 4, CHIP_ADC4, 4},
    { "P1_5.5", 5, CHIP_ADC4, 3},
    { "P1_5.6", 6, CHIP_ADC4, 2},
    { "P1_5.7", 7, CHIP_ADC4, 1},
    { "P1_5.8", 8, CHIP_ADC4, 0},
};

const struct pin_info HEADER_P1_6[] = {
    { "P1_6.0", 0, NOT_USED , 0},   // Header Pin 0
    { "P1_6.1", 1, CHIP_ADC5, 7},   // Header Pin 1 Info
    { "P1_6.2", 2, CHIP_ADC5, 6},   // Header Pin 2 Info
    { "P1_6.3", 3, CHIP_ADC5, 5},
    { "P1_6.4", 4, CHIP_ADC5, 4},
    { "P1_6.5", 5, CHIP_ADC5, 3},
    { "P1_6.6", 6, CHIP_ADC5, 2},
    { "P1_6.7", 7, CHIP_ADC5, 1},
    { "P1_6.8", 8, CHIP_ADC5, 0},
};

#define	ARRARY_SIZE(x)	(sizeof(x) / sizeof(x[0]))

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// function prototype
//------------------------------------------------------------------------------
static  void                toupperstr      (char *p);

static  int                 read_pin        (int fd, struct pin_info *info);
static  int                 convert_to_mv   (unsigned short adc_value);
static  struct pin_info     *header_info    (const char *h_name, int pin_no, int *p_cnt);
static  int                 check_devices   (int fd);

        int adc_board_read  (int fd, const char *name, int *read_value, int *cnt);
        int adc_board_init  (const char *i2c_dev_node);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
static void toupperstr (char *p)
{
    int i, c = strlen(p);

    for (i = 0; i < c; i++, p++)
        *p = toupper(*p);
}

//------------------------------------------------------------------------------
static int read_pin (int fd, struct pin_info *info)
{
    int read_val = 0, retry = 3;

    if (info->adc_idx == NOT_USED)
        return 0;

    while (i2c_set_addr(fd, ADC_I2C_ADDR [info->adc_idx]) && retry --)
        usleep(100);

    if (retry) {
        // Dummy read for chip wake up & conversion
        i2c_read_word(fd, ADC_CH_ADDR [info->ch_idx]);
        read_val  = i2c_read_word(fd, ADC_CH_ADDR [info->ch_idx]);
        read_val  = (read_val < 0) ? 0 : read_val;
        read_val  = (SWAP_WORD (read_val) >> 4) & 0xFFF;
    }
    return read_val;
}

//------------------------------------------------------------------------------
static int convert_to_mv (unsigned short adc_value)
{
    int volt = adc_value * ADC_WEIGHT_uV;
    return	(volt / 1000);
}

//------------------------------------------------------------------------------
static struct pin_info *header_info(const char *h_name, int pin_no, int *p_cnt)
{
    const struct pin_info *p;

    if          (!strncmp("CON1", h_name, strlen ("CON1"))) {
        pin_no  = pin_no < (int)ARRARY_SIZE(HEADER_CON1) ? pin_no : 0;
        p       = pin_no ? &HEADER_CON1 [pin_no] : &HEADER_CON1[1];
        *p_cnt  = pin_no ? 1 : ARRARY_SIZE(HEADER_CON1) -1;
    } else if   (!strncmp("P3"  , h_name, strlen ("P3"))) {
        pin_no  = pin_no < (int)ARRARY_SIZE(HEADER_P3) ? pin_no : 0;
        p       = pin_no ? &HEADER_P3 [pin_no] : &HEADER_P3[1];
        *p_cnt  = pin_no ? 1 : ARRARY_SIZE(HEADER_P3) -1;
    } else if   (!strncmp("P13" , h_name, strlen ("P13"))) {
        pin_no  = pin_no < (int)ARRARY_SIZE(HEADER_P13) ? pin_no : 0;
        p       = pin_no ? &HEADER_P13 [pin_no] : &HEADER_P13[1];
        *p_cnt  = pin_no ? 1 : ARRARY_SIZE(HEADER_P13) -1;
    } else if   (!strncmp("P1_1", h_name, strlen ("P1_1"))) {
        pin_no  = pin_no < (int)ARRARY_SIZE(HEADER_P1_1) ? pin_no : 0;
        p       = pin_no ? &HEADER_P1_1 [pin_no] : &HEADER_P1_1[1];
        *p_cnt  = pin_no ? 1 : ARRARY_SIZE(HEADER_P1_1) -1;
    } else if   (!strncmp("P1_2", h_name, strlen ("P1_2"))) {
        pin_no  = pin_no < (int)ARRARY_SIZE(HEADER_P1_2) ? pin_no : 0;
        p       = pin_no ? &HEADER_P1_2 [pin_no] : &HEADER_P1_2[1];
        *p_cnt  = pin_no ? 1 : ARRARY_SIZE(HEADER_P1_2) -1;
    } else if   (!strncmp("P1_3", h_name, strlen ("P1_3"))) {
        pin_no  = pin_no < (int)ARRARY_SIZE(HEADER_P1_3) ? pin_no : 0;
        p       = pin_no ? &HEADER_P1_3 [pin_no] : &HEADER_P1_3[1];
        *p_cnt  = pin_no ? 1 : ARRARY_SIZE(HEADER_P1_3) -1;
    } else if   (!strncmp("P1_4", h_name, strlen ("P1_4"))) {
        pin_no  = pin_no < (int)ARRARY_SIZE(HEADER_P1_4) ? pin_no : 0;
        p       = pin_no ? &HEADER_P1_4 [pin_no] : &HEADER_P1_4[1];
        *p_cnt  = pin_no ? 1 : ARRARY_SIZE(HEADER_P1_4) -1;
    } else if   (!strncmp("P1_5", h_name, strlen ("P1_5"))) {
        pin_no  = pin_no < (int)ARRARY_SIZE(HEADER_P1_5) ? pin_no : 0;
        p       = pin_no ? &HEADER_P1_5 [pin_no] : &HEADER_P1_5[1];
        *p_cnt  = pin_no ? 1 : ARRARY_SIZE(HEADER_P1_5) -1;
    } else if   (!strncmp("P1_6", h_name, strlen ("P1_6"))) {
        pin_no  = pin_no < (int)ARRARY_SIZE(HEADER_P1_6) ? pin_no : 0;
        p       = pin_no ? &HEADER_P1_6 [pin_no] : &HEADER_P1_6[1];
        *p_cnt  = pin_no ? 1 : ARRARY_SIZE(HEADER_P1_6) -1;
    } else {
        p = &HEADER_CON1[0];
        *p_cnt	= 0;
    }
    return (struct pin_info *)p;
}

//------------------------------------------------------------------------------
// 6개의 ADC가 정상적으로 응답하는지 확인함.
//------------------------------------------------------------------------------
static int check_devices (int fd)
{
    int i;

    for (i = 0; i < (int)ARRARY_SIZE(ADC_I2C_ADDR); i++) {
        i2c_set_addr(fd, ADC_I2C_ADDR[i]);
        if(i2c_read_word(fd, ADC_I2C_ADDR[i]) < 0) {
            return 0;
        }
    }
#if defined (__LIB_I2CADC_APP__)
    printf ("%s pass : fd = %d\n", __func__, fd);
#endif
    return 1;
}

//------------------------------------------------------------------------------
// Header name 및 Pin 번호를 입력. CON1.1(1개의 데이터 읽어옴) or CON1 (40개의 데이터 읽어옴)
// read_value에 mv값으로 저장함.
//------------------------------------------------------------------------------
int adc_board_read (int fd, const char *h_name, int *read_value, int *cnt)
{
    char *p_name, *pin_str, str[10];
    int pin_no = 0, pin_cnt = 0, i;
    struct pin_info *p;

    if ((h_name == NULL) || !fd)
        return -1;

    memset(str,   0x00, sizeof(str));
    memcpy(str, h_name, strlen(h_name));

    if ((p_name  = strtok(str, ".")) != NULL)
        toupperstr(p_name);

    if ((pin_str = strtok(NULL, " ")) != NULL)
        pin_no = atoi(pin_str);

    p = header_info(p_name, pin_no, &pin_cnt);

// DEBUG
#if defined (__LIB_I2CADC_APP__)
    printf ("%s : header = %s, pin = %d, pin_cnt = %d\n", h_name, h_name, pin_no, pin_cnt);
#endif

    if (pin_cnt) {
        for (i = 0; i < pin_cnt; i++, p++) {
            read_value[i] = convert_to_mv (read_pin (fd, p));
#if defined (__LIB_I2CADC_APP__)
            printf ("%s.%d, value = %d mV\n",
                p_name, (pin_cnt == 1) ? pin_no : i+1, read_value[i]);
#endif
        }
        *cnt = pin_cnt;
        return 1;
    }
#if defined (__LIB_I2CADC_APP__)
    else
        printf ("can't found %s pin or header\n", h_name);
#endif

    return 0;
}

//------------------------------------------------------------------------------
int adc_board_init (const char *i2c_dev_node)
{
    int fd;

    if ((fd = i2c_open(i2c_dev_node)) < 0)
        return 0;

    if (check_devices (fd))
        return fd;

    printf ("Can not found adc board. i2c_dev = %s\n", i2c_dev_node);
    close (fd);

    return -1;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------