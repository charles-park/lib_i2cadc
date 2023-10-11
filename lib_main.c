//------------------------------------------------------------------------------
/**
 * @file lib_main.c
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
//------------------------------------------------------------------------------
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/fb.h>
#include <getopt.h>

#include "lib_i2cadc.h"

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#if defined(__LIB_I2CADC_APP__)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
static void print_usage (const char *prog)
{
    puts("");
    printf("Usage: %s [-D:device] [-p:pin name] [-v]\n", prog);
    puts("\n"
         "  -D --Device         Control Device node(i2c dev)\n"
         "  -p --pin name       Header pin name in adc board (con1, con1.1...)\n"
         "  -v --view all port  ALL Haader pin info display.\n"
         "\n"
         "  e.g) ./lib_i2cadc -D /dev/i2c-0 -p con1.1\n"
         "\n"
    );
    exit(1);
}

//------------------------------------------------------------------------------
/* Control server variable */
//------------------------------------------------------------------------------
static char *OPT_DEVICE_NODE    = NULL;
static char *OPT_PIN_NAME       = NULL;
static char  OPT_VIEW_INFO      = 0;

//------------------------------------------------------------------------------
// 문자열 변경 함수. 입력 포인터는 반드시 메모리가 할당되어진 변수여야 함.
//------------------------------------------------------------------------------
static void tolowerstr (char *p)
{
    int i, c = strlen(p);

    for (i = 0; i < c; i++, p++)
        *p = tolower(*p);
}

//------------------------------------------------------------------------------
static void toupperstr (char *p)
{
    int i, c = strlen(p);

    for (i = 0; i < c; i++, p++)
        *p = toupper(*p);
}

//------------------------------------------------------------------------------
static void parse_opts (int argc, char *argv[])
{
    while (1) {
        static const struct option lopts[] = {
            { "Device",     1, 0, 'D' },
            { "read_word",  1, 0, 'p' },
            { "read_byte",  0, 0, 'v' },
            { NULL, 0, 0, 0 },
        };
        int c;

        c = getopt_long(argc, argv, "D:p:vh", lopts, NULL);

        if (c == -1)
            break;

        switch (c) {
        case 'D':
            OPT_DEVICE_NODE = optarg;
            break;
        /* Header pin name */
        case 'p':
            OPT_PIN_NAME = optarg;
            break;
        /* ALL header info view */
        case 'v':
            OPT_VIEW_INFO = 1;
            break;
        case 'h':
        default:
            print_usage(argv[0]);
            break;
        }
    }
}

//------------------------------------------------------------------------------------------------------------
int print_pin_info (int fd, const char *h_name)
{
    int r_pin_mv[100], r_pin_cnt = 0, i;

    if (h_name == NULL)
        return -1;

    if (adc_board_read   (fd, h_name, r_pin_mv, &r_pin_cnt)) {
        if (r_pin_cnt) {
            printf ("%10s\t%s\n", "PIN Name","mV");
            printf ("--------------------------\n");
            for (i = 0; i < r_pin_cnt; i++) {
                if (r_pin_cnt > 1)
                    printf ("%8s.%02d\t%d\n", h_name, i+1, r_pin_mv[i]);
                else
                    printf ("%10s\t%d\n", h_name, r_pin_mv[i]);
            }
        }
    }
    else
        printf ("can't found %s pin or header\n", h_name);

    return 0;
}

//------------------------------------------------------------------------------------------------------------
void print_all_info (int fd)
{
    print_pin_info(fd, "CON1");
    print_pin_info(fd, "P3");
    print_pin_info(fd, "P13");
    print_pin_info(fd, "P1_1");
    print_pin_info(fd, "P1_2");
    print_pin_info(fd, "P1_3");
    print_pin_info(fd, "P1_4");
    print_pin_info(fd, "P1_5");
    print_pin_info(fd, "P1_6");
}

//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
int main (int argc, char *argv[])
{
    int fd;

    parse_opts(argc, argv);

    if (OPT_DEVICE_NODE == NULL)
        print_usage(argv[0]);

    if ((fd = adc_board_init (OPT_DEVICE_NODE)) < 0)
        return -1;

    if (OPT_VIEW_INFO)
        print_all_info (fd);

    if (OPT_PIN_NAME)
        print_pin_info (fd, OPT_PIN_NAME);

    close(fd);

    return 0;
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#endif  // #if defined(__LIB_I2CADC_APP__)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
