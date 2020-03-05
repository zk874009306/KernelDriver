/*
 * common.c
 *
 *  Created on: Oct 22, 2019
 *      Author: binnary
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "common.h"
#include "logging.h"


#define ZEROPAD 0x01       /* pad with zero */
#define SIGN    0x02       /* unsigned/signed long */
#define PLUS    0x04       /* show plus */
#define SPACE   0x08       /* space if plus */
#define LEFT    0x10       /* left justified */
#define SPECIAL 0x20       /* need 0x */
#define LARGE   0x40       /* use 'ABCDEF' instead of 'abcdef' */



static char sprint_buf[512];
int logging_level= LOG_INFO;
void set_log_level(uint8_t level)
{
	if(LOG_DEBUG < level){
		return;
	}
	logging_level=level;
}

static int skip_atoi(const char **s)
{
	int i = 0;
	while (isdigit(**s)) {
		i = i * 10 + *((*s)++) - '0';
	}
	return i;
}
static char *number(char *buf, unsigned long num, int base, int size, int precision, int type)
{
    /* we are called with base 8, 10 or 16, only, thus don't need "G..."  */
    static const char digits[16] = "0123456789ABCDEF";

	char tmp[66];
	char sign;
	char locase;
	int need_pfx = ((type & SPECIAL) && base != 10);
	int i;

	locase = (type & SPECIAL);
	if (type & LEFT) {
		type &= ~ZEROPAD;
	}
	sign = 0;
	if (type & SIGN) {
		if ((signed long) num < 0) {
			sign = '-';
			num = -(signed long) num;
			size--;
		} else if (type & PLUS) {
			sign = '+';
			size--;
		} else if (type & SPACE) {
			sign = ' ';
			size--;
		}
	}

	if (need_pfx) {
		size--;
		if (base == 16)
			size--;
	}

	/* generate full string in tmp[], in reverse order */
	i = 0;
	if (num == 0) {
		tmp[i++] = '0';
	} else {
		if (base != 10) {/* 8 or 16 */
			int mask = base - 1;
			int shift = 3;
			if (base == 16) {
				shift = 4;
			}

			do {
				tmp[i++] = (digits[((unsigned char) num) & mask] | locase);
				num >>= shift;
			} while (num);
		} else { /* base 10 */
			do {
				tmp[i++] = digits[num % base];
				num /= base;
			} while (num);
		}
	}

	/* printing 100 using %2d gives "100", not "00" */
	if (i > precision) {
		precision = i;
	}
	/* leading space padding */
	size -= precision;
	if (!(type & (ZEROPAD + LEFT))) {
		while (--size >= 0) {
			*buf++ = ' ';
		}
	}
	/* sign */
	if (sign)
		*buf++ = sign;
	/* "0x" / "0" prefix */
	if (need_pfx) {
		*buf++ = '0';
		if (base == 16) {
			*buf++ = ('X' | locase);
		}
	}
	/* zero or space padding */
	if (!(type & LEFT)) {
		char c = (type & ZEROPAD) ? '0' : ' ';
		while (--size >= 0) {
			*buf++ = c;
		}
	}
	/* hmm even more zero padding? */
	while (i <= --precision)
		*buf++ = '0';
	/* actual digits of result (reverse)*/
	while (--i >= 0) {
		*buf++ = tmp[i];
	}
	/* trailing space padding */
	while (--size >= 0)
		*buf++ = ' ';
	return buf;
}
static int _vsprintf(char *buf, const char *fmt, va_list args)
{
    int len;
    unsigned int num;
    int i, base;
    char * str;
    const char *s;
    int flags;          /* flags to number() */
    int field_width;    /* width of output field */
    int precision;      /* min. # of digits for integers; max
                           number of chars for from string */
    int qualifier;      /* 'h', 'l', or 'q' for integer fields */

    for (str=buf ; *fmt ; ++fmt) {
        if (*fmt != '%') {
            *str++ = *fmt;
            continue;
        }
        /* process flags */
        flags = 0;
repeat:
        ++fmt;      /* this also skips first '%' */
        switch (*fmt) {
            case '-': flags |= LEFT; goto repeat;
            case '+': flags |= PLUS; goto repeat;
            case ' ': flags |= SPACE; goto repeat;
            case '#': flags |= SPECIAL; goto repeat;
            case '0': flags |= ZEROPAD; goto repeat;
        }

        /* get field width */
        field_width = -1;

        if (isdigit(*fmt)){
            field_width = skip_atoi(&fmt);
        } else if (*fmt == '*') {
            ++fmt;
            field_width = va_arg(args, int); /* it's the next argument */
            if (field_width < 0) {
                field_width = -field_width;
                flags |= LEFT;
            }
        }

        precision = -1;/* get the precision */
        if (*fmt == '.') {
            ++fmt;
            if (isdigit(*fmt)) {
                precision = skip_atoi(&fmt);
            } else if (*fmt == '*') {
                ++fmt;
                precision = va_arg(args, int);/* it's the next argument */
            }
            if (precision < 0) {
                precision = 0;
            }
        }

        qualifier = -1;/* get the conversion qualifier */
        if (*fmt == 'h' || *fmt == 'l') {
            qualifier = *fmt;
            ++fmt;
        }

        base = 10;/* default base */
        switch (*fmt) {
        case 'c':{
                if (!(flags & LEFT)) {
                    while (--field_width > 0)
                        *str++ = ' ';
                }
                *str++ = (unsigned char) va_arg(args, int);

                while (--field_width > 0) {
                    *str++ = ' ';
                }
            }
            continue;

        case 's':{
                s = va_arg(args, char *);
                if (!s) {
                    s = "<NULL>";
                }

                len = strnlen(s, precision);
                if (!(flags & LEFT)) {

                    while (len < field_width--){
                        *str++ = ' ';
                    }
                }
                for (i = 0; i < len; ++i) {
                    *str++ = *s++;
                }
                while (len < field_width--) {
                    *str++ = ' ';
                }
            }
            continue;
        case 'n':{
                if (qualifier == 'l') {
                    long * ip = va_arg(args, long *);
                    *ip = (str - buf);

                } else {
                    int * ip = va_arg(args, int *);
                    *ip = (str - buf);
                }
            }
            continue;

        case '%':
            *str++ = '%';
            continue;
        /* integer number formats - set up the flags and "break" */
        case 'o':
            base = 8;
            break;
        case 'X':
            flags |= LARGE;
        case 'x':
            base = 16;
            break;

        case 'd':
        case 'i':
            flags |= SIGN;
        case 'u':
            break;

        default:
            *str++ = '%';
            if (*fmt)
                *str++ = *fmt;
            else
                --fmt;
            continue;
        }

        if (qualifier == 'l') {
            num = va_arg(args, unsigned long);

        } else if (qualifier == 'h') {
            num = (unsigned short) va_arg(args, int);

            if (flags & SIGN) {
                num = (short) num;
            }

        } else if (flags & SIGN) {
            num = va_arg(args, int);

        } else {
            num = va_arg(args, unsigned int);
        }
        str = number(str, num, base, field_width, precision, flags);
    }
    *str = '\0';
    return str-buf;
}
int logging_vputs(const char *fmt,...)
{
	int n;
	va_list args;
	va_start(args, fmt);
	n = _vsprintf(sprint_buf, fmt, args);
	va_end(args);
	USBAcmSend(0,(uint8_t*)sprint_buf,n);
	return n;
}
