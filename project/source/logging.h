/*
 * logging.h
 *
 *  Created on: 2017年3月30日
 *      Author: binnary
 */

#ifndef INCLUDE_LOGGING_H_
#define INCLUDE_LOGGING_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum{
    LOG_EMERG,//  KERN_SOH "0"    /* system is unusable */
    LOG_ALERT,//  KERN_SOH "1"    /* action must be taken immediately */
    LOG_CRIT,//   KERN_SOH "2"    /* critical conditions */
    LOG_ERR,//    KERN_SOH "3"    /* error conditions */
    LOG_WARNING,// KERN_SOH "4"    /* warning conditions */
    LOG_NOTICE,// KERN_SOH "5"    /* normal but significant condition */
    LOG_INFO,//   KERN_SOH "6"    /* informational */
    LOG_DEBUG,//  KERN_SOH "7"    /* debug-level messages */
}loglevel;
#define STR_LOG_EMERG   "0 "    /* system is unusable */
#define STR_LOG_ALERT   "1 "    /* action must be taken immediately */
#define STR_LOG_CRIT    "2 "    /* critical conditions */
#define STR_LOG_ERR     "3 "    /* error conditions */
#define STR_LOG_WARNING "4 "    /* warning conditions */
#define STR_LOG_NOTICE  "5 "    /* normal but significant condition */
#define STR_LOG_INFO    "6 "    /* informational */
#define STR_LOG_DEBUG   "7 "    /* debug-level messages */

extern int logging_vputs(const char *fmt,...) ;
extern int logging_level;

void logging_set_level(int level);


#define log_error(fmt, ...) \
    do{\
          if (logging_level >= LOG_ERR){ \
              logging_vputs(fmt, ##__VA_ARGS__);\
          }\
    }while(0)
#define log_warning(fmt, ...) \
    do{\
        if (logging_level >= LOG_WARNING){ \
            logging_vputs(fmt, ##__VA_ARGS__);\
        }\
    }while(0)

#define log_notice(fmt, ...) \
    do{\
        if (logging_level >= LOG_NOTICE){ \
            logging_vputs(STR_LOG_NOTICE fmt, ##__VA_ARGS__);\
        }\
    }while(0)

#define log_info(fmt, ...) \
    do{\
        if (logging_level >= LOG_INFO){ \
            logging_vputs(STR_LOG_INFO fmt, ##__VA_ARGS__);\
        }\
    }while(0)

#define log_debug(fmt, ...) \
    do{\
        if (logging_level >= LOG_DEBUG){ \
            logging_vputs(STR_LOG_DEBUG fmt, ##__VA_ARGS__);\
        }\
    }while(0)

extern void loggingThread(void *arg0);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_LOGGING_H_ */
