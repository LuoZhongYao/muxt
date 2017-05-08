/*************************************************
 * Anthor  : LuoZhongYao@gmail.com
 * Modified: 2017/05/05
 ************************************************/
#ifndef __LOG_H__
#define __LOG_H__

#include <syslog.h>

#define LOGLEVEL_ERROR      0x0001
#define LOGLEVEL_WARNING    0x0002
#define LOGLEVEL_INFO       0x0004
#define LOGLEVEL_DEBUG      0x0008

extern int loglevel;
//#define LOG(level, fmt, ...) printf(fmt, ##__VA_ARGS__)
#define LOG(level, fmt, ...) syslog(loglevel, fmt, ##__VA_ARGS__)
#define LOGD(fmt,...) do {if(loglevel & LOGLEVEL_DEBUG) LOG(LOG_DEBUG, fmt, ##__VA_ARGS__);} while(0)
#define LOGW(fmt,...) do {if(loglevel & LOGLEVEL_WARNING) LOG(LOG_WARN, fmt, ##__VA_ARGS__);} while(0)
#define LOGE(fmt,...) do {if(loglevel & LOGLEVEL_ERROR) LOG(LOG_ERR, fmt, ##__VA_ARGS__);} while(0)
#define LOGI(fmt,...) do {if(loglevel & LOGLEVEL_INFO) LOG(LOG_INFO, fmt, ##__VA_ARGS__);} while(0)

#endif

