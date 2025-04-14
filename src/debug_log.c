//===========================================================//
//= Include files.                                          =//
//===========================================================//
#include "Debug_log.h"
#include <debug_log_bsp.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

//===========================================================//
//= Macro definition.                                       =//
//===========================================================//
#define DBG_LOG_TX_BUF_LEN  (128)

//===========================================================//
//= Macro definition.                                       =//
//===========================================================//
static char s_dbg_log_buf[DBG_LOG_TX_BUF_LEN];

//===========================================================//
//= Static function declare.                                =//
//===========================================================//

//===========================================================//
//= Static variable defition.                               =//
//===========================================================//
void debug_init(void)
{
    debug_log_bsp_init();
}

void dbg_log_fmt(const char *fmt, ...)
{
    va_list args;
    /* Waiting for last transfer done. */
    while(debug_log_bsp_isbusy());
    va_start(args, fmt);
    int log_size = vsnprintf(&s_dbg_log_buf[0], DBG_LOG_TX_BUF_LEN, fmt, args);
    va_end(args);
    /* Write console content. */
    debug_log_bsp_send_data(s_dbg_log_buf, log_size);
}

void dbg_log_text(const char* txt)
{
    uint32_t txt_len = strlen(txt);
    debug_log_bsp_send_data(txt, txt_len);
}

void dbg_log_raw(const void *data, uint32_t len)
{
    debug_log_bsp_send_data(data, len);
}
