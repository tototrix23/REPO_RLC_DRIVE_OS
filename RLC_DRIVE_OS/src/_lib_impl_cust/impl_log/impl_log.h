#ifndef IMPL_LOG_H
#define IMPL_LOG_H

#include <stdarg.h>
#include <_core/c_typedefs.h>
#include <_core/c_return_codes.h>
#include <_core/c_error/c_error.h>
#include <_interfaces/i_log/i_log.h>

#define LOG_BUFFER_SIZE    64

#define LOG_COLOR_ERROR    12
#define LOG_COLOR_WARN     6
#define LOG_COLOR_INFO     11
#define LOG_COLOR_DEBUG    14
#define LOG_COLOR_DEFAULT  15



typedef struct st_log_t
{
    uint8_t mode;
    uint8_t color;
    uint16_t line;
    char module[16];
    char func[32];
    char text[64];
}
__attribute__((packed))      // Remove interfield padding.
__attribute__((aligned(4)))  // Set alignment and add tail padding.
log_t;



void impl_log_write(uint8_t mode,uint8_t color,char*module,  char* file,const char* func, uint16_t line, char *va_string);
void impl_log_write_e(uint8_t mode,char *module, char* s, char* file,const char* func, uint16_t line, va_list argp);
void impl_log_write_w(uint8_t mode,char *module, char* s, char* file,const char* func, uint16_t line, va_list argp);
void impl_log_write_i(uint8_t mode,char *module, char* s, char* file,const char* func, uint16_t line, va_list argp);
void impl_log_write_d(uint8_t mode,char *module, char* s, char* file,const char* func, uint16_t line, va_list argp);


#endif
