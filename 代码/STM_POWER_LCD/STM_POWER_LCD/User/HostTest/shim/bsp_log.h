/* Host shim for bsp_log.h.
 *
 * can_proto.c logs through LOGI/LOGW/LOGE, which come from bsp_log.h. The real
 * header pulls in bsp_config.h -> main.h -> the STM32 HAL, so on a PC it cannot
 * be used. Shadowing the header is enough - can_proto.c needs no change.
 *
 * IMPORTANT: the macros must genuinely CONSUME their arguments. A no-op macro
 * that drops them makes GCC report the variables as unused, which under -Werror
 * is a build failure - and worse, it would hide a genuine unused variable in the
 * firmware. log_probe() therefore takes the format string plus the variadic
 * arguments so they stay live.
 *
 * This directory goes FIRST on the include path for the host build only.
 */
#ifndef BSP_LOG_H
#define BSP_LOG_H
#include <stdint.h>
void bsp_log_scan_hit(uint16_t hal_addr);

void host_log_reset(void);
int  host_log_count(const char *needle);
void host_log_probe(const char *level, const char *tag, const char *fmt, ...);

#define LOGI(tag, ...) host_log_probe("INFO ", tag, __VA_ARGS__)
#define LOGW(tag, ...) host_log_probe("WARN ", tag, __VA_ARGS__)
#define LOGE(tag, ...) host_log_probe("ERROR", tag, __VA_ARGS__)

#endif
