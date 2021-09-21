#ifndef _MAVLINK_UTIL_H_
#define _MAVLINK_UTIL_H_

#include <stdint.h>

// Same as mavlink severity.
enum SEVERITY {
   SEVERITY_EMERGENCY=0, /* System is unusable. This is a "panic" condition. | */
   SEVERITY_ALERT=1, /* Action should be taken immediately. Indicates error in non-critical systems. | */
   SEVERITY_CRITICAL=2, /* Action must be taken immediately. Indicates failure in a primary system. | */
   SEVERITY_ERROR=3, /* Indicates an error in secondary/redundant systems. | */
   SEVERITY_WARNING=4, /* Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning. | */
   SEVERITY_NOTICE=5, /* An unusual event has occurred, though not an error condition. This should be investigated for the root cause. | */
   SEVERITY_INFO=6, /* Normal operational messages. Useful for logging. No action is required for these messages. | */
   SEVERITY_DEBUG=7, /* Useful non-operational messages that can assist in debugging. These should not occur during normal operation. | */
   SEVERITY_ENUM_END=8, /*  | */
};

int mavlink_printf(uint8_t severity, const char *fmt, ...);

int mavlink_send_parameter(const char *key);

int mavlink_send_parameter_list();

#endif // _MAVLINK_UTIL_H_
