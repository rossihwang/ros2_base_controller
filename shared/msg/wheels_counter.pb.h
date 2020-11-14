/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.2-dev */

#ifndef PB_WHEELS_COUNTER_PB_H_INCLUDED
#define PB_WHEELS_COUNTER_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _WheelsCounter {
    int32_t left;
    int32_t right;
} WheelsCounter;


/* Initializer values for message structs */
#define WheelsCounter_init_default               {0, 0}
#define WheelsCounter_init_zero                  {0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define WheelsCounter_left_tag                   1
#define WheelsCounter_right_tag                  2

/* Struct field encoding specification for nanopb */
#define WheelsCounter_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, INT32,    left,              1) \
X(a, STATIC,   SINGULAR, INT32,    right,             2)
#define WheelsCounter_CALLBACK NULL
#define WheelsCounter_DEFAULT NULL

extern const pb_msgdesc_t WheelsCounter_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define WheelsCounter_fields &WheelsCounter_msg

/* Maximum encoded size of messages (where known) */
#define WheelsCounter_size                       22

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif