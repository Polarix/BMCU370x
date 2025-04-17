#ifndef _INCLUDED_BAMBU_BUS_QUEUE_H_
#define _INCLUDED_BAMBU_BUS_QUEUE_H_
//===========================================================//
//= Include files.                                          =//
//===========================================================//
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include <raw_queue.h>

//===========================================================//
//= Macro definition.                                       =//
//===========================================================//

//===========================================================//
//= Data type definition.                                   =//
//===========================================================//

//===========================================================//
//= Public function declaration.                            =//
//===========================================================//
#ifdef __cplusplus
extern "C"
{
#endif

void bambu_bus_queue_init(void);
bool bambu_bus_read_byte(uint8_t* byte);
uint32_t bambu_bus_queue_read(uint8_t* buffer, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif // _INCLUDED_BAMBU_BUS_QUEUE_H_
