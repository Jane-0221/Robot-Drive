#ifndef __OMNI_WHEEL_H__
#define __OMNI_WHEEL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern volatile float x;
extern volatile float y;
extern volatile float w;

void Omni_Wheel_Init(void);
void Omni_Wheel_Update(void);
void Omni_Wheel_RxCallback(uint32_t ext_id, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif
