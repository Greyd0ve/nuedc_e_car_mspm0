#ifndef __APP_AIM_LINK_H
#define __APP_AIM_LINK_H

/*
 * Reserved link interface for a future K230 aiming module.
 *
 * The current project uses the available UART for serial tuning, so this
 * module is not wired into the scheduler by default. Call these functions only
 * after assigning a UART/link path for the aiming module.
 */
void AimLink_Init(void);
void AimLink_Task100ms(void);
void AimLink_SendCarState(void);
void AimLink_OnAimPacket(const char *packet);

#endif
