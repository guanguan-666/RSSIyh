#ifndef _APP_LORA_GATEWAY_H_
#define _APP_LORA_GATEWAY_H_

void Gateway_SendBeacon(void);
void Gateway_Task_Loop(void);
void Gateway_Send_Join_ACK(uint8_t node_id, uint8_t slot_id);
int Gateway_Find_Free_Slot(uint8_t node_id);
void Gateway_Listen_CAP(int duration_ms);
void Gateway_Listen_TDMA(int duration_ms);
void Gateway_Send_Reply(uint8_t dest_id, uint8_t cmd, float param);

#endif
