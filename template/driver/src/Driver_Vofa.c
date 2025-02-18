#include "Driver_Vofa.h"
void Task_Vofa_Send(void * Parameters){
  TickType_t LastWakeTime = xTaskGetTickCount();
  ProtocolInfo_Type *protocolInfo = Parameters;
  Node_Type * node = protocolInfo->node;
  uint32_t deviceID = node->deviceID;
  while (1)
  {
    /* code */
    DMA_Disable(USARTx_Tx);
    DMA_Enable(USARTx_Tx, protocolInfo->length);
    vTaskDelayUntil(&LastWakeTime, 1000.0f/protocolInfo->frequency);
  }
  
}

void Vofa_Send(uint16_t frequency, uint16_t heapDeth, Node_Type * node, uint32_t commandID){
  ProtocolData.debugInfo.debugData.vofaTail = vofaTail_t;
  ProtocolInfo_Type *protocolInfo = Protocol_Get_Info_Handle(commandID);
  if (protocolInfo->taskHandle != 0) {
    vTaskDelete(protocolInfo->taskHandle);
  }
  protocolInfo->node = node;
  protocolInfo->frequency = frequency;
  uint32_t deviceID = node->deviceID;
  BSP_DMA_Init(USARTx_Tx, ProtocolData.debugInfo.data, Protocol_Buffer_Length);
  xTaskCreate(Task_Vofa_Send, "Vofa_Send", heapDeth, (void *)protocolInfo, 6, protocolInfo->taskHandle);
};