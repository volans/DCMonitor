/**
  ******************************************************************************
  * @file    usb_endp.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Endpoint routines
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
//#include "usb_bot.h"
#include "usb_istr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO u8 USB_R_Buffer[256];
extern __IO u8 USB_S_Buffer[256];
extern __IO u8 USB_Receive_Flag, USB_Send_Flag;
extern __IO u16 USB_S_Size, USB_R_Size;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    : EP1 IN Callback Routine
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void)
{
   USB_Send_Flag = 0xff;
  //Mass_Storage_In();
}

/*******************************************************************************
* Function Name  : EP2_OUT_Callback.
* Description    : EP2 OUT Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP2_OUT_Callback(void)
{
  USB_R_Size = GetEPRxCount(ENDP2);
  PMAToUserBufferCopy((u8 *)USB_R_Buffer, ENDP2_RXADDR, USB_R_Size);
  SetEPRxStatus(ENDP2, EP_RX_VALID); /* enable the next transaction*/
  USB_Receive_Flag = 0xff;
  //Mass_Storage_Out();
}

/*******************************************************************************
* Function Name  : EP1_Send_Callback.
* Description    : EP1 Sending data Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_Send_Callback(void)
{
  UserToPMABufferCopy((u8 *)USB_S_Buffer, ENDP1_TXADDR, USB_S_Size);
  SetEPTxCount(ENDP1, USB_S_Size);
  SetEPTxStatus(ENDP1, EP_TX_VALID);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

