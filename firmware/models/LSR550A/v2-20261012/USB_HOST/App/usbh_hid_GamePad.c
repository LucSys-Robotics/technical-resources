#include "usbh_hid_GamePad.h"
#include "usbh_hid.h"

#include "FreeRTOS.h"
#include "timers.h"

//Handle support pack
#include "bsp_gamepad.h"

//Xbox 360 controller
#include "xbox360_gamepad.h"

//Wired USB PS2 controller
#include "WiredPS2_gamepad.h"

//Characteristic parameters of game controllers: when PID and VID are the same, these parameters are used to distinguish different brands
char GamePad_Manufacturer[100] = { 0 };
char GamePad_iSerialNum[100] = { 0 } ;

//GamePad_HID class-related function declarations
static USBH_StatusTypeDef USBH_HID_InterfaceInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_InterfaceDeInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_ClassRequest(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_Process(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_SOFProcess(USBH_HandleTypeDef *phost);
static void USBH_HID_ParseHIDDesc(HID_DescTypeDef *desc, uint8_t *buf);

//usb controller initialization declaration
extern USBH_StatusTypeDef USBH_HID_GamePadInit(USBH_HandleTypeDef *phost);

//Handle types for special initialization needs
void SwitchPro_GamePad_Init(USBH_HandleTypeDef *phost);

//Define the hid class for PS2
USBH_ClassTypeDef  GamePad_HID_Class =
{
  .Name = "HID",
  .ClassCode = USB_HID_CLASS, //Defaults to an HID device
  .Init = USBH_HID_InterfaceInit,
  .DeInit = USBH_HID_InterfaceDeInit,
  .Requests = USBH_HID_ClassRequest,
  .BgndProcess = USBH_HID_Process,
  .SOFProcess = USBH_HID_SOFProcess,
  .pData = NULL,
};

//Nonstandard HID
USBH_ClassTypeDef  GamePad_NonStdHID_Class =
{
  .Name = "HID",
  .ClassCode = 0xff, //Some controllers are not standard HID devices but 0xff
  .Init = USBH_HID_InterfaceInit,
  .DeInit = USBH_HID_InterfaceDeInit,
  .Requests = USBH_HID_ClassRequest,
  .BgndProcess = USBH_HID_Process,
  .SOFProcess = USBH_HID_SOFProcess,
  .pData = NULL,
};


/**
  * @brief  USBH_HID_InterfaceInit
  *         The function init the HID class.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_InterfaceInit(USBH_HandleTypeDef *phost)
{
  USBH_StatusTypeDef status;
  HID_HandleTypeDef *HID_Handle;
  uint16_t ep_mps;
  uint8_t max_ep;
  uint8_t num = 0U;
  uint8_t interface;

	USBH_UsrLog("start find interface now");
	
	//Interface matching, mainly matching the following array interface
	//phost->device.CfgDesc.Itf_Desc[0,1,2,3...,n...].bInterfaceSubClass

	//0xff indicates matching all types
	interface = USBH_FindInterface(phost, 0xFFU, 0xFFU, 0xFFU);
	
	if ((interface == 0xFFU) || (interface >= USBH_MAX_NUM_INTERFACES)) /* No Valid Interface */
	{
		USBH_DbgLog("Cannot Find the interface for %s class.", phost->pActiveClass->Name);
		return USBH_FAIL;
	}

  status = USBH_SelectInterface(phost, interface);

  if (status != USBH_OK)
  {
    return USBH_FAIL;
  }

  phost->pActiveClass->pData = (HID_HandleTypeDef *)USBH_malloc(sizeof(HID_HandleTypeDef));
  HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  if (HID_Handle == NULL)
  {
    USBH_DbgLog("Cannot allocate memory for HID Handle");
    return USBH_FAIL;
  }

  /* Initialize hid handler */
  (void)USBH_memset(HID_Handle, 0, sizeof(HID_HandleTypeDef));

  HID_Handle->state = USBH_HID_ERROR;

  //Identify different device types based on the PID and VID of HID devices
  if( phost->device.DevDesc.idProduct == Wired_PS2_PID && phost->device.DevDesc.idVendor==Wired_PS2_VID )
  {	  //PID/VID for wired PS2 controllers
	  USBH_UsrLog("Wired PS2 device found!");
	  gamepad_brand = PS2_USB_Wired;
	  GamePadInterface = &Wired_USB_PS2Gamepad;
	  HID_Handle->Init = USBH_HID_GamePadInit;
  }
  else if( phost->device.DevDesc.idProduct == WiredV2_PS2_PID && phost->device.DevDesc.idVendor==WiredV2_PS2_VID )
  {
	  //Second-generation USB wired
	  USBH_UsrLog("Wired PS2 V2 device found!");
	  gamepad_brand = PS2_USB_WiredV2;
	  GamePadInterface = &Wired_USB_PS2Gamepad;
	  HID_Handle->Init = USBH_HID_GamePadInit;
  }
  
//  else if( phost->device.DevDesc.idProduct == Wireless_PC_PS2_PID && phost->device.DevDesc.idVendor==Wireless_PC_PS2_VID )
//  { // Wireless PS2 controller PC mode PID/VID
//	  USBH_UsrLog("Wireless PC PS2 device found!");
//	  gamepad_brand = PS2_Wiredless_PC;
//	  HID_Handle->Init = USBH_HID_GamePadInit;
//  }
  
  else if( phost->device.DevDesc.idProduct == Xbox360_GamePad_PID && phost->device.DevDesc.idVendor==Xbox360_GamePad_VID )
  {	
		//Distinguish identical PID and VID products using manufacturer data
		if( strcmp(GamePad_Manufacturer,"Flydigi") == 0 )
		{
			gamepad_brand = Xbox360;
		}
		else
		{
			gamepad_brand = PS2_USB_Wiredless;
		}
		//XBOX360 platform
		USBH_UsrLog("XBox360 device found!");
		GamePadInterface = &Xbox360Gamepad;
		HID_Handle->Init = USBH_HID_GamePadInit;
  }
  
  else
  {
	gamepad_brand = UnKnown_Dev;//Unknown device type
	USBH_UsrLog("Protocol not supported.PID:%X\tVID:%X\r\n",phost->device.DevDesc.idProduct,phost->device.DevDesc.idVendor);
    return USBH_FAIL;
  }

  HID_Handle->state     = USBH_HID_INIT;
  HID_Handle->ctl_state = USBH_HID_REQ_INIT;
  HID_Handle->ep_addr   = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].bEndpointAddress;
  HID_Handle->length    = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].wMaxPacketSize;
  HID_Handle->poll      = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].bInterval;
  
  if (HID_Handle->poll < HID_MIN_POLL)
  {
    HID_Handle->poll = HID_MIN_POLL;
  }

  /* Check of available number of endpoints */
  /* Find the number of EPs in the Interface Descriptor */
  /* Choose the lower number in order not to overrun the buffer allocated */
  max_ep = ((phost->device.CfgDesc.Itf_Desc[interface].bNumEndpoints <= USBH_MAX_NUM_ENDPOINTS) ?
            phost->device.CfgDesc.Itf_Desc[interface].bNumEndpoints : USBH_MAX_NUM_ENDPOINTS);


  /* Decode endpoint IN and OUT address from interface descriptor */
  for (num = 0U; num < max_ep; num++)
  {
//	USBH_UsrLog("Found endpoint %d: Address = 0x%02X, Type = 0x%02X", num,
//				phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress,
//				phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bmAttributes);
	  
    if ((phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress & 0x80U) != 0U)
    {
      HID_Handle->InEp = (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress);
      HID_Handle->InPipe = USBH_AllocPipe(phost, HID_Handle->InEp);
      ep_mps = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].wMaxPacketSize;

      /* Open pipe for IN endpoint */
      (void)USBH_OpenPipe(phost, HID_Handle->InPipe, HID_Handle->InEp, phost->device.address,
                          phost->device.speed, USB_EP_TYPE_INTR, ep_mps);

      (void)USBH_LL_SetToggle(phost, HID_Handle->InPipe, 0U);
    }
    else
    {
      HID_Handle->OutEp = (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress);
      HID_Handle->OutPipe = USBH_AllocPipe(phost, HID_Handle->OutEp);
      ep_mps = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].wMaxPacketSize;

      /* Open pipe for OUT endpoint */
      (void)USBH_OpenPipe(phost, HID_Handle->OutPipe, HID_Handle->OutEp, phost->device.address,
                          phost->device.speed, USB_EP_TYPE_INTR, ep_mps);

      (void)USBH_LL_SetToggle(phost, HID_Handle->OutPipe, 0U);
    }
  }
  
  return USBH_OK;
}

/**
  * @brief  USBH_HID_InterfaceDeInit
  *         The function DeInit the Pipes used for the HID class.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_InterfaceDeInit(USBH_HandleTypeDef *phost)
{
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  if (HID_Handle->InPipe != 0x00U)
  {
    (void)USBH_ClosePipe(phost, HID_Handle->InPipe);
    (void)USBH_FreePipe(phost, HID_Handle->InPipe);
    HID_Handle->InPipe = 0U;     /* Reset the pipe as Free */
  }

  if (HID_Handle->OutPipe != 0x00U)
  {
    (void)USBH_ClosePipe(phost, HID_Handle->OutPipe);
    (void)USBH_FreePipe(phost, HID_Handle->OutPipe);
    HID_Handle->OutPipe = 0U;     /* Reset the pipe as Free */
  }

  if ((phost->pActiveClass->pData) != NULL)
  {
    USBH_free(phost->pActiveClass->pData);
    phost->pActiveClass->pData = 0U;
  }

  //Deinitialization will occur when the device is removed
  USB_GamePad_PullOutCallback();
  
  return USBH_OK;
}

/**
  * @brief  USBH_HID_ClassRequest
  *         The function is responsible for handling Standard requests
  *         for HID class.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_ClassRequest(USBH_HandleTypeDef *phost)
{

  USBH_StatusTypeDef status         = USBH_BUSY;
  USBH_StatusTypeDef classReqStatus = USBH_BUSY;
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  /* Switch HID state machine */
  switch (HID_Handle->ctl_state)
  {
    case USBH_HID_REQ_INIT:
    case USBH_HID_REQ_GET_HID_DESC:

      USBH_HID_ParseHIDDesc(&HID_Handle->HID_Desc, phost->device.CfgDesc_Raw);

      HID_Handle->ctl_state = USBH_HID_REQ_GET_REPORT_DESC;

      break;
    case USBH_HID_REQ_GET_REPORT_DESC:

      /* Get Report Desc */
      classReqStatus = USBH_HID_GetHIDReportDescriptor(phost, HID_Handle->HID_Desc.wItemLength);
      if (classReqStatus == USBH_OK)
      {
        /* The descriptor is available in phost->device.Data */
        HID_Handle->ctl_state = USBH_HID_REQ_SET_IDLE;
      }
      else if (classReqStatus == USBH_NOT_SUPPORTED)
      {
        USBH_ErrLog("Control error: HID: Device Get Report Descriptor request failed");
        status = USBH_OK;
      }
      else
      {
        /* .. */
      }

      break;

    case USBH_HID_REQ_SET_IDLE:

      classReqStatus = USBH_HID_SetIdle(phost, 0U, 0U);

      /* set Idle */
      if (classReqStatus == USBH_OK)
      {
        HID_Handle->ctl_state = USBH_HID_REQ_SET_PROTOCOL;
      }
      else
      {
        if (classReqStatus == USBH_NOT_SUPPORTED)
        {
          HID_Handle->ctl_state = USBH_HID_REQ_SET_PROTOCOL;
        }
      }
      break;

    case USBH_HID_REQ_SET_PROTOCOL:
      /* set protocol */
      classReqStatus = USBH_HID_SetProtocol(phost, 0U);
      if (classReqStatus == USBH_OK)
      {
        HID_Handle->ctl_state = USBH_HID_REQ_IDLE;

        /* all requests performed */
        phost->pUser(phost, HOST_USER_CLASS_ACTIVE);
        status = USBH_OK;
      }
      else if (classReqStatus == USBH_NOT_SUPPORTED)
      {
		  USBH_ErrLog("TODO:Cannel:-->Control error: HID: Device Set protocol request failed");
        status = USBH_OK;
      }
      else
      {
        /* .. */
      }
      break;

    case USBH_HID_REQ_IDLE:
    default:
      break;
  }

  return status;
}

/**
  * @brief  USBH_HID_Process
  *         The function is for managing state machine for HID data transfers
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_Process(USBH_HandleTypeDef *phost)
{
  USBH_StatusTypeDef status = USBH_OK;
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
  uint32_t XferSize;

  switch (HID_Handle->state)
  {
    case USBH_HID_INIT:
      status = HID_Handle->Init(phost);

      if (status == USBH_OK)
      {
        HID_Handle->state = USBH_HID_IDLE;
      }
      else
      {
        USBH_ErrLog("HID Class Init failed");
        HID_Handle->state = USBH_HID_ERROR;
        status = USBH_FAIL;
      }

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, 0U);
#endif
#endif
      break;

    case USBH_HID_IDLE:
		
	//Send a GetReport request to request the HID device to return data. Before this, HID_Handle->pData must complete initialization, which is usually done in the HID_Handle-> Init function; otherwise, the request will not succeed
	
      status = USBH_HID_GetReport(phost, 0x01U, 0U, HID_Handle->pData, (uint8_t)HID_Handle->length);

      if (status == USBH_OK)
      {
        HID_Handle->state = USBH_HID_SYNC;
      }
      else if (status == USBH_BUSY)
      {
        HID_Handle->state = USBH_HID_IDLE;
        status = USBH_OK;
      }
      else if (status == USBH_NOT_SUPPORTED)
      {
        HID_Handle->state = USBH_HID_SYNC;
        status = USBH_OK;
      }
      else
      {
        HID_Handle->state = USBH_HID_ERROR;
        status = USBH_FAIL;
      }

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, 0U);
#endif
#endif
      break;

    case USBH_HID_SYNC:
      /* Sync with start of Even Frame */
      if ((phost->Timer & 1U) != 0U)
      {
        HID_Handle->state = USBH_HID_GET_DATA;
      }

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, 0U);
#endif
#endif
      break;

    case USBH_HID_GET_DATA:
		
	//Receive data from HID devices and save it in HID_Handle->pData
      (void)USBH_InterruptReceiveData(phost, HID_Handle->pData,
                                      (uint8_t)HID_Handle->length,
                                      HID_Handle->InPipe);

      HID_Handle->state = USBH_HID_POLL;
      HID_Handle->timer = phost->Timer;
      HID_Handle->DataReady = 0U;
      break;

    case USBH_HID_POLL:
      if (USBH_LL_GetURBState(phost, HID_Handle->InPipe) == USBH_URB_DONE)
      {
        XferSize = USBH_LL_GetLastXferSize(phost, HID_Handle->InPipe);

        if ((HID_Handle->DataReady == 0U) && (XferSize != 0U) && (HID_Handle->fifo.buf != NULL))
        {
		  //Write HID device data stored in HID_Handle->pData to FIFO, which then retrieves HID device data through FIFO
          (void)USBH_HID_FifoWrite(&HID_Handle->fifo, HID_Handle->pData, HID_Handle->length); 
          HID_Handle->DataReady = 1U;
          USBH_HID_EventCallback(phost);
	  
#if (USBH_USE_OS == 1U)
          phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
          (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
          (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, 0U);
#endif
#endif
        }
      }
      else
      {
        /* IN Endpoint Stalled */
        if (USBH_LL_GetURBState(phost, HID_Handle->InPipe) == USBH_URB_STALL)
        {
          /* Issue Clear Feature on interrupt IN endpoint */
          if (USBH_ClrFeature(phost, HID_Handle->ep_addr) == USBH_OK)
          {
            /* Change state to issue next IN token */
            HID_Handle->state = USBH_HID_GET_DATA;
          }
        }
      }
      break;

    default:
      break;
  }

  return status;
}

/**
  * @brief  USBH_HID_SOFProcess
  *         The function is for managing the SOF Process
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_SOFProcess(USBH_HandleTypeDef *phost)
{
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  if (HID_Handle->state == USBH_HID_POLL)
  {
    if ((phost->Timer - HID_Handle->timer) >= HID_Handle->poll)
    {
      HID_Handle->state = USBH_HID_GET_DATA;

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, 0U);
#endif
#endif
    }
  }
  return USBH_OK;
}

/**
  * @brief  USBH_ParseHIDDesc
  *         This function Parse the HID descriptor
  * @param  desc: HID Descriptor
  * @param  buf: Buffer where the source descriptor is available
  * @retval None
  */
static void USBH_HID_ParseHIDDesc(HID_DescTypeDef *desc, uint8_t *buf)
{
  USBH_DescHeader_t *pdesc = (USBH_DescHeader_t *)buf;
  uint16_t CfgDescLen;
  uint16_t ptr;

  CfgDescLen = LE16(buf + 2U);

  if (CfgDescLen > USB_CONFIGURATION_DESC_SIZE)
  {
    ptr = USB_LEN_CFG_DESC;

    while (ptr < CfgDescLen)
    {
      pdesc = USBH_GetNextDesc((uint8_t *)pdesc, &ptr);

      if (pdesc->bDescriptorType == USB_DESC_TYPE_HID)
      {
        desc->bLength = *(uint8_t *)((uint8_t *)pdesc + 0U);
        desc->bDescriptorType = *(uint8_t *)((uint8_t *)pdesc + 1U);
        desc->bcdHID = LE16((uint8_t *)pdesc + 2U);
        desc->bCountryCode = *(uint8_t *)((uint8_t *)pdesc + 4U);
        desc->bNumDescriptors = *(uint8_t *)((uint8_t *)pdesc + 5U);
        desc->bReportDescriptorType = *(uint8_t *)((uint8_t *)pdesc + 6U);
        desc->wItemLength = LE16((uint8_t *)pdesc + 7U);
        break;
      }
    }
  }
}

///////////////////////// Initialization, Data Decoding, ///////////////////////////////
//Handle general input report
static uint8_t GamePadReportData[128] = { 0 }; //Used to store data sent from HID device

//PS2 initialization function
USBH_StatusTypeDef USBH_HID_GamePadInit(USBH_HandleTypeDef *phost)
{
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
	
	//HID_Handle->length is determined by the HID device when initializing the device
	if (HID_Handle->length > sizeof(GamePadReportData))
	{
		HID_Handle->length = (uint16_t)sizeof(GamePadReportData);
	}
	
	//Initializing pData is a necessary step.
	HID_Handle->pData = GamePadReportData;
	
	if ((HID_QUEUE_SIZE * sizeof(GamePadReportData)) > sizeof(phost->device.Data))
	{	//sizeof(phost->device. Data) size is configured by USBH_MAX_DATA_BUFFER
		return USBH_FAIL;
	}
	else
	{
		//Initialize the FIFO
		USBH_HID_FifoInit(&HID_Handle->fifo, phost->device.Data, (uint16_t)(HID_QUEUE_SIZE * sizeof(GamePadReportData)));
	}
	
	//USB plugged in
	USB_GamePad_InsertCallback();
	
	//switch pro handle, requires special initialization
	if( gamepad_brand == SwitchPro )
	{
		SwitchPro_GamePad_Init(phost);
	}
	
	return USBH_OK;
}

//PS2 data decoding, input parameter set to HID device. Operating environment set to task.
USBH_StatusTypeDef USBH_HID_PS2_Decode(USBH_HandleTypeDef *phost)
{
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
	
	//Check whether the handle type is successfully identified
	if( gamepad_brand == UnKnown_Dev ) return USBH_FAIL;
	
	//Check if the HID device has data
	if ((HID_Handle->length == 0U) || (HID_Handle->fifo.buf == NULL))
	{
		return USBH_FAIL;
	}
	
	//Read HID data from the FIFO, place it into GamePadReportData, and parse it
	if (USBH_HID_FifoRead(&HID_Handle->fifo, (uint8_t* )GamePadReportData, HID_Handle->length) == HID_Handle->length)
	{
		/* Call different processing functions depending on the actual game controller you access */
		GamePadDebug.ready = 1;//Handle debugging, indicating data reception
		
		//Decode for Xbox 360 format
		//LX GamePadReportData[7] left section 128-0, right section 0-127
		//LY GamePadReportData[9] lower section 128-0, upper section 0-127
		//RX GamePadReportData[11] Left stick 128-0, right stick 0-127
		//RY GamePadReportData[13] Down 128-0, Up 0-127
		//LT GamePadReportData[4] 0-255
		//RT GamePadReportData[5] 0-255
		// GamePadReportData[3] 1:LB 2:RB 3:logo 4:? 5:A 6:B 7:X 8:Y
		// GamePadReportData[2] 1234: Up, down, left, right 5:start 6:seltec 7:Lrock 8:Rrock

		if( gamepad_brand == Xbox360 || gamepad_brand == PS2_USB_Wiredless )
		{
			Xbox360_gamepad_Decode(phost,GamePadReportData,HID_Handle->length);
		}
		else if( gamepad_brand == PS2_USB_Wired )
		{
			Wired_USB_PS2gamepad_Decode(phost,GamePadReportData,HID_Handle->length);
		}
		else if( gamepad_brand == PS2_USB_WiredV2 )
		{
			Wired_USB_V2_PS2gamepad_Decode(phost,GamePadReportData,HID_Handle->length);
		}
		
		return USBH_OK;
	}

	return   USBH_FAIL;

}

//Initialize the SwitchPro controller
void SwitchPro_GamePad_Init(USBH_HandleTypeDef *phost)
{
	#define DelayTime 100
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
	
	USBH_UsrLog("switch pro init now...");
	
	/* The following data is sourced from reverse engineering packet capture; the meaning is not required */
	
	uint8_t initcode[64] = { 0 };
	initcode[0] = 0x80; initcode[1] = 0x01;
	while( USBH_OK!= USBH_InterruptSendData(phost,initcode,64,HID_Handle->OutPipe) ) ; 
	USBH_Delay(DelayTime);
	
	initcode[1] = 0x02;
	while( USBH_OK!= USBH_InterruptSendData(phost,initcode,64,HID_Handle->OutPipe) ) ; 
	USBH_Delay(DelayTime);
	
	initcode[1] = 0x03;
	while( USBH_OK!= USBH_InterruptSendData(phost,initcode,64,HID_Handle->OutPipe) ) ; 
	USBH_Delay(DelayTime);
	
	initcode[1] = 0x02;
	while( USBH_OK!= USBH_InterruptSendData(phost,initcode,64,HID_Handle->OutPipe) ) ; 
	USBH_Delay(DelayTime);
	
	initcode[1] = 0x04;
	while( USBH_OK!= USBH_InterruptSendData(phost,initcode,64,HID_Handle->OutPipe) ) ;
	USBH_Delay(DelayTime);

	USBH_UsrLog("switch pro init ok!");
	
	/* Only after writing the above info will the Switch Pro controller activate data output */
}

