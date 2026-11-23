  /**************************************************************************
By LucSys Robotics
**************************************************************************/

#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H
 
 
extern unsigned char DataScope_OutPut_Buffer[42];	   //Frame data cache waiting to be sent


void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    // Write channel data to the frame data buffer to be sent

unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // Send frame data generation function
 
 
#endif 



