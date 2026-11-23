 
#include "DataScope_DP.h"
  /**************************************************************************
By LucSys Robotics
**************************************************************************/
unsigned char DataScope_OutPut_Buffer[42] = {0};	   //Serial port send buffer


//Function description: Converts single-precision floating-point data into 4 bytes and stores it at the specified address
//Additional note: users do not need to call this function directly
//target: target single-precision data
//buf: Array to be written
//beg: Specifies from which element of the array the write begins
//function does not return
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //Get float address
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}
 
 
//Function description: Writes single-precision floating-point data from the channel to be sent into the send buffer
//Data: Channel data
//Channel: Select channels (1-10)
//function does not return
void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
	if ( (Channel > 10) || (Channel == 0) ) return;  //If channel count is greater than 10 or equal to 0, return immediately without executing the function
  else
  {
     switch (Channel)
		{
      case 1:  Float2Byte(&Data,DataScope_OutPut_Buffer,1); break;
      case 2:  Float2Byte(&Data,DataScope_OutPut_Buffer,5); break;
		  case 3:  Float2Byte(&Data,DataScope_OutPut_Buffer,9); break;
		  case 4:  Float2Byte(&Data,DataScope_OutPut_Buffer,13); break;
		  case 5:  Float2Byte(&Data,DataScope_OutPut_Buffer,17); break;
		  case 6:  Float2Byte(&Data,DataScope_OutPut_Buffer,21); break;
		  case 7:  Float2Byte(&Data,DataScope_OutPut_Buffer,25); break;
		  case 8:  Float2Byte(&Data,DataScope_OutPut_Buffer,29); break;
		  case 9:  Float2Byte(&Data,DataScope_OutPut_Buffer,33); break;
		  case 10: Float2Byte(&Data,DataScope_OutPut_Buffer,37); break;
		}
  }	 
}


//Function Description: Generates frame formats correctly recognized by DataScope V1.0
//Channel_Number, the number of channels to be sent
//Return number of data items in the transmit buffer
//Return 0 if frame generation failed
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
	if ( (Channel_Number > 10) || (Channel_Number == 0) ) { return 0; }  //If channel count is greater than 10 or equal to 0, return immediately without executing the function
  else
  {	
	 DataScope_OutPut_Buffer[0] = '$';  //Frame header
		
	 switch(Channel_Number)   
   { 
		 case 1:   DataScope_OutPut_Buffer[5]  =  5; return  6;  
		 case 2:   DataScope_OutPut_Buffer[9]  =  9; return 10;
		 case 3:   DataScope_OutPut_Buffer[13] = 13; return 14; 
		 case 4:   DataScope_OutPut_Buffer[17] = 17; return 18;
		 case 5:   DataScope_OutPut_Buffer[21] = 21; return 22;  
		 case 6:   DataScope_OutPut_Buffer[25] = 25; return 26;
		 case 7:   DataScope_OutPut_Buffer[29] = 29; return 30; 
		 case 8:   DataScope_OutPut_Buffer[33] = 33; return 34; 
		 case 9:   DataScope_OutPut_Buffer[37] = 37; return 38;
     case 10:  DataScope_OutPut_Buffer[41] = 41; return 42; 
   }	 
  }
	return 0;
}











