#include "timx_callback.h"

/**************************************************************************
Function: Model aircraft remote control receiving interrupt
Input   : none
Output  : none
Function: Model model remote control receiving interrupt (Timer X capture interrupt)
Entry parameters: None
Return value: none
**************************************************************************/
void REMOTE_TIM_IRQHandler(void)
{
	//Input the capture flag for channel,
	//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
	//Channel input-capture flags: upper two bits for capture flags, lower 6 bits for overflow flags
	static u8 RemoteCH1_CAPTURE_STA = 0;
	static u8 RemoteCH2_CAPTURE_STA = 0;
	static u8 RemoteCH3_CAPTURE_STA = 0;
	static u8 RemoteCH4_CAPTURE_STA = 0;

	static u16 RemoteCH1_CAPTURE_UPVAL;
	static u16 RemoteCH1_CAPTURE_DOWNVAL;

	static u16 RemoteCH2_CAPTURE_UPVAL;
	static u16 RemoteCH2_CAPTURE_DOWNVAL;

	static u16 RemoteCH3_CAPTURE_UPVAL;
	static u16 RemoteCH3_CAPTURE_DOWNVAL;

	static u16 RemoteCH4_CAPTURE_UPVAL;
	static u16 RemoteCH4_CAPTURE_DOWNVAL;
	
	//Remote-control value history
	static REMOTER_t last_remoter;
	
	//Deviation value, assisting in calculating captured values
	u32 CH1_Diff;
	u32 CH2_Diff;
	u32 CH3_Diff;
	u32 CH4_Diff;

	//Filter abrupt channel changes
    static u8 ch1_filter_times=0,ch2_filter_times=0,ch3_filter_times=0,ch4_filter_times=0;

    //After connecting the RC transmitter, push the forward stick before RC control of the chassis is enabled
    //After connecting the remote controller of the model aircraft,
    //you need to push down the forward lever to officially control the car of the model aircraft
	static uint16_t state_filter=0;//Status filtering to prevent false detection of model aircraft control modes
    if(remoter.ch2>1600&& remoter.check_count< Filter_Threshold &&Get_Control_Mode(_RC_Control)==0&&SysVal.Time_count>=CONTROL_DELAY)
    {
		state_filter++;
		if( state_filter > 250 ) //Only activate model control after the model remote has been activated for a certain period to prevent interference signals. 250 has no specific unit and is related to the frequency of interrupts entering the timer.
		{
			Set_Control_Mode(_RC_Control);//Set to RC transmitter mode
		}     
    }
	else
	{
		state_filter = 0;//For non-RC control conditions, reset the filtered value
	}
	
    //Channel 1 // Channel 1
    if ((RemoteCH1_CAPTURE_STA & 0X80) == 0)
    {
        if ( Get_CH1_State != RESET) //A capture event occurred on channel 1 // Capture event occurred on channel 1
        {
            Clear_CH1_State; //Clear the interrupt flag bit // Clear the interrupt flag bit
            if (RemoteCH1_CAPTURE_STA & 0X40)	//A falling edge is caught // Captured a falling edge
            {
                RemoteCH1_CAPTURE_DOWNVAL = Get_CH1_CNT ; //Record the timer value at this point // Record the timer value at this moment
                if (RemoteCH1_CAPTURE_DOWNVAL < RemoteCH1_CAPTURE_UPVAL)
                {
                    CH1_Diff = 9999;
                }
                else
                    CH1_Diff = 0;
                remoter.ch1 = RemoteCH1_CAPTURE_DOWNVAL - RemoteCH1_CAPTURE_UPVAL + CH1_Diff;	//Time to get the total high level // Time to get the total high level
				
                if(abs(remoter.ch1-last_remoter.ch1)>500)
                {
                    ch1_filter_times++;
                    if( ch1_filter_times<=5 ) remoter.ch1=last_remoter.ch1; //Filter //Filter
                    else ch1_filter_times=0;
                }
                else
                {
                    ch1_filter_times=0;
                }
                last_remoter.ch1 = remoter.ch1;

                RemoteCH1_CAPTURE_STA = 0; //Capture flag bit to zero // Capture flag bit to zero
				Set_CH1_Rising; //Set to rising edge capture // Set to rising edge capture
            }
            else
            {
                //When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
                //Capture time occurred but it wasn¡¯t a falling edge; at the first rising edge captured, record the timer count at that moment
                RemoteCH1_CAPTURE_UPVAL = Get_CH1_CNT; //Obtain rising edge data
                RemoteCH1_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge // The flag has been captured on the rising edge
                Set_CH1_Falling; //Set to Falling Edge Capture // Set to falling edge capture
            }
        }
    }
    //Channel 2 // Channel 2
    if ((RemoteCH2_CAPTURE_STA & 0X80) == 0)
    {
        if ( Get_CH2_State != RESET)	//A capture event occurred on channel 2 // Capture event occurred on channel 2
        {
            Clear_CH2_State; //Clear the interrupt flag bit // Clear the interrupt flag bit
            if (RemoteCH2_CAPTURE_STA & 0X40)	//A falling edge is caught // Captured a falling edge
            {
                RemoteCH2_CAPTURE_DOWNVAL = Get_CH2_CNT; //Record the timer value at this point // Record the timer value at this moment
                if (RemoteCH2_CAPTURE_DOWNVAL < RemoteCH2_CAPTURE_UPVAL)
                {
                    CH2_Diff = 9999;
                }
                else
                    CH2_Diff = 0;
                remoter.ch2 = RemoteCH2_CAPTURE_DOWNVAL - RemoteCH2_CAPTURE_UPVAL + CH2_Diff; //Time to get the total high level // Time to get the total high level
                if(abs(remoter.ch2-last_remoter.ch2)>500)
                {
                    ch2_filter_times++;
                    if( ch2_filter_times<=5 ) remoter.ch2=last_remoter.ch2; //Filter //Filter
                    else ch2_filter_times=0;
                }
                else
                {
                    ch2_filter_times=0;
                }
                last_remoter.ch2=remoter.ch2;

                RemoteCH2_CAPTURE_STA = 0; //Capture flag bit to zero // Capture flag bit to zero
                Set_CH2_Rising; //Set to rising edge capture // Set to rising edge capture
            }
            else
            {
                //When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
                //Capture time occurred but it wasn¡¯t a falling edge; at the first rising edge captured, record the timer count at that moment
                RemoteCH2_CAPTURE_UPVAL = Get_CH2_CNT; //Obtain rising edge data
                RemoteCH2_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge // The flag has been captured on the rising edge
                Set_CH2_Falling; //Set to Falling Edge Capture // Set to falling edge capture
            }
        }
    }
    //Channel 3 // Channel 3
    if ((RemoteCH3_CAPTURE_STA & 0X80) == 0)
    {
        if (Get_CH3_State != RESET)	//A capture event occurred on channel 3 // Capture event occurred on channel 3
        {
			remoter.check_count = 0;//RC transmitter check reset. If this position is not reset frequently, it indicates interference signals.
			
            Clear_CH3_State; //Clear the interrupt flag bit // Clear the interrupt flag bit
            if (RemoteCH3_CAPTURE_STA & 0X40)	//A falling edge is caught // Captured a falling edge
            {
                RemoteCH3_CAPTURE_DOWNVAL = Get_CH3_CNT; //Record the timer value at this point // Record the timer value at this moment
                if (RemoteCH3_CAPTURE_DOWNVAL < RemoteCH3_CAPTURE_UPVAL)
                {
                    CH3_Diff = 9999;
                }
                else
                    CH3_Diff = 0;
                remoter.ch3 = RemoteCH3_CAPTURE_DOWNVAL - RemoteCH3_CAPTURE_UPVAL + CH3_Diff; //Time to get the total high level // Time to get the total high level
                if(abs(remoter.ch3-last_remoter.ch3)>500)
                {
                    ch3_filter_times++;
                    if( ch3_filter_times<=5 ) remoter.ch3=last_remoter.ch3; //Filter //Filter
                    else ch3_filter_times=0;
                }
                else
                {
                    ch3_filter_times=0;
                }
                last_remoter.ch3=remoter.ch3;
                RemoteCH3_CAPTURE_STA = 0; //Capture flag bit to zero // Capture flag bit to zero
                Set_CH3_Rising; //Set to rising edge capture // Set to rising edge capture
            }
            else
            {
                //When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
                //Capture time occurred but it wasn¡¯t a falling edge; at the first rising edge captured, record the timer count at that moment
                RemoteCH3_CAPTURE_UPVAL = Get_CH3_CNT; //Obtain rising edge data
                RemoteCH3_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge // The flag has been captured on the rising edge
                Set_CH3_Falling; //Set to Falling Edge Capture // Set to falling edge capture
            }
        }
    }
	
    //Channel 4 // Channel 4
    if ((RemoteCH4_CAPTURE_STA & 0X80) == 0)
    {
        if (Get_CH4_State != RESET)	//A capture event occurred on channel 4 // A capture event occurred on channel 4
        {
            Clear_CH4_State; //Clear the interrupt flag bit // Clear the interrupt flag bit
            if (RemoteCH4_CAPTURE_STA & 0X40)	//A falling edge is caught // Captured a falling edge
            {
                RemoteCH4_CAPTURE_DOWNVAL = Get_CH4_CNT; //Record the timer value at this point // Record the timer value at this moment
                if (RemoteCH4_CAPTURE_DOWNVAL < RemoteCH4_CAPTURE_UPVAL)
                {
                    CH4_Diff = 9999;
                }
                else
                    CH4_Diff = 0;
                remoter.ch4 = RemoteCH4_CAPTURE_DOWNVAL - RemoteCH4_CAPTURE_UPVAL + CH4_Diff; //Time to get the total high level // Time to get the total high level
                if(abs(remoter.ch4-last_remoter.ch4)>500)
                {
                    ch4_filter_times++;
                    if( ch4_filter_times<=5 ) remoter.ch4=last_remoter.ch4; //Filter //Filter
                    else ch4_filter_times=0;
                }
                else
                {
                    ch4_filter_times=0;
                }
                last_remoter.ch4=remoter.ch4;
                RemoteCH4_CAPTURE_STA = 0; //Capture flag bit to zero // Capture flag bit to zero
                Set_CH4_Rising; //Set to rising edge capture // Set to rising edge capture
            }
            else
            {
                //When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
                //Capture time occurred but it wasn¡¯t a falling edge; at the first rising edge captured, record the timer count at that moment
                RemoteCH4_CAPTURE_UPVAL = Get_CH4_CNT; //Obtain rising edge data
                RemoteCH4_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge // The flag has been captured on the rising edge
                Set_CH4_Falling; //Set to Falling Edge Capture // Set to falling edge capture
            }
        }
    }
	
}





