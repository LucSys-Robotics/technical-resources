#include "auto_recharge.h"
#include "can.h"
#include "robot_init.h"

CHARGER_t charger;


//Automatic recharge software variable reset
void auto_recharge_reset(void)
{
	//Automatic recharge mode
	charger.AllowRecharge = 0;
	
	//Is it charging?
	charger.Charging = 0;
	
	//Charging current
	charger.ChargingCurrent = 0;
	
	//Navigation takeover flag position
	charger.NavWalk = 0;
	
	//Offline monitoring
	charger.OutLine_Check = 0;
	
	//Default docking speed setting
	charger.Dock_MoveX = -0.1f;
	charger.Dock_MoveY = 0;
	charger.Dock_MoveZ = 0.2f;
	
	//4 infrared signals
	charger.L_A = 0;
	charger.L_B = 0;
	charger.R_A = 0;
	charger.R_B = 0;
	charger.RED_STATE = 0;
	
	//Infrared refresh time
	#if defined AKM_CAR
		//Ackermann chassis configuration
		charger.RedRefalsh_Time = 50;
	#else
		//For non-Ackermann chassis, set 0 to use the charging equipment's default time
		charger.RedRefalsh_Time = 0;
	#endif
	
	//Infrared control, navigation takeover, and speed reset
	charger.Red_MoveX = 0;
	charger.Red_MoveY = 0;
	charger.Red_MoveZ = 0;
	charger.Up_MoveX = 0;
	charger.Up_MoveY = 0;
	charger.Up_MoveZ = 0;
	
	//Check if charging equipment is available
	Find_Charging_HardWare();
}


//Check if charging equipment hardware is currently present
//Send extended frame ID 0x12345678, content {1,1,1,1,1,1,1,1} to charging equipment. Charging equipment will return the same ID and content, indicating it is online and successfully connected
void Find_Charging_HardWare(void)
{
    u8 tmpdata[8]= {1,1,1,1,1,1,1,1};
    CAN1_Send_EXTid_Num(0x12345678,tmpdata);
}

//Set the docking speed for charging equipment and set the infrared refresh time
void CAN_Send_AutoRecharge(void)
{
	u8 CAN_SENT[8];
	
	//Reserved bit
	CAN_SENT[0]=0;
	
	//Set the speed of the infrared interconnection, unit m/s
	//Set the speed and magnitude of infrared docking, unit: mm/s
	CAN_SENT[1]=((short)(charger.Dock_MoveX*1000))>>8;
	CAN_SENT[2]=((short)(charger.Dock_MoveX*1000));
	CAN_SENT[3]=((short)(charger.Dock_MoveY*1000))>>8;
	CAN_SENT[4]=((short)(charger.Dock_MoveY*1000));
	CAN_SENT[5]=((short)(charger.Dock_MoveZ*1000))>>8;
	CAN_SENT[6]=((short)(charger.Dock_MoveZ*1000));
	CAN_SENT[7] = charger.RedRefalsh_Time;	
	CAN1_Send_Num(0x105,CAN_SENT);

}


//Automatic docking logic for non-Ackermann chassis
#if !defined AKM_CAR
void Handle_Normal_AutoRecharge(uint8_t* temp_rxbuf)
{
	//Calculate the three-axis target velocity, unit: mm/s
	//Calculate the three-axis target velocity, unit: mm/s
	charger.Red_MoveX=((float)((short)((temp_rxbuf[0]<<8)+(temp_rxbuf[1]))))/1000; 
	charger.Red_MoveY = 0;
	//charger.Red_MoveY=((float)((short)((temp_rxbuf[2]<<8)+(temp_rxbuf[3]))))/1000;
	charger.Red_MoveZ=((float)((short)((temp_rxbuf[4]<<8)+(temp_rxbuf[5]))))/1000;
	
	//Differential Model 5 with rear ultrasound
	#if defined DIFF_CAR
	if( (s21c_board.rangerE < 0.5f || s21c_board.rangerF < 0.5f) && robot.type==5 )
		charger.Red_MoveX/=2.0f;
	#elif defined MEC_CAR
	if( (s21c_board.rangerE < 0.5f || s21c_board.rangerF < 0.5f) && robot.type==9 )
		charger.Red_MoveX/=2.0f;
	#endif
	
	charger.Charging=temp_rxbuf[6]&1;		//Charging status indicator
	charger.L_A = (temp_rxbuf[6]>>5)&0x01;
	charger.L_B = (temp_rxbuf[6]>>4)&0x01;
	charger.R_B = (temp_rxbuf[6]>>3)&0x01;
	charger.R_A = (temp_rxbuf[6]>>2)&0x01;
	charger.RED_STATE = charger.L_A + charger.L_B + charger.R_B + charger.R_A;
	
	//No infrared signals or vehicles charging, clear target speed
	if( 0 == charger.RED_STATE || 1 == charger.Charging  ) charger.Red_MoveX = 0 , charger.Red_MoveY = 0 , charger.Red_MoveZ = 0;
	
	//Charging current conversion
	if(temp_rxbuf[7]>128) charger.ChargingCurrent=-(256-temp_rxbuf[7])*30;
	else charger.ChargingCurrent=(temp_rxbuf[7]*30);
}
#endif




#if defined AKM_CAR
//Internal variables that facilitate debugging and can serve as global output during debugging
u8 liar_adjust=0;           //Single-side charging mode
u8 last_state=0;            //Save the previous state of the infrared signal
u16 adjust_timeout=0;       //Adjust timeout settings to track whether the adjustment time is too long
u16 double_adjust_timeout=0;//Dual-side adjustment timeout setting, used to check if the adjustment time is too long
u8 red_now_state;           //Current status of the infrared signal
u8 touch_state;             //The contact status of the charging pile with the spring fragments of the charging equipment

void Handle_AKM_AutoRecharge(uint8_t* temp_rxbuf)
{
	static u8 tmp_state=0;
	static u8 last_touchstate=0;
	
	//State lock after contact with the spring
	static u8 state_lock = 0;
	
	//Posture marker when leaving after contact with shrapnel
	static u8 change_state = 0;
	
	//Time-running core when leaving after contacting the spring piece
	static u16 time_core = 0;
	
	//Filtered anti-shake variables when contacting the spring
	static u16 filter_cur = 0,filter_vol = 0;
	
	//Number of adjustments to the car's charging posture
	static u8 adjust_times = 0;
	static u8 adjust_vol = 0 , adjust_cur = 0;
	
	//One-sided charging marker
	static u8 liar_charge = 0;
	
	//Both sides touch, adjust the marker once
	if(adjust_vol&&adjust_cur) adjust_vol = 0,adjust_cur = 0,adjust_times++;
	
	//Infrared state calculation
	charger.L_A = (temp_rxbuf[6]>>5)&0x01;
	charger.L_B = (temp_rxbuf[6]>>4)&0x01;
	charger.R_B = (temp_rxbuf[6]>>3)&0x01;
	charger.R_A = (temp_rxbuf[6]>>2)&0x01;
	
	//Used to check contact in charging and pressure testing areas
	touch_state = temp_rxbuf[2];
	if( last_touchstate!=0xAA&&touch_state==0xAA ) adjust_timeout=0,liar_adjust++;//Unilateral state adjustment records
	if( (last_touchstate!=0xAA&&touch_state==0xAA) || (last_state!=0xBB&&touch_state==0xBB) ) double_adjust_timeout=0;//Clears bilateral delay time when any side contacts
	last_touchstate = touch_state;
	
	//If neither side has no contact for more than 5 seconds, the number of clearing adjustments will be calibrated
	if(adjust_times>0) double_adjust_timeout++;
	if( double_adjust_timeout>250 ) adjust_times=0,double_adjust_timeout=0;
	
	//Charging edge is contactless for 5 seconds; adjust the number of times to clear the charging edge and calibrate
	if( liar_adjust>0 ) adjust_timeout++;
	if( adjust_timeout>250 ) liar_adjust=0,adjust_timeout=0;//If the contact interval is greater than 5 seconds, it is judged as an abnormal situation and the adjustment count record is cleared.
	
	
	//If it is detected to have detached from the charging port, reset is required
	if(liar_charge==1&&touch_state!=0xAA&&touch_state!=0xCF) 
	{
		if( touch_state==0xAB && robot.voltage >=25.0f )//Fully charged
		{
			charger.AllowRecharge=0;
		}
		adjust_times = 0,liar_charge=0;
	}	
	
	//Unilateral charging with customizable touch_state marks
	if(liar_charge) touch_state = 0xFC;
	
	/* Set different parameters for different chassis types */
	u16 leave_times = 75; //Time to leave after contact with shrapnel: 75*20ms = 1.5s
	u8 yuzhi = 40;        //Time to de-shake after contact with the spring: 40 * 20 ms = 800 ms
	if(charger.AllowRecharge==0) 
	{

		adjust_times = 0,liar_charge=0,liar_adjust=0;//When auto-recharge is turned off, the calibration adjustment count is 0
		if( robot.type==2 || robot.type==3 )  Set_Robot_PI_Param(300,300); 
		if( robot.type==4 ) Set_Robot_PI_Param(400,100); 
		if( robot.type==5 ) Set_Robot_PI_Param(50,200); 
	}
	else
	{	
		if( robot.type==0 || robot.type==1 ) leave_times=80, charger.RedRefalsh_Time=100; //Original 80, 50
		if( robot.type==2 || robot.type==3 ) Set_Robot_PI_Param(600,600) , charger.RedRefalsh_Time=100; 
		if( robot.type==4 || robot.type==5 ) 
		{
			//Red_Docker_X=-0.1f, Red_Docker_Y=0, Red_Docker_Z=0.2f; default values
			charger.Dock_MoveX = -0.08f;
			charger.Dock_MoveZ = 0.15f;
			Set_Robot_PI_Param(600,600);
			leave_times=113;//Top-spec independent-suspension motor responds more slowly and needs a longer adjustment time
			charger.RedRefalsh_Time=150;//Infrared status refresh time (the slower the motor response, the longer the recommended duration; this is related to the motor reduction ratio and the car's load)
		}
	}

	//Infrared fused into a single numerical state representation, making it easy to assign status numbers
	red_now_state = charger.L_A << 3 | charger.L_B << 2 | charger.R_B << 1 | charger.R_A << 0 ;
	
	if( charger.AllowRecharge )//Status is recorded only after automatic recharge is enabled
	{
		//When the infrared state changes, save the previous infrared state
		if(red_now_state!=tmp_state) last_state = tmp_state;	
		tmp_state = red_now_state;
	}
	
	////////////////////////////  Docking logic processing Start ////////////////////////////
	charger.RED_STATE = charger.L_A + charger.L_B + charger.R_B + charger.R_A;
	
		 if(charger.L_A==0&&charger.L_B==0&&charger.R_B==0&&charger.R_A==1)  front_right; //1
		 
	else if(charger.L_A==0&&charger.L_B==0&&charger.R_B==1&&charger.R_A==0)  back_left;  //2
	
	else if(charger.L_A==0&&charger.L_B==0&&charger.R_B==1&&charger.R_A==1)  front_right;  //3

	else if(charger.L_A==0&&charger.L_B==1&&charger.R_B==0&&charger.R_A==0)  //4
	{
		front_left; 
	}	
	
	// charger. L_A==0&&charger. L_B==1&&charger. R_B==0&&charger. R_A==1 does not exist 5
	
	else if(charger.L_A==0&&charger.L_B==1&&charger.R_B==1&&charger.R_A==0) //6
	{
		front_left;   
		if(last_state==2) back;
		
	}	
	
	else if(charger.L_A==0&&charger.L_B==1&&charger.R_B==1&&charger.R_A==1)  back; //7
	
	else if(charger.L_A==1&&charger.L_B==0&&charger.R_B==0&&charger.R_A==0) 
	{
		back_right; //8
		if( last_state==9 ) back;
	}

	else if(charger.L_A==1&&charger.L_B==0&&charger.R_B==0&&charger.R_A==1)   //9
	{
		front_right;
		if( last_state==8||last_state==1 ) back;
	}
	
	else if (charger.L_A==1&&charger.L_B==0&&charger.R_B==1&&charger.R_A==0)  back; //10
	
	else if(charger.L_A==1&&charger.L_B==0&&charger.R_B==1&&charger.R_A==1) // 11
	{
		front_right;
		if(last_state==9) back;
	}
	
	else if(charger.L_A==1&&charger.L_B==1&&charger.R_B==0&&charger.R_A==0)  back_right;  //12	
	
	else if(charger.L_A==1&&charger.L_B==1&&charger.R_B==0&&charger.R_A==1)
	{
		back; //13
	}			
	
	else if(charger.L_A==1&&charger.L_B==1&&charger.R_B==1&&charger.R_A==0)  //14
	{
		front_left; 
		if(last_state==6||last_state==3) back;
	}			

	else if(charger.L_A==1&&charger.L_B==1&&charger.R_B==1&&charger.R_A==1)  back;		  //15
	
	//In other cases, all 0s or 5s
	else
	{
		charger.RED_STATE = 0;//Infrared recognition status set to 0
		stop;
	}

	
	if(touch_state==0xAA||state_lock==1)//Charging equipment touches the charging area
	{
		stop;		

		filter_cur++;   
		if(filter_cur>yuzhi) //Delayed stabilization
		{
			adjust_cur = 1; //Tag contact
			
			filter_cur = yuzhi+1;//Filter variables lock themselves to prevent loop overflow
			
			state_lock = 1;//Latch contact state so this branch continues until the state is updated
			
			cur_front_right;//Adjust posture
			
			//The number of adjustments exceeds the limit, and the adjustment is touched by the charging area again, so no adjustment is done; Charging on one side is used
			u8 allow_adjust = 2; //The number of times pose adjustments are allowed after docking
			if(adjust_times==allow_adjust||( liar_adjust>=3&&adjust_times>0 ))
			{
				stop;		
				liar_charge = 1;//No adjustment, enable unilateral charging
				liar_adjust=0;
			}
		}
		
	}
	else if(touch_state==0xBB||state_lock==2)//Contacted the pressure measurement zone
	{
		stop;
		
		filter_vol++;   
		if(filter_vol>yuzhi) //Delayed stabilization
		{  
			adjust_vol = 1; //Record 1 contact event
			
			filter_vol = yuzhi+1;
			state_lock = 2;
			cur_front_left;//Adjust posture
		}
	}
	else if(touch_state==0xCF||touch_state==0xFC)
	{
		liar_charge=1;//When simultaneously touching the pressure measurement and charging zones, single-side charging is enabled to improve stability
		liar_adjust=0;
		stop;
	}
	
	//Spring contact status update
	if(state_lock!=0)
	{
		time_core++;
		if(time_core> leave_times )
		{
			if(state_lock==2) change_state = 2 ;
			if(state_lock==1) change_state = 1 ;
			state_lock = 0,time_core=0,filter_vol=0,filter_cur=0;
		}				
	}
	else
		time_core =  0;
	
	//After the shrapnel status update, all infrared effects except all 1 are blocked
	//If it contacts the spring and then leaves, it only receives the full-infrared backward data; all other data moves forward without turning
	if(change_state!=0)
	{
		static u8 shielding=0;
		static u8 leaveCount=0;	

		if(++leaveCount==255) leaveCount=0, change_state = 0,shielding=0;//Limit total adjustment time
		
		if(red_now_state==9) {front_right;shielding++;}//Special status, too far to the edge
		else if( red_now_state==6 ) {front_left;shielding++;}//Special status, too far to the edge
		else if( red_now_state!=15 ) {front;shielding=0;}
		
		//If the variable keeps incrementing, it means the shielding period has been too long, so the number of clearing adjustments will start over
		if(shielding>250) adjust_times=0,liar_adjust=0;
		
		if(last_state==15 || last_state==9 || last_state==6 ) leaveCount=0,change_state = 0,shielding=0;
	}
	
	//Convert Z velocity to angle
	charger.Red_MoveZ = Akm_Vz_to_Angle(charger.Red_MoveX,charger.Red_MoveZ);

	////////////////////////////  Docking logic processing End ////////////////////////////
	
	if(liar_charge==1) charger.Charging = 1;//The robot is charging
	else charger.Charging = 0;
	
	if(charger.Charging==1) charger.Red_MoveX = 0, charger.Red_MoveY = 0 , charger.Red_MoveZ = 0;
	
	//Charging current conversion
	if(temp_rxbuf[7]>128) charger.ChargingCurrent=-(256-temp_rxbuf[7])*30;
	else  charger.ChargingCurrent=(temp_rxbuf[7]*30);
}
#endif


