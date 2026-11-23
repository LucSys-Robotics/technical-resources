#include "check.h"

//Define self-test-related structures
Check_parameter check_parameter;		
//Buzzer count value
u32 Buzzer_count1 = 0;

void Check_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	for(int i=0; i<6; i++)
	{
		check_parameter.Servo_count[i] = 500;
	}
	check_parameter.Motor_forward = 300;
	check_parameter.Motor_retreat = 500;
	while(1)
	{
		// This task runs at a frequency of 100Hz (10ms control once)
		//This task runs at 100Hz (controlled once every 10ms)
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));
		//Time count is no longer needed after 30 seconds
		//Time count, no longer needed after 30 seconds
		if(SysVal.Time_count<3000) SysVal.Time_count++;
		
		Buzzer_count1++;
		//Get the encoder data, that is, the real time wheel speed, 
		//and convert to transposition international units
		//Obtain encoder data, i.e., the real-time speed of the wheels, and convert bits in international units
		Get_Velocity_Form_Encoder();
		//Click the user button to proceed to the next step
		//Double-click to exit the self-check mode
		//Click the user button to proceed to the next step, then double-click to exit self-test mode
		Check_Key();
		switch(check_parameter.Process)
		{
			case 3:				//Motor self-check
				if(check_parameter.Motor_forward > 0)				check_parameter.Motor_forward--,Full_rotation = 16799;		//Forward rotation
				else if(check_parameter.Motor_retreat > 0)	check_parameter.Motor_retreat--,Full_rotation = -16799;		//Reverse
				switch(Car_Mode)
				{
					case Mec_Car:case Mec_Car_V550: 
						Set_Pwm( Full_rotation, -Full_rotation, -Full_rotation, Full_rotation, 0    ); break; 									//Mecanum wheel car
					case Omni_Car:      Set_Pwm(-Full_rotation,  Full_rotation, -Full_rotation, Full_rotation, 0    ); break; //Omni car
					case Akm_Car:       Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation, 0    ); break; //Ackermann structure car // Ackermann car
					case Diff_Car:      Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation, 0    ); break; //Differential car // Two-wheel differential car
					case FourWheel_Car:case FourWheel_Car_V550:
						Set_Pwm( Full_rotation, -Full_rotation, -Full_rotation, Full_rotation, 0    ); break; 									//FourWheel car // Four-wheel car
					case Tank_Car:      Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation, 0    ); break; //Tank Car // Tracked Vehicle
				} 
				if(!(check_parameter.Motor_forward > 0) && !(check_parameter.Motor_retreat > 0))				Set_Pwm(0,0,0,0,0);
				break;
			case 4:				//Exit motor self-test and stop the motor
				Set_Pwm(0,0,0,0,0);
				break;
			case 6:				//Initialize the servo
				TIM8_SERVO_Init(9999, 168-1);
				break;
			case 7:
				//Servo rotates forward
				if(check_parameter.Servo_Dir[0]==0 && check_parameter.Servo_count[0]<2500)
					check_parameter.Servo_count[0] = check_parameter.Servo_count[0] + 5;
				//After the forward rotation ends, switch the flag position
				if(check_parameter.Servo_Dir[0]==0 && check_parameter.Servo_count[0]>=2500)
					check_parameter.Servo_Dir[0] = 1;
				//Servo reversal
				if(check_parameter.Servo_Dir[0]==1 && check_parameter.Servo_count[0]>500)
					check_parameter.Servo_count[0] = check_parameter.Servo_count[0] - 5;
				//After the forward rotation ends, switch the flag position
				if(check_parameter.Servo_Dir[0]==1 && check_parameter.Servo_count[0]<=500)
					check_parameter.Servo_Dir[0] = 2;
				//Set duty cycle to control servo rotation
				TIM12->CCR2=check_parameter.Servo_count[0];
				break;
			case 8:
				//If the previous servo does not complete rotation and the verification jumps to the next servo, reset the previous servo
				if((check_parameter.Servo_Dir[0]!=2))				
					check_parameter.Servo_count[0]=500,TIM12->CCR2=check_parameter.Servo_count[0];
				//Servo rotates forward
				if(check_parameter.Servo_Dir[1]==0 && check_parameter.Servo_count[1]<2500)
					check_parameter.Servo_count[1] = check_parameter.Servo_count[1] + 5;
				//After the forward rotation ends, switch the flag position
				if(check_parameter.Servo_Dir[1]==0 && check_parameter.Servo_count[1]>=2500)
					check_parameter.Servo_Dir[1] = 1;
				//Servo reversal
				if(check_parameter.Servo_Dir[1]==1 && check_parameter.Servo_count[1]>500)
					check_parameter.Servo_count[1] = check_parameter.Servo_count[1] - 5;
				//After the forward rotation ends, switch the flag position
				if(check_parameter.Servo_Dir[1]==1 && check_parameter.Servo_count[1]<=500)
					check_parameter.Servo_Dir[1] = 2;
				//Set duty cycle to control servo rotation
				TIM12->CCR1=check_parameter.Servo_count[1];
				break;
			case 9:
				//If the previous servo does not complete rotation and the verification jumps to the next servo, reset the previous servo
				if((check_parameter.Servo_Dir[1]!=2))				
					check_parameter.Servo_count[1]=500,TIM12->CCR1=check_parameter.Servo_count[1];
				//Servo rotates forward
				if(check_parameter.Servo_Dir[2]==0 && check_parameter.Servo_count[2]<2500)
					check_parameter.Servo_count[2] = check_parameter.Servo_count[2] + 5;
				//After the forward rotation ends, switch the flag position
				if(check_parameter.Servo_Dir[2]==0 && check_parameter.Servo_count[2]>=2500)
					check_parameter.Servo_Dir[2] = 1;
				//Servo reversal
				if(check_parameter.Servo_Dir[2]==1 && check_parameter.Servo_count[2]>500)
					check_parameter.Servo_count[2] = check_parameter.Servo_count[2] - 5;
				//After the forward rotation ends, switch the flag position
				if(check_parameter.Servo_Dir[2]==1 && check_parameter.Servo_count[2]<=500)
					check_parameter.Servo_Dir[2] = 2;
				//Set duty cycle to control servo rotation
				TIM8->CCR4=check_parameter.Servo_count[2];
				break;
			case 10:
				//If the previous servo does not complete rotation and the verification jumps to the next servo, reset the previous servo
				if((check_parameter.Servo_Dir[2]!=2))				
					check_parameter.Servo_count[2]=500,TIM8->CCR4=check_parameter.Servo_count[2];
				//Servo rotates forward
				if(check_parameter.Servo_Dir[3]==0 && check_parameter.Servo_count[3]<2500)
					check_parameter.Servo_count[3] = check_parameter.Servo_count[3] + 5;
				//After the forward rotation ends, switch the flag position
				if(check_parameter.Servo_Dir[3]==0 && check_parameter.Servo_count[3]>=2500)
					check_parameter.Servo_Dir[3] = 1;
				//Servo reversal
				if(check_parameter.Servo_Dir[3]==1 && check_parameter.Servo_count[3]>500)
					check_parameter.Servo_count[3] = check_parameter.Servo_count[3] - 5;
				//After the forward rotation ends, switch the flag position
				if(check_parameter.Servo_Dir[3]==1 && check_parameter.Servo_count[3]<=500)
					check_parameter.Servo_Dir[3] = 2;
				//Set duty cycle to control servo rotation
				TIM8->CCR3=check_parameter.Servo_count[3];
				break;
			case 11:
				//If the previous servo does not complete rotation and the verification jumps to the next servo, reset the previous servo
				if((check_parameter.Servo_Dir[3]!=2))				
					check_parameter.Servo_count[3]=500,TIM8->CCR3=check_parameter.Servo_count[3];
				//Servo rotates forward
				if(check_parameter.Servo_Dir[4]==0 && check_parameter.Servo_count[4]<2500)
					check_parameter.Servo_count[4] = check_parameter.Servo_count[4] + 5;
				//After the forward rotation ends, switch the flag position
				if(check_parameter.Servo_Dir[4]==0 && check_parameter.Servo_count[4]>=2500)
					check_parameter.Servo_Dir[4] = 1;
				//Servo reversal
				if(check_parameter.Servo_Dir[4]==1 && check_parameter.Servo_count[4]>500)
					check_parameter.Servo_count[4] = check_parameter.Servo_count[4] - 5;
				//After the forward rotation ends, switch the flag position
				if(check_parameter.Servo_Dir[4]==1 && check_parameter.Servo_count[4]<=500)
					check_parameter.Servo_Dir[4] = 2;
				//Set duty cycle to control servo rotation
				TIM8->CCR2=check_parameter.Servo_count[4];
				break;
			case 12:
				//If the previous servo does not complete rotation and the verification jumps to the next servo, reset the previous servo
				if((check_parameter.Servo_Dir[4]!=2))				
					check_parameter.Servo_count[4]=500,TIM8->CCR2=check_parameter.Servo_count[4];
				//Servo rotates forward
				if(check_parameter.Servo_Dir[5]==0 && check_parameter.Servo_count[5]<2500)
					check_parameter.Servo_count[5] = check_parameter.Servo_count[5] + 5;
				//After the forward rotation ends, switch the flag position
				if(check_parameter.Servo_Dir[5]==0 && check_parameter.Servo_count[5]>=2500)
					check_parameter.Servo_Dir[5] = 1;
				//Servo reversal
				if(check_parameter.Servo_Dir[5]==1 && check_parameter.Servo_count[5]>500)
					check_parameter.Servo_count[5] = check_parameter.Servo_count[5] - 5;
				//After the forward rotation ends, switch the flag position
				if(check_parameter.Servo_Dir[5]==1 && check_parameter.Servo_count[5]<=500)
					check_parameter.Servo_Dir[5] = 2;
				//Set duty cycle to control servo rotation
				TIM8->CCR1=check_parameter.Servo_count[5];
				break;
			case 13:				//After servo self-test ends, reset all servos and related parameters
				for(int i=0; i<6; i++){
					check_parameter.Servo_Dir[i] = 0;
					check_parameter.Servo_count[i] = 500;
				}
				TIM8->CCR1 = check_parameter.Servo_count[5];
				TIM8->CCR2 = check_parameter.Servo_count[4];
				TIM8->CCR3 = check_parameter.Servo_count[3];
				TIM8->CCR4 = check_parameter.Servo_count[2];
				TIM12->CCR1 = check_parameter.Servo_count[1];
				TIM12->CCR2 = check_parameter.Servo_count[0];
				break;;
			case 14:				//Buzzer self-check related
				if((Buzzer_count1/100)%2)			Buzzer = 1;
				else													Buzzer = 0;
				break;
			case 15:				//Turn off the buzzer when leaving buzzer self-test
				Buzzer = 0;
				break;
			case 19:				//Serial port 3 self-test; if a flag bit is detected, the buffer area characters are stored
				if(check_parameter.Usart3_check_flag==1)
				{
					USART3_Return();
					check_parameter.Usart3_check_flag = 0;
					check_parameter.Usart3_rec_count = 0;
				}
				break;
			default:
				break;
		}
	}
}

void Check_Key(void)
{
	u8 tmp;

  //Frequency of incoming tasks
  tmp=KEY_Scan(RATE_100_HZ,0);
	if(tmp==single_click)				//Click
	{
		check_parameter.Process++;
		if(check_parameter.Process==21)			//Self-test completes and exits
		{
			delay_ms(50);
			NVIC_SystemReset();
		}
	}
	else if(tmp==double_click)
	{
		delay_ms(50);
		NVIC_SystemReset();
	}
}




