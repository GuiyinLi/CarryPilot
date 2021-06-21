/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
/*----------------------------------------------------------------------------------------------------------------------/
*               本程序只供购买者学习使用，版权著作权属于无名科创团队，无名科创团队将飞控程序源码提供给购买者，
*               购买者要为无名科创团队提供保护，未经作者许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载， 
*               更不能以此销售牟利，如发现上述行为，无名科创团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
*               生命不息、奋斗不止；前人栽树，后人乘凉！！！
*               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
*               学习优秀者，简历可推荐到DJI、ZEROTECH、XAG、AEE、GDU、AUTEL、EWATT、HIGH GREAT等公司就业
*               求职简历请发送：15671678205@163.com，需备注求职意向单位、岗位、待遇等
*               无名科创开源飞控QQ群：540707961
*               CSDN博客：http://blog.csdn.net/u011992534
*               优酷ID：NamelessCotrun无名小哥
*               B站教学视频：https://space.bilibili.com/67803559/#/video
*               客户使用心得、改进意见征集贴：http://www.openedv.com/forum.php?mod=viewthread&tid=234214&extra=page=1
*               淘宝店铺：https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               百度贴吧:无名科创开源飞控
*               公司官网:www.nameless.tech
*               修改日期:2019/4/12
*               版本：躺赢者——CarryPilot_V1.0
*               版权所有，盗版必究。
*               Copyright(C) 2017-2025 武汉无名创新科技有限公司 
*               All rights reserved
*               重要提示：
*               正常淘宝咸鱼转手的飞控、赠送朋友、传给学弟的都可以进售后群学习交流，
*               不得直接在网上销售无名创新资料，无名创新代码有声明版权，他人不得将
*               资料代码传网上供他人下载，不得以谋利为目的销售资料代码，发现有此操
*               作者，公司会提前告知，请1天内及时处理，否则你的学校、单位、姓名、电
*               话、地址信息会被贴出在公司官网、官方微信公众平台、官方技术博客、知乎
*               专栏以及淘宝店铺首页予以公示公告，此种所作所为，会成为个人污点，影响
*               升学、找工作、社会声誉、很快就很在无人机界出名，后果很严重。
*               因此行为给公司造成重大损失者，会以法律途径解决，感谢您的合作，谢谢！！！
----------------------------------------------------------------------------------------------------------------------*/
#include "Headfile.h"
#include "WP_Ctrl.h"
#include "control_config.h"

/*****************遥控器行程设置**********************/
//默认 关闭定高 关闭GPS 姿态自稳模式 关闭自定义SDK 关闭光流
uint16 Motor_PWM_1,Motor_PWM_2,Motor_PWM_3,Motor_PWM_4,Motor_PWM_5,Motor_PWM_6;//六个电机输出PWM
uint16 Last_Motor_PWM_1,Last_Motor_PWM_2,Last_Motor_PWM_3,Last_Motor_PWM_4,Last_Motor_PWM_5,Last_Motor_PWM_6;//上次六个电机输出PWM
uint8_t Controler_High_Mode=1,Last_Controler_High_Mode=1;             
uint8_t Controler_Horizontal_Mode=1,Last_Controler_Horizontal_Mode=1;  //默认关闭GPS 姿态自稳模式
uint8_t Controler_Land_Mode=0,Last_Controler_Land_Mode=0;        //默认非返航模式
uint8_t Reserve_Mode=1;                      //默认关闭光流
uint8_t Reserve_Mode_Fast_Exchange_Flag=0,Reserve_Mode_Fast_Exchange_Cnt=0;
uint16_t Reserve_Mode_Cnt=0;
uint8_t Control_Mode_Change=0;
uint16_t High_Hold_Throttle=0;
uint8_t  Pos_Hold_SetFlag=0;
uint8_t SDK_Ctrl_Mode=0;
uint8_t SDK_Take_Over_Ctrl = 2;  // 1：SDK控制接管来源于OPENMV、无线数传   2：SDK控制接管来源于人为事先指定  默认为人为指定
uint8_t Start_Task_Flag=0,Start_Fly=0;
uint8_t key_press_flag=0;
/***************************************************
函数名: void Controler_Mode_Select(void)
说明:	控制器模式选择函数
入口:	无
出口:	无
备注:	中断任务调度持续运行
****************************************************/
void Controler_Mode_Select()
{
	//用于切换检测用
  Last_Controler_High_Mode=Controler_High_Mode;//上次高度控制模式
  Last_Controler_Horizontal_Mode=Controler_Horizontal_Mode;//上次位置控制模式
  Last_Controler_Land_Mode=Controler_Land_Mode;//上次返航模式
	//通道5失效    通道6用于一键执行任务 通道7用于控制飞行与降落  通道8失效
  if(PPM_Databuf[4]>=(RC_Calibration[4].max-RC_Calibration[4].deadband))      
  {
		Controler_Land_Mode=2;//急停
		Task_Reset();
		Bling_Set(&Light_2,100,500,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_2,1); 
	}
	if(((PPM_Databuf[5]>=(RC_Calibration[5].max-RC_Calibration[5].deadband)&&key_press_flag==0)||Sun_Sky==1)&&Controler_Land_Mode!=2)  //一键起飞 开定高 开非返航
	{
		Start_Task_Flag=1;
		Controler_High_Mode=2;       //开定高
		Reserve_Mode=2;              //开光流
		Controler_Horizontal_Mode=1; //关GPS
	  Controler_Land_Mode=0;       //开启飞行模式 即非返航模式
		Start_Fly=1;
		key_press_flag=1;
		Sun_Sky=0;  //防止重复进入
	}
	else if(PPM_Databuf[5]<=(RC_Calibration[5].min+RC_Calibration[5].deadband)&&Controler_Land_Mode!=2&&realse_lock_status!=4)     //关定高 在执行任务中拨下 直接急停
	{
		Bling_Set(&Light_1,100,800,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_3,1);  //蓝
		if(Start_Task_Flag==1)  //急停之后必须重启才能继续自动执行任务
		{
			Start_Task_Flag=2;      //此处表示任务强制停止  0：正常执行完毕 1：任务正在执行 2：任务被强制停止
			Controler_Land_Mode=0;  //非返航
			Controler_High_Mode=1;
			Reserve_Mode=1;              //关光流
			SDK_Ctrl_Mode=0;
			key_press_flag=1;       //防止再次进入SDK模式
			Task_Reset();
		}
		if(Start_Task_Flag!=2)
		{
			key_press_flag=0;
		}
	  Controler_High_Mode=1;    //切回姿态自稳
	}
  
//  if(PPM_Databuf[6]>=(RC_Calibration[6].max-RC_Calibration[6].deadband))            {Controler_Land_Mode=1;}//返航模式
//  else if(PPM_Databuf[6]<=(RC_Calibration[6].middle+RC_Calibration[6].deadband))   {Controler_Land_Mode=0;}//非返航模式
//  
//  if(PPM_Databuf[6]>=(RC_Calibration[6].middle-RC_Calibration[6].deadband)
//     &&PPM_Databuf[6]<=(RC_Calibration[6].middle+RC_Calibration[6].deadband))//遥控器三段开关处于中位     
//  {
//    SDK_Ctrl_Mode=1;
//  }
//  else
//  {
//    SDK_Ctrl_Mode=0;
//		NCQ_SDK_Reset();
//		Rectangle_Reset();
//  }  
  
//  if(PPM_Databuf[7]>=(RC_Calibration[7].max-RC_Calibration[7].deadband))      {Reserve_Mode=2;} //八通道在高位  光流定点参与
//  else if(PPM_Databuf[7]<=(RC_Calibration[7].min+RC_Calibration[7].deadband)) 
//  {
//    Reserve_Mode=1;
//    OpticalFlow_SINS_Reset();
//    OpticalFlow_Ctrl_Reset();
//  }  //八通道在低位  光流定点不参与
  if(Start_Fly==1)
	{
		Start_Fly=0;
		Reserve_Mode_Fast_Exchange_Flag=1;	
		if(Hung_Thing)
			Total_Controller.High_Position_Control.Expect=NamelessQuad.Position[_YAW]+Hung_Thing_Target;  //起落架对地的高度
		else
			Total_Controller.High_Position_Control.Expect=NamelessQuad.Position[_YAW]+Auto_Launch_Target; 
		Unwanted_Lock_Flag=0;
	}
  if(Reserve_Mode_Fast_Exchange_Flag==1)//快速起飞	
  {
		if(!Hung_Thing)
		{
			Total_Controller.High_Acce_Control.Scale_Kp=1.0f;
			Total_Controller.High_Acce_Control.Scale_Ki=1.0f; 
			Total_Controller.High_Speed_Control.Scale_Kp=1.0;
			Total_Controller.High_Speed_Control.Scale_Ki=1.0f;
			Total_Controller.High_Position_Control.Scale_Kp=1.0;
			Total_Controller.High_Position_Control.Scale_Ki=1.0f; 
		}
    else //悬挂重物起飞
		{//原加速度环 0.3 1.0 1.0f

			Total_Controller.Yaw_Gyro_Control.Scale_Kp=1.3f;
			Total_Controller.Yaw_Gyro_Control.Scale_Ki=1.0f;
			Total_Controller.Yaw_Gyro_Control.Scale_Kd=1.8f;
			
			Total_Controller.Pitch_Gyro_Control.Scale_Kp=1.3f;
			Total_Controller.Pitch_Gyro_Control.Scale_Ki=1.0f;
			Total_Controller.Pitch_Gyro_Control.Scale_Kd=1.8f;

			Total_Controller.Roll_Gyro_Control.Scale_Kp=1.3f;
			Total_Controller.Roll_Gyro_Control.Scale_Ki=1.0f;
			Total_Controller.Roll_Gyro_Control.Scale_Kd=1.8f;
		}
    if(Total_Controller.High_Position_Control.Expect<=NamelessQuad.Position[_YAW])//反馈高度大于期望高度，即恢复正常 起飞完成 开始巡线
    {
      Reserve_Mode_Fast_Exchange_Flag=0;     //关闭快速起飞 
			SDK_Take_Over_Ctrl=2;                  //转换到自定义SDK模式
			SDK_Ctrl_Mode=1;                       //开启自定义SDK模式 进行巡线
			yellow_barcode_pos=OpticalFlow_SINS.Position[_ROLL];  //记录巡线前的光流位移
    }	
  }
  else 
  { 
    Total_Controller.High_Acce_Control.Scale_Kp=1.0f;
    Total_Controller.High_Acce_Control.Scale_Ki=1.0f; 
    Total_Controller.High_Speed_Control.Scale_Kp=1.0;
    Total_Controller.High_Speed_Control.Scale_Ki=1.0f; 
    Total_Controller.High_Position_Control.Scale_Kp=1.0;
    Total_Controller.High_Position_Control.Scale_Ki=1.0f; 
  }
  
  if(Unwanted_Lock_Flag==1)//定高模式解锁后，无任何操作
  {
    Thr_Push_Over_State=Thr_Push_Over_Deadband();
    if(Thr_Push_Over_State==2)//只要向上推过了中位死区，即把允许自动上锁操作
    {
      Unwanted_Lock_Flag=0;
    }
    else
    {
      Take_Off_Reset();//清积分
      //Throttle_Control_Reset();//清积分
    }
  }
  
  if(Controler_Land_Mode!=Last_Controler_Land_Mode)
  {
    if(Controler_Land_Mode==0)  
      land_reset();//返航模式切回正常模式
    Total_Controller.High_Position_Control.Expect=NamelessQuad.Position[_YAW];//将开关拨动瞬间的惯导高度设置为期望高度
  }
  
  if(Controler_High_Mode!=Last_Controler_High_Mode)
  {
    if(Controler_High_Mode==2)  {Control_Mode_Change=1;}//自稳切定高，设置基准油门值，悬停高度
    if(Controler_High_Mode==1)  {Control_Mode_Change=1;}//定高切自稳
  }
	if(Control_Mode_Change==1&&Start_Task_Flag!=1)//存在定高模式切换，并且为非自动模式
  {
    if(Controler_High_Mode==High_Hold_Mode)//本次为定高模式，即自稳切定高
    {
      High_Hold_Throttle=Throttle_Control;//保存当前油门值，只存一次
      /*******************将当前惯导竖直位置估计作为目标高度***************************/
    }
    else//本次为自稳模式，即定高切自稳
    {
      //Throttle_Control_Reset();
    }
    Control_Mode_Change=0;//将模式切换位置0,有且仅处理一次
  }
}



uint16 Value_Limit(uint16 Min,uint16 Max,uint16 Data)
{
  if(Data>=Max) Data=Max;
  else if(Data<=Min) Data=Min;
  return Data;
}



void Angle_Control()//角度环节
{
  static uint16 Yaw_Cnt=0;
  //角度反馈
  Total_Controller.Pitch_Angle_Control.FeedBack=(Pitch-Pitch_Offset);
  PID_Control(&Total_Controller.Pitch_Angle_Control);
  Total_Controller.Roll_Angle_Control.FeedBack =(Roll-Roll_Offset);
  PID_Control(&Total_Controller.Roll_Angle_Control);
  
  if(Yaw_Control==0)//偏航杆置于中位
  {
    if(Yaw_Cnt<=500)//无头模式、飞机上电后一段时间锁定偏航角，磁力计、陀螺仪融合需要一段时间，这里取500
    {
      Yaw_Cnt++;
    }
    if(Total_Controller.Yaw_Angle_Control.Expect==0//回中时赋角度期望值
       ||Yaw_Cnt<=500
         ||Landon_Earth_Flag==1)//或者满足触地条件、复位偏航角期望
    {
      Total_Controller.Yaw_Angle_Control.Expect=Yaw;
    }
    Total_Controller.Yaw_Angle_Control.FeedBack=Yaw;//偏航角反馈
    PID_Control_Yaw(&Total_Controller.Yaw_Angle_Control);//偏航角度控制
    Total_Controller.Yaw_Gyro_Control.Expect=Total_Controller.Yaw_Angle_Control.Control_OutPut;//偏航角速度环期望，来源于偏航角度控制器输出
    
    if(SDK_Line.line_ctrl_enable==1&&SDK_Ctrl_Mode==1)//SDK模式下的循迹模式，偏航角速度期望来源于SDK数据
    {
      Total_Controller.Yaw_Gyro_Control.Expect=3*SDK_Target_Yaw_Gyro;
      Total_Controller.Yaw_Angle_Control.Expect=0;//偏航角期望给0,不进行角度控制
      //SDK_Line.flag=0;
    }
  }
  else//波动偏航方向杆后，只进行内环角速度控制
  {
    Total_Controller.Yaw_Angle_Control.Expect=0;//偏航角期望给0,不进行角度控制
    Total_Controller.Yaw_Gyro_Control.Expect=Yaw_Control;//偏航角速度环期望，直接来源于遥控器打杆量
  }
  //Total_Controller.Yaw_Gyro_Control.Expect=Yaw_Control;//偏航角速度环期望，直接来源于遥控器打杆量
}



uint16_t Yaw_Control_Fault_Cnt=0;
float Last_Yaw_Gyro_Control_Expect=0,Yaw_Gyro_Control_Expect_Delta=0;
float Last_Pitch_Gyro_Control_Expect=0,Pitch_Gyro_Control_Expect_Delta=0;
float Last_Roll_Gyro_Control_Expect=0,Roll_Gyro_Control_Expect_Delta=0;
float Pitch_Roll_Feedforward_Kp=0.0f,Pitch_Roll_Feedforward_Kd=0.0f;// 1.45  0.25         0.85     //0.45
float Yaw_Feedforward_Kp=0.0f,Yaw_Feedforward_Kd=0.05f;//偏航角前馈控制 0.15   1.0  0.3         //0.05
void Gyro_Control()//角速度环
{
  if(GYRO_CONTROL_MODE==PID_MODE)//俯仰、横滚方向姿态内环角速度控制器采用PID控制器
  {
    /***************内环角速度期望****************/
    Total_Controller.Pitch_Gyro_Control.Expect=Total_Controller.Pitch_Angle_Control.Control_OutPut;
    Total_Controller.Roll_Gyro_Control.Expect=Total_Controller.Roll_Angle_Control.Control_OutPut;
    
    /***************内环角速度反馈****************/
    Total_Controller.Pitch_Gyro_Control.FeedBack=Pitch_Gyro;
    Total_Controller.Roll_Gyro_Control.FeedBack=Roll_Gyro;
    
    /***************内环角速度控制****************/
    //PID_Control_Div_LPF(&Total_Controller.Pitch_Gyro_Control);
    //PID_Control_Div_LPF(&Total_Controller.Roll_Gyro_Control);    
    /***************内环角速度控制：微分参数动态调整****************/
    PID_Control_Div_LPF_For_Gyro(&Total_Controller.Pitch_Gyro_Control);
    PID_Control_Div_LPF_For_Gyro(&Total_Controller.Roll_Gyro_Control);
    
    Pitch_Gyro_Control_Expect_Delta=1000*(Total_Controller.Pitch_Gyro_Control.Expect-Last_Pitch_Gyro_Control_Expect
                                          /Total_Controller.Pitch_Gyro_Control.PID_Controller_Dt.Time_Delta);
    Roll_Gyro_Control_Expect_Delta=1000*(Total_Controller.Roll_Gyro_Control.Expect-Last_Roll_Gyro_Control_Expect
                                         /Total_Controller.Roll_Gyro_Control.PID_Controller_Dt.Time_Delta);
    
    Last_Pitch_Gyro_Control_Expect=Total_Controller.Pitch_Gyro_Control.Expect;
    Last_Roll_Gyro_Control_Expect=Total_Controller.Roll_Gyro_Control.Expect;
    
    Total_Controller.Pitch_Gyro_Control.Control_OutPut+=Pitch_Roll_Feedforward_Kd*Pitch_Gyro_Control_Expect_Delta
      +Pitch_Roll_Feedforward_Kp*Total_Controller.Pitch_Gyro_Control.Expect;
    Total_Controller.Pitch_Gyro_Control.Control_OutPut=constrain_float(Total_Controller.Pitch_Gyro_Control.Control_OutPut,
                                                                       -Total_Controller.Pitch_Gyro_Control.Control_OutPut_Limit,
                                                                       Total_Controller.Pitch_Gyro_Control.Control_OutPut_Limit);
    
    Total_Controller.Roll_Gyro_Control.Control_OutPut+=Pitch_Roll_Feedforward_Kd*Roll_Gyro_Control_Expect_Delta
      +Pitch_Roll_Feedforward_Kp*Total_Controller.Roll_Gyro_Control.Expect;
    Total_Controller.Roll_Gyro_Control.Control_OutPut=constrain_float(Total_Controller.Roll_Gyro_Control.Control_OutPut,
                                                                      -Total_Controller.Roll_Gyro_Control.Control_OutPut_Limit,
                                                                      Total_Controller.Roll_Gyro_Control.Control_OutPut_Limit);
    
  }
  else if(GYRO_CONTROL_MODE==ADRC_MODE)//俯仰、横滚方向姿态内环角速度控制器采用ADRC自抗扰控制器
  {
    
  }
  else//测试用、正常只选择一种模式
  {
    /***************内环角速度期望****************/
    Total_Controller.Pitch_Gyro_Control.Expect=Total_Controller.Pitch_Angle_Control.Control_OutPut;
    Total_Controller.Roll_Gyro_Control.Expect=Total_Controller.Roll_Angle_Control.Control_OutPut;
    /***************内环角速度反馈****************/
    Total_Controller.Pitch_Gyro_Control.FeedBack=Pitch_Gyro;
    Total_Controller.Roll_Gyro_Control.FeedBack=Roll_Gyro;
    
    /***************内环角速度控制****************/
    PID_Control_Div_LPF(&Total_Controller.Pitch_Gyro_Control);
    PID_Control_Div_LPF(&Total_Controller.Roll_Gyro_Control);
  }
  
  //偏航角前馈控制
  //Total_Controller.Yaw_Gyro_Control.FeedBack=Yaw_Gyro;
  Total_Controller.Yaw_Gyro_Control.FeedBack=Yaw_Gyro_Earth_Frame;//Yaw_Gyro;
  
  PID_Control_Div_LPF(&Total_Controller.Yaw_Gyro_Control);
  Yaw_Gyro_Control_Expect_Delta=1000*(Total_Controller.Yaw_Gyro_Control.Expect-Last_Yaw_Gyro_Control_Expect)
    /Total_Controller.Yaw_Gyro_Control.PID_Controller_Dt.Time_Delta;
  //**************************偏航角前馈控制**********************************
  Total_Controller.Yaw_Gyro_Control.Control_OutPut+=Yaw_Feedforward_Kp*Total_Controller.Yaw_Gyro_Control.Expect
    +Yaw_Feedforward_Kd*Yaw_Gyro_Control_Expect_Delta;//偏航角前馈控制
  Total_Controller.Yaw_Gyro_Control.Control_OutPut=constrain_float(Total_Controller.Yaw_Gyro_Control.Control_OutPut,
                                                                   -Total_Controller.Yaw_Gyro_Control.Control_OutPut_Limit,
                                                                   Total_Controller.Yaw_Gyro_Control.Control_OutPut_Limit);
  Last_Yaw_Gyro_Control_Expect=Total_Controller.Yaw_Gyro_Control.Expect;
  /*******偏航控制异常情况判断，即偏航控制量很大时，偏航角速度很小，如此时为强外力干扰、已着地等******************************/
  if(ABS(Total_Controller.Yaw_Gyro_Control.Control_OutPut)>Total_Controller.Yaw_Gyro_Control.Control_OutPut_Limit/2//偏航控制输出相对较大
     &&ABS(Yaw_Gyro)<=30.0f)//偏航角速度相对很小
  {
    Yaw_Control_Fault_Cnt++;
    if(Yaw_Control_Fault_Cnt>=500) Yaw_Control_Fault_Cnt=500;
  }
  else Yaw_Control_Fault_Cnt/=2;//不满足，快速削减至0
  
  if(Yaw_Control_Fault_Cnt>=400)//持续5ms*400=2S,特殊处理
  {
    PID_Integrate_Reset(&Total_Controller.Yaw_Gyro_Control);//清空偏航角速度控制的积分
    PID_Integrate_Reset(&Total_Controller.Yaw_Angle_Control);//清空偏航角控制的积分
    Total_Controller.Yaw_Angle_Control.Expect=Yaw;//将当前偏航角，作为期望偏航角
    Yaw_Control_Fault_Cnt=0;
  }
  /*******偏航控制异常处理结束******************************/
}

uint16 Throttle=0,Last_Throttle=0;
void Main_Leading_Control(void)
{
  /*********************根据遥控器切换档位，飞控进入不同模式****************************/
  if(Controler_Land_Mode==0)//非返航着陆模式
  {
    if(Controler_High_Mode==1//姿态自稳定模式
       &&Controler_Horizontal_Mode==1)//GPS定点档位未设置   默认遥控器控制
    {
      Total_Controller.Pitch_Angle_Control.Expect=Target_Angle[0];
      Total_Controller.Roll_Angle_Control.Expect=Target_Angle[1];
      
      if(Throttle_Control<=1000)   Throttle=1000;
      else Throttle=Throttle_Control;//油门直接来源于遥控器油门给定
      Last_Throttle=Throttle;
    }
    else if(Controler_High_Mode==2//定高模式
            &&Controler_Horizontal_Mode==1)//GPS定点档位未设置
    {
      /**************************定高模式，水平姿态期望角来源于遥控器******************************************/
      
#if  (Optical_Enable==0)
      Total_Controller.Pitch_Angle_Control.Expect=Target_Angle[0];
      Total_Controller.Roll_Angle_Control.Expect=Target_Angle[1];
			ncq_control_althold();//高度控制
#else   //光流辅助悬停
      if(Reserve_Mode==2&&OpticalFlow_Is_Work==1
				&&(Sensor_Flag.Hcsr04_Health==1||tfdata.health==1))//超声波有效且存在光流外设时，才允许进入光流模式
      {  
        if(SDK_Take_Over_Ctrl==1)        //遇到条形码或者二维码则进入此函数段
        {
          OpticalFlow_Control(0);//普通光流模式、无线数传与OPENMV参与的SDK模式
          ncq_control_althold();//高度控制
        }
        else if(SDK_Take_Over_Ctrl==2)  //先进入此代码段  进行普通巡线
        {
					if(SDK_Ctrl_Mode==1)    
					{
						rectangle_duty(365,55,0);//用户事先指定的SDK开发者模式 
					}
					else  
					{
						OpticalFlow_Control_Pure(0);//普通光流模式
						ncq_control_althold();//高度控制
					}
        }
      }
      else   //光流无效时 仍然由遥控器控制
      {
        Total_Controller.Pitch_Angle_Control.Expect=Target_Angle[0];
        Total_Controller.Roll_Angle_Control.Expect=Target_Angle[1];
				ncq_control_althold();//高度控制
      }
#endif
      
    }
    else//其它
    {
      Total_Controller.Pitch_Angle_Control.Expect=Target_Angle[0];
      Total_Controller.Roll_Angle_Control.Expect=Target_Angle[1];
      if(Throttle_Control<=1000)   Throttle=1000;
      else Throttle=Throttle_Control;//油门直接来源于遥控器油门给定
      Last_Throttle=Throttle;
    }
    land_state_check();
  }
  else if(Controler_Land_Mode==1)//返航着陆模式
  {
    land_run();
  }
	else if(Controler_Land_Mode==2) //急停模式
	{
		Throttle=Thr_Min;
		Auto_Relock_Flag=1;
	}
}
/************姿态环控制器：角度+角速度****************/
void Attitude_Control(void)
{
  Angle_Control();//角度控制
  Gyro_Control();//角速度控制
}


float Active_Para1,Active_Para2;
/***************************************************
函数名: void Total_Control(void)
说明:	总控制器运行，大体分三步：
1、根据遥控器输入、当前状态，给定运行模式（自稳+油门手动、定高+自稳、定高+定点（控速）等）
2、主导上层控制器给定姿态期望，高度控制等
3、自稳（姿态）控制
入口:	无
出口:	无
备注:	上电初始化，运行一次
****************************************************/
void Total_Control(void)
{
  /*************主导控制器******************/
  Main_Leading_Control();		
  /*************姿态环控制器*****************/
  Attitude_Control();
}


void CarryPilot_Control(void)
{
  static uint8_t ctrl_cnt=0;
	static uint16_t lock_cnt=0;
  ctrl_cnt++;  
	if(realse_lock_status==1)
	{
		lock_cnt++;
		if(lock_cnt>=1000)  //按下按键5s后开始解锁
		{
			lock_cnt=0;
			realse_lock_status=2;
		}
	}
	else if(realse_lock_status==3)  //解锁完成 通过延时来一键日天
	{
		lock_cnt++;
		//更改 减少怠速延时时间 防止高度融合出现波动
		if(lock_cnt>=100)  //解锁500ms后开始执行任务
		{
			lock_cnt=0;
			realse_lock_status=4;   //解锁状态归零
			Sun_Sky=1;         //开启一键日天
		}
	}
  /*************控制器模式选择******************/
  if(ctrl_cnt>=4)//改此计数器值，可以调整控制周期
  {  
    Controler_Mode_Select();
    ctrl_cnt=0;
  }
  Total_Control();//总控制器：水平位置+水平速度+姿态（角度+角速度）控制器，高度位置+高度速度+高度加速度控制器	
  Control_Output();//控制量总输出
}
uint16_t Throttle_Output=0;
void Throttle_Angle_Compensate()//油门倾角补偿
{
  float CosPitch_CosRoll=ABS(Cos_Pitch*Cos_Roll);
  float Throttle_Makeup=0;
  float Temp=0;
  if(CosPitch_CosRoll>=0.999999f)  CosPitch_CosRoll=0.999999f;
  if(CosPitch_CosRoll<=0.000001f)  CosPitch_CosRoll=0.000001f;
  if(CosPitch_CosRoll<=0.50f)  CosPitch_CosRoll=0.50f;//Pitch,Roll约等于30度
  if(Throttle>=Thr_Start)//大于起转油门量
  {
    Temp=(uint16_t)(MAX(ABS(100*Pitch),ABS(100*Roll)));
    Temp=constrain_float(9000-Temp,0,3000)/(3000*CosPitch_CosRoll);
    Throttle_Makeup=(Throttle-Thr_Start)*Temp;//油门倾角补偿
    Throttle_Output=(uint16_t)(Thr_Start+Throttle_Makeup);
    Throttle_Output=(uint16_t)(constrain_float(Throttle_Output,Thr_Start,2000));
  }
  else Throttle_Output=Throttle;
}


/**************************************************************
***************************************************************
X型安装方式，电机序号与姿态角关系
                   -
                 Pitch
					3#             1#
					   *          *
-   Roll          *         Roll   +
						 *          *
					2#             4#
				         Pitch
								   +
加速度传感器轴向与载体X、Y、Z同轴，沿轴向原点看，逆时针旋转角度为+
Y Aixs
*
*
*
*
*
*
* * * * * * * *   X Axis
(0)
*******************************************************************
******************************************************************/
uint16_t Idel_Cnt=0;
#define Idel_Transition_Gap 4//怠速递增间隔时间 4*5=20ms
#define Idel_Transition_Period (Thr_Idle-Thr_Min)//怠速启动最大计数器  20ms*100=1s
uint16_t Thr_Idle_Transition_Cnt=0;
void Control_Output()
{
  Throttle_Angle_Compensate();//油门倾角补偿
  landon_earth_check();//着陆条件自检
  if(Controler_State==Unlock_Controler)//解锁
  {
    if(Landon_Earth_Flag==1)//检测到着陆条件
    {
      if(Last_Motor_PWM_1<=Thr_Min
         &&Last_Motor_PWM_2<=Thr_Min
           &&Last_Motor_PWM_3<=Thr_Min
             &&Last_Motor_PWM_4<=Thr_Min
               //&&Last_Motor_PWM_5<=Thr_Min
               //&&Last_Motor_PWM_6<=Thr_Min
               )//只有上锁后再解锁时才会满足
      {
        //如果上次油门输出值为最低位，进入怠速时，安排过渡过程
        Thr_Idle_Transition_Cnt=Idel_Transition_Period;
      }
      else//其他时刻进入着陆条件
      {
        if(Last_Landon_Earth_Flag==0)//上次为起飞状态，本次为着陆状态，上锁电机
        {
          Controler_State=Lock_Controler;
          Bling_Set(&Light_1,3000,200,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_1,0);
          Bling_Set(&Light_2,3000,200,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);
          Bling_Set(&Light_3,3000,200,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_3,0);
        }
      }
      
      Idel_Cnt++;
      if(Idel_Cnt>=Idel_Transition_Gap)
      {
        if(Thr_Idle_Transition_Cnt>=1)
          Thr_Idle_Transition_Cnt--;
        Idel_Cnt=0;
      }
      Motor_PWM_1=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;//油门怠速
      Motor_PWM_2=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;
      Motor_PWM_3=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;
      Motor_PWM_4=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;
//      Motor_PWM_5=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;
//      Motor_PWM_6=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;
      Take_Off_Reset();//清积分
      OpticalFlow_SINS_Reset();
      OpticalFlow_Ctrl_Reset();
    }
    else  //解锁后不满足着陆条件，默认起飞
    {
      if(Controler_High_Mode==1)//姿态档位
      {
        if(Throttle>=Thr_Fly_Start)//大于起飞油门
        {
          if(GYRO_CONTROL_MODE==PID_MODE)//水平姿态环角速度读控制器来源于PID
          {
            Motor_PWM_1=Int_Sort(
                                 Moter1_Thr_Scale*Throttle_Output
                                   +Moter1_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                     +Moter1_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                       +Moter1_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
            Motor_PWM_2=Int_Sort(
                                 Moter2_Thr_Scale*Throttle_Output
                                   +Moter2_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                     +Moter2_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                       +Moter2_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
            Motor_PWM_3=Int_Sort(
                                 Moter3_Thr_Scale*Throttle_Output
                                   +Moter3_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                     +Moter3_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                       +Moter3_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
            Motor_PWM_4=Int_Sort(
                                 Moter4_Thr_Scale*Throttle_Output
                                   +Moter4_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                     +Moter4_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                       +Moter4_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
//            Motor_PWM_5=Int_Sort(
//                                 Moter5_Thr_Scale*Throttle_Output
//                                   +Moter5_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
//                                     +Moter5_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
//                                       +Moter5_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
//            Motor_PWM_6=Int_Sort(
//                                 Moter6_Thr_Scale*Throttle_Output
//                                   +Moter6_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
//                                     +Moter6_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
//                                       +Moter6_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
//            
          }
          else//水平姿态环角速度读控制器来源于ADRC
          {
            
          }
        }
        else//小于起飞油门
        {
          Motor_PWM_1=Int_Sort(Throttle_Output);
          Motor_PWM_2=Int_Sort(Throttle_Output);
          Motor_PWM_3=Int_Sort(Throttle_Output);
          Motor_PWM_4=Int_Sort(Throttle_Output);
//          Motor_PWM_5=Int_Sort(Throttle_Output);
//          Motor_PWM_6=Int_Sort(Throttle_Output);
          Take_Off_Reset();//清积分
        }
        Motor_PWM_1=Value_Limit(Thr_Idle,2000,Motor_PWM_1);//总输出限幅
        Motor_PWM_2=Value_Limit(Thr_Idle,2000,Motor_PWM_2);
        Motor_PWM_3=Value_Limit(Thr_Idle,2000,Motor_PWM_3);
        Motor_PWM_4=Value_Limit(Thr_Idle,2000,Motor_PWM_4);
        Motor_PWM_5=Value_Limit(Thr_Idle,2000,Motor_PWM_5);
        Motor_PWM_6=Value_Limit(Thr_Idle,2000,Motor_PWM_6);
				
				Throttle_Control_Reset();
      }
      else if(Controler_High_Mode==2)//油门托管、定高档位
      {
        if(GYRO_CONTROL_MODE==PID_MODE)//水平姿态环角速度读控制器来源于PID
        {
          Motor_PWM_1=Int_Sort(
                               Moter1_Thr_Scale*Throttle_Output
                                 +Moter1_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                   +Moter1_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                     +Moter1_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
          Motor_PWM_2=Int_Sort(
                               Moter2_Thr_Scale*Throttle_Output
                                 +Moter2_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                   +Moter2_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                     +Moter2_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
          Motor_PWM_3=Int_Sort(
                               Moter3_Thr_Scale*Throttle_Output
                                 +Moter3_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                   +Moter3_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                     +Moter3_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
          Motor_PWM_4=Int_Sort(
                               Moter4_Thr_Scale*Throttle_Output
                                 +Moter4_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                   +Moter4_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                     +Moter4_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
//          Motor_PWM_5=Int_Sort(
//                               Moter5_Thr_Scale*Throttle_Output
//                                 +Moter5_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
//                                   +Moter5_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
//                                     +Moter5_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
//          Motor_PWM_6=Int_Sort(
//                               Moter6_Thr_Scale*Throttle_Output
//                                 +Moter6_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
//                                   +Moter6_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
//                                     +Moter6_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
        }
        else//水平姿态环角速度读控制器来源于ADRC
        {
          
          
        }
        Motor_PWM_1=Value_Limit(Thr_Idle,2000,Motor_PWM_1);//总输出限幅
        Motor_PWM_2=Value_Limit(Thr_Idle,2000,Motor_PWM_2);
        Motor_PWM_3=Value_Limit(Thr_Idle,2000,Motor_PWM_3);
        Motor_PWM_4=Value_Limit(Thr_Idle,2000,Motor_PWM_4);
        Motor_PWM_5=Value_Limit(Thr_Idle,2000,Motor_PWM_5);
        Motor_PWM_6=Value_Limit(Thr_Idle,2000,Motor_PWM_6);
      }
    }
  }
  else//未解锁，油门置于最低位，停转
  {
    Motor_PWM_1=Thr_Min;
    Motor_PWM_2=Thr_Min;
    Motor_PWM_3=Thr_Min;
    Motor_PWM_4=Thr_Min;
//    Motor_PWM_5=Thr_Min;
//    Motor_PWM_6=Thr_Min;
    Take_Off_Reset();//清积分
    Throttle_Control_Reset();
	}
  Last_Motor_PWM_1=Motor_PWM_1;
  Last_Motor_PWM_2=Motor_PWM_2;
  Last_Motor_PWM_3=Motor_PWM_3;
  Last_Motor_PWM_4=Motor_PWM_4;
  Last_Motor_PWM_5=Motor_PWM_5;
  Last_Motor_PWM_6=Motor_PWM_6;
  
  Motor_PWM_1=Value_Limit(0,2000,Motor_PWM_1);//总输出限幅
  Motor_PWM_2=Value_Limit(0,2000,Motor_PWM_2);
  Motor_PWM_3=Value_Limit(0,2000,Motor_PWM_3);
  Motor_PWM_4=Value_Limit(0,2000,Motor_PWM_4);
  Motor_PWM_5=Value_Limit(0,2000,Motor_PWM_5);
  Motor_PWM_6=Value_Limit(0,2000,Motor_PWM_6);
  if(	Controler_Land_Mode==2)
	{
		Motor_PWM_1=0,Motor_PWM_2=0,Motor_PWM_3=0,Motor_PWM_4=0;  //四桨输出为1000 停止转动
	}
  PWM_Output(1.25*Motor_PWM_1,1.25*Motor_PWM_2,1.25*Motor_PWM_3,1.25*Motor_PWM_4,Motor_PWM_5,Motor_PWM_6,0,0);//Motor_PWM_5->PITCH   Motor_PWM_6->ROLL
}




