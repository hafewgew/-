//#include "Wire.h"
//#include "I2Cdev.h"
//#include "MPU6050.h"

#include "Wire.h"
#include <MPU6050_light.h>
#include <U8g2lib.h>
#include <SoftwareSerial.h>
//-------mpu6050相关 开始-------------------------------

MPU6050 mpu(Wire);
unsigned long timer = 0;
unsigned long time;
float yaw;
/*======================定义OLED对象=================*/
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);
//-------mpu6050相关 结束-------------------------------
int startTime=5; //启动等待时间，5秒
int speedLeft;
int speedRight;
int myspeed=10.5;


//A 左前 B左后  C 右后，D右前
unsigned int Motor_AIN1=5;       //控制A电机的PWM引脚  一定改成自己用的
unsigned int Motor_AIN2=6;  
unsigned int Motor_BIN1=8;       //控制B电机的PWM引脚  一定改成自己用的
unsigned int Motor_BIN2=7; 
unsigned int Motor_CIN1=11;       //控制C电机的PWM引脚  一定改成自己用的
unsigned int Motor_CIN2=12; 
unsigned int Motor_DIN1=44;       //控制D电机的PWM引脚  一定改成自己用的
unsigned int Motor_DIN2=46;       

int valueA;                       //用于存储通过PI控制器计算得到的用于调整电机转速的PWM值的整形变量 
int valueB; 
int valueC; 
int valueD; 
float F1,F2,F3,F4,F5,F6,F7,F8;
int f1,f2,f3,f4,f5,f6,f7,f8;
int x=0,y=0;
int cq=0;
float yawBegin=0;

#include <FlexiTimer2.h>         //定时中断头文件库
/***********编码器引脚************/
#define ENCODERA_A 2              //编码器A相引脚
#define ENCODERA_B 51             //编码器B相引脚
#define ENCODERB_A 3              //编码器A相引脚
#define ENCODERB_B 53            //编码器B相引脚
#define ENCODERC_A 18              //编码器A相引脚
#define ENCODERC_B 52            //编码器B相引脚
#define ENCODERD_A 19              //编码器A相引脚
#define ENCODERD_B 50             //编码器B相引脚

int VelocityA,CountA=0;            //Count计数变量 Velocity存储设定时间内A相上升沿和下降沿的个数
float VelocityA_KP =7.2, VelocityA_KI =0.68,TargetA;//Velocity_KP,Velocity_KI.PI参数  Target目标值

int VelocityB,CountB=0;            //Count计数变量 Velocity存储设定时间内A相上升沿和下降沿的个数
float VelocityB_KP =7.2, VelocityB_KI =0.68,TargetB;//Velocity_KP,Velocity_KI.PI参数  Target目标值

int VelocityC,CountC=0;            //Count计数变量 Velocity存储设定时间内A相上升沿和下降沿的个数
float VelocityC_KP =7.2, VelocityC_KI =0.68,TargetC;//Velocity_KP,Velocity_KI.PI参数  Target目标值

int VelocityD,CountD=0;            //Count计数变量 Velocity存储设定时间内A相上升沿和下降沿的个数
float VelocityD_KP =7.2, VelocityD_KI =0.68,TargetD;//Velocity_KP,Velocity_KI.PI参数  Target目标值

/*********** 限幅************
*以下两个参与让输出的PWM在一个合理区间
*当输出的PWM小于40时电机不转 所以要设置一个启始PWM
*arduino mega 2560 单片机的PWM不能超过255 所以 PWM_Restrict 起到限制上限的作用
*****************************/
int startPWMA=15;                 //初始PWM
int PWMA_Restrict=80;            //startPW+PWM_Restric=255<256
int startPWMB=15;                 //初始PWM
int PWMB_Restrict=80;            //startPW+PWM_Restric=255<256
int startPWMC=15;                 //初始PWM
int PWMC_Restrict=80;            //startPW+PWM_Restric=255<256
int startPWMD=15;                 //初始PWM
int PWMD_Restrict=80;            //startPW+PWM_Restric=255<256

int IncrementalA_PI (int EncoderA,int TargetA)
{  
static float BiasA,PWMA,Last_biasA;
   BiasA=TargetA-EncoderA;                                  //计算偏差
   PWMA+=VelocityA_KP*(BiasA-Last_biasA)+VelocityA_KI*BiasA;   //增量式PI控制器
   Last_biasA=BiasA;                                       //保存上一次偏差 
   return PWMA;                                           //增量输出
}
int IncrementalB_PI (int EncoderB,int TargetB)
{  
static float BiasB,PWMB,Last_biasB;
   BiasB=TargetB-EncoderB;                                  //计算偏差
   PWMB+=VelocityB_KP*(BiasB-Last_biasB)+VelocityB_KI*BiasB;   //增量式PI控制器
   Last_biasB=BiasB;                                       //保存上一次偏差 
   return PWMB;                                           //增量输出
}

int IncrementalC_PI (int EncoderC,int TargetC)
{  
                                         
static float BiasC,PWMC,Last_biasC;
   BiasC=TargetC-EncoderC;                                  //计算偏差
   PWMC+=VelocityC_KP*(BiasC-Last_biasC)+VelocityC_KI*BiasC;   //增量式PI控制器
   Last_biasC=BiasC;                                       //保存上一次偏差 
   return PWMC;                                           //增量输出
}
int IncrementalD_PI (int EncoderD,int TargetD)
{  
                                 
static float BiasD,PWMD,Last_biasD;
   BiasD=TargetD-EncoderD;                                  //计算偏差
   PWMD+=VelocityD_KP*(BiasD-Last_biasD)+VelocityD_KI*BiasD;   //增量式PI控制器
   Last_biasD=BiasD;                                       //保存上一次偏差 
   return PWMD;                                           //增量输出
}

//SoftwareSerial mySerial(0, 1); // RX, TX
SoftwareSerial mySerial1(22, 23); // RX, TX
/**********************************************/
//屏幕定义
#define TJC mySerial1
//#define Serial mySerial
void setup() 
{
  Serial.begin(115200); //打开串口,Serial通信
  Serial2.begin(115200);//机械臂通信
  TJC.begin(115200);//屏幕通信

//屏幕通信
while (TJC.read() >= 0){ //清空串口缓冲区
  TJC.write("page main\xff\xff\xff"); //发送命令让屏幕跳转到main页面
}

//陀螺仪初始化

  mpu_init(); 

//  Serial.println("/*****开始驱动*****/");
  pinMode(ENCODERA_A,INPUT);     //设置两个相线为输入模式
  pinMode(ENCODERA_B,INPUT);
  pinMode(ENCODERB_A,INPUT);     //设置两个相线为输入模式
  pinMode(ENCODERB_B,INPUT);
  pinMode(ENCODERC_A,INPUT);     //设置两个相线为输入模式
  pinMode(ENCODERC_B,INPUT);
  pinMode(ENCODERD_A,INPUT);     //设置两个相线为输入模式
  pinMode(ENCODERD_B,INPUT);
  pinMode(Motor_AIN1,OUTPUT);   //设置两个驱动引脚为输出模式
  pinMode(Motor_AIN2,OUTPUT); 
  pinMode(Motor_BIN1,OUTPUT);   //设置两个驱动引脚为输出模式
  pinMode(Motor_BIN2,OUTPUT); 
  pinMode(Motor_CIN1,OUTPUT);   //设置两个驱动引脚为输出模式
  pinMode(Motor_CIN2,OUTPUT); 
  pinMode(Motor_DIN1,OUTPUT);   //设置两个驱动引脚为输出模式
  pinMode(Motor_DIN2,OUTPUT); 
 
  FlexiTimer2::set(10, control); //5毫秒定时中断函数
  FlexiTimer2::start ();        //中断使能 
  
  attachInterrupt(0, READ_ENCODERA_A, CHANGE);      //开启对应2号引脚的0号外部中断,触发方式为CHANGE 即上升沿和下降沿都触发,触发的中断函数为 READ_ENCODER_A 
  attachInterrupt(1, READ_ENCODERB_A, CHANGE);
  attachInterrupt(5, READ_ENCODERC_A, CHANGE);
  attachInterrupt(4, READ_ENCODERD_A, CHANGE);
  

}

void loop()
{
  sbsaomiao();
 //陀螺仪：
   time = millis();
  // Serial.println(time);
  // Serial2.println(time);


  if(time<startTime*1000)
  {
    speedLeft=0.0;
    speedRight=0.0;
    yawBegin=mpu_getyaw();
    //Serial.println("初始值：");
    //Serial.println(yawBegin);
  }
  else
  {
    
  float currentYaw=mpu_getyaw();
  float deltaYaw=currentYaw-yawBegin;
  //Serial.println("yaw初始值:");
 // Serial.println(yawBegin);
 // Serial.println("currentYaw:");
  //Serial.println(currentYaw);
  speedLeft=myspeed-int(myspeed *(deltaYaw*0.01745)*10);
  speedRight=-myspeed-int(myspeed *(deltaYaw*0.01745)*10);
 // Serial.print(deltaYaw);
 // Serial.print("speedLeft ");
 // Serial.println(speedLeft);
  //Serial.print(" speedRight ");
 // Serial.println(speedRight);
  
stepone();
steptwo();

pingmufason();
stepthree(); 
stepfour();
stepfive();
stepsix();
stepseven();
stepeight();

 }
// go();

}
//原代码
void turn()   
{
   TargetA=10;
 TargetB=10;
 TargetC=10;
 TargetD=10;  
}
void turn1()   
{
   TargetA=-10;
 TargetB=-10;
 TargetC=-10;
 TargetD=-10;  
}

void go()
{
 TargetA=-10;
 TargetB=10;
 TargetC=-10;
 TargetD=10;
}

void go1()
{while (Serial2.available() > 0);
 delay(100);
  char c = Serial2.read();
  if(c=='o'){
    go();
    delay(2000);   
}
else{stop1();}
} 
//由于重力在各个轮子上的大小，以及地面的粗糙度不一样，导致轮子的摩擦阻力不一样，转速就不一样，最终导致偏向。
void go00()
{
 TargetA=speedLeft;
 TargetB=speedLeft;
 TargetC=speedRight;
 TargetD=speedRight;
}



void back()
{
 TargetA=10;
 TargetB=-10;
 TargetC=10;
 TargetD=-10;
}

void goj()
{
 TargetA=TargetA+1;
 TargetB=TargetB-1;
 TargetC=TargetC-1;
 TargetD=TargetD+1;
}


void zuo()
{
 TargetA=10;
 TargetB=10;
 TargetC=-10;
 TargetD=-10;
}

void you()
{
  TargetA=-10;
 TargetB=-10;
 TargetC=10;
TargetD=10;
}

void stop1()
{
 TargetA=0;
 TargetB=-0;
 TargetC=-0;
 TargetD=0;
}
void stop2()
{
 TargetA=0;
 TargetB=-0;
 TargetC=-0;
 TargetD=0;
 Serial2.write('s'); //发送信息
  delay(1000); //等待1秒
}

//三《o》中断
void sanozongduan()
{
static int count = 0;
  char c;
  
  if (Serial2.available() > 0) { // 判断是否有数据可读
    c = Serial2.read();
    if (c == 'o') {
      count++;
      if (count == 3) {
        // 执行下一步操作
        go1();
        count = 0; // 重置计数器
      }
    } else {
      count = 0; // 如果不是字母 'o'，则重置计数器
      stop1();   
       }}}

void qxj()
{
  F1=analogRead(5);
  F2=analogRead(6);
  F3=analogRead(7);
  F4=analogRead(8);
  F5=analogRead(9);
  F6=analogRead(10);
  F7=analogRead(11);
  F8=analogRead(12); 
if (f1==1 and cq==0) 
{
  x=x+1;
   }
  if (f1!=cq)
{ 
  delay(200);
   }
   if(F1<300&&F2<300&&F5<300&&F6<300)
   {
 TargetA=-10;
 TargetB=10;
 TargetC=-10;
 TargetD=10;
    }
    if(F1>300&&F2<300&&F5<300&&F6<300)
   {TargetA=-8;
    TargetB=12;
    TargetC=-8;
    TargetD=12;
    }
   if(F1<300&&F2>300&&F5<300&&F6<300)
   {
   TargetA=-12;
    TargetB=8;
    TargetC=-12;
    TargetD=8;
    }
    if(F1<300&&F2<300&&F5<300&&F6>300)
   {
    TargetA=-12;
    TargetB=8;
    TargetC=-12;
    TargetD=8;
    }
   if(F1>300&&F2<300&&F5<300&&F6>300)
   {
    TargetA=-8;
    TargetB=12;
    TargetC=-12;
    TargetD=8;
    }
     if(F1<300&&F2>300&&F5>300&&F6<300)
   {
    TargetA=-12;
    TargetB=8;
    TargetC=-8;
    TargetD=12;
    }
}

 


/**********外部中断触发计数器函数************
*根据转速的方向不同我们将计数器累计为正值或者负值(计数器累计为正值为负值为计数器方向)
*只有方向累计正确了才可以实现正确的调整,否则会出现逆方向满速旋转
*
*※※※※※※超级重点※※※※※※
*
*所谓累计在正确的方向即
*(1)计数器方向
*(2)电机输出方向(控制电机转速方向的接线是正着接还是反着接)
*(3)PI 控制器 里面的误差(Basi)运算是目标值减当前值(Target-Encoder),还是当前值减目标值(Encoder-Target)
*三个方向只有对应上才会有效果否则你接上就是使劲的朝着一个方向(一般来说是反方向)满速旋转

我例子里是我自己对应好的,如果其他驱动单片机在自己尝试的时候出现满速旋转就是三个方向没对应上

下列函数中由于在A相线上升沿触发时,B相是低电平,和A相线下降沿触发时B是高电平是一个方向,在这种触发方式下,我们将count累计为正,另一种情况将count累计为负
********************************************/
void READ_ENCODERA_A() 
{
    
    if (digitalRead(ENCODERA_A) == HIGH) 
    {     
     if (digitalRead(ENCODERA_B) == LOW)      
       CountA++;  //根据另外一相电平判定方向
     else      
       CountA--;
    }
    
    else 
    {    
     if (digitalRead(ENCODERA_B) == LOW)      
     CountA--; //根据另外一相电平判定方向
     else      
     CountA++;
    }
    
}
void READ_ENCODERB_A() 
{
    
    if (digitalRead(ENCODERB_A) == HIGH) 
    {     
     if (digitalRead(ENCODERB_B) == LOW)      
       CountB++;  //根据另外一相电平判定方向
     else      
       CountB--;
    }
    
    else 
    {    
     if (digitalRead(ENCODERB_B) == LOW)      
     CountB--; //根据另外一相电平判定方向
     else      
     CountB++;
    }
    
}
void READ_ENCODERC_A() 
{
    
    if (digitalRead(ENCODERC_A) == HIGH) 
    {     
     if (digitalRead(ENCODERC_B) == LOW)      
       CountC++;  //根据另外一相电平判定方向
     else      
       CountC--;
    }
    
    else 
    {    
     if (digitalRead(ENCODERC_B) == LOW)      
     CountC--; //根据另外一相电平判定方向
     else      
     CountC++;
    }
    
}
void READ_ENCODERD_A() 
{
    
    if (digitalRead(ENCODERD_A) == HIGH) 
    {     
     if (digitalRead(ENCODERD_B) == LOW)      
       CountD++;  //根据另外一相电平判定方向
     else      
       CountD--;
    }
    
    else 
    {    
     if (digitalRead(ENCODERD_B) == LOW)      
     CountD--; //根据另外一相电平判定方向
     else      
     CountD++;
    }
    
}
/**********定时器中断触发函数*********/

void control() {
    VelocityA=CountA;  
    VelocityB=CountB;
    VelocityC=CountC;
    VelocityD=CountD;//单位时间内读取位置信息
        CountA=0; 
        CountB=0; 
        CountC=0; 
        CountD=0; //并清零
        valueA=IncrementalA_PI_A(VelocityA,TargetA);
        valueB=IncrementalB_PI_B(VelocityB,TargetB);
        valueC=IncrementalC_PI_C(VelocityC,TargetC);
        valueD=IncrementalD_PI_D(VelocityD,TargetD);//通过目标值和当前值在这个函数下算出我们需要调整用的PWM值
        Set_PWMA(valueA);  
        Set_PWMB(valueB);
        Set_PWMC(valueC);
        Set_PWMD(valueD);//将算好的值输出给电机
  
}

void controlA()
{     

        VelocityA=CountA;    //单位时间内读取位置信息
        CountA=0;           //并清零
        valueA=IncrementalA_PI_A(VelocityA,TargetA);  //通过目标值和当前值在这个函数下算出我们需要调整用的PWM值
        Set_PWMB(valueA);    //将算好的值输出给电机
        
}
void controlB()
{     

        VelocityB=CountB;    //单位时间内读取位置信息
        CountB=0;           //并清零
        valueB=IncrementalB_PI_B(VelocityB,TargetB);  //通过目标值和当前值在这个函数下算出我们需要调整用的PWM值
        Set_PWMB(valueB);    //将算好的值输出给电机
        
}
void controlC()
{     

        VelocityC=CountC;    //单位时间内读取位置信息
        CountC=0;           //并清零
        valueC=IncrementalC_PI_C(VelocityC,TargetC);  //通过目标值和当前值在这个函数下算出我们需要调整用的PWM值
        Set_PWMC(valueA);    //将算好的值输出给电机
        
}
void controlD()
{     

        VelocityD=CountD;    //单位时间内读取位置信息
        CountD=0;           //并清零
        valueD=IncrementalD_PI_D(VelocityD,TargetD);  //通过目标值和当前值在这个函数下算出我们需要调整用的PWM值
        Set_PWMD(valueD);    //将算好的值输出给电机
        
}
/***********PI控制器****************/
int IncrementalA_PI_A (int EncoderA,int TargetA)
{  
   static float BiasA,PWMA,Last_biasA;                      //定义全局静态浮点型变量 PWM,Bias(本次偏差),Last_bias(上次偏差)
   BiasA=TargetA-EncoderA;                                  //计算偏差,目标值减去当前值
   PWMA+=VelocityA_KP*(BiasA-Last_biasA)+VelocityA_KI*BiasA;   //增量式PI控制计算
   
   if(PWMA>PWMA_Restrict)
   PWMA=PWMA_Restrict;                                     //限幅
   
   if(PWMA<-PWMA_Restrict)
   PWMA=-PWMA_Restrict;                                    //限幅  
   
   Last_biasA=BiasA;                                       //保存上一次偏差 
   return PWMA;                                           //增量输出
}
int IncrementalB_PI_B (int EncoderB,int TargetB)
{  
   static float BiasB,PWMB,Last_biasB;                      //定义全局静态浮点型变量 PWM,Bias(本次偏差),Last_bias(上次偏差)
   BiasB=TargetB-EncoderB;                                  //计算偏差,目标值减去当前值
   PWMB+=VelocityB_KP*(BiasB-Last_biasB)+VelocityB_KI*BiasB;   //增量式PI控制计算
   
   if(PWMB>PWMB_Restrict)
   PWMB=PWMB_Restrict;                                     //限幅
   
   if(PWMB<-PWMB_Restrict)
   PWMB=-PWMB_Restrict;                                    //限幅  
   
   Last_biasB=BiasB;                                       //保存上一次偏差 
   return PWMB;                                           //增量输出
}
int IncrementalC_PI_C(int EncoderC,int TargetC)
{  
   static float BiasC,PWMC,Last_biasC;                      //定义全局静态浮点型变量 PWM,Bias(本次偏差),Last_bias(上次偏差)
   BiasC=TargetC-EncoderC;                                  //计算偏差,目标值减去当前值
   PWMC+=VelocityC_KP*(BiasC-Last_biasC)+VelocityC_KI*BiasC;   //增量式PI控制计算
   
   if(PWMC>PWMC_Restrict)
   PWMC=PWMC_Restrict;                                     //限幅
   
   if(PWMC<-PWMC_Restrict)
   PWMC=-PWMC_Restrict;                                    //限幅  
   
   Last_biasC=BiasC;                                       //保存上一次偏差 
   return PWMC;                                           //增量输出
}
int IncrementalD_PI_D(int EncoderD,int TargetD)
{  
   static float BiasD,PWMD,Last_biasD;                      //定义全局静态浮点型变量 PWM,Bias(本次偏差),Last_bias(上次偏差)
   BiasD=TargetD-EncoderD;                                  //计算偏差,目标值减去当前值
   PWMD+=VelocityD_KP*(BiasD-Last_biasD)+VelocityD_KI*BiasD;   //增量式PI控制计算
   
   if(PWMD>PWMD_Restrict)
   PWMD=PWMD_Restrict;                                     //限幅
   
   if(PWMD<-PWMD_Restrict)
   PWMD=-PWMD_Restrict;                                    //限幅  
   
   Last_biasD=BiasD;                                       //保存上一次偏差 
   return PWMD;                                           //增量输出
}
/**********实际控制函数*********/
void Set_PWMA(int motora)                        
{ 
  if (motora > 0)                                  //如果算出的PWM为正
  {
    analogWrite(Motor_AIN1, motora+startPWMA);      //让PWM在设定正转方向(我们认为的正转方向)正向输出调整
    analogWrite(Motor_AIN2, 0); 
  }
  else if(motora == 0)                             //如果PWM为0停车
  {
    analogWrite(Motor_AIN2, 0);
    analogWrite(Motor_AIN1, 0);   
  }
  else if (motora < 0)                              //如果算出的PWM为负
  {
    analogWrite(Motor_AIN2, -motora+startPWMA);      //让PWM在设定反转方向反向输出调整
    analogWrite(Motor_AIN1, 0);
  }
}
void Set_PWMB(int motorb)                        
{ 
  if (motorb > 0)                                  //如果算出的PWM为正
  {
    analogWrite(Motor_BIN1, motorb+startPWMB);      //让PWM在设定正转方向(我们认为的正转方向)正向输出调整
    analogWrite(Motor_BIN2, 0); 
  }
  else if(motorb == 0)                             //如果PWM为0停车
  {
    analogWrite(Motor_BIN2, 0);
    analogWrite(Motor_BIN1, 0);   
  }
  else if (motorb < 0)                              //如果算出的PWM为负
  {
    analogWrite(Motor_BIN2, -motorb+startPWMB);      //让PWM在设定反转方向反向输出调整
    analogWrite(Motor_BIN1, 0);
  }
}
void Set_PWMC(int motorc)                        
{ 
  if (motorc > 0)                                  //如果算出的PWM为正
  {
    analogWrite(Motor_CIN1, motorc+startPWMC);      //让PWM在设定正转方向(我们认为的正转方向)正向输出调整
    analogWrite(Motor_CIN2, 0); 
  }
  else if(motorc == 0)                             //如果PWM为0停车
  {
    analogWrite(Motor_CIN2, 0);
    analogWrite(Motor_CIN1, 0);   
  }
  else if (motorc < 0)                              //如果算出的PWM为负
  {
    analogWrite(Motor_CIN2, -motorc+startPWMC);      //让PWM在设定反转方向反向输出调整
    analogWrite(Motor_CIN1, 0);
  }
}

void Set_PWMD(int motord)                        
{ 
  if (motord > 0)                                  //如果算出的PWM为正
  {
    analogWrite(Motor_DIN1, motord+startPWMD);      //让PWM在设定正转方向(我们认为的正转方向)正向输出调整
    analogWrite(Motor_DIN2, 0); 
  }
  else if(motord == 0)                             //如果PWM为0停车
  {
    analogWrite(Motor_DIN2, 0);
    analogWrite(Motor_DIN1, 0);   
  }
  else if (motord < 0)                              //如果算出的PWM为负
  {
    analogWrite(Motor_DIN2, -motord+startPWMD);      //让PWM在设定反转方向反向输出调整
    analogWrite(Motor_DIN1, 0);
  }
}


//mpu6050 函数定义---------------------------------------------
void mpu_init()
{
   time = millis();
//     Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
//  Serial.print(F("MPU6050 status: "));
//  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
//? Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
//  Serial.println("Done!\n");

}

float mpu_getyaw()
{
  mpu.update();
  
  if ((millis() - timer) > 10) {
    // 打印 Z 轴上的角度值

    //Serial3.print(", Z: ");
    //Serial3.println(mpu.getAngleZ());
    timer = millis();  
  }

  // 获取 Z 轴上的角度值并返回
  return mpu.getAngleZ();
}
float mpu_getxaw()
{
  mpu.update();
  
  if ((millis() - timer) > 10) {
    // 打印 X 轴上的角度值
    //Serial3.print("X: ");
    //Serial3.print(mpu.getAngleX());
  
    timer = millis();  
  }

  // 获取 X 轴上的角度值并返回
  return mpu.getAngleX();
}
float mpu_getzaw()
{
  mpu.update();
  
  if ((millis() - timer) > 10) {
    // 打印 Y 轴上的角度值

   // Serial3.print(", Y: ");
    //Serial3.print(mpu.getAngleY());
 
    timer = millis();  
  }

  // 获取 Y 轴上的角度值并返回
  return mpu.getAngleY();
}
