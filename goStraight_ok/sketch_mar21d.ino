/*--------------------------------------*
              步骤一：离开出发区
  --------------------------------------*/
void stepone()
{
    zuo();
    delay(1600);
    control();
    stop1();
    delay(2000);
    
}


/*--------------------------------------*
             步骤二：停留扫描二维码
  --------------------------------------*/
void steptwo()
{
   go();
   delay(2900);
   control();
   delay(1000);
   stop2();
   delay(2000);
}

/*--------------------------------------*
             步骤三：停留抓取放置
  --------------------------------------*/
void stepthree()
   {
    go1();
    delay(5000);
    stop1();
/**************抓取程序部分***************/  
 fasongzhuaqu1();
 delay(1000);
 sanozongduan();
  delay(1000);
   }
  
/*--------------------------------------*
             步骤四：前往放置区
  --------------------------------------*/
void stepfour()
{
     mpu_init();

    go1();
    delay(2000);
     controlA();
    controlB();
     controlC();
    controlD();
    stop1();
    delay(1000);
    turn();
    delay(1850);
      controlA();
    controlB();
     controlC();
    controlD();
    stop1();
      go();
    delay(4700);
    stop1();
    delay(5000);
/********************缺少放置程序*********************************/

   sanozongduan();
  delay(5000); 
}
/*--------------------------------------*
             步骤五：前往二次放置区
  --------------------------------------*/

void stepfive()
{    
   
    
    go();
    delay(3600);
     controlA();
    controlB();
     controlC();
    controlD();
    stop1();
    delay(2000);    
    turn();
    delay(1850);
      controlA();
    controlB();
     controlC();
    controlD();
    stop1();
    delay(2100);  
    go();
    delay(4500);
    stop1();
    delay(5000);
/********************放置程序*********************************/
fasongfangzhi();
delay(30000);
sanozongduan();
  delay(5000);
}
/*--------------------------------------*
             步骤六：返回取料区
  --------------------------------------*/

void stepsix()
{
    turn();//调转方向
    delay(3680);
     controlA();
    controlB();
     controlC();
    controlD();
    stop1();
    delay(1000);
    go();//到达向右转区
    delay(4500);
    stop1();
    delay(5000);
    turn1();//向右转
    delay(1850);
    controlA();
    controlB();
     controlC();
    controlD();
   stop1();
    delay(4500);
    go();//到达向右转区2
    delay(8300);
     controlA();
    controlB();
     controlC();
    controlD();
    stop1();
    delay(5000);
    turn1();//右转
    delay(1850);
     controlA();
    controlB();
     controlC();
    controlD();
    stop1();
    delay(1000);
    go();//到达二次抓取区
    delay(2000);
     controlA();
    controlB();
     controlC();
    controlD();
    stop1();
    delay(5000);
      turn();//调转方向
    delay(3680);
       
}
/*--------------------------------------*
             步骤七：返回二次放置区
  --------------------------------------*/
  void stepseven()
  {
        
     go();//返回二次转向点
    delay(2000);
     controlA();
    controlB();
     controlC();
    controlD();
    stop1();
    delay(5000);
     turn();
    delay(1850);
      controlA();
    controlB();
     controlC();
    controlD();
    stop1();
    go();//到达向左转区
    delay(8300);
     controlA();
    controlB();
     controlC();
    controlD();
    stop1();
    delay(5000);
    turn();//右转
    delay(1850);
     controlA();
    controlB();
     controlC();
    controlD();
    stop1();
    delay(1000);
      go();
    delay(4500);
    controlA();
    controlB();
     controlC();
    controlD();
    stop1();
    delay(1000);
    controlA();
    controlB();
     controlC();
    controlD();
    delay(1000);
 
  }
  
/*--------------------------------------*
             步骤八：返回出发区区
  --------------------------------------*/
  void stepeight()
  {
    go();
    delay(3000);
    controlA();
    controlB();
     controlC();
    controlD();
    stop1();
    delay(1000);
    turn();
    delay(1850);
    controlA();
    controlB();
     controlC();
    controlD();
    stop1();
    delay(1000);
    stop1();
    delay(1000);
    go();
    delay(8300);
    controlA();
    controlB();
     controlC();
    controlD();
    stop1();
    delay(30000);
    
    
  }

  
   
