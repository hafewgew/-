
char data[7];

String qr_data = "";

String qr_data_converted = "";


//发送的是抓取转台上顺序'r''g''b'对应红绿蓝
void fasongzhuaqu1()
{
String QR =qr_data_converted;  
 String first_three = QR.substring(0, 3);
Serial2.write(first_three.c_str()); // 输出 "rgb"
}

//发送的是二次抓取台的抓取顺序
void fasongzhuaqu2()
{
    //读取后三个
String QR = qr_data_converted;    
String last_four = QR.substring(QR.length() - 3);
Serial2.write(last_four.c_str()); // 输出 ""

}

//发送放置粗加工区
void fasongfangzhi()
{   
Serial2.write("uvw", 3);  // 发送三个字节
}


//发送从粗加工区抓取
void fasongzhuaqu3()
{
Serial2.write("ijk", 3);  // 发送三个字节
}

void fasongfangzhi1()
{
Serial2.write("cxy", 3);  // 发送三个字节
}

//二维码扫描加颜色识别加字符转换
void sbsaomiao(){
  if (Serial.available() > 0) { // 判断是否有数据可读
     Serial2.write('z');
    char incomingData = Serial.read();
    Serial.print(incomingData);   // 输出接收到的数据到串口
    if (incomingData == 'S') {    // 如果接收到的是起始字符
       qr_data = "";        // 创建空字符串
       int counter =0;
       int MAX_DATA_LENGTH =8;
      while (Serial.available() > 0 && counter < MAX_DATA_LENGTH) {   // 不断接收数据，直到接收到结束字符
        incomingData = Serial.read();
        if (incomingData == 'E') {
          Serial.println();    
          Serial.print("QR code: ");
/*********（这里就是接收二维码数据的函数，屏幕就引用这个）**********/
          Serial.println(qr_data);    
          break;  // 退出循环
        }
        
         
        else {
          qr_data += incomingData;  // 将接收到的数据添加到字符串末尾
          counter++;
          }
           }
             // 如果读取的字符数超过了最大长度，直接退出
         if (counter >= MAX_DATA_LENGTH) {
        Serial.println("Data length exceeded maximum, exiting.");
        return;
         }

        qr_data_converted = ""; // 创建空字符串，用于存储转换后的二维码数据
      for (int i = 0; i < qr_data.length(); i++) { // 遍历二维码数据的每一个字符
        char ch = qr_data.charAt(i);
        if (ch == '1') {      // 如果是红色
          qr_data_converted += "r"; // 转换为数字1
        } else if (ch == '2') { // 如果是绿色
          qr_data_converted += "g"; // 转换为数字2
        } else if (ch == '3') { // 如果是蓝色
          qr_data_converted += "b"; // 转换为数字3
        } else {              // 如果是其他字符
          qr_data_converted += ch;  // 直接添加到转换后的字符串中
        }
      }      
         Serial.print("QR code converted: ");
/**************输出转换后的二维码数据 (引用这个函数)**************/
         Serial.println(qr_data_converted); 
}}
delay(1000);
}


//屏幕识别转换区
void pingmufason() {
  // 读取需要发送的字符串

  String s = qr_data;

  // 将字符串转换成 TJP 指令
  byte buf[128];
  int len;
  string_to_tjp(s, buf, &len);

  // 发送 TJP 指令
  TJC.write(buf,len);

  // 延时一段时间
  delay(1000);
}

// 将字符串转换成 TJP 指令
void string_to_tjp(String s, byte* buf, int* len) {
  // 开始标志
  buf[0] = 0x7E;
  // 命令字
  buf[1] = 0x01;
  // 数据长度
  buf[2] = 0x00;
  buf[3] = s.length() + 11;
  // 指令字
  buf[4] = 0x01;
  // 操作类型
  buf[5] = 0x0C;
  // 页面编号
  buf[6] = 0x00;
  // 控件编号
  buf[7] = 0x03;
  // 控件类型
  buf[8] = 0x06;
  // 控件参数1
  buf[9] = 0x00;
  buf[10] = s.length();
  // 控件参数2
  buf[11] = 0x00;
  buf[12] = 0x00;
  // 控件参数3
  buf[13] = 0x00;
  buf[14] = 0x00;
  // 字符串数据
  for (int i = 0; i < s.length(); i++) {
    buf[i + 15] = s.charAt(i);
  }
  // 结束标志
  buf[s.length() + 15] = 0x7E;
  // 数据长度
  buf[2] = (s.length() + 14) >> 8;
  buf[3] = (s.length() + 14) & 0xFF;
  // 设置数据长度
  *len = s.length() + 14;
}

