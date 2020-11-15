#include<ros/ros.h>
#include<iostream>
#include<serial/serial.h>
#include<sensor_msgs/Imu.h>
#include<imu_p/imu_p_pgam.h>

//$PGAM, GGGG.xx, GGGG.yy, GGGG.zz, AA.xxxx, AA.yyyy, AA.zzzz, MXXXXXX,MYYYYYY, MZZZZZZ, PPPPPP, ttttttttt,TTT.t,VV.v,SSSS*CC<CR><LF>
//$PGAM每帧的数据长度最大为114位字符，包括头尾字符


struct FormatPGAM
{
	
    double  gyro_x;//x轴角速度deg/s
    double  gyro_y;//y轴角速度deg/s
    double  gyro_z;//z轴角速度deg/s
    double  acc_x;//x轴加速度g
    double  acc_y;//y轴加速度g
    double  acc_z;//z轴加速度g
	double  Mag_x;//x轴磁力nT
	double 	Mag_y;//y轴磁力nT
	double 	Mag_z;//z轴磁力nT
	double 	Pressure;//气压Pa
	double 	timestampss;//时间ms，从imu启动时开始计时
	double 	Temperature;//温度V
    double 	Vinp;//电压V默认为0
	int		Status;//系统状态
};

//全局变量
serial::Serial ser;//声明串口对象 
int StateParser = 0;//解析处理状态机状态
int CntByte = 0;//用于记录OneFrame中的实际数据长度
int PosDelimiter[15] = {0};//用于记录分隔符位置
int field_len[14];//字符串长度
int CntDelimiter = 0;//分隔符计数
unsigned char rbuf[500];//接收缓冲区，要足够大，需要通过测试得出
char OneFrame[200];//存放一帧数据，长度大于114即可，这里取200
double imutime_pre=0;//上一个imu时间
double delta_t;
char str[3];
unsigned int tmpint = 0;
int cscomputed;//计算得到的校验，除去$*CC<CR><LF>共6个字符
int csreceived;//接收到的校验
char strtemp[3];
char temp_field[30] = {0};
sensor_msgs::Imu imu_output;
imu_p::imu_p_pgam imu_p_output;

ros::Publisher msg_imu_pub;
ros::Publisher msg_imu_p_pub;
/*****************************
  功能：计算校验，字符串中所有字符的异或
  返回：返回一个无符号整数
  输入参数：<1>字符串指针，<2>字符串长度（指有效长度，不包括字符串结束标志）
  输出参数：校验结果
******************************/
unsigned int GetXorChecksum(const char *pch, int len) 
{
    unsigned int cs = 0;
    int i;

    for(i=0; i<len; i++)
        cs ^= pch[i];

    return cs;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_p");
    ros::NodeHandle nh;
	std::string port_name;
	ros::param::get("~port_name", port_name);

	msg_imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_output",1000,true);
	msg_imu_p_pub=nh.advertise<imu_p::imu_p_pgam>("/imu_p_output",1000,true);
	try 
    { 
    	//设置串口属性，并打开串口 
        ser.setPort(port_name); 
        ser.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); //超时定义，单位：ms
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
		std::cout<<port_name+" open failed，please check the permission of port ,run command \"sudo chmod 777 "+port_name+"\" and try again！"<<std::endl;
		getchar(); 
		return -1;
	}

	 //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    {
		std::cout<<port_name+" open successed！"<<std::endl;
    } 
    else 
    { 
		std::cout<<port_name+" open failed！"<<std::endl;
		getchar(); 
		return -1;
	} 

	ros::Rate loop_rate(100);//设置循环频率为100Hz,最大循环频率与波特率和信息格式有关
    ser.flushInput();//在开始正式接收数据前先清除串口的接收缓冲区 	
	memset(OneFrame, 0, sizeof(OneFrame));//清空imu字符串
	int framecnt = 0;
	CntByte = 0;//指向OneFrame的第一个位置
	while (ros::ok())
	{
		int i, j;
		int start;//当前位置
		int pos;//下一个分隔符的位置
		int numinbuf;
		int numgetted;
		
		try
		{
			numinbuf = ser.available();//available()返回从串口缓冲区读回的字符数
			//initrd.img.oldCLEAR();
			//printf("bytes in buf = %d\n",numinbuf);
		}
		catch (serial::IOException& e)
		{
			std::cout<<"Port crashed！ Please check cable!"<<std::endl;
			getchar(); 
			return -1;
		}

		if(numinbuf > 0)//串口缓冲区有数据
		{ 
            numgetted = ser.read(rbuf, numinbuf);//串口缓冲区数据读到rbuf中
			//std::cout<<static_cast<int>(rbuf[3])<<std::endl;
			if(numgetted == numinbuf)//取回的数据个数与缓冲区中有的数据个数相同，说明读串口成功
			{
				
				for(int i=0; i<numgetted; i++)//对收到的字符逐个处理
				{
					//printf("%s\n",rbuf);
					//在一帧数据的接收过程中，只要遇到非$PGAM帧头就重新开始
					//此处理具有最高优先级，会重置状态机
					if(rbuf[i]=='$' &&rbuf[i+1] != 'P'&&rbuf[i+2] != 'G'&&rbuf[i+3] != 'A'&&rbuf[i+4] != 'M')
					{
						memset(OneFrame, 0, sizeof(OneFrame));
						StateParser = 0;

						break;//中断循环
					}

					//正常处理过程
					switch (StateParser)
					{
						//等待语句开始标志'$'
						case 0:
							if(rbuf[i] == '$'&&rbuf[i+1] == 'P'&&rbuf[i+2] == 'G'&&rbuf[i+3] == 'A'&&rbuf[i+4] == 'M')//收到语句开始标志
							{
								//std::cout<<"找到$PGAM"<<std::endl;
								memset(OneFrame, 0, sizeof(OneFrame));
								OneFrame[0] = '$';
								CntByte = 1;//开始对帧长度的计数
								StateParser = 1;
							}
							break;
						
						//寻找帧头"$PGAM,"
						case 1:
							OneFrame[CntByte] = rbuf[i];
							CntByte++;//指向下一个空位
						
							if(rbuf[i]==',')
							{
								if(strncmp(OneFrame, "$PGAM,", 6) == 0)
								{
									CntDelimiter = 0;//分隔符计数从0开始
									PosDelimiter[0] = CntByte - 1;//记录分隔符在OneFrame中的位置
									//std::cout<<"PosDelimiter[0]"<<PosDelimiter[0]<<std::endl;
									StateParser = 2;
									//std::cout<<"寻找帧头$GPCHC完成"<<std::endl;
								}	
								else//帧头错误
								{
									memset(OneFrame, 0, sizeof(OneFrame));
									StateParser = 0;
								}
							}
							break;

						//接收各数据域
						case 2:
							//std::cout<<"开始接受各个数据域"<<std::endl;
							OneFrame[CntByte] = rbuf[i];
							//std::cout<<"接受字符"<<rbuf[i]<<std::endl;
							CntByte++;//指向下一个空位

							if(rbuf[i]==','||rbuf[i]=='*')
							{
								CntDelimiter++;//分隔符计数
								//std::cout<<"分隔符个数："<<CntDelimiter<<std::endl;
								PosDelimiter[CntDelimiter] = CntByte - 1;//记下分隔符位置
								//std::cout<<"PosDelimiter["<<CntDelimiter<<"]"<<PosDelimiter[CntDelimiter]<<std::endl;
								field_len[CntDelimiter-1] = PosDelimiter[CntDelimiter] - PosDelimiter[CntDelimiter-1] - 1;
								//std::cout<<"第"<<CntDelimiter<<"段数据长"<<field_len[CntDelimiter]<<std::endl;
								if(CntDelimiter == 14)//0-14，共15个分隔符，开始数据解析
								{
									// //计算出每个字段的长度
									// for(int j=0; j<14; j++)//0-13，14个字段
									// {
									// 	field_len[j] = PosDelimiter[j+1] - PosDelimiter[j] - 1;
									// 	//std::cout<<"第"<<j<<"段数据长"<<field_len[j]<<std::endl;
									// }
									
									if(field_len[0] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[0]+1], field_len[0]);
										//imu_output.angular_velocity.x = atof(temp_field);
										imu_p_output.wx= atof(temp_field);
										imu_output.angular_velocity.x=imu_p_output.wx;
										
									}

									if(field_len[1] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[1]+1], field_len[1]);
										//imu_output.angular_velocity.y = atof(temp_field);
										imu_p_output.wy= atof(temp_field);
										imu_output.angular_velocity.y =imu_p_output.wy;
									}

									if(field_len[2] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[2]+1], field_len[2]);
										//imu_output.angular_velocity.z = atof(temp_field);
										imu_p_output.wz= atof(temp_field);
										imu_output.angular_velocity.z=imu_p_output.wz;
									}

									if(field_len[3] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[3]+1], field_len[3]);
										//imu_output.linear_acceleration.x = atof(temp_field);
										imu_p_output.ax= atof(temp_field);
										imu_output.linear_acceleration.x=imu_p_output.ax;
									}

									if(field_len[4] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[4]+1], field_len[4]);
										//imu_output.linear_acceleration.y = atof(temp_field);
										imu_p_output.ay= atof(temp_field);
										imu_output.linear_acceleration.y =imu_p_output.ay;
									}
									
									if(field_len[5] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[5]+1], field_len[5]);
										//imu_output.linear_acceleration.z = atof(temp_field);
										imu_p_output.az= atof(temp_field);
										imu_output.linear_acceleration.z =imu_p_output.az;
									}

									if(field_len[6] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[6]+1], field_len[6]);
										imu_p_output.magx= atof(temp_field);
									}

									if(field_len[7] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[7]+1], field_len[7]);
										imu_p_output.magy= atof(temp_field);
										
									}

									if(field_len[8] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[8]+1], field_len[8]);
										imu_p_output.magz= atof(temp_field);
										
									}

									if(field_len[9] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[9]+1], field_len[9]);
										imu_p_output.pressure= atof(temp_field);
										
									}

									if(field_len[10] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[10]+1], field_len[10]);
										imu_p_output.timestampss= atof(temp_field);
										
									}

									if(field_len[11] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[11]+1], field_len[11]);
										imu_p_output.temperature = atof(temp_field);
										
									}

									if(field_len[12] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field)); 
										strncpy(temp_field, &OneFrame[PosDelimiter[12]+1], field_len[12]);
										imu_p_output.vinp= atof(temp_field);
										
									}

									if(field_len[13] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[13]+1], field_len[13]);
										imu_p_output.status= atoi(temp_field);
										
									}

									StateParser = 3;
								}
							}
							break;

							//校验和第一个字符
						case 3:
							OneFrame[CntByte] = rbuf[i];
							CntByte++;//指向下一个空位
							if(rbuf[i-1]=='*'&&((rbuf[i]>='0' && rbuf[i]<='9') || (rbuf[i]>='A' && rbuf[i]<='F')))//校验和字节应是一个十六进制数
							{
								StateParser = 4;
							}
							else
							{
								memset(OneFrame, 0, sizeof(OneFrame));
								StateParser = 0;
							}
							break;

							//校验和第二个字符	
						case 4:
							OneFrame[CntByte] = rbuf[i];
							CntByte++;//指向下一个空位

							if((rbuf[i]>='0' && rbuf[i]<='9') || (rbuf[i]>='A' && rbuf[i]<='F'))//校验和字节应是一个十六进制数
							{
								//检查校验
								cscomputed = GetXorChecksum((char*)(OneFrame+1), CntByte-4);//计算得到的校验，除去$*CC<CR><LF>共6个字符
								csreceived = 0;//接收到的校验
								strtemp[0] = OneFrame[CntByte-2];
								strtemp[1] = OneFrame[CntByte-1];
								strtemp[2] = '\0';//字符串结束标志
								sscanf(strtemp, "%x", &csreceived);//字符串strtemp转换为16进制数
											
								//检查校验是否正确
								if(cscomputed != csreceived)//校验和不匹配
								{
									memset(OneFrame, 0, sizeof(OneFrame));
									StateParser = 0;
								}
								else//校验和匹配
								{		
									StateParser = 5;
								}	
							}//校验和字节是hex
							else
							{
								memset(OneFrame, 0, sizeof(OneFrame));
								StateParser = 0;
							}
							
							break;

							//等待结束标志<CR>=0x0d
						case 5:
							OneFrame[CntByte] = rbuf[i];
							CntByte++;//指向下一个空位
							if(rbuf[i] == '\r')
							{
								StateParser = 6;
							}
							else
							{
								memset(OneFrame, 0, sizeof(OneFrame));
								StateParser = 0;
							}
							break;
						
						//等待结束标志<LF>=0x0a
						case 6:
							OneFrame[CntByte] = rbuf[i];
							if(rbuf[i]=='\n')
							{
								delta_t = imu_p_output.timestampss - imutime_pre;//前后两帧之间的时间差
								imutime_pre = imu_p_output.timestampss;
							}
							
							imu_p_output.header.stamp = ros::Time::now();//ros时刻
							imu_p_output.header.frame_id="imu";
							imu_output.header.stamp = ros::Time::now();
							imu_output.header.frame_id = "imu";
							msg_imu_pub.publish(imu_output);//发布nav消息
							msg_imu_p_pub.publish(imu_p_output);//发布IMU消息
							//std::cout<<"发布成功"<<std::endl;

							memset(OneFrame, 0, sizeof(OneFrame));
							StateParser = 0;
							break;

						default:
							memset(OneFrame, 0, sizeof(OneFrame));
							StateParser = 0;
						 	break;
					}//switch(StateParser)
				}//for(int i=0; i<numgetted; i++)
			}//if(numgetted == numinbuf)
		}
		ros::spinOnce();//执行等待队列中所有回调函数
		loop_rate.sleep();
	}//while
    return 0;
}