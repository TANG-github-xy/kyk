#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_demo/info.h"
#include <string>
#include <iostream>
using namespace std;
ros::Publisher chatter_pub;
ros::Publisher GPS_IMU_state_pub;

int lenth(int advance_data, int len, const std_msgs::String::ConstPtr& msg)
{
	/*数据先验,计算字符串长度*/
	if(msg->data[0]=='$')
		{
			 for(int i=0; i<advance_data; i++)
    			{
					len++;
					if(msg->data[i]=='*') break;

				}
         
		}
		
		return len+2;
}


int GPS_Check(char c1,char c2, int dex)
{
	int int1, int2, mask;
        if(c1 >='A' && c1<='F'){
            int1 = 10+(c1-'A');
        }else{
            int1 = c1 - '0';
        }
        if(c2 >='A' && c2<='F'){
            int2 = 10+(c2-'A');
        }else{
            int2 = c2 - '0';
        }
        mask = int1*16 + int2; //16进制转10进制
		//cout<<"十六进制："<<mask<<"****"<<dex;;
		if(mask==dex)//如果两个字符串相等则为0
    {
        return 1;
    }
    else
    {
        return 0;
        
    }

}

int GPS_Data_Check(uint8_t a[],int length)//数据校验
{
    int head_n=5;
    string head_str; //帧头
    string CS_str; //数据帧校验位
    int flag=1; //数据标志位
    int CS_dex; 
    unsigned char CS=0;
    int chang=0;
    for(int i=0;i<length-1;i++)
    {
        if(i<head_n)
        {
            head_str+=a[i];
        }
        if(flag==3)
        {
            CS_str+=a[i];//在一帧数据中本来存在的校验位
        }
        if(a[i]=='*')
        {
            flag=3;
        }
        if(flag==2)
        {
            CS^=a[i];//异或出来的校验位
            chang++;
        }
        if(a[i]=='$')
        {
            flag=2;
        }
    }
    CS_dex=CS;  //十进制ASCLL码
	//cout<<"校验位："<<CS_str[0]<<CS_str[1]<<"十进制"<<CS_dex<<"\n";
     if((head_str=="$KSXT")&&(GPS_Check(CS_str[0],CS_str[1],CS_dex)))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

struct GPS_IMU
{
        string Header;
        string GPSTIME;
        string Longitude;
        string Lattitude;
        string Altitude;
        string Yaw;
        string Pitch;
        string V_direction_Angle;
        string speed;
        string Roll;
        string Position_state_1;
        string Position_state_2;
        string f_satellite;
        string b_satellite;
        string E_Distance;
        string N_Distance;
        string D_Distance;
        string ED_Speed;
        string ND_Speed;
        string DD_Speed;

}data;


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
    
    int len=lenth(1000,1, msg);

	 uint8_t a[len];
    for(int i=0; i<len; i++)
	{
		a[i] = msg->data[i];
	}
    
    int m =GPS_Data_Check(a,len);
    //cout<<"较验值情况为："<<m<<"\n";
    struct GPS_IMU D;
	if((len>230||len<190)||(m==0)){

		cout<<"此数据异常不可取\n******************************************\n\n\n";
	}
   else
   {
       int num=0;
       for(int i=0; i<len; i++)
            {
                if(a[i]==',')
                {
                    num++;
                    
                }
                else 
                {
                    
                    if(num==0)
                    {

                         D.Header += a[i];
                    }
                    else if(num==1)
                    {
                        D.GPSTIME += a[i];
                    }
                    else if(num==2)
                    {
                        D.Longitude += a[i];
                    }
                    else if(num==3)
                    {
                        D.Lattitude += a[i];
                    }
                    else if(num==5)
                    {
                        D.Yaw += a[i];
                    }
                    else if(num==6)
                    {
                        D.Pitch += a[i];
                    }
                    else if(num==7)
                    {
                        D.V_direction_Angle += a[i];
                    }
                    else if(num==8)
                    {
                        D.speed += a[i];
                    }
                    else if(num==9)
                    {
                        D.Roll += a[i];
                    }
                    else if(num==10)
                    {
                        D.Position_state_1 += a[i];
                    }
                    else if(num==11)
                    {
                        D.Position_state_2 += a[i];
                    }
                    else if(num==12)
                    {
                        D.f_satellite += a[i];
                    }
                    else if(num==13)
                    {
                        D.b_satellite += a[i];
                    }
                    else if(num==14)
                    {
                        D.E_Distance += a[i];
                    }
                    else if(num==15)
                    {
                        D.N_Distance += a[i];
                    }
                    else if(num==16)
                    {
                        D.D_Distance += a[i];
                    }
                    else if(num==17)
                    {
                        D.ED_Speed += a[i];
                    }
                    else if(num==18)
                    {
                        D.ND_Speed += a[i];
                    }
                    else if(num==19)
                    {
                        D.DD_Speed += a[i];
                    }

                }
           }
    }
     
     //cout<<"\n head:"<<D.Header<<"******"<<D.DD_Speed;

     ros_demo::info data_state;
     data_state.head = D.Header;
	 data_state.state_1 = D.Position_state_1;
     data_state.state_2 = D.Position_state_2;
     data_state.f_satellite = D.f_satellite;
     data_state.b_satellite = D.b_satellite;
     GPS_IMU_state_pub.publish(data_state);
	 
   

}

int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "data_deel");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("read", 1, chatterCallback);
	chatter_pub = n.advertise<std_msgs::String>("/chatter", 1000);
    GPS_IMU_state_pub = n.advertise<ros_demo::info>("/GPS_IMU_State", 1000);		
	ros::spin();

	return 0;
}