#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_demo/info.h"
#include <iostream>
using namespace std;
ros::Publisher chatter_pub;

int lenth(int advance_data, int len, const std_msgs::String::ConstPtr& msg)
{
	/*数据先验,计算为之字符串长度*/
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

	if((len>230||len<190)||(m==0)){

		cout<<"此数据异常不可取\n******************************************\n\n\n";
	}
   else
   {
        

   }
	std_msgs::String info = *msg;
   
	chatter_pub.publish(info);
}



int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "data_deel");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("read", 1, chatterCallback);
	chatter_pub = n.advertise<std_msgs::String>("/chatter", 1000);		
	ros::spin();

	return 0;
}