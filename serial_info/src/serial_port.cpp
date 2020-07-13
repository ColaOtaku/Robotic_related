#include<ros/ros.h>
#include<serial/serial.h>
#include<vector>
#include<serial_info/SerialInfo.h>
typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef signed short int16_t;
typedef unsigned short uint16_t;
typedef signed int int32_t;
typedef unsigned int uint32_t;

#define PORT "/dev/ttyUSB0"

using namespace std;

int cnt=0;

typedef enum
{
    MODBUS_RS_HEADER = 0,
    MODBUS_RS_TYPE,
    MODBUS_RS_DATA,
    MODBUS_RS_CHECK
} MODBUS_RS;

typedef struct
{
    MODBUS_RS rs;
    unsigned char data[15];

    int count_data;
    int count_header;

    int count_corret;
    int count_error;
}MODBUS_HANDLE, *P_MODBUS_HANDLE;


class Serial_{
    public:
        Serial_(){
           phdl=(P_MODBUS_HANDLE) malloc(sizeof(MODBUS_HANDLE));
           phdl->count_data=phdl->count_header = phdl->count_corret = phdl->count_error=0;
           memset(phdl->data,0,sizeof(phdl->data));
           buffs.clear();
           serial_port_init();
        }
        ~Serial_(){sp.close();}
        serial::Serial sp; 
        bool serial_offline = false;
        std::vector<uint8_t> buffs;

        P_MODBUS_HANDLE phdl;

        serial_info::SerialInfo  serial_port_transfer(void);
        int serial_port_analysis(unsigned char data);
        int serial_port_init(void);
        int serial_port_check(void);
        void serial_port_write(string s);
};

int Serial_::serial_port_init(void)
{
    try 
    { 
        sp.setPort(PORT); 
        sp.setBaudrate(1000000); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        sp.setTimeout(to);
        sp.open(); 
    } 
    catch(serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port.");  
        return -1; 
    } 

    if(sp.isOpen())
    {
        sp.flushInput();
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
        return -1;

    return 1;
}

int Serial_::serial_port_check()
{
    if(!serial_offline)
    {
        try 
        { 
            sp.available();
        } 
        catch(serial::IOException& e) 
        { 
            serial_offline = true;
            sp.close();
            sp.setPort(PORT); 
            sp.setBaudrate(115200); 
            serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
            sp.setTimeout(to);
            ROS_ERROR_STREAM("serial lost!");  
        }
    }
    if(serial_offline)
    {
        try 
        { 
            //打开串口 
            sp.open(); 
        } 
        catch(serial::IOException& e) 
        { 
            ROS_ERROR_STREAM("Unable to reopen port. Ignore cmd this time.");  
            return -1; 
        } 
        serial_offline = false;
    }
    return 1;
}

int Serial_::serial_port_analysis(unsigned char data){
    uint8_t sum = 0;
    bool ends= false;
    if(data == 0xff)
        phdl->count_header++;
    else
        phdl->count_header = 0;
    
    if(phdl->count_header == 4)
    {
        phdl->count_header = 0;
        phdl->rs = MODBUS_RS_TYPE;
        return ends;
    }
    
    switch (phdl->rs)
    {
        case MODBUS_RS_HEADER:
            break;

        case MODBUS_RS_TYPE:
            if(data != 0x01)
                phdl->rs = MODBUS_RS_HEADER;
            else
                phdl->rs = MODBUS_RS_DATA;
            break;

        case MODBUS_RS_DATA:
            phdl->data[phdl->count_data] = data;
            phdl->count_data++;
            if(phdl->count_data >= 15)
                phdl->rs = MODBUS_RS_CHECK;
            break;

        case MODBUS_RS_CHECK:

            for(int i = 0; i < 15; i++)
            {
                sum ^= phdl->data[i];
         
                // printf("%d %d %d\n",i+1,sum,phdl->data[i]);
            }
            // printf("16 %d %d\n",sum,data);
            if(sum == data)
            {
                cout<<"CheckSUm Right"<<endl;
                phdl->count_corret++;
                phdl->rs = MODBUS_RS_HEADER;
                ends=true;
            }
            else
            {
                ROS_INFO("error\r\n");
                phdl->count_error++;
                 ends=false;
            }
            break;

        default:
            phdl->rs = MODBUS_RS_HEADER;
            break;
    }
    return ends;
}

serial_info::SerialInfo Serial_::serial_port_transfer(void){
    serial_port_check();
    cout<<"------Cnt:"<<cnt++<<"-------"<<endl;
    bool ends = false;
  
    while(1)
    {
        if(!sp.available()) continue;

        buffs.clear();
        sp.read(buffs,sp.available());
        vector<uint8_t>::iterator it = buffs.begin();
        while( it != buffs.end() && (!ends)) 
        {
            ends=serial_port_analysis(*it);
            it++;
        }

           phdl->count_data=phdl->count_header = phdl->count_corret = phdl->count_error=0;
         if(ends) break;
    }

    cout<<"Push into Msg"<<endl;
   
    serial_info::SerialInfo ret;
    ret.Times=(uint32_t)(phdl->data[0]);
    ret.Times+=(uint32_t)(phdl->data[1])<<8;
    ret.Times+=(uint32_t)(phdl->data[2])<<16;
    ret.Times+=(uint32_t)(phdl->data[3])<<24;
    
    ret.Time = (uint16_t)phdl->data[5]<<8|phdl->data[4];
    ret.Period = (uint8_t)phdl->data[6];
    ret.Motor_1= (uint8_t)phdl->data[8] <<8 |phdl->data[7];
    ret.Motor_2= (uint16_t)phdl->data[10]<<8 |phdl->data[9];
    ret.Motor_3= (uint16_t)phdl->data[12]<<8 |phdl->data[11];
    ret.Motor_4= (uint16_t)phdl->data[14]<<8 | phdl->data[13];

    return ret;
}

void Serial_::serial_port_write(string s){
    sp.write(s);
}

//=====================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial");
    ros::NodeHandle n;
    ros::Publisher serial_pub = n.advertise<serial_info::SerialInfo>("/serial_info", 100);

    serial_info::SerialInfo msg;
    Serial_ ser;
    ros::Rate loop_rate(50); 
    while(1){
        serial_pub.publish(ser.serial_port_transfer());
    }


   return 0;
}
