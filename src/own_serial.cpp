#include "own_serial.h"
int fd = 0;
uint8_t sbuff[64];
uint8_t rbuff[20];

own_serial::own_serial(const char *port,
          uint32_t baudrat)
{
    std::cout<<"1\n";
    this_port = port;
    this_baudrate = baudrat;
}
own_serial::~own_serial()
{
}

int own_serial::init(int &fd1)
{

    fd1 = open(this_port, O_RDWR| O_NOCTTY| O_NDELAY);
    
    if (fd1 == -1)
    {
    perror("open_port: Unable to open serial\n");
    return 0;
    }
    // saio.sa_handler = signal_handler_IO;
    // //signal_handler_IO(ifSerial);
    // saio.sa_flags = 0;
    // saio.sa_restorer = NULL; 
    // sigaction(SIGIO,&saio,NULL);
    // fcntl(fd1, F_SETFL, FNDELAY);
    // fcntl(fd1, F_SETOWN, getpid());
    // fcntl(fd1, F_SETFL, O_NDELAY| O_ASYNC ); 
    tcgetattr(fd1,&termAttr);
    //baudRate = B115200;          /* Not needed */
    cfsetispeed(&termAttr,this_baudrate);
    cfsetospeed(&termAttr,this_baudrate);
    termAttr.c_cflag &= ~PARENB;
    termAttr.c_cflag &= ~CSTOPB;
    termAttr.c_cflag &= ~CSIZE;
    termAttr.c_cflag |= CS8;
    termAttr.c_cflag |= (CLOCAL | CREAD);
    termAttr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    termAttr.c_iflag &= ~(IXON | IXOFF | IXANY);
    termAttr.c_oflag &= ~OPOST;
    termAttr.c_cc[VMIN] = 0;
    termAttr.c_cc[VTIME] = 1;
    tcflush(fd1,TCIOFLUSH);
    tcsetattr(fd1,TCSANOW,&termAttr);
    fd = fd1;
    std::cout<<"serial configured....\n";
}

int own_serial::isopen(void)
{
    return(ifSerial);
}
int own_serial::writeData(float distanceLeft,float distanceRight,float angel,int status)
{
    uint8_t buffData[17];
    dataUnion leftDisUnion;
    dataUnion rightDisUnion;
    dataUnion angleUnion;

    leftDisUnion.data = distanceLeft;
    rightDisUnion.data = distanceRight;
    angleUnion.data = angel;

    buffData[0] = 'A';
    buffData[1] = 'T';
    
    switch (status)
    {
    case 0:
        buffData[2] = 0;
        break;
    case 1:
        buffData[2] = 1;
        break;
    case 2:
        buffData[2] = 2;
        break;
    case 3:
        buffData[2] = 3;
        break;
    default:
        break;
    }

    CopyData(&leftDisUnion.dataChar[0],&buffData[3],4);
    CopyData(&rightDisUnion.dataChar[0],&buffData[7],4);
    CopyData(&angleUnion.dataChar[0],&buffData[11],4);

    buffData[15] = '\r';
    buffData[16] = '\n';

    int ifWrite =0;
    for (size_t i = 0; i < 17; i++)
    {
        ifWrite = write(fd,&buffData[i],1);
    }
}
int own_serial::writeData(char handMsg)
{
    uint8_t buffData[5];
 
    buffData[0] = 'G';
    buffData[1] = 'S';
    buffData[2] = handMsg;
    buffData[3] = '\r';
    buffData[4] = '\n';
    int ifWrite =0;
    for (size_t i = 0; i < 5; i++)
    {
        ifWrite = write(fd,&buffData[i],1);
    }
    return ifWrite;
}
static int cnt =0;
void signal_handler_IO(int status)
{
    uint8_t data;
    cnt++;
    //static int fd =status;
    int nread = read(fd, &rbuff, 4);
    std::cout <<" nread "<<nread<<std::endl;
    if (nread>0)
    {
        std::cout<<rbuff<<" "<<cnt<<std::endl;
    }
    
  
}
void receiveData(int fd)
{
    struct pollfd fdset[2];
    int ret;
    fdset[0].fd = fd;
    int rc , timeOut = 1;
    fdset[0].events = POLLIN;
    // int cnt;
    tcflush(fd,TCIOFLUSH);
    int status = 0;
    while (true)
    {
        
        rc = poll(&fdset[0],1,timeOut);
        if(rc>0)
        {
            if(fdset[0].revents & POLLIN)
            {
                cnt++;
                
                read(fdset[0].fd, rbuff , 1);
                
                std::cout<<rbuff<<" "<<cnt<<std::endl;
                //tcflush(fd,TCIFLUSH);
            }
        }
        switch (status)
        {
        case 0:
            if(rbuff[0] == 'A')
                status++;
            break;
        case 1:
            if(rbuff[0] == 'T')
                status++;
            else 
            {
                status = 0;
                break;
            }
        case 2:
            std::cout<<"serialok\r\n";
            status = 0;
        default:
            break;
        }
    }
}
void CopyData(uint8_t* origen, uint8_t* afterTreat, int size)
{
    
	for (size_t i = 0; i < size; i++)
	{
		*afterTreat = *origen;
		afterTreat++;
		origen++;
	}
}
