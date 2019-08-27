#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <signal.h>
#include <string>
#include <iostream>
#include <poll.h>

union dataUnion
{
	uint8_t dataChar[4];
	float data;
};
 /* definition of signal handler */
void CopyData(uint8_t* origen, uint8_t* afterTreat, int size);
void signal_handler_IO (int status);  
void receiveData(int fd);
class own_serial
{
private:
    /* data */
public:
    own_serial(const char * port = "",
          uint32_t baudrate = B115200);
    ~own_serial();
    int isopen(void);
    int init(int &fd1);
    int writeData(float distanceLeft = 0.0,float distanceRight = 0.0,float angel = 0.0,int status =0);//status为状态 0没有找到 1找到2条边 3找到左边的那条边 4找到右边的那条边
    //int signal_handler_IO(int stuas);
    int this_fd = 0;
private:
    struct termios termAttr;
    struct sigaction saio;
    int ifSerial = 0;
    const char * this_port;
    uint32_t this_baudrate = 9600;  
public:

};




// int main(int argc, char *argv[])
// {    int i;
// pthread_t ntid;
//      fd = open("/dev/ttyACM4", O_RDWR| O_NOCTTY);// | O_NOCTTY | O_NDELAY
//      if (fd == -1)
//      {
//         perror("open_port: Unable to open /dev/ttyO1\n");
//         exit(1);
//      }
 
//      saio.sa_handler = signal_handler_IO;
//      saio.sa_flags = 0;
//      saio.sa_restorer = NULL; 
//      sigaction(SIGIO,&saio,NULL);
 
//      fcntl(fd, F_SETFL, FNDELAY);
//      fcntl(fd, F_SETOWN, getpid());
//      fcntl(fd, F_SETFL, O_NDELAY| O_ASYNC ); /**<<<<<<------This line made it work.**/
 
//      tcgetattr(fd,&termAttr);
//      //baudRate = B115200;          /* Not needed */
//      cfsetispeed(&termAttr,B115200);
//      cfsetospeed(&termAttr,B115200);
//      termAttr.c_cflag &= ~PARENB;
//      termAttr.c_cflag &= ~CSTOPB;
//      termAttr.c_cflag &= ~CSIZE;
//      termAttr.c_cflag |= CS8;
//      termAttr.c_cflag |= (CLOCAL | CREAD);
//      termAttr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
//      termAttr.c_iflag &= ~(IXON | IXOFF | IXANY);
//      termAttr.c_oflag &= ~OPOST;
//      tcsetattr(fd,TCSANOW,&termAttr);
//      printf("serial configured....\n");
     
//      std::pthread_create(&ntid,NULL,sender,NULL);
   
//      while(true){
// 	for (i=0;i<10;i++) sleep(10);
//      }
 
//      close(fd);
//      exit(0);             
// }
 
