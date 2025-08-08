#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <fcntl.h>
#include <termios.h>
#include<string.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#define SERIAL_PORT0 "/dev/ttyS0"  // Replace with your actual serial port
#define RS485_ENABLE_PIN0 4 

#define PACKET_START_BYTE 0xAA     // Start of packet marker
#define PACKET_END_BYTE 0x55       // End of packet marker

long long getCurrentTimeMillis(){
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000LL + ts.tv_nsec /1000000LL;
}
int configureSerialPort(int serialPort);
void rs485ReceiveData(int serialPort0);
int Send_ACK(int serialPort0, void *mydata);
// Define your data structure
typedef  struct {
    uint8_t STX;
    uint8_t SEQ;
    uint8_t NTX;
    uint8_t realData[7];
    uint8_t ETX;
    uint8_t BCC1;
    uint8_t BCC2;
} dgwDataT;

int configureSerialPort(int serialPort) {
    struct termios options;

    if (tcgetattr(serialPort, &options) != 0) {
        perror("Error getting serial port attributes");
        return -1;
    }
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(serialPort, TCIFLUSH);
    if (tcsetattr(serialPort, TCSANOW, &options) != 0) {
        perror("Error setting serial port attributes");
        return -1;
    }
    return 0;
}
int Send_ACK(int serialPort0, void *mydata){
  //sleep(10);
   dgwDataT *tmp = (dgwDataT *) mydata;
   if (tmp->SEQ==1){
  //mp->ledStatus=1;
   printf("-------------------Sending_ACK ------------------\n");
   //   tmp->rcvData[i]=i+1;
   uint8_t frame[17] = { 0, };
   frame[0] = PACKET_START_BYTE;
   memcpy(&frame[1], tmp, sizeof(dgwDataT));
   // push a ledStatus index
   for(int i = 0; i < sizeof(dgwDataT); i++)
   {
      printf("sent_ACK[%d]=%d\n", i+1, frame[i+1]);
   }

  return  write(serialPort0, &frame, sizeof(frame));
	}
  else{
 	 printf("ACK is not true to send for Master in the next frame\n");
	 return 0;
	} 
}

void rs485ReceiveData(int serialPort0){
   uint8_t received_byte;
   uint8_t received_data[17] = { 0, };
   int byte_counter = 0;
   int in_packet = 0;
   //set timeout to 500 miliseconds
   long long timeout_ms=100;
   // set polling time to 1000 milliseconds
   long long polling_time = 1000;
   //get current time
   long long start_time, end_time;
   // calculate timeout time by adding milliseconds to current time
   while(1){ 
   digitalWrite(RS485_ENABLE_PIN0, HIGH);
   start_time= getCurrentTimeMillis();
   while (getCurrentTimeMillis()-start_time<timeout_ms){
   if (read(serialPort0, &received_byte, 1) > 0) {
         
         if (received_byte == PACKET_START_BYTE && in_packet==0) {
            in_packet = 1;
            byte_counter = 0;
            continue;
         }
         if (in_packet) {
            if(byte_counter <sizeof(dgwDataT))
            {
               received_data[byte_counter] = received_byte;
               byte_counter++;
            }else{
               printf("-----------------------------------------starting to receive data---------------------------------------------------------\n");
               in_packet=0;
               int i;
               for(i = 0; i < byte_counter; i++)
               {
                printf("received_data[%d] = %x \n", i, received_data[i]);

               }
               printf("-----------------------------------------starting to send data---------------------------------------------------------\n");
               delay(10) ;
               digitalWrite(RS485_ENABLE_PIN0, LOW);
               delay(1);
               Send_ACK(serialPort0, &received_data);
               printf("successful sent\n");
               delay(5);
               digitalWrite(RS485_ENABLE_PIN0, HIGH);
            }
        }
  }
 else{
 printf("timeout reading\n");
}
}
#if 1
end_time= getCurrentTimeMillis();
long long elapsed_time= end_time - start_time;
long long remaining_time = polling_time -elapsed_time;
if (remaining_time >0){
	usleep(remaining_time * 1000);
	break;
}
}
#endif
}
int main() {
    if (wiringPiSetupGpio() == -1) {
        perror("Error setting up WiringPi");
        return 1;
    }

    pinMode(RS485_ENABLE_PIN0, OUTPUT);
    int serialPort0 = open(SERIAL_PORT0, O_RDWR | O_NOCTTY);

    if (serialPort0 == -1) {
        perror("Error opening serial port");
        return 1;
    }
    // Configure the serial port
    if (configureSerialPort(serialPort0) == -1) {
        perror("Error configuring serial port");
        close(serialPort0);
        return 1;
    }
    serialFlush(serialPort0);
    printf("-----------------------------------------starting to receive data---------------------------------------------------------\n");
    rs485ReceiveData(serialPort0);
   
    close(serialPort0);
    return 0;
}
