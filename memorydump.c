#define FORLINUX

#include "modbus.h"
#include "modbus_attack.h"

int main(int argc, char *argv[]){

  unsigned char addr, fc, data[256], crc[2];
  int datalen;
  char inport[100], outport[100];
  int i, j, len, err;
  int port;
  struct modbus_pdu *pdu;
  int cnt1 = 0x0, cnt2 = 0x0, cnt;
  
  strcpy(inport, "/dev/ttyUSB0");
  strcpy(outport, "/dev/ttyUSB2");
  
  for(i=1; i<argc; i++){
    if(strcmp(argv[i], "-inport") == 0){
      len = strlen(argv[i+1]);
      if(len > 100){
        printf("Error: Port name must be less than 100 characters long.\n");
        return(-1);
      }
      strncpy(inport, argv[i+1], len);
      i++;
      printf("INFO: Input port = %s\n", inport);
    } else if(strcmp(argv[i], "-outport") == 0){
      len = strlen(argv[i+1]);
      if(len > 100){
        printf("Error: Port name must be less than 100 characters long.\n");
        return(-1);
      }
      strncpy(outport, argv[i+1],len);
      i++;      
    }else{
      printf("ERROR: Illegal arguments (%s)\n", argv[i]);
      printf("Usage: %s -inport /dev/ttyXX -outport /dev/ttyYY\n", argv[0]);
      return (-1);
    }
  }
   
  port = openport(outport);
  if(port < 0)
    return (port);
  printf("open %s ok port = %d\n", outport, port);
  
  char *str;
  unsigned char lrc, top, bottom;
  addr = (unsigned char) 0x7; 
  while(1){
  //fc =0x01 read coil,
  for(cnt1=0x0; cnt1<0xff; cnt1++){
  for(cnt2=0x0; cnt2<0xff; cnt2++){
  fc = (unsigned char) 0x01;
  datalen = 4;
  data[0] = cnt1;
  data[1] = cnt2;
  data[2] = 0x00;
  data[3] = 0x01;

  str = malloc(datalen+2);
  str[0] = addr;
  str[1] = fc;
  memcpy(str+2, data, datalen);
  lrc = LRC(str, datalen+2);     
  free(str);
  byte2ascii(lrc, &top, &bottom);
  crc[0] = top;
  crc[1] = bottom;
  
  pdu = mkpdu(addr, fc, data, datalen, crc);
  sendmodbus_ascii(port, pdu);
  freepdu(pdu);
  printf("number is %d\n", cnt1++);
  sleep(4);
  }
  } 
  //fc = 0x02 Read Discrete Inputs, not implement excode = 01
  fc = (unsigned char)0x02;
  for(cnt1=0x0; cnt1<0xff; cnt1++){
  for(cnt2=0x0; cnt2<0xff; cnt2++){
  datalen = 4;
  data[0] = cnt1;
  data[1] = cnt2;
  data[2] = 0x00;
  data[3] = 0x01;
  
  str = malloc(datalen+2);
  str[0] = addr;
  str[1] = fc;
  memcpy(str+2, data, datalen);
  lrc = LRC(str, datalen+2);
  free(str);
  byte2ascii(lrc, &top, &bottom);
  crc[0] = top;
  crc[1] = bottom;
  
  pdu = mkpdu(addr, fc, data, datalen, crc);
  sendmodbus_ascii(port, pdu);
  freepdu(pdu);
  printf("number is %d\n", cnt++);
  sleep(4);
  }
  }

  //fc = 0x03 Read Holding Registers, not implement excode = 01
  fc = 0x03;
  for(cnt1=0x0; cnt1<0xff; cnt1++){
  for(cnt2=0x0; cnt2<0xff; cnt2++){
  datalen = 4;
  data[0] = cnt1;
  data[1] = cnt2;
  data[2] = 0x00;
  data[3] = 0x01;

  str = malloc(datalen+2);
  str[0] = addr;
  str[1] = fc;
  memcpy(str+2, data, datalen);
  lrc = LRC(str, datalen+2);
  free(str);
  byte2ascii(lrc, &top, &bottom);
  crc[0] = top;
  crc[1] = bottom;
  
  pdu = mkpdu(addr, fc, data, datalen, crc);
  sendmodbus_ascii(port, pdu);
  freepdu(pdu);
  printf("numb is %d\n", cnt++);
  sleep(4);
  }
  }

  //fc = 0x04 Read Input Registers, not implement excode = 01
  fc = 0x04;
  for(cnt1=0x0; cnt1<0xff; cnt1++){
  for(cnt2=0x0; cnt2<0xff; cnt2++){
  datalen = 4;
  data[0] = cnt1;
  data[1] = cnt2;
  data[2] = 0x00;
  data[3] = 0x01;
  
  str = malloc(datalen+2);
  str[0] = addr;
  str[1] = fc;
  memcpy(str+2, data, datalen);
  lrc = LRC(str, datalen+2);
  free(str);
  byte2ascii(lrc, &top, &bottom);
  crc[0] = top;
  crc[1] = bottom;

  pdu = mkpdu(addr, fc, data, datalen, crc);
  sendmodbus_ascii(port, pdu);

  freepdu(pdu);
  printf("number is %d\n", cnt++);
  sleep(4);
  }
  }
  //fc = 0x18, read FIFO
  fc = 0x18;
  for(cnt1=0x0; cnt1<0xff; cnt1++){
  for(cnt2=0x0; cnt2<0xff; cnt2++){
   datalen = 4;
  data[0] = cnt1;
  data[1] = cnt2;
  data[2] = 0x00;
  data[3] = 0x01;

  str = malloc(datalen+2);
  str[0] = addr;
  str[1] = fc;
  memcpy(str+2, data, datalen);
  lrc = LRC(str, datalen+2);
  free(str);
  byte2ascii(lrc, &top, &bottom);
  crc[0] = top;
  crc[1] = bottom;

  pdu = mkpdu(addr, fc, data, datalen, crc);
  sendmodbus_ascii(port, pdu);
  freepdu(pdu);
  printf("number is %d\n", cnt++);
  sleep(4);
}
  }
  } 
}
