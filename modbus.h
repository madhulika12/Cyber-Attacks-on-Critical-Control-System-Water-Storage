

#ifdef FORLINUX
#include <stdlib.h>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <pthread.h>
#include <sys/time.h>
#include <time.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#endif
#ifdef FORMICROBLAZE
#include <stdlib.h>
#include "xparameters.h"

// GNU C headers
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

// Xilinx provided headers (driver code)
#include "xuartns550_l.h"
#include "xuartns550.h"
#include "xbasic_types.h"
#include "xgpio.h"
#include "lcd.h"
#include "uart.h"
#include "xsysace_l.h"
#include "xsysace.h"
#include "xutil.h"
#include "xtmrctr_l.h"
#include "sysace_stdio.h"
#include "memcard.h"
#define printf xil_printf
#ifdef __MICROBLAZE__
#include "mb_interface.h"
#else
#include "xexception_l.h"
#endif
#endif

#define ASCII_COLON    0x3A    // Hex for ASCII ":" 
#define ASCII_CR       0x0D    // Hex for ASCII CR
#define ASCII_LF       0x0A    // Hex for ASCII LF
#define TTYS0 "/dev/ttyS0"
#define TTYS1 "/dev/ttyS1"

#define RTU_TIMEOUT 10


#ifdef FORLINUX
#define BAUDRATE B9600
#endif

#ifndef TIMEOUT_ENABLE
#define TIMEOUT_ENABLE 0
#endif

#ifndef TIMEOUT_VAL
#define TIMEOUT_VAL 5
#endif

struct modbustcp_mbap {
  unsigned short transid;
  unsigned short protid;
  unsigned short length;
  unsigned char unitid;
};

struct modbustcp_adu {
  struct modbustcp_mbap header;
  unsigned char fc;
  unsigned char *data;
  int datalen;
};

struct modbus_pdu {
   unsigned char addr;
   unsigned char fc;
   unsigned char *data;
   unsigned char crc[2];
   int datalen;
};
struct modbus_record {
  struct modbus_pdu *pdu;
#ifdef FORLINUX
  struct timeval endtime;
#endif
#ifdef FORMICROBLAZE
  int endtime;
#endif
};

void printbytes(void *buf, int blen);
void portflush(int port);
static unsigned char LRC(unsigned char *auchMsg, unsigned short usDataLen);
void calc_crc(struct modbus_pdu *pdu);
int pdu2ascii(struct modbus_pdu *pdu, char *str);
struct modbus_pdu *mkpdu(unsigned char addr, unsigned char fc, unsigned char *data, int datalen, unsigned char crc[2]);
int getbyte(int port, char *byte);
int sendbyte(int port, char byte);
unsigned char char2hex(char letter);
char hex2char(unsigned char byte);
void byte2ascii(unsigned char byte, char *ascii1, char *ascii2);
void tobinary(unsigned char *byte, char ascii1, char ascii2); 
struct modbus_pdu *getmodbus_ascii(int port);
struct modbus_record *getmodbus_ascii_timed(int port);
void sendmodbus_ascii(int port, struct modbus_pdu *pdu);
void sendmodbus_ascii_timed(int port, struct modbus_record *rec);
void printmodbuspdu (struct modbus_pdu *pdu);
void printmodbusrec (struct modbus_record *rec);
int openport(char *portstr);
struct modbus_pdu *ascii2pdu(char *str, int len);
void sendmodbus_RTU(int port, struct modbus_pdu *pdu);
struct modbus_pdu *getmodbus_RTU(int port);

unsigned short unpack_pdu(struct modbus_pdu* pdu, unsigned char** pdu_string);

#ifdef FORMICROBLAZE
unsigned int posl0 = 0;
unsigned int posl1 = 0;
void lcd_print(unsigned int line, unsigned int *pos, const char *instr, unsigned int len);
XUartNs550 UartNs550;
XStatus UartNs550Init( XUartNs550 *UartInstancePtr, Xuint16 UartDeviceId);
XUartNs550 RS232_Uart_2_UartNs550;
XUartNs550 RS232_Uart_1_UartNs550;
#endif

/* Table of CRC values for highorder byte */ 
static unsigned char auchCRCHi[] = { 
0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,
0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,
0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,
0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,
0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,
0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,
0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,
0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
0x40
} ; 

static char auchCRCLo[] = { 
0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2,0xC6,0x06,0x07,0xC7,0x05,0xC5,0xC4,
0x04,0xCC,0x0C,0x0D,0xCD,0x0F,0xCF,0xCE,0x0E,0x0A,0xCA,0xCB,0x0B,0xC9,0x09,
0x08,0xC8,0xD8,0x18,0x19,0xD9,0x1B,0xDB,0xDA,0x1A,0x1E,0xDE,0xDF,0x1F,0xDD,
0x1D,0x1C,0xDC,0x14,0xD4,0xD5,0x15,0xD7,0x17,0x16,0xD6,0xD2,0x12,0x13,0xD3,
0x11,0xD1,0xD0,0x10,0xF0,0x30,0x31,0xF1,0x33,0xF3,0xF2,0x32,0x36,0xF6,0xF7,
0x37,0xF5,0x35,0x34,0xF4,0x3C,0xFC,0xFD,0x3D,0xFF,0x3F,0x3E,0xFE,0xFA,0x3A,
0x3B,0xFB,0x39,0xF9,0xF8,0x38,0x28,0xE8,0xE9,0x29,0xEB,0x2B,0x2A,0xEA,0xEE,
0x2E,0x2F,0xEF,0x2D,0xED,0xEC,0x2C,0xE4,0x24,0x25,0xE5,0x27,0xE7,0xE6,0x26,
0x22,0xE2,0xE3,0x23,0xE1,0x21,0x20,0xE0,0xA0,0x60,0x61,0xA1,0x63,0xA3,0xA2,
0x62,0x66,0xA6,0xA7,0x67,0xA5,0x65,0x64,0xA4,0x6C,0xAC,0xAD,0x6D,0xAF,0x6F,
0x6E,0xAE,0xAA,0x6A,0x6B,0xAB,0x69,0xA9,0xA8,0x68,0x78,0xB8,0xB9,0x79,0xBB,
0x7B,0x7A,0xBA,0xBE,0x7E,0x7F,0xBF,0x7D,0xBD,0xBC,0x7C,0xB4,0x74,0x75,0xB5,
0x77,0xB7,0xB6,0x76,0x72,0xB2,0xB3,0x73,0xB1,0x71,0x70,0xB0,0x50,0x90,0x91,
0x51,0x93,0x53,0x52,0x92,0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,0x9C,0x5C,
0x5D,0x9D,0x5F,0x9F,0x9E,0x5E,0x5A,0x9A,0x9B,0x5B,0x99,0x59,0x58,0x98,0x88,
0x48,0x49,0x89,0x4B,0x8B,0x8A,0x4A,0x4E,0x8E,0x8F,0x4F,0x8D,0x4D,0x4C,0x8C,
0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,0x43,0x83,0x41,0x81,0x80,
0x40
}; 

char pbuf[1000];
void ldpbuf(char *str) {
  strncat(pbuf, str, strlen(str));
   //lcd_print(0, &posl0, pbuf, 1);
}

void prnpbuf() {
  //printf("%s", pbuf);
  strcpy(pbuf, "");
}

#ifdef FORLINUX
void printbytes(void *buf, int blen) {
   int i;
   unsigned char *mem;

   for (i=0;i<blen;i+=sizeof(unsigned char)) {
      mem = buf+i;
      if (i % 16 == 0) {
        printf("%p(%3d): %.2x",mem,i,*mem);
     } else if (i % 16 == 15) {
        printf("%.2x\n",*mem);
      } else {
        printf("%.2x",*mem);
      }
   } //end for
   printf("\n");
} //end printbytes
#endif
#ifdef FORMICROBLAZE
void printbytes(void *buf, int blen) {
   int i;
   unsigned char *mem;

   for (i=0;i<blen;i+=sizeof(unsigned char)) {
      mem = buf+i;
      if (i % 16 == 0) {
        xil_printf("%.2x",*mem);
     } else if (i % 16 == 15) {
        xil_printf("%.2x\n",*mem);
      } else {
        xil_printf("%.2x",*mem);
      }
   } //end for
   xil_printf("\n");
} //end printbytes
#endif

void portflush(int port) {
  int err;
#ifdef FORLINUX
  err = tcflush(port, TCIOFLUSH);
  if (err < 0) {
    printf("Flush unsuccessful\n");
    perror;
  }
#endif

}

// LRC routine taken from
// MODBUS over Serial Line, Specification and Implementation Guide, V1.02
// Appendix B
//http://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
static unsigned char LRC(unsigned char *auchMsg, unsigned short usDataLen)  /* the function returns the LRC as a type unsigned char */
//unsigned char *auchMsg ;                      /* message to calculate LRC upon */
//unsigned short usDataLen ;                    /* quantity of bytes in message */
{
  unsigned char uchLRC = 0 ;                  /* LRC char initialized */
  while (usDataLen--)                         /* pass through message buffer */
  uchLRC += *auchMsg++ ;                      /* add buffer byte without carry */
  return ((unsigned char)(-((char)uchLRC))) ; /* return twos complement */
}


void calc_crc(struct modbus_pdu *pdu) {

  unsigned char *puchMsg ;  /* message to calculate CRC upon */ 
  unsigned short usDataLen ; /* quantity of bytes in message  */ 
  int g = 0;


  unsigned char uchCRCHi = 0xFF ; /* high byte of CRC initialized  */ 
  unsigned char uchCRCLo = 0xFF ; /* low byte of CRC initialized  */ 
  unsigned uIndex ; /* will index into CRC lookup table  */ 
 
  usDataLen= unpack_pdu(pdu, &puchMsg);

  while (usDataLen--) /* pass through message buffer  */ 
  { 
    g++ ;   /* calculate the CRC   */ 
    uIndex = uchCRCLo ^ *puchMsg++;
    uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex] ; 
    uchCRCHi = auchCRCLo[uIndex] ; 
  } 
	
  pdu->crc[0] = uchCRCLo; //Lowbyte is sent first
  pdu->crc[1] = uchCRCHi; 
}

 
//This function is req'd for the spec implementations of the LRC & CRC
//pdu is a ptr to the modbus pkt to be converted into a single bytearray
//pdu_string is a double pointer to the byte array that's generated
//pdu_length is the length of that byte array
unsigned short unpack_pdu(struct modbus_pdu* pdu, unsigned char** pdu_string)
{
	     int i=0;
        unsigned short pdu_length;

	//The length of the pdu string  the size of the data (datalen) + 2 (1 for addr, 1 for fc)
	pdu_length= pdu->datalen + 2;

	*pdu_string=malloc(pdu_length);
	(*pdu_string)[0]=pdu->addr;
	(*pdu_string)[1]=pdu->fc;
	memcpy((*pdu_string) + 2,pdu->data, pdu->datalen);//Copy the data	
        printbytes((*pdu_string), pdu_length);
	return pdu_length;
}

struct modbus_pdu *ascii2pdu(char *str, int len) {
  char byte;
  int j = 0, i;
  struct modbus_pdu *pdu;
  int top, bottom;

  pdu = malloc(sizeof(struct modbus_pdu));
  pdu->datalen = len/2 - 3;
  pdu->data = malloc(pdu->datalen);

  top = j++; bottom = j++;
  tobinary(&pdu->addr, str[top], str[bottom]); 

  top = j++; bottom = j++;
  tobinary(&pdu->fc, str[top], str[bottom]); 

  for(i=0;i<pdu->datalen;i++) {
    top = j++; bottom = j++;
    tobinary(&pdu->data[i], str[top], str[bottom]); 
  }

  top = j++; bottom = j++;
  pdu->crc[0] = str[top];
  pdu->crc[1] = str[bottom];

  return pdu;
}

unsigned char pdulrc(struct modbus_pdu *pdu) {
  char *str;
  unsigned char crc;

  str = malloc(pdu->datalen + 2); //datalen + 1 byte addr + 1 byte fc
  str[0] = pdu->addr;
  str[1] = pdu->fc;
  memcpy(str+2, pdu->data, pdu->datalen);
  crc = LRC(str, pdu->datalen+2);  
  free(str);
  return crc;
}

void printadu (struct modbustcp_adu *adu) {
  printf("--------------------------------\n");
  printf("adu->header.transid = %d\n", adu->header.transid);
  printf("adu->header.protid = %d\n", adu->header.protid);
  printf("adu->header.length = %d\n", adu->header.length);
  printf("adu->header.unitid = %d\n", adu->header.unitid);
  printf("adu->fc = %d\n", adu->fc);
  printf("adu->datalen = %d\n", adu->datalen);
  printf("adu->data = \n");
  printbytes(adu->data, adu->datalen);
  printf("--------------------------------\n");
}

struct modbustcp_adu *array2adu(char *array) {
  struct modbustcp_adu *temp;
  int remaining;
  unsigned short tshort;

  temp = malloc(sizeof(struct modbustcp_adu));

  memcpy(&tshort, array, sizeof(unsigned short));
  temp->header.transid = ntohs(tshort);
  
  memcpy(&tshort, array+2, sizeof(unsigned short));
  temp->header.protid = ntohs(tshort);

  memcpy(&tshort, array+4, sizeof(unsigned short));
  temp->header.length = ntohs(tshort);
  remaining = temp->header.length; 
  printf("remaining = %d\n", remaining);

  temp->header.unitid = *(array+6);
  remaining -= 1; 
  temp->fc = *(array+7);
  remaining -= 1; 

  temp->datalen = remaining;
  //copy data
  temp->data = malloc(remaining);
  memcpy(temp->data,array+8,remaining); 

  return temp;
}

struct modbus_pdu *adu2pdu(struct modbustcp_adu *adu) {
  struct modbus_pdu *pdu;
  pdu = malloc(sizeof(struct modbus_pdu));
  //pdu->data = malloc(datalen);
  pdu->datalen = adu->datalen;
  pdu->fc = adu->fc;
  pdu->addr = adu->header.unitid;
  pdu->data = malloc(pdu->datalen);
  memcpy(pdu->data, adu->data, pdu->datalen);
  pdu->crc[0] = 0;
  pdu->crc[1] = 0;
}

unsigned char *pdu2mbtcpadu(struct modbus_pdu *pdu, int transid) {
  unsigned char *array;
  int i, offset=0;
  unsigned short temp;

  // array size = 7 bytes for MBAP header + 1 byte function code + pdu data length 
  array = malloc(pdu->datalen + 8);

  //MBAP header (transaction identifier (2B), protocol identifier (2B), length (2B), unit identifier (1B))
  // transaction identifier
  // 2 bytes, created by MTU, copied by RTU, unique to each transaction
  temp = htons((unsigned short int) transid);
  memcpy(array+offset, &temp, sizeof(unsigned short));
  offset+=sizeof(unsigned short);

  // protocol identifier
  // 2 bytes, always 0
  temp = htons((unsigned short int) 0);
  memcpy(array+offset, &temp, sizeof(unsigned short));
  offset+=sizeof(unsigned short);

  // length
  // 2 bytes, number of following bytes
  // 1 (unit identifier) + 1 (function code) + pdu data length
  //*(array+4) 
  temp = htons((unsigned short int) (2+pdu->datalen));
  memcpy(array+offset, &temp, sizeof(unsigned short));
  memcpy(&temp, array+offset, sizeof(unsigned short));
  temp= ntohs(temp);

  offset+=sizeof(unsigned short);
 
  // unit identifier
  // 1 byte, replaces the address field
  memcpy(array+offset, &(pdu->addr), sizeof(unsigned char));
  offset+=sizeof(unsigned char);

  //function code
  memcpy(array+offset, &(pdu->fc), sizeof(unsigned char));
  offset+=sizeof(unsigned char);

  //data field
  memcpy(array+offset, pdu->data, pdu->datalen);

  return array;
}

unsigned char *pdu2array(struct modbus_pdu *pdu) {
  unsigned char *array;
  int i;

  array = malloc(pdu->datalen + 4);
  array[0] = pdu->addr;
  array[1] = pdu->fc;
  for (i=0;i<pdu->datalen;i++) {
    array[i+2] = pdu->data[i];
  }
  array[pdu->datalen+2] = pdu->crc[0];
  array[pdu->datalen+3] = pdu->crc[1];

  return array;
}

int pdu2ascii(struct modbus_pdu *pdu, char *str) {
  int i, j = 0;
  char top, bottom;
  unsigned char crc;

  byte2ascii(pdu->addr, &top, &bottom);
  str[j++] = top;
  str[j++] = bottom;
  
  byte2ascii(pdu->fc, &top, &bottom);
  str[j++] = top;
  str[j++] = bottom;

  for (i=0;i<pdu->datalen;i++) {
    byte2ascii(pdu->data[i], &top, &bottom);
    str[j++] = top;
    str[j++] = bottom;
  }

  crc = pdulrc(pdu);  
  byte2ascii(crc, &top, &bottom);
  str[j++] = top;
  str[j++] = bottom;
 
  return j; //return the length of the string
}

struct modbus_pdu *mkpdu(unsigned char addr, unsigned char fc, unsigned char *data, int datalen, unsigned char crc[2]) {
  struct modbus_pdu *pdu;
  pdu = malloc(sizeof(struct modbus_pdu));
  pdu->data = malloc(datalen);

  pdu->addr = addr;
  pdu->fc = fc;
  pdu->datalen = datalen;
  memcpy(pdu->data, data, datalen);

  if (crc != NULL) {
    pdu->crc[0]= crc[0];
    pdu->crc[1]= crc[1];
  }

  return (pdu);
}

void freepdu(struct modbus_pdu *pdu) {
  free(pdu->data);
  free(pdu);

  return;
}

void freerec(struct modbus_record *rec) {
  freepdu(rec->pdu);
  free(rec);

  return;
}

// Note: we can have only FIFO_SIZE-1 elements in the list at any given time.
#define FIFO_SIZE 2048
int iHead = 0;
int iTail = 0;
unsigned char aiElements[FIFO_SIZE];
int put(unsigned char i) {
  int iTemp = (iTail + 1) % FIFO_SIZE;
  if (iTemp == iHead) {
    printf("Error: FIFO Overflow. iHead = %d, iTail = %d, iTemp = %d\n", iHead, iTail, iTemp);
    exit(1);
  }
  aiElements[iTail] = i;
  iTail = iTemp;
  //printf("Element added: i = %c, iHead = %d, iTail = %d, iTemp = %d\n", i, iHead, iTail, iTemp);
  return;
} //end put

unsigned char *get() {
  unsigned char *i;
  i = malloc(sizeof(unsigned char));
  if (iHead == iTail) {
    free(i);
    return NULL; /* FIFO is empty */
  }
  *i = aiElements[iHead];
  iHead = (iHead + 1) % FIFO_SIZE;
  //printf("*i = %c, iHead = %d, iTail = %d\n", *i, iHead, iTail);
  return i;
}

int getbyte(int port, char *byte) {
   int len = 0;
   int i;
   char tempbuf[128];
   int gotone = 0;
   unsigned char *temp;

   do {
#ifdef FORLINUX
     len = read(port, tempbuf, 1); 
#endif
#ifdef FORMICROBLAZE
     if (port ==XPAR_RS232_UART_1_DEVICE_ID) {
       len = XUartNs550_Recv(&RS232_Uart_1_UartNs550, (u8 *) tempbuf,128);
     } else if (port ==XPAR_RS232_UART_2_DEVICE_ID) {
       len = XUartNs550_Recv(&RS232_Uart_2_UartNs550, (u8 *) tempbuf,128);
     }
#endif

	 
     if (len > 0) {
       //printf("Got %d bytes\n", len);
       //printbytes(tempbuf, len);

       for (i=0;i<len;i++) {
          put(tempbuf[i]);
       }
     } 

     #if TIMEOUT_ENABLE==1
     //This is a time out case
     else {
       return 0;
     }
     #endif
     if (temp = get()) {
       *byte = *temp;
	   free(temp);
       gotone = 1;
//       if (*byte == ASCII_CR) {
//         printf("R");
//       } else if (*byte == ASCII_LF) {
//         printf("L\n");
//       } else if (*byte == ASCII_COLON) {
//         printf("RX: %c", *byte);
//       } else {
//         printf("%c", *byte);
//       }
     }
   } while (!gotone);

   return 1;
}


int sendbyte(int port, char byte) {
  int len;
  struct timespec req, rem;

  req.tv_sec = 0;
  req.tv_nsec = 2e8;
  
  do {
#ifdef FORLINUX
    len = write(port, &byte, 1);
#endif
#ifdef FORMICROBLAZE
	if (port == XPAR_RS232_UART_2_DEVICE_ID) {
      len = XUartNs550_Send(&RS232_Uart_2_UartNs550, &byte,1);
     
    } else if (port == XPAR_RS232_UART_1_DEVICE_ID) {
      len = XUartNs550_Send(&RS232_Uart_1_UartNs550, &byte,1);
    }
#endif
    //if (byte == ASCII_CR) {
    //  printf("TX byte = <CR> (0x%.2x)\n", (unsigned char) byte);
    //} else if (byte == ASCII_LF) {
    // printf("TX byte = <LF> (0x%.2x)\n", (unsigned char) byte);
    //} else {
    //  printf("TX byte = %x (0x%.2x)\n", byte, (unsigned char) byte);
    //}
    //if (len < 1) { 
    // //printf("write returned len = %d\n", len); 
    // //nanosleep(&req, &rem);
    //} //else {
    // //printf("write returned len = %d, sent %c\n", len, byte); 
    //}
  } while (len < 1);
}

unsigned char char2hex(char letter) {
  unsigned char temp;

  switch (letter) {
    case '0' : temp = 0; break;
    case '1' : temp = 1; break;
    case '2' : temp = 2; break;
    case '3' : temp = 3; break;
    case '4' : temp = 4; break;
    case '5' : temp = 5; break;
    case '6' : temp = 6; break;
    case '7' : temp = 7; break;
    case '8' : temp = 8; break;
    case '9' : temp = 9; break;
    case 'A' : temp = 10; break;
    case 'B' : temp = 11; break;
    case 'C' : temp = 12; break;
    case 'D' : temp = 13; break;
    case 'E' : temp = 14; break;
    case 'F' : temp = 15; break;
    default: temp = 0; 
  }
  return temp;
}

char hex2char(unsigned char byte) {
  char temp;
  switch (byte) {
    case 0 : temp = '0'; break;
    case 1 : temp = '1'; break;
    case 2 : temp = '2'; break;
    case 3 : temp = '3'; break;
    case 4 : temp = '4'; break;
    case 5 : temp = '5'; break;
    case 6 : temp = '6'; break;
    case 7 : temp = '7'; break;
    case 8 : temp = '8'; break;
    case 9 : temp = '9'; break;
    case 10: temp = 'A'; break;
    case 11: temp = 'B'; break;
    case 12: temp = 'C'; break;
    case 13: temp = 'D'; break;
    case 14: temp = 'E'; break;
    case 15: temp = 'F'; break;
    default: temp = '0'; 
  }
  return temp;
}

void byte2ascii(unsigned char byte, char *ascii1, char *ascii2) {
  char top, bottom;
  
  top= (byte & 0xf0) >> 4;
  bottom= (byte & 0x0f);
 
  *ascii1 = hex2char(top);
  *ascii2 = hex2char(bottom);
}

void tobinary(unsigned char *byte, char ascii1, char ascii2) {
  unsigned char temp1, temp2;
  temp1 = char2hex(ascii1);
  temp2 = char2hex(ascii2);
  *byte = (temp1<<4) | temp2;
  return ;
}

struct modbus_pdu *getmodbus_ascii(int port) {
  int done = 0;
  int pdulen = 0, len = 0, datalen;
  int i, end;
  char byte;
  unsigned char tempbytes[255]; 
  struct modbus_pdu *pdu;

  unsigned char addr, fc, *data, *crc;

  end = 0;
  while (!end) {
     len = getbyte(port, &byte); 
#if TIMEOUT_ENABLE==1
     if (len<1){
       return NULL;
     }
#endif
     if (byte != ASCII_COLON) {
       tempbytes[pdulen] = byte;
       pdulen += len;
       if (pdulen >= 255) {
         printf("Error: %d bytes returned after \':\' without CR-LF (limit = 255)\n", pdulen);
         printf("BR:Bytes received were: \n" );
         printbytes(tempbytes, 255);
         exit(1);
       }
    }
    end = (tempbytes[pdulen-2] == ASCII_CR)  &&(tempbytes[pdulen-1] == ASCII_LF);
  } // end while
  if (pdulen < 2) {
    printf("Error: Invalid PDU size, %d bytes returned after colon.\n", pdulen);
    exit(1);
  }
  pdu = ascii2pdu(tempbytes, pdulen-2);
  if (pdu->datalen < 1) {
    printf("Error: Invalid PDU Data length = %d\n", pdu->datalen);
    exit(1);
  }
  return pdu;
}

struct modbus_pdu *get_send_modbus_ascii(int inp, int outp) {
  int done = 0;
  int pdulen = 0, len = 0, datalen;
  int i, end;
  char byte;
  char bstr[2];
  unsigned char tempbytes[255]; 
  struct modbus_pdu *pdu;

  unsigned char addr, fc, *data, *crc;

#ifdef MICROBLAZE
  int line, pos;
  if (inp == XPAR_RS232_UART_1_DEVICE_ID) {
    line = 0;
	 pos = posl0;
  } else {
    line = 1;
	 pos = posl1;
  }
#endif

  end = 0;
  while (!end) {
     len = getbyte(inp, &byte); 

   
#if TIMEOUT_ENABLE==1
     if (len<1){
       return NULL;
     }
#endif
     if(len==1)
	  {
	  /* if (byte == ASCII_CR) {
       ldpbuf("R");
     } else if (byte == ASCII_LF) {
       ldpbuf("L\n");
     } else if ((byte < 32) || (byte > 126)) {
       ldpbuf("!");
     } else {
       strncpy(bstr, &byte, 1);
       ldpbuf(bstr);
     }*/
     sendbyte(outp, byte); 
	  }
     if (byte != ASCII_COLON) {
       tempbytes[pdulen] = byte;
       pdulen += len;
       if (pdulen >= 255) {
         printf("Error: %d bytes returned after \':\' without CR-LF (limit = 255)\n", pdulen);
         exit(1);
       }
    }
    end = (tempbytes[pdulen-2] == ASCII_CR)  &&(tempbytes[pdulen-1] == ASCII_LF);
  } // end while
  if (pdulen < 2) {
    printf("Error: Invalid PDU size, %d bytes returned after colon.\n", pdulen);
    exit(1);
  }
  pdu = ascii2pdu(tempbytes, pdulen-2);
  if (pdu->datalen < 1) {
    printf("Error: Invalid PDU Data length = %d\n", pdu->datalen);
    exit(1);
  }
  return pdu;
}

int getmodbus_ascii_str(int inp, int *cnt, char *str) {
  int len = 0;
  int end;
  char byte;

  unsigned char addr, fc, *data, *crc;

  end = 0;
  len = getbyte(inp, &byte); 
  *cnt += len;

  strncat(str, &byte, 1);

  if (*cnt > 2) { 
    end = (str[*cnt-2] == ASCII_CR)  &&(str[*cnt-1] == ASCII_LF);
  }

  return end;
}


struct modbus_record *getmodbus_ascii_timed(int port) {
  struct modbus_record *rec;
  rec= malloc(sizeof(struct modbus_record));
  rec->pdu = getmodbus_ascii(port);
#ifdef FORLINUX
  gettimeofday(&rec->endtime, NULL);
#endif
  return rec;
}

void sendmodbus_ascii(int port, struct modbus_pdu *pdu) {
  int i, len; 

  char *str;

  str = malloc(2*(pdu->datalen+3));
  len = pdu2ascii(pdu, str); 

  pdu->crc[0] = str[len-2];
  pdu->crc[1] = str[len-1];
  
  sendbyte(port, ASCII_COLON); //start byte

  //send the message
  for (i=0;i<len;i++) {
    sendbyte(port, str[i]);
  }

  sendbyte(port, ASCII_CR); //CR followed by LF marks the end of a PDU transmission
  sendbyte(port, ASCII_LF);
 
  free(str);
 
  //portflush(port);
  return;
}

void sendmodbus_ascii_timed(int port, struct modbus_record *rec) {
  sendmodbus_ascii(port, rec->pdu);
#ifdef FORLINUX
  gettimeofday(&rec->endtime, NULL);
#endif
  return;  
}

void printmodbuspdu (struct modbus_pdu *pdu) {
  printf ("Address = 0x%x, FC = 0x%x, data length = %d\n", pdu->addr, pdu->fc, pdu->datalen); 
  printf ("Data = \n");
  printbytes(pdu->data, pdu->datalen);

  printf ("CRC = %.2x%.2x\n", pdu->crc[0], pdu->crc[1]);
}
#ifdef FORLINUX
void printmodbusrec (struct modbus_record *rec) {
  long milliseconds;
  struct tm *ptm;
  char stime[40];
  ptm = localtime(&rec->endtime.tv_sec);
  strftime(stime, sizeof(stime), "%Y-%m-%d %H:%M:%S", ptm);
  if (rec->pdu != NULL) {
    printmodbuspdu(rec->pdu);
  }
  milliseconds = rec->endtime.tv_usec /1000;
  printf("Endtime = %s.%3ld\n", stime, milliseconds);  
}
#endif

#ifdef FORLINUX
int openport(char *portstr ){
  int port, err; 
  struct termios config;

  printf("opening port %s...\n", portstr);
  port = open (portstr, O_RDWR | O_NOCTTY);
  printf("opening port %s(%d)\n", portstr, port);

  if (port < 0) {
    perror("MSUModbus unable to open port: ");
    exit(0);
  }

  if(tcgetattr(port, &config) < 0) { 
      printf("MSUModbus Error: Can not retrieve %s attributes\n", portstr); 
      exit(0); 
  }
  config.c_iflag = IGNPAR;
  config.c_oflag = 0;
  config.c_lflag = 0;
  config.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;

  #if TIMEOUT_ENABLE==1
    // return amount of bytes requested by read call or 
    // time out after TIMEOUT_VAL character times (TIMEOUT_VAL tenths of second)
    config.c_cc[VMIN]  = 0;
    config.c_cc[VTIME] = TIMEOUT_VAL;
  #else
    // return if atleast 1 byte is availble
    // no time out, block until 1 byte is availble 
    //config.c_cc[VMIN]  = 1;
    config.c_cc[VMIN]  = 0;
    config.c_cc[VTIME] = 0;
  #endif

  // Communication speed (simple version, using the predefined
  // constants)
  if(cfsetispeed(&config, BAUDRATE) < 0 || cfsetospeed(&config, BAUDRATE) < 0) {
    printf("Unable to set baud rate for %s\n", portstr);
  }
  //
  // Finally, apply the configuration
  //
  if(tcsetattr(port, TCSAFLUSH, &config) < 0) { printf("Error: Unable to apply config to %s\n", portstr); exit; }
  //portflush(port);
  return port;
}
#endif

#ifdef FORMICROBLAZE
int openport(char *portstr){
    XStatus status;
    int device;
  if (!strcmp(portstr, TTYS0)) {
      status = UartNs550Init(&RS232_Uart_1_UartNs550, XPAR_RS232_UART_1_DEVICE_ID);
      device = XPAR_RS232_UART_1_DEVICE_ID;
  } else if (!strcmp(portstr, TTYS1)) {
      status = UartNs550Init(&RS232_Uart_2_UartNs550, XPAR_RS232_UART_2_DEVICE_ID);
		device = XPAR_RS232_UART_2_DEVICE_ID;
  }
  return device;
}
#endif

#ifdef FORLINUX
void printcrc(int port, char *str) {
}
#endif

#ifdef FORMICROBLAZE
void printcrc(int port, char *str) {
  if (port ==XPAR_RS232_UART_1_DEVICE_ID) {
    lcd_goto(0, 6);  
  } else if (port ==XPAR_RS232_UART_2_DEVICE_ID) {
    lcd_goto(1, 6);  
  }
  lcd_puts(str); 
} //end printcrc
#endif

int check_crc(struct modbus_pdu *pdu, int port){
  char message[4];
  char crc, top, bottom;

  crc = pdulrc(pdu);  
  byte2ascii(crc, &top, &bottom);
 
  if ((pdu->crc[1] == bottom) && (pdu->crc[0] == top)){
    message[0] = 'G';
    message[1] = top;
    message[2] = bottom;
    message[3] = '\0';
    //printcrc(port, message);
    return 1;//good
  } else {
    message[0] = 'X';
    message[1] = top;
    message[2] = bottom;
    message[3] = '\0';
    //printcrc(port, message);
    return 0;
  }
}

char dig2alpha(int digit) {
  char alpha;
  switch (digit) {
    case 0 : alpha = '0';
             break;
    case 1 : alpha = '1';
             break;
    case 2 : alpha = '2';
             break;
    case 3 : alpha = '3';
             break;
    case 4 : alpha = '4';
             break;
    case 5 : alpha = '5';
             break;
    case 6 : alpha = '6';
             break;
    case 7 : alpha = '7';
             break;
    case 8 : alpha = '8';
             break;
    default : alpha = '9';
             break;
  } //end switch
  return alpha;
} //end dig2alpha

void itoa(int num, char *str, int radix) {
  int rem, modulus, i, j;
  int digits[33];
  char sdig;
  char *orig;
  orig = str;

  i = 32;
  rem = num;
  //xil_printf("rem = %d\n", rem);
  if (rem == 0) {
    digits[i] = 0; 
    i--;
  }

  while (rem > 0) {
    modulus = rem % radix;
    rem = (rem - modulus) / radix; 
    digits[i] = modulus; 
    //xil_printf("modulus = %d, rem = %d, digits[%d] = %d\n", modulus, rem, i, digits[i]);
    i--;
  }  // end while
   
  for(j=i+1;j<33;j++) {
    sdig = dig2alpha(digits[j]);
    //xil_printf("sdig = %c, digits[%d] = %d\n", sdig, j, digits[j]);
    //strncat(str, &sdig, 1);
    *(str++) = sdig;
  }
  *str++ = '\0';
  //xil_printf("str = %s (strlen = %d)\n", orig, strlen(orig));
}

#ifdef FORLINUX
void printdiag(int port, char* str) {
}
#endif
#ifdef FORMICROBLAZE
void printdiag(int port, char* str) {
  if (port ==XPAR_RS232_UART_1_DEVICE_ID) {
    lcd_goto(0, 10);  
  } else if (port ==XPAR_RS232_UART_2_DEVICE_ID) {
    lcd_goto(1, 10);  
  }
  lcd_puts(str); 
} //end printdiag
#endif

#ifdef FORLINUX
void printrxcnt(int port, int cnt) {
}
#endif
#ifdef FORMICROBLAZE
void printrxcnt(int port, int cnt) {
  char str[34];
  if (port ==XPAR_RS232_UART_1_DEVICE_ID) {
    lcd_goto(0, 0);  
  } else if (port ==XPAR_RS232_UART_2_DEVICE_ID) {
    lcd_goto(1, 0);  
  }
  itoa(cnt, str, 10);
  //xil_printf("str = %s\n", str);
  lcd_puts(str); 
} //end printdiag
#endif

#ifdef FORMICROBLAZE
void lcd_print(unsigned int line, unsigned int *pos, const char *instr, unsigned int len) {
   char tmpstr[16];
	char str[16];
	strcpy(str, instr);
   //str[len]= '\0'; //terminate the str before printing
   if ((*pos + len) < 16) {
     lcd_goto(line,*pos); 
     lcd_puts(str); 
     *pos = (*pos + len) % 16;
   } else {
     int fits, extra;
     extra = len;
     fits = 16 - *pos;
     strncpy(tmpstr, str, fits);

     while (extra > 0) {
       lcd_goto(line,*pos); 
       lcd_puts(tmpstr); 
       fits = 16 - *pos;
       extra = len - fits;
       strncpy(tmpstr, str + fits, extra);
       *pos = (*pos + fits) % 16;
     }
   }
}
#endif
#ifdef FORMICROBLAZE

int com_flash1(struct modbus_pdu *pdu)
 {
  int len; 
  int i;
  char str[256];
   len = pdu2ascii(pdu,str);
   xil_printf("string values");
  for (i=0;i<=strlen(str);i++){
     xil_printf("%x",str[i]);
  }
   if (len > 0){    
     Sysace_appendLog(str);	 
	 }
 
   free(str);
 }

#endif

#ifdef FORLINUX
struct modbus_pdu *getmodbus_RTU(int port)
{
 int nread;
 int i = 0;
 int timeout = 0;
 unsigned char  *temp;
 temp = malloc(16);
 unsigned char  buff[256];

 double timeuse;
 double oldtime;
 double newtime = 0;

 struct timeval tpstart, tpend;
 double starttime, endtime;

 unsigned char addr, fc, *data, *crc;
 struct modbus_pdu *pdu;
 int pdulen = 0,  datalen;

  gettimeofday(&tpstart, NULL);
  starttime = tpstart.tv_sec + tpstart.tv_usec/1e6;

  while(!timeout) {
     next: nread = read(port, temp, 1);
     //printf("nread = %d\n", nread);
     if(nread ==1) {
       buff[i++] = temp[0];
       printf("buff[%d] =  %.2x \n", i-1, buff[i-1]);
       gettimeofday(&tpstart, NULL);
       starttime = tpstart.tv_sec + tpstart.tv_usec/1e6;
       printf("start time = %d S : %d uS\n", (int)tpstart.tv_sec,(int)tpstart.tv_usec);
     } else {
       gettimeofday(&tpend, NULL); 
       endtime = tpend.tv_sec + tpend.tv_usec/1e6;
       //printf("current time = %d S : %d uS\n", tpend.tv_sec,tpend.tv_usec);
       //printf("starttime = %f endtime = %f timeuse = %f\n", starttime, endtime, timeuse);
       timeuse = endtime - starttime;
       timeuse*=1000;
     }
     if((timeuse > RTU_TIMEOUT) && (i != 0)) {
       timeout = 1;
       printf("end time = %d S : %d uS\n", (int)tpend.tv_sec,(int)tpend.tv_usec);
       printf("timeuse = %f mS\n", timeuse);
     } if(timeuse > RTU_TIMEOUT) { 
       gettimeofday(&tpstart, NULL);
       starttime = tpstart.tv_sec + tpstart.tv_usec/1e6;
       //printf("(i=0) current time = %d S : %d uS\n", tpend.tv_sec,tpend.tv_usec);
       //printf("timeuse = %f mS\n", timeuse);
     }
     
  }
  addr = buff[0];
  fc = buff[1];
  data = (unsigned char *)buff+2;
  if( i <= 4)
    datalen = 0;
  else
    datalen = i - 4;
  crc = (unsigned char *)buff + datalen + 2;
  pdu = mkpdu(addr, fc, data, datalen, crc);
  return pdu;
}

void sendmodbus_RTU(int port, struct modbus_pdu *pdu)
{
 int i, len;
 unsigned char *array;

 array = pdu2array(pdu);
 len = write(port, array, pdu->datalen+4);
 if (len != (pdu->datalen+4)) {
   printf("error: write sent %d bytes (supposed to send %d bytes).\n", len, pdu->datalen+4);
 }
 free(array);
// sendbyte(port, pdu->addr);
// sendbyte(port, pdu->fc);

// for (i = 0; i < pdu->datalen; i++)
// {
//  sendbyte(port,pdu->data[i]);
// }
// sendbyte(port, pdu->crc[0]);
// sendbyte(port, pdu->crc[1]);
 return;
}
#endif
