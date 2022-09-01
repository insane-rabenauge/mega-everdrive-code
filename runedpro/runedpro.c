/*
 * Copyright 2022 insane/Rabenauge^tSCc <insane.atari@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>      // File control definitions
#include <termios.h>    // POSIX terminal control definitions
#include <sys/ioctl.h>
#include <errno.h>

#define BLOCK_LEN 8192
#define ACK_BLOCK_SIZE 1024
#define MAX_ROM_SIZE 0xF80000
#define ADDR_ROM 0x0000000
#define ADDR_SRAM 0x1000000
#define ADDR_BRAM 0x1080000
#define ADDR_CFG 0x1800000
#define ADDR_SSR  0x1802000
#define ADDR_FIFO 0x1810000
#define SIZE_ROMX 0x1000000
#define SIZE_SRAM 0x80000
#define SIZE_BRAM 0x80000
#define ADDR_FLA_MENU 0x00000
#define ADDR_FLA_FPGA 0x40000
#define ADDR_FLA_ICOR 0x80000
#define FAT_READ 0x01
#define FAT_WRITE 0x02
#define FAT_OPEN_EXISTING 0x00
#define FAT_CREATE_NEW 0x04
#define FAT_CREATE_ALWAYS 0x08
#define FAT_OPEN_ALWAYS 0x10
#define FAT_OPEN_APPEND 0x30
#define HOST_RST_OFF    0
#define HOST_RST_SOFT   1
#define HOST_RST_HARD   2
#define CMD_STATUS 0x10
#define CMD_GET_MODE 0x11
#define CMD_IO_RST 0x12
#define CMD_GET_VDC 0x13
#define CMD_RTC_GET 0x14
#define CMD_RTC_SET 0x15
#define CMD_FLA_RD 0x16
#define CMD_FLA_WR 0x17
#define CMD_FLA_WR_SDC 0x18
#define CMD_MEM_RD 0x19
#define CMD_MEM_WR 0x1A
#define CMD_MEM_SET 0x1B
#define CMD_MEM_TST 0x1C
#define CMD_MEM_CRC 0x1D
#define CMD_FPG_USB 0x1E
#define CMD_FPG_SDC 0x1F
#define CMD_FPG_FLA 0x20
#define CMD_FPG_CFG 0x21
#define CMD_USB_WR 0x22
#define CMD_FIFO_WR 0x23
#define CMD_UART_WR 0x24
#define CMD_REINIT 0x25
#define CMD_SYS_INF 0x26
#define CMD_GAME_CTR 0x27
#define CMD_UPD_EXEC 0x28
#define CMD_HOST_RST 0x29
#define CMD_DISK_INIT 0xC0
#define CMD_DISK_RD 0xC1
#define CMD_DISK_WR 0xC2
#define CMD_F_DIR_OPN 0xC3
#define CMD_F_DIR_RD 0xC4
#define CMD_F_DIR_LD 0xC5
#define CMD_F_DIR_SIZE 0xC6
#define CMD_F_DIR_PATH 0xC7
#define CMD_F_DIR_GET 0xC8
#define CMD_F_FOPN 0xC9
#define CMD_F_FRD 0xCA
#define CMD_F_FRD_MEM 0xCB
#define CMD_F_FWR 0xCC
#define CMD_F_FWR_MEM 0xCD
#define CMD_F_FCLOSE 0xCE
#define CMD_F_FPTR 0xCF
#define CMD_F_FINFO 0xD0
#define CMD_F_FCRC 0xD1
#define CMD_F_DIR_MK 0xD2
#define CMD_F_DEL 0xD3
#define CMD_USB_RECOV 0xF0
#define CMD_RUN_APP 0xF1
#define STATUS_OK 0x00
#define STATUS_GAME 0x04

int usbser;
struct termios oldtty;
char defdev[]="/dev/ttyACM0";
char *usbdev;
uint8_t *buf;
unsigned long long filelen;
int is_ssf=0;
int is_sega=0;

void ser_exit() {
  tcflush(usbser,TCIFLUSH);
  tcsetattr(usbser,TCSANOW,&oldtty);
  close(usbser);
};

int ser_init() {
  struct termios tty;
  printf("opening device \"%s\"\n",usbdev);
  usbser = open( usbdev, O_RDWR | O_NOCTTY  );
  memset(&tty, 0, sizeof (tty));
  if (tcgetattr ( usbser, &tty ) != 0 ) {
    printf("unable to open device \"%s\"\n",usbdev);
    close(usbser);
    return 0;
  };
  memcpy(&oldtty,&tty,sizeof(tty));

  cfsetispeed(&tty, B230400); 
  cfsetospeed(&tty, B230400); //shouldn't be needed due to usbser

  cfmakeraw(&tty); // enable bits needed for raw mode
  tty.c_cflag &= ~CSTOPB; // 1 stopbit, could be ignored due to not having a real serial port
  tty.c_cflag &= ~CRTSCTS; // no flow control
  tty.c_cflag |= CLOCAL|CREAD;
  tty.c_cc[VTIME] = 5;
  tty.c_cc[VMIN] = 1; // at least 1 character must be readable

  if (tcsetattr(usbser,TCSANOW,&tty)<0) {
    printf("failed to set serial attributes\n");
    exit(0);
  };
  tcflush(usbser,TCIFLUSH);
  atexit(ser_exit);
  return 1;
};

void txCMD(int cmd) {
  static uint8_t buf[4];
  buf[0]='+';
  buf[1]='+'^0xff;
  buf[2]=cmd;
  buf[3]=cmd^0xff;
  write(usbser,buf,4);
};

void txData(uint8_t *buf,int offs,int len) {
  while (len>0) {
    int blen=(BLOCK_LEN>len)?len:BLOCK_LEN;
    write(usbser,&buf[offs],blen);
    len-=blen;offs+=blen;
  };
};

void tx32(uint32_t arg) {
  static uint8_t buf[4];
  buf[0]=arg>>24;
  buf[1]=arg>>16;
  buf[2]=arg>>8;
  buf[3]=arg;
  write(usbser,buf,4);
};

uint32_t rx32() {
  static uint8_t buf[4];
  int bread=read(usbser,buf,4);
  if (bread!=4) return -1;
  return buf[3]|(buf[2]<<8)|(buf[1]<<16)|(buf[0]<<24);
};

void tx16(uint16_t arg) {
  static uint8_t buf[2];
  buf[0]=arg>>8;
  buf[1]=arg;
  write(usbser,buf,2);
};

uint16_t rx16() {
  static uint8_t buf[2];
  int bread=read(usbser,buf,2);
  if (bread!=2) return -1;
  return buf[1]|(buf[0]<<8);
};

void tx8(uint8_t arg) {
  static uint8_t buf;
  buf=arg;
  write(usbser,&buf,1);
};

uint8_t rx8() {
  static uint8_t buf;
  int bread=read(usbser,&buf,1);
  if (bread!=1) return -1;
  return buf;
};

void memWR(uint32_t addr,uint8_t *buf, int offs, int len) {
  if (len==0) return;
  txCMD(CMD_MEM_WR);
  tx32(addr);
  tx32(len);
  tx8(0);//exec
  txData(buf,offs,len);
};

void fifoWR(char* data, int offs, int len) {
  memWR(ADDR_FIFO,(uint8_t*)data,offs,len);
};

void fifoTX32(int arg) {
  static char buf[4];
  buf[0]=(arg >> 24);
  buf[1]=(arg >> 16);
  buf[2]=(arg >> 8);
  buf[3]=(arg);
  fifoWR(buf, 0, 4);
}

void fifoTX16(int arg) {
  static char buf[2];
  buf[0]=(arg >> 8);
  buf[1]=(arg);
  fifoWR(buf, 0, 2);
}

void fifoTXstr(char *str) {
  int sl=strlen(str);
  fifoTX16(sl);
  fifoWR(str,0,sl);
};

int getstatus() {
  txCMD(CMD_STATUS);
  int resp=rx16();
  if ((resp&0xff00)!=0xA500) printf("operation error %04X\n",resp);
  if ((resp&0xA500)!=0xA500) resp=-1;else resp&=0xff;
  return (resp);
};

int hosttest() {
  fifoWR("*t",0,2);
  int resp=rx8();
  return resp;
};

void hostreset(int rst) {
  txCMD(CMD_HOST_RST);
  tx8(rst);
};

void file_init(char * dev, char * file) {
  FILE *f;

  f=fopen(file,"rb");
  if (f==NULL) {
    printf("unable to open file \"%s\"\n",file);
    exit(-1);
  };
  fseek(f,0,SEEK_END);
  filelen=ftell(f);
  rewind(f);
  if (filelen>MAX_ROM_SIZE) {
    printf("file is too big\n");
    fclose(f);
    exit(-1);
  };
  
  buf=(uint8_t*)calloc(filelen,sizeof(uint8_t));
  if (buf) {
    fread(buf,1,filelen,f);
  } else {
    printf("Error allocating %lli bytes\n",filelen);
    fclose(f);
    exit(-1);
  };
  fclose(f);
  is_sega=(memcmp(&buf[0x100],"SEGA",4)==0)?1:0;
  if (!is_sega) is_sega=(memcmp(&buf[0x100]," SEGA",5)==0)?1:0; // same as TMSS does - " SEGA"@100 in addition to "SEGA"@100
  is_ssf =(memcmp(&buf[0x105],"SSF ",4)==0)?1:0;
  usbdev=malloc(strlen(dev));
  strcpy(usbdev,dev);
};

void file_done() {
  free(buf);
  free(usbdev);
};

void do_transfer() {
  if (getstatus()<0) {
    printf("Everdrive PRO not found\n");
    return;
  };
  printf("sending ROM\n");
  hostreset(HOST_RST_SOFT);
  memWR(ADDR_ROM,buf,0,filelen);
  hostreset(HOST_RST_OFF);
  int resp=rx8();
  if (resp!='r') printf("unexpected response: %04X\n",resp);
  hosttest();
  fifoWR("*g",0,2);
  fifoTX32(filelen);
  if (is_sega) {
    fifoTXstr("USB:ROM.MD");
  } else {
    fifoTXstr("USB:ROM.SMS");
  };
};

int main(int argc, char **argv) {
  printf("insane's Everdrive PRO ROM Runner\n");
  if((argc<2)||(argc>3)) {
    printf("usage: runmd [/dev/ttyACM0] rom.md\n");
    return 0;
  };
  if (argc==3) {
    file_init(argv[1],argv[2]);
  } else {
    file_init(defdev,argv[1]);
  };

  if (!ser_init()) {
    exit(0);
  };
 
  if (is_sega) {
    if (is_ssf) printf("Mapper activated\n");
  } else printf("No 68K SEGA ID in ROM - SMS activated, hard reset needed\n");  
  do_transfer();
  file_done();

  return(0);
};
