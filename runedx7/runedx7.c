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

#define MAX_FILE_SIZE 0xF00000
#define BLOCK_SIZE 0x10000

int usbser;
struct termios oldtty;
char defdev[]="/dev/ttyUSB0";
char *usbdev;
uint8_t *buf;
unsigned long long filelen;
unsigned long long buflen;
uint8_t blocks;
int is_ssf=0;
int is_sega=0;

char md_check[]="    *T";
char md_OK='k';
char md_DataOK='d';
char md_LoadGame[]="*g";
char md_RunGame[]="*rm";
char md_RunSSF[]="*rS";

void ser_done() {
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
  return 1;
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
  if (filelen>MAX_FILE_SIZE) {
    printf("file is too big\n");
    fclose(f);
    exit(-1);
  };
  buflen=(filelen+BLOCK_SIZE-1)&(~(BLOCK_SIZE-1));
  blocks=buflen/BLOCK_SIZE;
  
  buf=(uint8_t*)calloc(buflen,sizeof(uint8_t));
  if (buf) {
    fread(buf,1,filelen,f);
  } else {
    printf("Error allocating %lli bytes\n",buflen);
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

char getresult() {
  char res;
  int bread;
  do {
    bread=read(usbser,&res,1);
  } while (bread<=0);
  return(res);
};

void do_transfer() {
  printf("Transfer started\n");
  write(usbser,md_check,strlen(md_check));

  if (getresult()!=md_OK) {
    printf("Everdrive X7 not found\n");
    return;
  };
  printf("Everdrive X7 found\n");

  write(usbser,md_LoadGame,strlen(md_LoadGame));
  write(usbser,&blocks,1);

  if (getresult()!=md_OK) {
    printf("Error during transfer init\n");
    return;
  };

  write(usbser,buf,buflen);

  if (getresult()!=md_DataOK) {
    printf("Error transferring ROM\n");
    return;
  };

  if (is_ssf) {
    write(usbser,md_RunSSF,strlen(md_RunSSF));
  } else {
    write(usbser,md_RunGame,strlen(md_RunGame));
  };

  if (getresult()!=md_OK) {
    printf("Error during ROM execute\n");
    return;
  };
};

int main(int argc, char **argv) {
  printf("insane's Everdrive X7 ROM Runner\n");
  if((argc<2)||(argc>3)) {
    printf("usage: runmd [/dev/ttyUSB0] rom.md\n");
    return 0;
  };
  if (argc==3) {
    file_init(argv[1],argv[2]);
  } else {
    file_init(defdev,argv[1]);
  };
  if (is_ssf) printf("Mapper activated\n");

  if (!ser_init()) {
    exit(0);
  };
  
  if (is_sega) {
    do_transfer();
  } else {
    printf("ROM corrupt - SEGA ID not found\n");
  };

  ser_done();
  file_done();

  return(0);
};
