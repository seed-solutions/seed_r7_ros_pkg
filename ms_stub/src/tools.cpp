#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <pty.h>
#include <string.h>
#include <unistd.h>
#include <pty.h>
#include <utmp.h>
#include <stdint.h>
#include <pthread.h>
#include <stdbool.h>

void dump(const char* title,uint16_t address,uint8_t len,uint8_t datalen,uint8_t* data,int ofst){
    int idx = 0;
    printf("---[%s]---------------------------\n",title);
    printf("address: 0x%04x\n",address);
    printf("len: %d(datalen:%d)\n",len,datalen);
    printf("data: [");
    for (idx = 0; idx < datalen; idx += 1) {//ad-adress2byte - cmd1byte = datasize
        if (idx == 0) {
        } else if (idx % 32 == 0) {
            printf("\n");
        } else if (idx % 4 == 0) {
            printf("    ");
        } else if (idx % 2 == 0) {
            printf(" ");
        }
        printf("%02x", data[ofst + idx]);
    }
    printf("]\n");
}

