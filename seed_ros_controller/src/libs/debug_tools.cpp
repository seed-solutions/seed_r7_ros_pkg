#include "debug_tools.hpp"
#include <cstdio>

void dump(const char* str, uint8_t* data, int len){
    printf("[%s] data size: %02d hex: ",str,len);
    for(int idx = 0;idx < len;++idx){
        if(idx%4 == 0){
            printf("  ");
        }
        printf("%02x",data[idx]);
    }
    printf("\n");
}
