#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/time.h>
#include <unistd.h>

#define BUF_SZIE 128        /* <= 4MB */
unsigned char buf[BUF_SZIE];
unsigned char rcv_buf[BUF_SZIE];

int main(int argc, char **argv)
{
    int fd,flag=0, counter = 0;    
    unsigned int i;
    //struct timeval tv;
    //long long start_time, stop_time, delta_time;
    //off_t offset;
    //float speed;
    //int ntimes;

    for (i = 0; i < BUF_SZIE; ++i) {
        buf[i] = BUF_SZIE - i;
        rcv_buf[i] = 0;
    }

    fd = open("/dev/dsppcie",O_RDWR);

    if(fd<0){
        printf("open fail\n");
        return 0;
    }
    
    write(fd,buf,sizeof(buf));

    /* 等待DSP简单处理数据 */
    sleep(1);
    //offset = lseek(fd,0,SEEK_SET);
    read(fd,rcv_buf,sizeof(rcv_buf));

    for (i = 0; i < 20; ++i) {
        printf("buf[%d] = %d\trcv_buf[%d] = %d\n", i, buf[i], i, rcv_buf[i]);
    }

    for (i=0; i<BUF_SZIE; i++){
        if((~(rcv_buf[i]) & 0xFF) != buf[i]){
            flag=1;
            counter++;
        }
    }

    if (flag){
        printf("DMA Test Failed with %d locations!\n", counter);
    }
    else{
        printf("DMA Test Passed!\n");
    }
    return 0;
}
