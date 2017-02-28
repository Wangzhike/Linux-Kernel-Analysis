#include <unistd.h>
#include <pthread.h>
//#include <linux/inotify.h>
#include <sys/inotify.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dlfcn.h>
#include "readConfig.h"
#include "messageTable.h"

#define CAL_CONF_PATH    "./cal.conf"
#define MAX_BUF_SIZE 1024

void *lib;
pMessage header;
pMessage tailer;
char cmd_prompt[MAX_BUF_SIZE];
int update;

void* notifyEvent(void *arg){
    int conf;
    int wd_conf;
    char buffer[MAX_BUF_SIZE];
    int len;
    //创建配置文件的inotify实例
    conf = inotify_init();
    if(conf < 0){
        printf("Fail to initialize inotify.\n");
        exit(EXIT_FAILURE);
    }
    //为配置文件添加事件监控对象
    wd_conf = inotify_add_watch(conf, CAL_CONF_PATH, IN_ALL_EVENTS);
    if (wd_conf < 0) {
        printf("Can't add watch for %s.\n", CAL_CONF_PATH);
        exit(EXIT_FAILURE);
    }
    while(1){
        len = read(conf, buffer, MAX_BUF_SIZE);
        if(len != -1){
            deleteMessageTable(&header, &tailer);
            readConfig(&header, &tailer);
            printf("%s", cmd_prompt);
//             printf("配置文件被改变了！\n");
        }
        sleep(5);
    }
    return ((void*)0);
}

int main(int argc, char **argv){
    pthread_t id;
    int ret;
    readConfig(&header, &tailer);
    printf("%s", cmd_prompt);
    ret = pthread_create(&id, NULL, notifyEvent, NULL);
    if(ret != 0){
        printf("Create pthread error!\n");
        exit(1);
    }
    int ch;
    while( (ch = getchar()) != EOF && ch != 'q'){
        pFunc pfunc = NULL;
        pfunc = searchMessageTable(&header, &tailer, ch);
        if(pfunc == NULL){
            printf("请输入正确的关键字！\n");
            printf("%s", cmd_prompt);
        }
        else{
            printf("请输入两个整数的操作数，以空格隔开:\n");
            type_t opnd1, opnd2, result;
            scanf("%d %d", &opnd1, &opnd2);
            result = pfunc(opnd1, opnd2);
            printf("计算结果为：%d\n", result);
        }
        getchar();      //丢弃紧随的回车
    }
    deleteMessageTable(&header, &tailer);
    dlclose(lib);
    if(ch == 'q'){
       pthread_cancel(id);
    }
    pthread_join(id, NULL);
    return 0;
}
