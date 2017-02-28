#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dlfcn.h>
#include "readConfig.h"

#define BUFFER_SIZE 50
#define CAL_CONF_PATH    "./cal.conf"

extern void *lib;
extern char cmd_prompt[];

void readConfig(pMessage* pHeader, pMessage* pTailer) {
    char bufLine[BUFFER_SIZE];
    char linePrompt[BUFFER_SIZE];
    char key, name[BUFFER_SIZE], libName[BUFFER_SIZE], libPath[BUFFER_SIZE], func[BUFFER_SIZE];

    FILE *calConf = fopen(CAL_CONF_PATH, "r");
    if( NULL == calConf ){
        perror("Open file cal.conf failed!");
        exit(EXIT_FAILURE);
    }
    creatMessageTable(pHeader, pTailer);
    int cnt = 0;
    while( fgets(bufLine, BUFFER_SIZE, calConf) != NULL ){
        memset((char*)name, '\0', BUFFER_SIZE);
        memset((char*)libName, '\0', BUFFER_SIZE);
        memset((char*)libPath, '\0', BUFFER_SIZE);
        memset((char*)func, '\0', BUFFER_SIZE);
        memset((char*)linePrompt, '\0', BUFFER_SIZE);
        if( sscanf(bufLine, "%c %s %s %s", &key, name, libName, func) != 4 ){
            fprintf(stdout, "Bad line data: %s in cal.conf\n", bufLine);
            continue;
        }
        char tmp[BUFFER_SIZE];
        sprintf(tmp, "Press %c: %s\n", key, name);
        strcat(cmd_prompt, tmp);
//        printf("Press %c: ", key);
//        printf("%s\n", name);
        libPath[0] = '.'; libPath[1] = '/';
        libPath[2] = 's'; libPath[3] = 'o';
        libPath[4] = '/';                       //路径名称"./so/***"
        strcpy(libPath+5, libName);
        // printf("%c\t%s\t%s\t%s\n", key, name, libPath, func);
        lib = dlopen(libPath, RTLD_LAZY);
        if( NULL == lib){
            fprintf(stdout, "Open %s failed!\n", libPath);
            exit(EXIT_FAILURE);
        }
        pFunc pfunc = (pFunc)dlsym(lib, func);
        if( NULL == pfunc){
            fprintf(stdout, "Find function %s failed!\n", func);
            exit(EXIT_FAILURE);
        }
        cnt++;
        insertMessageTable(pHeader, pTailer, key, pfunc);
    }
    fclose(calConf);
    strcat(cmd_prompt, "Press q: exit\n\0");
//    printf("Press q: exit\n");
}
