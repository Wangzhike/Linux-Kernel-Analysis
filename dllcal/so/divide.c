//
// Created by qiuyu_mac on 2016/11/15.
//

#include "calType.h"
#include <stdio.h>

type_t Divide(type_t opnd1, type_t opnd2){
    if(opnd2 != 0){
        return (type_t)((float)opnd1 / opnd2);
    }else{
        printf("除法运算的除数不能为0！\n");
        return (type_t)0;
    }
}

