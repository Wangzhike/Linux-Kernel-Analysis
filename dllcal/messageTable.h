//
// Created by qiuyu_mac on 2016/11/14.
//

#ifndef MESSAGETABLE_H
#define MESSAGETABLE_H

#include "messageTable.h"
#include "calType.h"

struct _message;
typedef struct _message message;
typedef message* pMessage;


void
creatMessageTable(pMessage* pHeader, pMessage* pTailer);
pMessage
insertMessageTable(pMessage* pHeader, pMessage* pTailer, const char ch, pFunc pfunc);
pFunc
searchMessageTable(pMessage* pHeader, pMessage* pTailer, const char ch);
void
deleteMessageTable(pMessage* pHeader, pMessage* pTailer);
static pMessage
creatMessage(const char ch, pFunc pfunc);
static pMessage
insertAsPred(pMessage pcur, const char ch, pFunc pfunc);
static pMessage
insertAsSucc(pMessage pcur, const char ch, pFunc pfunc);
static void
removeMessage(pMessage pcur);

#endif //MESSAGETABLE_H
