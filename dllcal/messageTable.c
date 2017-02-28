//
// Created by qiuyu_mac on 2016/11/14.
//

#include "messageTable.h"
#include <stdlib.h>

struct _message{
    char ch;
    pFunc pfunc;
    struct _message* pred;
    struct _message* succ;
};

//static pMessage header, tailer;
//static int size = 0;

void
creatMessageTable(pMessage* pHeader, pMessage* pTailer){
    pMessage header = creatMessage('\0', NULL);
    pMessage tailer = creatMessage('\0', NULL);
    header->pred = NULL; header->succ = tailer;
    tailer->pred = header; tailer->succ = NULL;
    *pHeader = header;
    *pTailer = tailer;
}

pMessage
insertMessageTable(pMessage* pHeader, pMessage* pTailer, const char ch, pFunc pfunc){
    return insertAsPred(*pTailer, ch, pfunc);
}

pFunc
searchMessageTable(pMessage* pHeader, pMessage* pTailer, const char ch){
    pMessage p = *pHeader;
    while( (p = p->succ) != *pTailer ){
        if( ch == p->ch )
            return p->pfunc;
    }
    return NULL;
}

void
deleteMessageTable(pMessage* pHeader, pMessage* pTailer) {
    pMessage p = (*pHeader)->succ;
    while (p != *pTailer) {
        pMessage q = p;
        p = q->succ;
        removeMessage(q);
    }
}


static pMessage
creatMessage(const char ch, pFunc pfunc){
    pMessage pmsg = (pMessage)malloc(sizeof(message));
    pmsg->ch = ch;
    pmsg->pfunc = pfunc;
    return pmsg;
}

static pMessage
insertAsPred(pMessage pcur, const char ch, pFunc pfunc){
    pMessage pmsg = creatMessage(ch, pfunc);
    //建立前向连接
    pmsg->pred = pcur->pred;
    pcur->pred->succ = pmsg;
    //建立后向连接
    pmsg->succ = pcur;
    pcur->pred = pmsg;
    return pmsg;
}

static pMessage
insertAsSucc(pMessage pcur, const char ch, pFunc pfunc){
    pMessage pmsg = creatMessage(ch, pfunc);
    //建立后向连接
    pmsg->succ = pcur->succ;
    pcur->succ->pred = pmsg;
    //建立前向连接
    pcur->succ = pmsg;
    pmsg->pred = pcur;
    return pmsg;
}

static void
removeMessage(pMessage pcur){
    pcur->pred->succ = pcur->succ;      //建立从前到后的连接
    pcur->succ->pred = pcur->pred;
    free(pcur);
}
