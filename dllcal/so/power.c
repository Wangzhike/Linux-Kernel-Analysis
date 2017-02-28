
#include "calType.h"

type_t Power(type_t opnd1, type_t opnd2){
    type_t result = 1;
    while( opnd2-- > 0 ){
        result *= opnd1;
    }
    return result;
}