#ifndef _PATH_ALLOC_H
#define _PATH_ALLOC_H

#include <errno.h>
#include <unistd.h>
#include <limits.h>
#include "/home/qiuyu/code/err_handlers/err_handlers.h"

extern char *path_alloc(size_t *sizep);

#endif	/* _PATH_ALLOC_H */
