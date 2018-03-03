#ifndef _MYLS_H
#define _MYLS_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "err_handlers/err_handlers.h"

/*
 * options_t struct is used to store options parsed by getopt.
 * For each option parsed, the corresponding flag is set.
 */
typedef struct options_t {
    int flag_a;     //all, do not ignore entries starting with .
    int flag_d;     //list directories instead of contents
    int flag_l;     //use a long listing format
    int flag_R;     //list subdirectories recursively
    int flag_S;     //sort by file size, largest first
    int flag_t;     //sort by modification time, newest first
} options_t;

int myls(options_t *opt_t, char **args, int n_args);

#endif /* _MYLS_H */
