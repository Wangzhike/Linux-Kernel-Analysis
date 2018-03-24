#ifndef _ERR_HANDLERS_H
#define _ERR_HANDLERS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdarg.h>
#include <limits.h>

#define MAXLINE 	2048
/* fatal error handlers */
	/* unrelated to a system call */
extern void err_quit(const char *fmt, ...);
extern void err_quit_err(int error, const char *fmt, ...);
	/* related to a system calll */
extern void err_sys(const char *fmt, ...);
extern void err_sys_exit(const char *fmt, ...);
extern void err_dump(const char *fmt, ...);
/* nonfatal error handlers */
	/* unrelated to a system call */
extern void err_msg(const char *fmt, ...);
extern void err_msg_err(int error, const char *fmt, ...);
	/* related to a system call */
extern void err_ret(const char *fmt, ...);

#endif /* _ERR_HANDLERS_H */
