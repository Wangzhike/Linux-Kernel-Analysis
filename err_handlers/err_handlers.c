#include "err_handlers.h"

static void err_handle(int errnoflag, int error, const char *fmt, va_list ap);

/*
 * Fatal error unrelated to a system call.
 * Print a message and terminate.
 */
void err_quit(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	err_handle(0, 0, fmt, ap);
	/* don't need call macro va_end()! */
	exit(EXIT_FAILURE);
}

/*
 * Fatal error unrelated to a system call.
 * Error code passed as explicit parameter.
 * Print a message and terminate.
 */
void err_quit_err(int error, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	err_handle(1, error, fmt, ap);
	/* don't need call macro va_end()! */
	exit(EXIT_FAILURE);
}

/*
 * Fatal error related to a system call.
 * Print a message and terminate.
 */
void err_sys(const char* fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	err_handle(1, errno, fmt, ap);
	/* don't need call macro va_end()! */
	exit(EXIT_FAILURE);
}

/*
 * Fatal error relate to a system call.
 * Print a message and terminate with _exit.
 */
void err_sys_exit(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	err_handle(1, errno, fmt, ap);
	/* don't need call macro va_end()! */
	_exit(EXIT_FAILURE);
}

/*
 * Fatal error related to a system call.
 * Print message, dump core, and terminate.
 */
void err_dump(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	err_handle(1, errno, fmt, ap);
	/* don't need call macro va_end()! */
	abort();	/* dump core and terminate */
	exit(EXIT_FAILURE);
}

/* 
 * Nonfatal error related to a system call.
 * Print a message and return.
 */
void err_ret(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	err_handle(1, errno, fmt, ap);
	/* don't need call macro va_end()! */
}

/*
 * Nonfatal error unrelated to a system call.
 * Print a message and return.
 */
void err_msg(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	err_handle(0, 0, fmt, ap);
	/* don't need call macro va_end()! */
}

/*
 * Nonfatal erron unrelated to a system call.
 * Error code passed as explicit parameter.
 * Print a message and return.
 */
void err_msg_err(int error, const char *fmt, ...)
{
	va_list ap;
	
	va_start(ap, fmt);
	err_handle(1, error, fmt, ap);
	/* don't need call macro va_end()! */
}

/*
 * Print a message and return to caller.
 * Caller specifies "errnoflag".
 */
static void err_handle(int errnoflag, int error, const char *fmt, va_list ap)
{
	char buf[MAXLINE];
	vsnprintf(buf, MAXLINE-1, fmt, ap);
	if (errnoflag)
		snprintf(buf+strlen(buf), MAXLINE-strlen(buf)-1, " : %s", strerror(error));
	strcat(buf, "\n");
	fflush(stdout);		/* in case stdout and stderr are the same */
	fputs(buf, stderr);
	fflush(NULL);		/* flushes all stdio output streams */
}
