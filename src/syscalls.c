/* Support files for GNU libc.  Files in the system namespace go here.
   Files in the C namespace (ie those that do not start with an
   underscore) go in .c.  */

#include <_ansi.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>
#include <errno.h>
#include <reent.h>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>

/* Forward prototypes.  */
caddr_t _sbrk(int);

extern unsigned int _heap_start;
extern unsigned int _heap_end;

caddr_t
_sbrk (int incr)
{
  //extern char   end asm ("end");    /* Defined by the linker.  */
  static char * heap_end;
  char *        prev_heap_end;

  if (heap_end == NULL)
    heap_end = (char*)&_heap_start;

  prev_heap_end = heap_end;

  if (heap_end + incr > (char*)(&_heap_end))
    {
      errno = ENOMEM;
      return (caddr_t) -1;
    }

  heap_end += incr;

  return (caddr_t) prev_heap_end;
}

int _kill (int pid, int sig)
{
    (void)(pid);
    (void)(sig);
    return -1;
}

void _exit (int status)
{
    _kill(status,-1);
    while(1);
}

int _getpid (int n)
{
    (void)(n);
    return 1;
}
