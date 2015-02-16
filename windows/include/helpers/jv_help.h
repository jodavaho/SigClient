#ifndef _HELPERS_H
#define _HELPERS_H
#include <stdlib.h>
#include <stdio.h>
#include <time.h>


#ifdef WIN32
#define WINDOWS
#else
#include <stdarg.h>
#endif

//Max number of timers: Override this to expand
#ifndef _MAX_TIMERS
#define _MAX_TIMERS 16
#endif


#ifdef WIRITING_MPI
int P, myid, name_len;
char proc_name[MPI_MAX_PROCESSOR_NAME];
#endif

///////////////////////////// HELPERS//////////////////////////
void _mecho(const char* pref, const char* str,const char* fs,int ll, int nl);
void _fmecho(const char* pref, const char* str,const char* fs,int ll, int nl,const char*s,...);

double get_time(const int which);
void start_timer(const int which);
double pause_timer(const int which);
void continue_timer(const int which);
double stop_timer(const int which);
double since_start(const int which);

#define FINFO(s,...) _fmecho("INFO","",__FILE__,__LINE__,0,s,__VA_ARGS__);
#define SINGLE(s) if(myid==0)_mecho("Root",s,__FILE__,__LINE__,1)
#define INFO(s) _mecho("INFO",s,__FILE__,__LINE__,1)
#define WARN(s) _mecho("WARN",s,__FILE__,__LINE__,1)
#define ERR(s) _mecho("ERROR",s,__FILE__,__LINE__,1)
#define CRASH(s) _mecho("FATAL",s,__FILE__,__LINE__,1);_dropout(-1)
#define FATAL_ERROR(s,i) _mecho("FATAL",s,__FILE__,__LINE__,1);_dropout(i)


#endif
