//
// Created by ys on 24. 2. 15.
//

#ifndef RBCANINE_THREADGENERATOR_HPP
#define RBCANINE_THREADGENERATOR_HPP
#include <iostream>
#include <pthread.h>
#include <limits.h>

int generateNrtThread(pthread_t &thread_nrt, void* (*thread_func)(void *), const char* name, int cpuNum, void *arg);
int generateRtThread(pthread_t &thread_rt, void* (*thread_func)(void *), const char* name, int cpuNum, int priority, void *arg);
void timeAddus(struct timespec *t, long us);
int timeCmp(struct timespec *a, struct timespec *b);
int timeDifferentMs(struct timespec *before, struct timespec *after);



#endif //RBCANINE_THREADGENERATOR_HPP
