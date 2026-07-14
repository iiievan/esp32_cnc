#ifndef _APP_TASKS_H
#define _APP_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

#define INVALID_SOCKET      -1
#define UDP_BUFFER_SIZE     256

void udp_server_task(void *arg);

void planner_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif //_APP_TASKS_H