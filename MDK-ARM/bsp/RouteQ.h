#ifndef __ROUTEQ_H__
#define __ROUTEQ_H__

#include "math.h"
#include "stdint.h"
#include <stdio.h>
#include "string.h"


#define MAX_SIZE 1024

typedef struct {
		uint8_t queue[MAX_SIZE];
		int head;
		int tail;
		int size;
}RouteQ;

void Cqueue_Init(RouteQ* q);
void Cqueue_push(RouteQ* q, uint8_t data);
int Cqueue_empty(RouteQ *q);
void Cqueue_pop(RouteQ* q);
uint8_t Cqueue_head(RouteQ* q);

#endif

