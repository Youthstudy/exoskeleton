#include "RouteQ.h"

void Cqueue_Init(RouteQ* q){
	memset(q->queue,0,sizeof(q->queue));
	q->head = 0;
	q->tail = 0;
	q->size = 0;
}


void Cqueue_push(RouteQ* q, uint8_t data){
	if(q->size >= MAX_SIZE){
		return ;
	}
	q->queue[q->tail] = data;
	q->tail = (q->tail ++)%MAX_SIZE;
	q->size ++;
}

void Cqueue_pop(RouteQ* q){
	if(q->size == 0){
		return ;
	}
	q->head = (q->head ++)%MAX_SIZE;
	q->size --;
}


int Cqueue_empty(RouteQ *q){
	return q->size? 0 : 1;
}

uint8_t Cqueue_head(RouteQ* q){
	return q->queue[q->head];
}




