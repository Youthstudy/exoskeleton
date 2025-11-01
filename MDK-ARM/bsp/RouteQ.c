#include "RouteQ.h"

void Cqueue_Init(RouteQ* q){
	memset(q->queue,0,sizeof(q->queue));
	q->head = 0;
	q->tail = 0;
	q->size = 0;
}

void Cqueue_push(RouteQ* q, uint8_t data){
	if(Cqueue_isFull(q)){
		return ;
	}
	q->queue[q->tail] = data;
	q->tail = (q->tail + 1) % MAX_SIZE;
	q->size ++;
}

int Cqueue_pop(RouteQ* q,uint8_t *data){
	if(Cqueue_empty(q)){
		return 0;
	}
	*data = q->queue[q->head];
	q->head = (q->head + 1) % MAX_SIZE;
	q->size --;
	return 1;
}


int Cqueue_empty(RouteQ *q){
	return q->size == 0;
}

int Cqueue_isFull(RouteQ *q) {
    return q->size == MAX_SIZE;
}






