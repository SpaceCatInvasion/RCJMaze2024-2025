#pragma once
#include <stdlib.h>
#include <string.h>
struct QueueNode {
	void* value;
	QueueNode* next;
	QueueNode* prev;
};

class Queue {
public:
	void* peek();
	void push(void* val);
	void popAndFree();
	void pop();
	Queue();
	void freeQueue();
	bool isEmpty();
private:
	QueueNode* head;
	QueueNode* tail;
};