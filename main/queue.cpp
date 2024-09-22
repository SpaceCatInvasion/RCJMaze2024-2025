#include "queue.h"


void* Queue::peek() {
    if (head == NULL) return NULL;
    return head->value;
}
Queue::Queue() {
    head = NULL;
    tail = NULL;
}
void Queue::push(void* val) {
    QueueNode* n = (QueueNode*)malloc(sizeof(QueueNode));
    if (n == NULL) {
      exit(-1);
    }
    n->value = val;
    n->next = NULL;
    n->prev = NULL;
    if (head == NULL) {
        head = n;
        tail = n;
    }
    else {
        tail->next = n;
        n->prev = tail;
        tail = n;
    }
}

void Queue::popAndFree() {
    if (head == NULL) return;
    if (head->next == NULL) {
        free(head);
        head = NULL;
        tail = NULL;
    }
    else {
        head = head->next;
        free(head->prev);
        head->prev = NULL;
    }
}
void Queue::pop() {
    if (head == NULL) return;
    if (head->next == NULL) {
        head = NULL;
        tail = NULL;
    }
    else {
        head = head->next;
        head->prev = NULL;
    }
}
void Queue::freeQueue() {
    for (QueueNode* n = head; n != NULL;) {
        if (n->next == NULL) {
            free(n); 
            head = NULL;
            tail = NULL;
            return;
        }
        n = n->next;
        free(n->prev);
    }
    head = NULL;
    tail = NULL;
}
bool Queue::isEmpty() {
    return head == NULL;
}