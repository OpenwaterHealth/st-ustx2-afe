/*
 * command_queue.c
 *
 *  Created on: Jan 9, 2024
 *      Author: gvigelet
 */

#include "command_queue.h"

void CommandQueue_Init(CommandQueue* queue, uint8_t* buffer, uint32_t size) {
    queue->buffer = buffer;
    queue->size = size;
    queue->front = 0;
    queue->rear = 0;
    queue->count = 0;
}

bool CommandQueue_IsEmpty(const CommandQueue* queue) {
    return (queue->count == 0);
}

bool CommandQueue_IsFull(const CommandQueue* queue) {
    return (queue->count == queue->size);
}

bool CommandQueue_Enqueue(CommandQueue* queue, uint8_t data) {
    if (CommandQueue_IsFull(queue)) {
        return false; // Queue is full, cannot enqueue
    }
    queue->buffer[queue->rear] = data;
    queue->rear = (queue->rear + 1) % queue->size;
    queue->count++;
    return true; // Enqueue successful
}

bool CommandQueue_Dequeue(CommandQueue* queue, uint8_t* data) {
    if (CommandQueue_IsEmpty(queue)) {
        return false; // Queue is empty, cannot dequeue
    }
    *data = queue->buffer[queue->front];
    queue->front = (queue->front + 1) % queue->size;
    queue->count--;
    return true; // Dequeue successful
}
