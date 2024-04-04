/*
 * command_queue.h
 *
 *  Created on: Jan 9, 2024
 *      Author: gvigelet
 */

#ifndef INC_COMMAND_QUEUE_H_
#define INC_COMMAND_QUEUE_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t* buffer;        // Pointer to the byte buffer
    uint32_t size;          // Maximum size of the queue
    uint32_t front;         // Front index for dequeueing
    uint32_t rear;          // Rear index for enqueueing
    uint32_t count;         // Number of elements in the queue
} CommandQueue;

void CommandQueue_Init(CommandQueue* queue, uint8_t* buffer, uint32_t size);
bool CommandQueue_IsEmpty(const CommandQueue* queue);
bool CommandQueue_IsFull(const CommandQueue* queue);
bool CommandQueue_Enqueue(CommandQueue* queue, uint8_t data);
bool CommandQueue_Dequeue(CommandQueue* queue, uint8_t* data);



#endif /* INC_COMMAND_QUEUE_H_ */
