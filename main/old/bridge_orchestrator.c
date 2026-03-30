// bridge_orchestrator.c

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_TASKS 10
#define QUEUE_SIZE 10

typedef struct {
    int id;
    char name[20];
} Task;

typedef struct {
    Task* tasks[MAX_TASKS];
    int front;
    int rear;
    int count;
} TaskQueue;

TaskQueue* createQueue() {
    TaskQueue* queue = (TaskQueue*)malloc(sizeof(TaskQueue));
    queue->front = 0;
    queue->rear = 0;
    queue->count = 0;
    return queue;
}

int isFull(TaskQueue* queue) {
    return queue->count == QUEUE_SIZE;
}

int isEmpty(TaskQueue* queue) {
    return queue->count == 0;
}

void enqueue(TaskQueue* queue, Task* task) {
    if (!isFull(queue)) {
        queue->tasks[queue->rear] = task;
        queue->rear = (queue->rear + 1) % QUEUE_SIZE;
        queue->count++;
    } else {
        printf("Queue is full\n");
    }
}

Task* dequeue(TaskQueue* queue) {
    if (!isEmpty(queue)) {
        Task* task = queue->tasks[queue->front];
        queue->front = (queue->front + 1) % QUEUE_SIZE;
        queue->count--;
        return task;
    } else {
        printf("Queue is empty\n");
        return NULL;
    }
}

void executeTask(Task* task) {
    printf("Executing task: %s\n", task->name);
    // Add task execution logic here
}

int main() {
    TaskQueue* queue = createQueue();
    
    // Add tasks to the queue
    for (int i = 0; i < 5; i++) {
        Task* task = (Task*)malloc(sizeof(Task));
        task->id = i;
        snprintf(task->name, sizeof(task->name), "Task-%d", i);
        enqueue(queue, task);
    }
    
    // Execute tasks
    for (int i = 0; i < 5; i++) {
        Task* task = dequeue(queue);
        if (task != NULL) {
            executeTask(task);
            free(task);
        }
    }
    
    free(queue);
    return 0;
}