// include non system dependent data types
#include <Arduino.h>
#include "priority_queue.h"

priority_queue::priority_queue()
{
    // Global map to record examined nodes/index in priority queue
    // 0 never in queue, 255 is examined, 1-127 is index in priority queue
    uint8_t heap_map[1024]={0};
    
    // Global min heap to act as priority queue
    node min_heap[128];
    int heap_size = 0;
}

// main functions: insert, pop 
void priority_queue::insert(node cell)
{
    if(heap_size == 128)
    {
        // error for full min_heap
        Serial.print("heap is full!");
    }
    // put new node in
    min_heap[heap_size + 1] = cell;
    // record this in heap map
    heap_map[cell.state]=heap_size+1;
    // swap new node up to correct position, increment size
    bubble_up(heap_size+1);
    heap_size++;
}

node priority_queue::pop()
{
    if(heap_size==0)
    {
        // impossible node to signify error
        return (node){1024, 0};
    }
    // remember top node
    node popped = min_heap[1];
    // record examined value
    heap_map[min_heap[1].state] = 255;
    // replace top node with last value, decrement size
    min_heap[1] = min_heap[heap_size];
    heap_size--;
    // record value that was moved
    heap_map[min_heap[heap_size].state] = 1;
    // swap last value down to correct position, return min value
    bubble_down(1);
    return popped;
    
}

// sub functions: bubble_up, bubble_down
// takes index of heap element and recursively swaps it up the tree to proper position
void priority_queue::bubble_up(int index)
{
    int parent = index/2;
    // done if at top node value or greater or equal to parent
    if(index == 1 || min_heap[index].value >= min_heap[parent].value)
    {
        return;
    }
    // swap node with parent, and continue
    swap(index, parent);
    bubble_up(parent);
}

// takes index of heap element and recursively swaps it down the tree to proper position
void priority_queue::bubble_down(int index)
{
    int child1 = index*2;
    int child2 = index*2 + 1;
    // done if at bottom or less or equal to both children
    if(index*2 > heap_size || (min_heap[index].value <= min_heap[child1].value && min_heap[index].value <= min_heap[child2].value))
    {
        return;
    }
    // swap node with smaller child, and continue
    if(min_heap[child1].value < min_heap[child2].value)
    {
        swap(index, child1);
        bubble_down(child1);
    }
    else
    {
        swap(index, child2);
        bubble_down(child2);
    }
}

// sub-sub functions: swap
// switch two nodes and record the change
void priority_queue::swap(int index1, int index2)
{
    // record swap in heap_map first
    heap_map[min_heap[index1].state] = index2;
    heap_map[min_heap[index2].state] = index1;
    // do the actual swap
    node temp = min_heap[index1];
    min_heap[index1] = min_heap[index2];
    min_heap[index2] = temp;
}

// compare a given value to that of a target cell, and update if new value is less
void priority_queue::update(uint16_t state, uint8_t new_value){
    int index = heap_map[state];
    if(index == 0){
        insert((node){state, new_value});
    }
    else if(new_value < min_heap[index].value){
        min_heap[index].value = new_value;
    }
}
