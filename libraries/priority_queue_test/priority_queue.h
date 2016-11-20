/**
 * priority_queue.h
 * Contains various methods for using the priority queue.
 */

#ifndef priority_queue_h
#define priority_queue_h

// define node to contain state and value
typedef struct node
{
    // row, col, orientation as a number from 0-1023
    uint16_t state;
    // distance from current
    uint8_t value;
}node;

class priority_queue
{
    public:
        // Constructor. 
        priority_queue();
          
        // main functions for use
        void insert(node cell);
        node pop();
        void update(uint16_t state, uint8_t new_value);
        
    private:
        // Global map to record examined nodes/index in priority queue
        // 0 never in queue, 255 is examined, 1-127 is index in priority queue
        uint8_t heap_map[1024]={0};
        // Global min heap to act as priority queue
        node min_heap[128];
        int heap_size = 0;
        void bubble_up(int index);
        void bubble_down(int index);
        void swap(int index1, int index2);
};

#endif
