/*
 * circular_buf_t.h
 *
 *  Created on: Jan 18, 2023
 *      Author: kgrela1
 *
 *      Implementation of circular buffer structure with access control
 *      FEATURES:
 *      > Two different threads can put and pop bytes to/from buffer (one thread puts and one pops data)
 *      > Access control: only one thread at a time can put data to buffer
 *      > Access control: only one thread at a time can pop data from buffer
 *      > Data can be read from buffer only by popping it -> reading data causes emptying of the buffer
 *      > head - position of first empty byte
 *      > tail - position of first byte that should be read
 *      > head == tail - buffer is empty
 *      > nextHeadPosition == tail - buffer is full
 *      > buffer capacity = buffer size - 1
 *
 *      TODO:
 *      > enumeration for empty/not_empty/full/ buffer
 *      > enumeration for access (tokens)
 *      > function callbacks if tokens are used
 *      > some functions should be static
 *      > token functions - rework
 */

#ifndef SRC_CIRCULAR_BUF_T_H_
#define SRC_CIRCULAR_BUF_T_H_

/*** INCLUDES ***/
#include "string.h"

/*** DEFINES ***/

/*** TYPEDEFS ***/

// circular buffer structure
struct circular_buf_t {
    unsigned char* buff_ptr;    // pointer to buffer
    unsigned char buff_size;    // size of buffer
    unsigned char head;         // head position (next put operation)
    unsigned char tail;         // tail position (next pop operation)
    unsigned char tokenPut;		// token: if == 1 - put is used by another thread
    unsigned char tokenPop;		// token: if == 1 - put is used by another thread
};

/*** FUNCTIONS ***/

/**** INITIALIZERS ****/
// initialize circular buffer
void circular_buf_Init(volatile struct circular_buf_t* buff_instance, volatile unsigned char* buff_ptr, unsigned char buff_size);


/**** CHECK STATE OF BUFFER ****/
// check if buffer is empty (position of head and tail is equal)
char circular_buff_isEmpty(volatile struct circular_buf_t buff_instance);

// check if buffer is full (next character for Head == Tail)
char circular_buff_isFull(volatile struct circular_buf_t buff_instance);

// get next position of head
unsigned char circular_buff_getNextHeadPos(volatile struct circular_buf_t buff_instance);

// get current position of head
unsigned char circular_buf_getHead(volatile struct circular_buf_t buff_instance);

// get next position of tail
unsigned char circular_buff_getNextTailPos(volatile struct circular_buf_t buff_instance);

// get current position of tail
unsigned char circular_buf_getTail(volatile struct circular_buf_t buff_instance);

// return current size of buffer
unsigned char circular_buf_CurrentSize(volatile struct circular_buf_t buff_instance);

/**** PUT/POP ****/
// put char to buffer
void circular_buf_putChar(volatile struct circular_buf_t* buff_instance, char input_char);

// put string to buffer (uses circular_buf_putChar(...) function)
void circular_buf_putStr(volatile struct circular_buf_t* buff_instance, char* input_str);

// read and pop char from buffer
unsigned char circular_buf_popChar(volatile struct circular_buf_t* buff_instance);

// reads entire buffer (uses circular_buf_popChar(...) function)
void circular_buf_readBuffer(volatile struct circular_buf_t* buff_instance, char* output_string);

/*** TOKENS ***/
// used when writing to buffer; checks token - if it is not used, access buffer and set token
unsigned char tryPut(volatile struct circular_buf_t* buff_instance);

// used when reading buffer; checks token - if it is not used, access buffer and set token
unsigned char tryPop(volatile struct circular_buf_t* buff_instance);

// reset token variable - used by tryPut and tryPop
void resetToken(volatile unsigned char* token);

/*** OTHER FUNCTIONS ***/






#endif /* SRC_CIRCULAR_BUF_T_H_ */
