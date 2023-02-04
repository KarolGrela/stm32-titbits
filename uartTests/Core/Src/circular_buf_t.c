/*
 * circular_buf_t.c
 *
 *  Created on: Jan 18, 2023
 *      Author: kgrela1
 */

/*** INCLUDES ***/
#include "circular_buf_t.h"

#include "usart.h"

/*** DEFINITIONS ***/


/*** STATIC FUNCTIONS ***/


/*** FUNCTIONS ***/

/**** INITIALIZERS ****/
// initialize circular buffer
void circular_buf_Init(volatile struct circular_buf_t* buff_instance, volatile unsigned char* buff_ptr, unsigned char buff_size)
{
    buff_instance->buff_ptr = buff_ptr;
    buff_instance->head = 0;
    buff_instance->tail = 0;
    buff_instance->buff_size = buff_size;
    buff_instance->tokenPut = 0;
    buff_instance->tokenPop = 0;
}


/**** CHECK STATE OF BUFFER ****/

// check if buffer is empty (position of head and tail is equal)
char circular_buff_isEmpty(volatile struct circular_buf_t buff_instance)
{
    if(buff_instance.head == buff_instance.tail)
    {
        // is empty
        return 1;
    }
    else
    {
        // is not empty
        return -1;
    }
}

// check if buffer is full (next character for Head == Tail)
char circular_buff_isFull(volatile struct circular_buf_t buff_instance)
{
    if(circular_buff_getNextHeadPos(buff_instance) == buff_instance.tail)
    {
        // is full
        return 1;
    }
    else
    {
        // is not full
        return -1;
    }
}

// get next position of head
unsigned char circular_buff_getNextHeadPos(volatile struct circular_buf_t buff_instance)
{
    if(buff_instance.head < buff_instance.buff_size - 1)
    {
        return buff_instance.head + 1;
    }
    else
    {
        return 0;
    }
}

// get current position of head
unsigned char circular_buf_getHead(struct circular_buf_t buff_instance)
{
    return buff_instance.head;
}

// get next position of tail
unsigned char circular_buff_getNextTailPos(volatile struct circular_buf_t buff_instance)
{
    if(buff_instance.tail < buff_instance.buff_size - 1)
    {
        return buff_instance.tail + 1;
    }
    else
    {
        return 0;
    }
}

// get current position of tail
unsigned char circular_buf_getTail(struct circular_buf_t buff_instance)
{
    return buff_instance.tail;
}

// return current size of buffer
unsigned char circular_buf_CurrentSize(volatile struct circular_buf_t buff_instance)
{
    if(buff_instance.head >= buff_instance.tail)
    {
        return buff_instance.head - buff_instance.tail;
    }
    else
    {
        return buff_instance.buff_size - (buff_instance.tail - buff_instance.head);
    }
}



/**** PUT/POP ****/

// put char to buffer
void circular_buf_putChar(volatile struct circular_buf_t* buff_instance, char input_char)
{
	if(tryPut(buff_instance) == 1)
	{
		if(circular_buff_isFull(*buff_instance) != 1)
		{
			buff_instance->buff_ptr[buff_instance->head] = input_char;
			buff_instance->head = circular_buff_getNextHeadPos(*buff_instance);

			resetToken(&(buff_instance->tokenPut));
		}
		else
		{
			resetToken(&(buff_instance->tokenPut));
			return;
		}
		//resetToken(&(buff_instance->tokenPut));
	}
	else
	{
		#pragma message("TODO: Token in use")
	}
}


// put string to buffer (uses circular_buf_putChar(...) function)
void circular_buf_putStr(volatile struct circular_buf_t* buff_instance, char* input_str)
{
	if(buff_instance->buff_size - circular_buf_CurrentSize(*buff_instance) >= strlen(input_str))
	{
		for (unsigned char i = 0; i < strlen(input_str); i++)
		{
			circular_buf_putChar(buff_instance, input_str[i]);
		}
	}

}

// read and pop char from buffer
unsigned char circular_buf_popChar(volatile struct circular_buf_t* buff_instance)
{
	if(tryPop(buff_instance) == 1)
	{
		if(circular_buff_isEmpty(*buff_instance) != 1)
		{
			char temp = buff_instance->buff_ptr[buff_instance->tail];
			buff_instance->tail = circular_buff_getNextTailPos(*buff_instance);

			resetToken(&(buff_instance->tokenPop));
			return temp;
		}
		else
		{
			resetToken(&(buff_instance->tokenPop));
			return (char)0;
		}
	}
	else
	{
		#pragma message("TODO: Token in use")
		// token already in use - TODO

		return (char)0;
	}
}

// reads entire buffer (uses circular_buf_popChar(...) function)
void circular_buf_readBuffer(volatile struct circular_buf_t* buff_instance, char* output_string)
{
    unsigned char i = 0;
    while(circular_buff_isEmpty(*buff_instance) != 1)
    {
        char local_char = circular_buf_popChar(buff_instance);
        if(local_char != 0)
        {
            output_string[i] = local_char;
        }
        i++;
    }
    output_string[i] = '\0';
}



/*** TOKENS ***/
// used when writing to buffer; checks token - if it is not used, access buffer and set token
unsigned char tryPut(volatile struct circular_buf_t* buff_instance)
{
	// token is not used
	if(buff_instance->tokenPut == 0)
	{
		buff_instance->tokenPut = 1;
		return 1;
	}
	else
	{
		// token already is use
		return 0;
	}
}

// used when reading buffer; checks token - if it is not used, access buffer and set token
unsigned char tryPop(volatile struct circular_buf_t* buff_instance)
{
	// token is not used
	if(buff_instance->tokenPop == 0)
	{
		buff_instance->tokenPop = 1;
		return 1;
	}
	else
	{
		// token already is use
		return 0;
	}
}

// reset token variable - used by tryPut and tryPop
void resetToken(volatile unsigned char* token)
{
	*token = 0;
}

/*** OTHER FUNCTIONS ***/


