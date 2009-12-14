 /*
 * MemoryFree.c
 * returns the number of free RAM bytes
 * Modified to include functions to log minimum RAM
*/

#include "WProgram.h"  
#include "MemoryFree.h"

extern unsigned int __data_start;
extern unsigned int __data_end;
extern unsigned int __bss_start;
extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;
int memory_min = 0x7FFF;

int freeMemory()
{
  int free_memory;

  if((int)__brkval == 0)
     free_memory = ((int)&free_memory) - ((int)&__bss_end);
  else
    free_memory = ((int)&free_memory) - ((int)__brkval);

  return free_memory;
}
 
int memoryMin()
{
  return memory_min;
}

void memoryCheck()
{
  int free_memory;

  if((int)__brkval == 0)
     free_memory = ((int)&free_memory) - ((int)&__bss_end);
  else
    free_memory = ((int)&free_memory) - ((int)__brkval);
  if (free_memory < memory_min) memory_min = free_memory;


  return;
}
