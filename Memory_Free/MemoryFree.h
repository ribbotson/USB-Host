// memoryFree header
// added check for minimum RAM memory

#ifndef	MEMORY_FREE_H
#define MEMORY_FREE_H

#ifdef __cplusplus
extern "C" {
#endif

int freeMemory();
int memoryMin();
void memoryCheck();

#ifdef  __cplusplus
}
#endif

#endif
 

