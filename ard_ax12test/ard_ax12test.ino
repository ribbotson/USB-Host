#include <ard_ax12.h>

// AX12 test
// 



 
void setup() 
{ 
  
  ax12Init(1000000);  
}
 
void loop() 
{ 
  int position;
  position = GetPosition(6);
  SetPosition(1,position);
  delay(50);
} 
