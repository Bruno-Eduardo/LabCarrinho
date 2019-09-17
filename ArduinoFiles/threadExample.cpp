// thread example
#include <iostream>       // std::cout
#include <thread>         // std::thread
#include <chrono>         // std::chrono::seconds

void delay(int a){;}
 
void foo(int *handler)
{
  while(1){
    if(*handler == 0){
      std::cout << "Changing handler to 1\n";
      *handler = 1;
      delay(1000);
    }
  }
}

void bar(int *handler)
{
  // do stuff...
  while(1){
    if(*handler == 1){
      std::cout << "Changing handler to 0\n";
      *handler = 0;
      delay(1000);
    }
  }
}

int mainThreadExample() 
{
  int handler = 0;

  //std::thread first (foo, &handler);     // spawn new thread that calls foo()
  //std::thread second (bar, &handler);  // spawn new thread that calls bar(0)
  osThreadDef(fooT, foo, osPriorityNormal, 0, 100);
  osThreadCreate(osThread(fooT), &handler);
  osThreadCreate(osThread(barT), &handler);
  std::cout << "main, foo and bar now execute concurrently...\n";

  while(1){
    std::cout << "keeping alive\n";
    delay(1000);
  }

  return 0;
}
