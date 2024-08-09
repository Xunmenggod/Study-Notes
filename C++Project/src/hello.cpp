#include "hello.h"

void HelloBot::sayHello()
{
    cout<<"Hello bot is saying hello to you!!!!"<<endl;
}

int main()
{
    HelloBot myBot;
    myBot.sayHello();
    return 0;
}
