# Programming tips
## VS Code keyboard shortcuts
- ctrl+shift+p to open the control command bar
- Multi-Cursor
    1. alt+mouse left button to choose the location of multi-cursors
    2. ctrl+alt+L to choose all locations with same word
    3. ctrl+D to choose locations of same word one by one
    4. press the mouse middle button to select the paragraphs that need to have multi cursors
- Use alt+<-(windows) or ctrl+alt+-(linux) to go back the previous cursor location

## C++ smart pointers (help on handling of dynamically allocated objects)
- **unique pointer**:
    ==Exclusive ownership== of the dynamically allocated object.
    Ensure that there is ==only one pointer== could pointed to the object during runing time.
    Examples
    ```C++
    std::unique_ptr<int> uPtr1(new int(5));
    std::unique_ptr<int> uPtr2 = uPtr2; // compilation error
    std::unique_ptr<int> uPtr3 = std::move(uPtr1); // move the uPtr1 to uPtr2 & uPtr1 = nullptr
    uPtr3.reset(); // release the dynamically allocated memory for new int(5)
    ```
- **shared pointer**:
    ==Shared ownership== of the dynamically allocated object
    Keep track of the number of shared_ptr instance that pointed to the object and ==automatically delete== the object if there is no shared_ptr instance for that object.
    Examples

- **weak pointer**:
    

## lvalues & rvalues
- Temporary object is considered as an **rvalue**, it can't be modified so that it could be hold by a const reference, eg.`int& a = 3; // error`
- const lvalue reference accptes an rvalue eg. `const int& a = 3;`
- rvalue reference is an **alias** of a temporary object/value, eg. T&& <variable> = <temporary object>
- rvalue ref must be initialized with temporary object

## Program tricks (format & multiple nested loop avoidance)
- return the function in advance for reversed if logic and assert() function
```C++
#include <iostream>
#include <assert.h>

#defin ifDebug 1
int main()
{
    // read the file
    File* fp = fopen("nonExistedFile", 'r');
    // debug mode check whether the readed file existed or not,
    // if no, assert() will terminate the program and report the location of failed assertion
    if (isDebug)
        assert(fp);
}
```
 
