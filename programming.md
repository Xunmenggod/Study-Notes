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

## `inline` & `template`
- inline should be added before the function definition instead of declartion
- member function definition inside the class body is regraded as inline function by default
- inline function definition is suggested to put in the header file instead of cpp file
- expression of using template for genetic programming
```C++
template <typename T>
// some functions has type T for parameters or return type,
// some class contains the type T member variables or functions
```
- template function or class member function definition should be defined in the header file
 

## CMAKE
- Basic commands:
Find all required packges of the c++ project
find_packages(...)

set(<variable name> <assignment value>)

Include diretories to let the compiler to find the required header files during building
include_directories(<...>)

Link directories to let the linker to find the required header file during linking
link_directories(<...>)

Declare a C++ library
add_library()

Declare a executable file with source files
add_executable(<executable file name> <source cpp files>)

Link the libraries
target_link_libraries(<executable file name> <library names>)

Set the compile options
target_compile_options(<executable file name> [PRIVATE|PUBLIC|INTERFACE] <option> ...)

Install the packages or libraies to designated destination directory
install(TARGETs <target name>
ARCHIVE DESTINATION <destination direction names>
LIBRARY DESTINATION <destination direction names>
RUNTIME DESTINATION <destination direction names>
)

This command helps to run commands in the terminal beforehand
execute_process(
    COMMAND <terminal commands>
    WORKING_DIRECCTORY <Directory name>
)

if(<boolean expression>)
    <some operations>
endif()

## Docker