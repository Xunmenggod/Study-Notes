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
- rvalue reference is an **alias** of a temporary object/value, eg. `T&& <variable> = <temporary object>`
- rvalue ref must be initialized with temporary object

## Class
- copy construction and copy assignment
```C++
// assuming A is a class with parameter construction
A a(1);
A b(2);
A c = a; // copy construction
c = b; // copy assignment
// initialization 
int a = 1 or int a(1) // they are equal for construction
// assignment 
a = 100 // this is same with the functionality as copy assignment
```
- compiler default generating construction and assignment function rule:
    1. 
    2. 
    3.     
    4.        



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
class A
{
    private:
        T val;
    public:
        A()
        {
            cout<<"constructor"<<endl;
        }
        void print();
}
template <typename T>
void A<T>::print()
{
    // function definition
}
```
- template function or class member function definition should be defined in the header file

## std11 atomic operation
- To use that, we need to add '#include <atomic>'
- Atomic type data type ensures taht there is only one thread could access that shared resource and the access sequence is correct which means it will not lead to deadlock
- Atomic could help avoid using the mutex or other lock to increase the efficiency
- Atomic flag usage example
```C++
#include <iostream>      
#include <atomic>        
#include <thread>         
#include <vector>          
#include <sstream>   

// atomic_flag works like a spin lock
std::atomic_flag lock = ATOMIC_FLAG_INIT; // if there is not initialization, the status for atomic_flag is unspecified; Now is clear status

std::stringstream stream;
 
void append_number(int x) {
    // test and set -> wait for obtain lock access an lock
    while (lock_stream.test_and_set()) {}
    stream << "thread #" << x << '\n';
    // unlock operation
    lock_stream.clear();
}
 
int main ()
{
    std::vector<std::thread> threads;
    for (int i=1; i<=10; ++i)
    {
        threads.push_back(std::thread(append_number,i));
    }
    
    for (auto& th : threads)
    {
        th.join();
    }
    
    std::cout << stream.str();
    return 0;
}
```
- Atomic data type including int, char, bool. double and float does not have the operator like '+='

## CMAKE
- Basic knowlege about the package construction: static(静态库 .a/.lib), dynamic(动态库 .so/.ddl)
- - Linking to static lib is done during building while the dynamic lib is inerted during running stage
- - source file(including .h .c .cpp) are pre-processed for #define, inline based on the compiler optimization. And then we build it to become object file (.o), then we could use other software to pack those object file to static or dynamic lib 
- - example
```shell
gcc -c -fpic test1.c test2.c
gcc -shared test1.o test2.o  -o libtest.so
```
- - 在使用动态库时需要提供相应的头文件以令使用者可以知道那些函数可以调用，所以动态库就相当与头文件中声明的函数的实现，并保证其内容不被看到。

- Basic commands:
Find all required packges of the c++ project
find_packages(...)

set(<variable name> <assignment value>)
- 动态库输出位置设定参数： CMAKE_LIBRARY_OUTPUT_DIRECTORY
- 动态库输出位置设定参数： CMAKE_ARCHIVE_OUTPUT_DIRECTORY

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
### Basic concept: images and container which are class and running instances respectively
### Dockerfile: the file contains all the dependencies and shell command to run the software. The following content shows a example of Dockerfile
```docker
FROM ros:noetic #<base-images>
COPY src_scripts.py des_scripts.py # COPY <src file path relative to the location of Dockerfile> <dest file>
CMD python ./scripts.py
```

### Frequent used docker command
```bash
docker run <contanier-name>
docker build -t <docker image name> <Docker file directory>
docker push/pull # push and pull the docker image from docker hub
docker images # show the existing docker images 
docker kill <running-container-name> # kill running contanier
docker rm <container-name> # rm one or more container 
docker restart <container-name>
docker ps # list all the containers
```


## Git
### Submodule
```C++
Add the submodule to the current project
git submodule add <Remote Repo(http/ssh)>
Commit the submodule to the remote repo of the current project
git commit -m "add submodule xxx"
```
- The whole project could be cloned to local by using `git clone --recursive <repo url>` or we could directly clone the project and run `git submodule init` and `git submodule update`

- If submodule repo is updated to a new version, we need to cd to the submodule and run `git pull origin main`. If you have multiple submodules, you could utilize the command of git submodule foreach. To update all submodules, the command `git submodule foreach 'git pull origin master'`.

- To remove a submodule, we could run `git submodule deinit <submodule project name>` and `git rm <submodule project name>`

### Branches 
- `git branch -vv` serach for the connection between local and remote branches
- `git branch -a` show all the local and remote branches
- `git checkout -b <branch name>` create and switch to the branch
- `git push -u origin <branch name>` push the local branch to the remote and build connection

### Sync between remote and local
1. add and commit the local changes to the local repo
2. git push to the remoet repo
3. if error, stash the commit to the stack by `git stash`
4. git pull
5. handle the conflict to merge 
6. run `git stash pop` to get the modified content and handle the conflict


## GNUPLOT (Scientific Plot)
- ; could be used to seperate different commands in same line
- \ could extend over several input lines by ending each line
- basic commands
```bash
set xrange [<low>:<high>]
set title '<title name>'
set xlable '<lable name>'
set ylabel '<lable name>'
# w stands for with and l for lines, lt for linetype
plot '<data file name>' using <col_num for x>:<col_num for y> w l t '<line title name>'
set datafile seperator # or comma (for csv files)
set key left top # put the legend related stuff to the left top corner
splot is for 3D graph plotting
# save output
set terminal png # or other formats
set output '<file name>.png'
```
- write a gnuplot script with .gp and run it by `gnuplot <file name>.gp`