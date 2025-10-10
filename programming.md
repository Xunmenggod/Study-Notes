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
    
## Algorithm for data structure of STL
- **Vector**
    1. construction:
        ```C++
            #include <vector>
            // empty int vector
            Vector<int> vec; 
            Vector<int> a{1,2,3,4,5}; // Vector<int> a = {1,2,3,4,6};
            Vector<int> zeros(6);
            // int vector contains six 5
            Vector<int> fives(6, 5);
            // part contains {3,4,5}
            Vector<int> part(a.begin()+2, a.end()-1);
        ```
    2. basic operation
        ```C++
            vector<int> vec(7,4);
            // indexing
            cout<<vec[2]<<endl;
            // traverse
            vector::iterator it;
            for (it = vec.begin(); it!=vec.end(); it++)
                cout<<*it<<endl;
            // insert: At index3 insert 8 into the vec
            vec.insert(3, 8);
            vec.erase(3);
            // erase the element in index [begin, end)
            vec.erase(begin, end);
            vec.size();
            vec.clear();
            int first = vec.front();
            int back = vec.pop_back();
            vec.push_back(2);
            vec.resize(new_size);
            bool isEmpty = vec.empty();
        ```
     3. algorithm:
         ```C++
             #include <algorithm>
             reverse(vec.begin(), vec.end());
             sort(vec.begin, vec.end());;
             fill(vec.begin(), vec.end(), val);
             fill_n(vec.begin(), 10, val); // fill ten numbers in the vec from the begining to be val
             // unique will first remove all repetitive elements to the end of the vector
             // and return the first repetitive element index by iterator
             vector<int>::iterator it = unique(vec.begin(), vec.end());
             // using unique to delete all the repetitive elements
             sort(vec.begin(), vec.end());
             vector<int>::iterator it = unique(vec.begin(), vec.end());
             vec.erase(it, vec.end());
         ```
- **set**
    1. basic operation and commonly used function, set is used to store unique element
        ```C++
            #include<set>
            set<char> charSet;
            charSet.empty();
            charSet.size();
            charSet.clear();
            charSet.erase(element);
            charSet.insert(element);
            // check whether the element is inside the set
            bool ret = charSet.count(element);
            // find the element position in the set and return the position iterator
            set<type>::iterator it = charSet.find(element);
        ```
- **Map**
    1.
        ```C++
            
        ```

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

## Eigen C++
```C++
// matrix and vector
Matrixxf a(10,15); // 10x15 matrix
VectorXf(4); // 4X1 vector
// fixed size
Matrix3f a{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
Matrix3f a; // 3x3 matrix
Vector3f b;

// matrix indexing
a_3_2 = a(2,1); // indexing with (row, col) 
b_3 = b[2] = b(2,1); // vector support [], becuz `[]` do not support multi parameter

// Matrix quick assignment
a << 1,2,3,
     4,5,6,
     7,8,9;

// matrix resize
a.resize(5,5);

// special matrix
a = MatrixXf::Zero(5,5) = a.setZero(); 
a = MatrixXf::Identity(5,5) = a.setIdentity();
a = MatrixXf::Random(5,5) = a.setRandom();

// block operation
a.block<2,2>(0,0) = a.block(0,0,2,2); // Starting from 0,0 index, 2x2 sub matrix
row_0 = a.row(0); // entire first row
col_0 = a.col(0); // entire first column
a.topLeftCorner(1,2); // top left 1x2 sub matrix
a.topRightCorner(1,2);
a.BottomLeftCorner(1,2);
a.BottomRightCorner(1,2);
a.topRows(2);
a.middleRows(2);
a.middleCols(2);

// vec segmention
b.segment(1,2); // Starting from index 1, size 2x1 sub vector

// math operation
a.transpose();
a.dot(b);
// cross operations only support 3x3 matrix
a.cross(b);
a *= 2 == (a = a*2);
// a * b is matrix calculation
```

## Docker
### Basic concept: images and container which are class and running instances respectively
### Dockerfile: the file contains all the dependencies and shell command to run the software. The following content shows a example of Dockerfile
```docker
FROM ros:noetic #<base-images>
COPY src_scripts.py des_scripts.py # COPY <src file path relative to the location of Dockerfile> <dest file>
CMD python ./scripts.py
```
### Docker basic commands
- docker images: show all images
- docker ps: show all containders
- docker rmi <image_id/name>: delete docker images
- docker save <docker_image_name>:<tag> -o <output_name>.tar: save docker images to a tar file
- docker load -i <docker_image_tar_file>: load tar file to docker image
- docker commit <container_id> <image_name>:<tag>: commit current containere to a new version docker image
- docker compose up <compose_file>.yml: up a compose service
- docker compose down <compose_file>.yml: down a compose service
### Docker file basic usage
- 

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

### Branch merge and rebase
1. checkout to your own local branch
2. git pull to get all remote branches
3. checkout to main branch
4. merge dev to branch by `git merge dev`
5. check whether there is any conflict and solve it
6. use `git status` to check whether the merge is successful
7. add and commit to indicate the merge operation
8. git push to update the remote repo

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

## Vscode debug related
- Python debug with other launching style
  - use debugpy method by `pip install debugpy` or `uv pip install debugpy`
  - configure the launch.json: 
    ``` bash
        {
            // Use IntelliSense to learn about possible attributes.
            // Hover to view descriptions of existing attributes.
            // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
            "version": "0.2.0",
            "configurations": [
                {
                    "name": "Python Debugger: Remote Attach",
                    "type": "debugpy",
                    "request": "attach",
                    "connect": {
                        "host": "localhost" # the host ip,
                        "port": 5678 # the port you are listening
                    },
                    "pathMappings": [
                        {
                            "localRoot": "${workspaceFolder}",
                            "remoteRoot": "."
                        }
                    ]
                }
            ]
        }
    ```
    - run your script by `python -m debubgpy --listen <host_ip>:<port> --wait-for-client <file_names> [args]`
    - Enable the debugger by using the vscode python debug and select the remote attach for debugging
  