# CPP-02 Modern C++:Compilation, Debugging, Functions, Headers/Source, Libraries, Cmake (2018,Igor

## Notes during the course [(slides)](https://www.ipb.uni-bonn.de/wp-content/uploads/2018/05/lecture_2.pdf)

- compilation flags and debugging 
    - Enable all warnings, treat them as errors:
        - -Wall, -Wextra, -Werror
    - Warning may not be a syntax error, but can be a potential error. therefore, fix the warnings as well 
    - debugging tools gdb
        - the best option is to use the gdb 
        - insanely popular and powerful
        - install the gdb GUI to debug the code as you want
- functions in C++
    - code can be organized into functions
    - functions create a scope
    - single return type from a function 
    - any number of input variables of any types
    - do one thing, name show what the function does 
    - write small function and have a good name 
    - do not return an reference of a local variable int& sum(int a, int b);
    - simple example
    ```
    #include <vector>
    using namespace std; //
    vector<int> CreateVectorOfFullSquares(int size){
        vector<int> result(size); // vector of size 'size'
        for(int i = 0; i < size; i++){ result[i] = i * i;}
        return result;
    }
    int main(){
        auto squares = CreateVectorOfFullSquares(10);
        return 0;
    }
    ```
    - declare your function before you use them
    - function **definition** holds the implementation of the function that can even be hidden from the users
    - passing big objects 
        - by default in C++, objects are copied when pass into functions
        - pass by reference to avoid copy  
        ```
        void DoSmth(std::string huge_string);
        void DoSmth(std::string& huge_string);
        ```
    - use const reference: void  DoSmth(const std::string huge_string)
        - non-const refs are mostly used in older code written before C++11

    - function overload
        - compilers infers a function from arguments
        - cannot overload based on return type 
        - return types plays no roles at all
        ```
        #include <iostream>
        #incldue <string>
        using namespace std;
        string Func(int num){ return "int";}
        string Func(const string& str){ return "string";}
        int main {
            cout << Func(1) <<endl;
            cout << Func("Hello") <<endl;
            return 0;
        }
        ``` 
    - default arguements
        - only use them when readability is much better
    - **do not reinvent the wheel**
        - when using the std::vector, std::array, etc. try to avoid writing your own functions 
        - use the #include <algorithm>
        ```
        std::vector<float> v;
        std::sort(v.begin(), v.end); // sort by ascending
        float sum = std::accumulate(v.begin(), v.end(),0.0f);
        float products = std::accumulate(v.begin(), v.end(),1.0f, std::multiplies<float>());
        ```
    - header/source seperation 
        - move all declareds to header files (*.h)
        - implementation goes to *.cpp or *.cc
    - how to build this?
        - the detail of this part can be found in the given link 


### Reference
1. [(slides)](https://www.ipb.uni-bonn.de/wp-content/uploads/2018/05/lecture_2.pdf)
2. [Google Code Styleguide](https://google.github.io/styleguide/cppguide.html)
3. [C++ tutorials](http://www.cplusplus.com/doc/tutorial/)


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)