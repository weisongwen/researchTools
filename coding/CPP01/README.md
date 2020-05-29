# CPP-01 Modern C++:Variables, Basic types, Control structure (2018,Igor)

## Notes during the course [(slides)](https://www.ipb.uni-bonn.de/wp-content/uploads/2018/05/lecture_1.pdf)

- naming variables
    - name varianbles with meaning 
    - do not be afrad of using long names
        - bool this_is_fun = false; 
        - char current_return = '\n';
        - auto number = 10;
    - operation on arithmetic types
        - a +=1 equals to a = a +1 ; same for -=, *=, /=, etc 
    - Avoid == for floating point types
    - do not use the de- increment operations within another expression, i,e. a = (a++) + ++b
    - % is modulo divisor i.e. 7 / 3 == 1
    - string with #incldue <string> to use the std::string
    - concatenate strings with + 
    - check if std is empty with stt.empty()
    - works out of the box with I/O streams
- std::array
    - use the std::array for fixed size collections of items
    - #incldue <array> to use the std:::array
    - store a collection of items of same type.  
    - remove all elements: arr.clear()
    - number of stored items: arr.size()
    - useful access to alis: arr.front() arr.back()
- std:vector
    - #include <vector>
    - add a new item in one of the two ways 
        - vec.emplace_back(value)  (prefered in C++11)
        - vec.push_back(value)  [historically better known]
    - use it! it is fast and flexible
    - optimize vector resizing 
        - many push_back operation force vector to change its size many times 
        - reserve(n) ensures that the vector has enough memory to store n
        - this is an very important optimization
    - solve the error from the button after compile
- variable live in scopes
    - there is a single global scope
    - all variables belongs to the scope where they have been declared 
    - all variables die in the end of their scope
    - any variable can be constant 
        - use the const to declare a const 
        - use the **const** to make the variable as constant, in google style, name constants starting with a smaller letter **k**
- reference to variables 
    - we can add a reference to any variables 
    - use the **&** to state that a variable is a reference 
        - float& ref = original_variable
        - std::string& hello_ref = hello;
        - in fact, both the origianl vairable and the ref one are exactly the same 
        - if one wants to copy large data, the reference can be a good option which is pretty cheap
        - const with reference
            - int num = 42; int& ref = num; const int& kRef = num;
- switch statement 
    - used to conditionally execute code 
    - can have many case statement 
    - **break** exits the switch block
- while loop 
    - usually used when the exact number of iterations is unknown before-wise
    - easy to form an endless loop by mistake  
- for loop 
    - in C++ the for loop is very fast, use them!
    - less flexible than while but less error-prone
    - use for when the number of iterations is fixed and while otherwise
- loop range 
    - for(float num:vec); for(const auto& num:vec);
- exit loops and iterations
    - use the break to **exit** the loop 
    - use the **continue** to skip to the next iteration 

### Reference
1. [(slides)](https://www.ipb.uni-bonn.de/wp-content/uploads/2018/05/lecture_1.pdf))
2. [Google Code Styleguide](https://google.github.io/styleguide/cppguide.html)
3. [C++ tutorials](http://www.cplusplus.com/doc/tutorial/)


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)