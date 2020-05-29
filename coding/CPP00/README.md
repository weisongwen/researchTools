# CPP-00 Modern C++: course introduction and hello world

## Notes during the course [(slides)](https://www.ipb.uni-bonn.de/wp-content/uploads/2018/05/lecture_0.pdf)
- **structure of typical linux commands**
    - pdfpc -h 
    - sudo kill -9 <pid> # use the htop to see the pid
    - ctrl+R <query>: search in history
    - sudo apt-get update: to update information about available packages
    - sudo apt-get install <programme> : to install the programme that you want 


- ** C++**
    - we won't teach you everything about C++ programming, C++ is used by numerous companies such as google, amazon, and other companies
    - where to write code: there are two operations 
        - use the C++ IDEs
            - Clion 
            - Qt creator
            - Eslipse 
        - use the C++ text editors
            - sublime editor
            - VS code 
            - VIM     
    - comments start with "//" or /* comments */
        - on ine line follows the //
    - good code style is important
        - recommend to use the clang_format your code 
        - use the  cpplint to check your style 
        - following a style guide will save your time and make the code more reliable 
        - we recommend to use the Google Code Style Sheets
    - #incldue directive
        - #include <file> -system include files
        #include "file"  --local include files
    - I/O streams for simple input and output
        - #inlcude <iostreams>
    - compile and run hello world 
        - we understand **text**
        - computer understand **machine code**
        - compilation is translation fropm text to machine code
        - **compilers** we can use on Linux
            - GCC 
            - Clang
        - compile and run hello world example
            - c++ -std=c++11 -o hello_world hello_world.cpp
            - ./hello_world


### Reference
1. [(slides)](https://www.ipb.uni-bonn.de/wp-content/uploads/2018/05/lecture_0.pdf)
2. [Google Code Styleguide](https://google.github.io/styleguide/cppguide.html)
3. [C++ tutorials](http://www.cplusplus.com/doc/tutorial/)


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)