#include <iostream>
#include <vector>
int main(){
    // std::cerr<<"hello world \n"<<std::endl;
    // std::vector<int> vec ={1, 2};
    // vec.push_back(3.1);
    // std::cout<<vec.front()<< "  "<< vec.back()<<std::endl;  

    int num = 42;
    int& ref = num;
    const int& kRef = num;
    ref = 0;
    std::cout<< ref << " "<< num << " "<< kRef << std::endl;
    num = 42;
    std::cout<< ref << " "<< num << " "<< kRef << std::endl;
    return 0;
}