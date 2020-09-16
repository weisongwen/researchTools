# Some practical matlab command in processing data


- how to read text file, for example 

    ```
    filename = '.\yourfilename.txt';
    [a1,a2,a3...] = textread(filename,'%s%s%s%s%s%s%s%s%s%s%s%s%s','delimiter',',');
    data = [a2,a3,a4];
    
    [data1,data2,data3]=textread('myfiles.txt','%n%n%n','delimiter', ',','headerlines', 1);
     textread中的headerlines指明了跳过几行，1可自由设定
    ```
- how to read excel file, for example
    ```
    data1 = xlsread('tool14');
    a = data1(:,1);
    %%acc_13 = data1(a==13,:);
    gyro_14 = data1(a==14,:);
    ```
- 待定
  ```
  coding...
  ```


### Reference


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)