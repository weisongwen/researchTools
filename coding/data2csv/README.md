# Writing data to CSV in C++

## Using fopen in C++ stdio.h
- **Solution 1**
    **initialize variable**
    ```
    FILE* groundTruth  = fopen( "../remoteSensing2020/src/result/groundTruth2.csv", "w+");
    ```
    groundTruth should be delared globally to gradually push back data into the csv.
    
    **write data into the folder gradually**
    ```
    fprintf(groundTruth, "%d ,%3.2f ,%3.2f  \n", count1, enu1(0), enu1(1));
    fflush(groundTruth); // make sure the data is saved
    ```
- **Solution 2**
    **initialize variable**
    ```
    // clean the file path
    std::ofstream fout1(groundTruthFolder, std::ios::out);
    fout1.close();

    // initialize the file path
    ofstream foutGt(groundTruthFolder, ios::app);
    foutGt.setf(ios::fixed, ios::floatfield);

    // set the precision of data storage, very convenient
    foutGt.precision(10);
    foutGt << time_now<< " ";
    foutGt.precision(5);
    foutGt << enu1(0) << " " // using space to separate the value, original: ","
        << enu1(1) << " "
        << enu1(2) << " "
        << gt_q[0] << " "
        << gt_q[1] << " "
        << gt_q[2] << " "
        << gt_q[3] << endl;
    // close the file 
    foutGt.close();
    ```
    groundTruth should be delared globally to gradually push back data into the csv.
    
    **write data into the folder gradually**
    ```
    fprintf(groundTruth, "%d ,%3.2f ,%3.2f  \n", count1, enu1(0), enu1(1));
    fflush(groundTruth); // make sure the data is saved
    ```

### Reference
1. [Visualization.cpp in VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/vins_estimator/src/utility/visualization.cpp#L161)
<!-- 2. [Quick Intro to Git and GitHub](https://hplgit.github.io/teamods/bitgit/Langtangen_bitgit_4print.pdf) -->


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)