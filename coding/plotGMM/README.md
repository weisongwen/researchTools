# Plot GMM in MATLAB based on Residuals 

## [GMM parameters estimation using Expectation Maximization](https://brilliant.org/wiki/gaussian-mixture-model/)

- file error_im_all.csv
    - the residuals from LiDAR super-resoluion for paper T-VT
- file SR_residual.m
    - MATLAB file for plotting/fitting Gaussian Mixture model
    ```
    REAL_DATA = 1; // using simulated data or real data
    histSize = 0.3; // bin size or histogram size
    ```
- Be noted that the MATLAB requirement 
```
'fitgmdist' requires Statistics and Machine Learning Toolbox.
```

### Reference
1. [(slides)](https://www.ipb.uni-bonn.de/wp-content/uploads/2018/05/lecture_1.pdf))
2. [Google Code Styleguide](https://google.github.io/styleguide/cppguide.html)
3. [C++ tutorials](http://www.cplusplus.com/doc/tutorial/)


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)