# Get jacobian matrix from factor graph optimization in ceres-solver

## get jacobian
- **Solution 1**
    
    ```C++
    #if enable_Jacobian_cal
    TicToc t_jac_cal;
    /** set basic options*/
    ceres::Problem::EvaluateOptions Options;
    Options.apply_loss_function = true;
    Options.num_threads = 8;

    /** set the states and factors you whish to evaluate (you can skipp this if you want )*/
    Options.residual_blocks = psrIDs;
    Options.parameter_blocks = state_array;

    /** compute jacobian*/
    ceres::CRSMatrix JacobianCRS;
    problem.Evaluate(Options, nullptr, nullptr, nullptr, &JacobianCRS);

    /** convert to eigen matrix (only for small dense problems, for bigger problems you better use a sparse matrix)*/
    Eigen::MatrixXd  Jacobian;
    CRSToMatrix(JacobianCRS, Jacobian);

    std::cout << "psrIDs.size() = " << psrIDs.size() << std::endl;
    std::cout << "Jacobian-> " << Jacobian.rows() << std::endl;
    std::cout << "Jacobian-> " << Jacobian.cols() << std::endl;
    std::cout << "t_jac_cal.toc()-> " << t_jac_cal.toc() << std::endl;

    #endif

    ```
    
- **define states in ceres-solver for factor graph optimization**
    **solution 1**
    ```C++
    double state_array[length][5]; 
    ...
    for(int i = 0;  i < length; i++,iter++) // initialize
    {
        nlosExclusion::GNSS_Raw_Array gnss_data = (iter->second);
        Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare(
                                    m_GNSS_Tools.getAllPositions(gnss_data),
                                    m_GNSS_Tools.getAllMeasurements(gnss_data),
                                    gnss_data, "WLS");
        // state_array[i][0] = eWLSSolutionECEF(0);
        // state_array[i][1] = eWLSSolutionECEF(1);
        // state_array[i][2] = eWLSSolutionECEF(2);
        // state_array[i][3] = eWLSSolutionECEF(3);
        // state_array[i][4] = eWLSSolutionECEF(4);

        state_array[i][0] = 0;
        state_array[i][1] = 0;
        state_array[i][2] = 0;
        state_array[i][3] = 0;
        state_array[i][4] = 0;

        problem.AddParameterBlock(state_array[i],5);
    }
    ```

    **solution 2**
    ```C++
    int length = gnss_raw_map.size();
    std::vector<double*> state_array;

    state_array.reserve(length);
    for(int i = 0; i < length;i++)
    {
        // state_array[i] = calloc(5, sizeof(double));
        state_array[i] = new double[5];
    }

    std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter;
    iter = gnss_raw_map.begin();
    for(int i = 0;  i < length; i++,iter++) // initialize
    {
        nlosExclusion::GNSS_Raw_Array gnss_data = (iter->second);
        Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare(
                                    m_GNSS_Tools.getAllPositions(gnss_data),
                                    m_GNSS_Tools.getAllMeasurements(gnss_data),
                                    gnss_data, "WLS");
        // state_array[i][0] = eWLSSolutionECEF(0);
        // state_array[i][1] = eWLSSolutionECEF(1);
        // state_array[i][2] = eWLSSolutionECEF(2);
        // state_array[i][3] = eWLSSolutionECEF(3);
        // state_array[i][4] = eWLSSolutionECEF(4);

        state_array[i][0] = 0;
        state_array[i][1] = 0;
        state_array[i][2] = 0;
        state_array[i][3] = 0;
        state_array[i][4] = 0;

        problem.AddParameterBlock(state_array[i],5);
    }

    }
    ```

### Reference
1. [Visualization.cpp in VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/vins_estimator/src/utility/visualization.cpp#L161)
<!-- 2. [Quick Intro to Git and GitHub](https://hplgit.github.io/teamods/bitgit/Langtangen_bitgit_4print.pdf) -->


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)