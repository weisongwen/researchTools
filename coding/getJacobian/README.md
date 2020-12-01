# Get jacobian matrix from factor graph optimization in ceres-solver, dynamic array definition and initialization

## get jacobian in Ceres solver 
- **Solution 1: get jacobian in Ceres solver by evaluate the whole problem**
    
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
    - **Solution 2: get jacobian in Ceres solver by evaluate a single factor**
    
    ```C++
    /* get the analytical Jacobian derived in ProjectionTwoFrameOneCamFactor */
    #if enable_Jacobian_cal_anallitical
    factorlist = factorlist_tmp;
    if(factorlist.size())
    {
        /** set up parameters */
        std::vector<double*> Params = {para_Pose[factorlist.back()->imu_i],
                                        para_Pose[factorlist.back()->imu_j],
                                        para_Ex_Pose[0],
                                        para_Feature[factorlist.back()->feature_index],
                                        para_Td[0]};

        // factorlist.back()->check(Params.data());

        /** set up residuals */
        double ResidualAnalytic[2];

        /** set up jecobians */
        std::vector<int> BlockSizes = factorlist.back()->parameter_block_sizes();
        double **JacAnalytic = new double *[BlockSizes.size()];

        Eigen::Matrix<double, 2, 7, Eigen::RowMajor> JacP1An, JacP2An, JacPexAn;
        Eigen::Matrix<double, 2, 1> JacDepAn, JacTdAn;
        JacAnalytic[0] = JacP1An.data();
        JacAnalytic[1] = JacP2An.data();
        JacAnalytic[2] = JacPexAn.data();
        JacAnalytic[3] = JacDepAn.data();
        JacAnalytic[4] = JacTdAn.data();

        /** query cost functions */
        factorlist.back()->Evaluate(Params.data(), ResidualAnalytic, JacAnalytic);

        /** print results */
        std::cout << "## Analytical cost function ## \nResiduals: " << ResidualAnalytic[0] << " " << ResidualAnalytic[1] << std::endl;
        std::cout << "Jacobians An:\n";
        std::cout << JacP1An << std::endl;
        std::cout << JacP2An << std::endl;
        std::cout << JacPexAn << std::endl;
        std::cout << JacDepAn.transpose() << std::endl;
        std::cout << JacTdAn.transpose() << std::endl;

        Eigen::Matrix<double,2,1> residualMat;
        residualMat << ResidualAnalytic[0], ResidualAnalytic[1];
        Eigen::Matrix<double,6,1> delta_P2;
        Eigen::Matrix<double,2,6> JacP2An_26;
        JacP2An_26 = JacP2An.block<2,6>(0,0);
        // delta_P2 = (JacP2An.transpose() * JacP2An + (JacP2An.transpose() * JacP2An).diagonal()).inverse() * JacP2An.transpose() * residualMat;
        delta_P2 = (JacP2An_26.transpose() * JacP2An_26 ).inverse() * JacP2An_26.transpose() * residualMat;
        std::cout << "delta_P2- > " <<delta_P2 << std::endl;
    }
    #endif // enable_Jacobian_cal_anallitical

    ```
    
## **define states in ceres-solver for factor graph optimization**
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
## **define new dynamic array**
```c++
/* define a array with size a[5][~], this can be very useful in Ceres-solver-based factor graph optimization when the size of state to be optimize is not constant (for example, the RTK calculation in GraphGNSSLib) */
double **parameter_temp = new double *[5]; // five parts
parameter_temp[0] = new double[1 * 7]; // residual vs pose i
parameter_temp[1] = new double[1 * 7]; // residual vs pose j
parameter_temp[2] = new double[1 * 7]; // residual vs extrinsic pose
parameter_temp[3] = new double[1 * 1]; // residual vs feature depth
parameter_temp[4] = new double[1 * 1]; // residual vs Td
factorlist.back()->check(parameter_temp);
```

## operate the vector array via function vector<double*>
```c++
// Factory method to create a CostFunction from a DDpseudorangeVSVConstraint to
// conveniently add to a ceres problem.
static DDpseudorangeVSVDynaCostFunction* Create(DDMeasurement dd_measurement, Eigen::Vector3d base_pos, int keyIndex, std::vector<double*>* state_array, std::vector<double*>* pose_parameter_blocks, std::vector<int*>* ar_state_num) {
    
    DDpseudorangeVSVConstraint* constraint = new DDpseudorangeVSVConstraint(
        dd_measurement, base_pos, keyIndex);
    
    DDpseudorangeVSVDynaCostFunction* cost_function = new DDpseudorangeVSVDynaCostFunction(constraint);
    
    pose_parameter_blocks->clear();
    // double a[5] = {1,2,3,4,5};
    // parameter_blocks->push_back(a);
    // parameter_blocks->push_back(&((*state_array)[keyIndex]));
    // parameter_blocks->push_back(state_array[keyIndex]);
    
    for(int i = 0; i <(keyIndex+1); i++)
    {
        pose_parameter_blocks->push_back((*state_array)[i]);
        cost_function->AddParameterBlock(3 + (*ar_state_num)[i][0]);
    }
    // std::cout << "parameter_blocks.size()-> " << parameter_blocks->size() << std::endl;
    // cost_function->AddParameterBlock(1);
    // cost_function->AddParameterBlock(5);
    
    cost_function->SetNumResiduals(1);
    return (cost_function);
}
```
**with** 
```c++
/* position state array of factor graph */
std::vector<double*> state_array;

/* ambiguity state array of factor graph */
std::vector<double*> ar_state_array;

/* array save the num of ambiguity unknowns for each epoch */
std::vector<int*> ar_state_num;

.......................................................

/* DynamicAutoDiffCostFunction-based cost function */
std::vector<double*> pose_parameter_blocks;
DDpseudorangeVSVConstraint::DDpseudorangeVSVDynaCostFunction* DD_pr_cost_function =
DDpseudorangeVSVConstraint::Create(DD_measurement, base_pose,k,&state_array, &pose_parameter_blocks,  &ar_state_num);
auto IDs = problem.AddResidualBlock(DD_pr_cost_function, loss_function, pose_parameter_blocks); 

```

### Reference
1. [Visualization.cpp in VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/vins_estimator/src/utility/visualization.cpp#L161)
<!-- 2. [Quick Intro to Git and GitHub](https://hplgit.github.io/teamods/bitgit/Langtangen_bitgit_4print.pdf) -->


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)