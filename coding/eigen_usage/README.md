# Usage of eigen for estimator, rotation, matrix

## C++ Array to eigen matrix 
    
```C++
// Covariance.
Eigen::Matrix<double, 15, 15> cov;
...
gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());
```

## absolute square, max values in eigen matrix 

```C++
/* Compute mean and std of the imu buffer*/
Eigen::Vector3d sum_acc(0., 0., 0.);
for (const auto imu_data : imu_buffer_) {
    sum_acc += imu_data->acc;
}
const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buffer_.size();

Eigen::Vector3d sum_err2(0., 0., 0.);
for (const auto imu_data : imu_buffer_) {
    sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
}
const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buffer_.size()).cwiseSqrt();

if (std_acc.maxCoeff() > kAccStdLimit) {
    LOG(WARNING) << "[ComputeG_R_IFromImuData]: Too big acc std: " << std_acc.transpose();
    return false;
}
```

## using matrix in armadillo
-  the **null** function in armadillo is fast and stable. The eigen seems does not provide such funcions.
        
    ```C++
    /* generate vector with all the elements equals to one */
    Eigen::MatrixXd onz;
    onz.resize(matSize, 1);
    for(int i = 0; i < matSize; i++)
    {
    onz(i,0) = 1;
    }

    /* get the null space using armadillo */
    /* Step 1: calculate null space in armadillo */
    arma::mat onzArma = arma::mat(onz.data(), onz.cols(), onz.rows(),
                            false, false);
    // onzArma.print("onzArma:"); 
    // std::cout<<"onzArma.size() -> \n" << onzArma.size() << std::endl;

    // std::cout<<"onzArma.n_rows() -> \n" << onzArma.n_rows << std::endl;
    // std::cout<<"onzArma.n_cols() -> \n" << onzArma.n_cols << std::endl;
    arma::mat nullSpaceMatArma = arma::null(onzArma);
    ```
- the full function to generate a left null space unitary matrix satisfying Ax=0 with x=[1,1,1,...].
    ```C++
    /* get the left null space + unitary matrix */
    Eigen::MatrixXd getLeftNullSpaceUnitaryMatrix(int matSize)
    {
        /* generate vector with all the elements equals to one */
        Eigen::MatrixXd onz;
        onz.resize(matSize, 1);
        for(int i = 0; i < matSize; i++)
        {
        onz(i,0) = 1;
        }

        /* get the null space using armadillo */
        /* Step 1: calculate null space in armadillo */
        arma::mat onzArma = arma::mat(onz.data(), onz.cols(), onz.rows(),
                                false, false);
        // onzArma.print("onzArma:"); 
        // std::cout<<"onzArma.size() -> \n" << onzArma.size() << std::endl;
        
        // std::cout<<"onzArma.n_rows() -> \n" << onzArma.n_rows << std::endl;
        // std::cout<<"onzArma.n_cols() -> \n" << onzArma.n_cols << std::endl;
        arma::mat nullSpaceMatArma = arma::null(onzArma);
        // nullSpaceMatArma.print("nullSpaceMatArma:"); 
        // std::cout<<"nullSpaceMatArma.size() -> \n" << nullSpaceMatArma.size() << std::endl;
        // std::cout<<"nullSpaceMatArma.n_rows() -> \n" << nullSpaceMatArma.n_rows << std::endl;
        // std::cout<<"nullSpaceMatArma.n_cols() -> \n" << nullSpaceMatArma.n_cols << std::endl;

        Eigen::MatrixXd nullSpaceEigen = Eigen::Map<Eigen::MatrixXd>(nullSpaceMatArma.memptr(),
                                                            nullSpaceMatArma.n_rows,
                                                            nullSpaceMatArma.n_cols);
        /* transform the armadillo matrix to Eigen matrix */
        // std::cout<<"nullSpaceEigen -> \n" << nullSpaceEigen<< std::endl;

        /************* get the H matrix ***************/
        /* Step 1: construct the random matrix */
        Eigen::MatrixXd H1, H2;
        Eigen::MatrixXcd H12;
        H1.resize(matSize-1,matSize-1);
        H2.resize(matSize-1,matSize-1);
        H12.resize(matSize-1,matSize-1);
        H1 = Eigen::MatrixXd::Random(matSize-1,matSize-1);
        H2 = Eigen::MatrixXd::Random(matSize-1,matSize-1);
        H12.real() = H1;
        H12.imag() = H2;
        H12 = 2 * H12;
        // std::cout<<"H1 -> \n" << H1<< std::endl;
        // std::cout<<"H2 -> \n" << H2<< std::endl;
        // std::cout<<"H12 -> \n" << H12<< std::endl;

        Eigen::MatrixXcd H34;
        H34.resize(matSize-1,matSize-1);
        Eigen::MatrixXd H34Value;
        H34Value.resize(matSize-1,matSize-1);
        for(int i = 0; i < matSize-1; i++)
        {
        for(int j = 0; j < matSize-1; j++)
        {
            H34Value(i,j) = 1.0;
        }
        }
        H34.real() = H34Value;
        H34.imag() = H34Value;
        // std::cout<<"H34 -> \n" << H34<< std::endl;

        /* Step 2: construct the H matrix */
        Eigen::MatrixXcd H;
        H.resize(matSize-1,matSize-1);
        H = H12 - H34;
        // std::cout<<"H -> \n" << H<< std::endl;
        // std::cout<<"H.real() -> \n" << H.real()<< std::endl;

        /* Step 3: get the Q matrix from H using QR decomposition */
        Eigen::ColPivHouseholderQR<Eigen::MatrixXcd> qrT(H);
        Eigen::MatrixXcd r = qrT.matrixQR().triangularView<Upper>();
        // std::cout<<"r -> \n" << r<< std::endl;
        Eigen::ColPivHouseholderQR< Eigen::MatrixXcd >::HouseholderSequenceType  seq = qrT.householderQ();
        // std::cout<<"Q -> \n" << (Eigen::MatrixXcd)seq<< std::endl;
        Eigen::MatrixXcd Qmatrix = (Eigen::MatrixXcd)seq;

        Eigen::MatrixXcd UnitaryMat;
        UnitaryMat.resize(matSize,matSize);
        UnitaryMat = nullSpaceEigen * Qmatrix * nullSpaceEigen.transpose();
        UnitaryMat = UnitaryMat + (1/(double)matSize) * onz * onz.transpose();
        std::cout<<"UnitaryMat -> \n" << UnitaryMat << std::endl;
        std::cout<<"UnitaryMat.imag() -> \n" << UnitaryMat.imag()<< std::endl;
        std::cout<<"UnitaryMat* onz -> \n" << UnitaryMat * onz<< std::endl;

    }
    ```

### Reference
1. [Visualization.cpp in VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/vins_estimator/src/utility/visualization.cpp#L161)
<!-- 2. [Quick Intro to Git and GitHub](https://hplgit.github.io/teamods/bitgit/Langtangen_bitgit_4print.pdf) -->


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)