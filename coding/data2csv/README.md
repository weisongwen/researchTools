# Writing data to CSV in C++ and reading data from CSV

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

## read data from CSV using C++ (e.g. read ublox solution from csv file)

```
void getUbloxNMEASolution()
  {
    while(1)
    {
      std::chrono::milliseconds dura(1000); // this thread sleep for any ms
      std::this_thread::sleep_for(dura);
      // load image list
      FILE* NMEAFile;
      // std::string NMEAPath = "../ion_GNSS_2020/src/data/ublox_190331_084530.csv";
      std::string NMEAPath = "../ion_GNSS_2020/src/data/ublox_190606_144040.csv";
      NMEAFile = std::fopen((NMEAPath).c_str() , "r");
      if(NMEAFile == NULL){
          printf("cannot find file: ublox NMEA File \n", NMEAPath.c_str());
          ROS_BREAK();
	    }
      double Timestamp;
      double latitude;
      double longitude;
      double altitude;
      char line[1024];
      NMEAData NMEAData_;
      map<int, NMEAData>::iterator iterNMEA;
      // while (fscanf(NMEAFile, "%lf %lf %lf %lf", &Timestamp, &latitude, &longitude, &altitude) != EOF)
      // {
      //   printf("%lf\n", latitude);
      // }
      // std::fclose(NMEAFile);
      if(!m_NMEAVector.size())
      {
         while ((fscanf(NMEAFile, "%[^\n]", line)) != EOF)
        {
          fgetc(NMEAFile);    // Reads in '\n' character and moves file
                            // stream past delimiting character
          printf("Line = %s \n", line);
          std::stringstream ss(line); // split into three string
          vector<string> result;
          while (ss.good())
          {
            string substr;
            getline(ss, substr, ',');
            result.push_back(substr);
            std::cout << std::setprecision(17);
          }
          NMEAData_.Timestamp =strtod((result[0]).c_str(), NULL);
          NMEAData_.latitude = strtod((result[1]).c_str(), NULL);
          NMEAData_.longitude = strtod((result[2]).c_str(), NULL);
          NMEAData_.altitude = strtod((result[3]).c_str(), NULL);
          std::cout << std::setprecision(17);
          m_NMEAVector[int(NMEAData_.Timestamp)] = NMEAData_;
          // std::cout<<"m_NMEAVector.size() = "<<m_NMEAVector.size()<<std::endl;
        }
      }
      std::fclose(NMEAFile);
      // std::cout<<"m_NMEAVector.size() = "<<m_NMEAVector.size()<<std::endl;
      if(m_GNSSMes.GNSS_Raws.size())
      {
        std::cout << std::setprecision(17);
        int GNSST = int(m_GNSSMes.GNSS_Raws[0].GNSS_time / 1000);
        iterNMEA = m_NMEAVector.find(GNSST);
        // std::cout<<"GNSS time-> "<< GNSST<<std::endl;
        if (iterNMEA != m_NMEAVector.end())
        {
          // std::cout<<"iterNMEA->second.latitude  "<<iterNMEA->second.latitude <<std::endl;
        }
      }
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