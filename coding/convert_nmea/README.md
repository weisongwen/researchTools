# Convert .ubx data to NMEA data saved by exceel format 
This function is mainly used to extract the loosely solution from the .ubx data collected from the ublox receiver. Even for .txt files from GNSSLOGGER of Google, the NMEA solutions can be extracted into the CSV file and the .kml file.

**Notes**
- the thing you need to change/specify is the **rcvNmeaPathName** in the following matlab code.
    

- file convert_nmea.m
    - MATLAB code for convert .ubx/txt file to excell file
    ```Matlab
    clc;
    clear all;
    close all;

    % rcvNmeaPathName = 'conventional\COM4_171208_093901.nmea';
    %rcvNmeaPathName = 'proposed\COM15_171208_093900.nmea';
    % rcvNmeaPathName = 'TST_20180521\raw\XPH_180521_115546.ubx';
    rcvNmeaPathName = 'gnss_log_2020_12_29_14_49_15.txt';


    % GPSTimeS = 31562;%109000;%[273451];
    % GPSTimeE = 35021;%122800;%[274095];

    GPSTimeS = 0;%109000;%[273451];
    GPSTimeE = 10000000000;%122800;%[274095];

    %% Load rcv nmea and time table
    nmeaGPSTIndexShift = 1;
    nmeaLatIndexShift = 2; 
    nmeaLonIndexShift = 4;
    nmeaHeightMSIIndexShift = 9;
    nmeaHeightGEOIndexShift = 11;
    nmeaGPSTIndexShift
    UTCGPSshift = 18;
    secondsInaDay = 86400;
    rcvNmea = textread(rcvNmeaPathName, '%s', 'whitespace', ',');
    rcvPosResult = zeros(1,3);
    rcvPosResultCount = 0;
    gpsTimeShift = fix(GPSTimeE/secondsInaDay)*secondsInaDay;
    GPSTime_last = 0;
    if GPSTimeE>=0 && GPSTimeS>=0 && GPSTimeE>GPSTimeS
        for rcvNmeaIdx = 1:size(rcvNmea,1)
            if ~isempty(rcvNmea{rcvNmeaIdx, 1})
            if rcvNmea{rcvNmeaIdx, 1}(1,1) == '$'
                %rcvNmeaIdx
                time = str2double(rcvNmea{rcvNmeaIdx + nmeaGPSTIndexShift, 1});
                lat = str2double(rcvNmea{rcvNmeaIdx + nmeaLatIndexShift, 1});
                lon = str2double(rcvNmea{rcvNmeaIdx + nmeaLonIndexShift, 1});
                hour = fix(time/10000);
                minu = fix((time-hour*10000)/100);
                sec = fix(time-hour*10000-minu*100);
                UTCTime = hour*3600 + minu * 60 + sec;
                GPSTime = UTCTime + UTCGPSshift + gpsTimeShift*0;       
                rcvNmea{34071,1}=0;
                height_msl = str2double(rcvNmea{rcvNmeaIdx + nmeaHeightMSIIndexShift, 1});
    %             height_geo = str2double(rcvNmea{rcvNmeaIdx + nmeaHeightGEOIndexShift, 1});
                

                if (GPSTime>=GPSTimeS&&GPSTime<=GPSTimeE) && (GPSTime ~= GPSTime_last)
                    rcvPosResultCount = rcvPosResultCount+1;
                    latDeg = fix(lat/100);
                    latMin = (lat-latDeg*100);
                    latInDegrees = latDeg+latMin/60.0;               
                    
                    lonDeg = fix(lon/100);
                    lonMin = (lon-lonDeg*100);
                    lonInDegrees = lonDeg+lonMin/60.0;
                    
                    rcvPosResult (rcvPosResultCount ,1) = GPSTime;
                    rcvPosResult (rcvPosResultCount ,2) = latInDegrees;
                    rcvPosResult (rcvPosResultCount ,3) = lonInDegrees;
                    rcvPosResult (rcvPosResultCount ,4) = height_msl;
    %                 rcvPosResult (rcvPosResultCount ,5) = height_geo;
                    GPSTime_last = GPSTime;
                end
            end
            end        
        end
    end
    idn = find(rcvPosResult(:,2) > 22);
    rcvPosResult = rcvPosResult(idn,:);

    idm = find(rcvPosResult(:,2) < 30);
    rcvPosResult = rcvPosResult(idm,:);
    dlmwrite('gnss_log_2020_12_29_14_49_15.csv', rcvPosResult, 'precision', '%12.8f');

    ublox_data = csvread('gnss_log_2020_12_29_14_49_15.csv');

    kmlwrite('gnss_log_2020_12_29_14_49_15.kml',ublox_data(:,2),ublox_data(:,3),'Icon',...
        'http://maps.google.com/mapfiles/kml/shapes/shaded_dot.png','IconScale',0.5,'Color',[1,0,0],'Name','  ');
    % csvtokml('conventional\data_con.kml',ublox_data,'cyan_dot');
    ```


### Reference
1. [(slides)](https://www.ipb.uni-bonn.de/wp-content/uploads/2018/05/lecture_1.pdf))
2. [Google Code Styleguide](https://google.github.io/styleguide/cppguide.html)
3. [C++ tutorials](http://www.cplusplus.com/doc/tutorial/)


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)