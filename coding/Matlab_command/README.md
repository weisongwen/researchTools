# Some practical matlab command in processing data, plot experimental results

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
- **plot double y-axis for errors (translation and rotation)**
  ```Matlab
  clear;
  close all;

  %%%%%%%%%%%% LeGOLOAMerrorTrans error
  LeGoLOAMTrans_Data = csvread('LeGOLOAMerrorTrans.csv'); % AGPCerrorTrans  LeGoLOAMerrorRot
  LeGoLOAMTrans_error = LeGoLOAMTrans_Data(:,1);
  LeGoLOAMTrans_epoch = LeGoLOAMTrans_error;
  LeGoLOAMTrans_epoch = 1:1:length(LeGoLOAMTrans_error);

  %%%%%%%%%%%% AGPCerrorTrans error
  AGPCSLAMTrans_Data = csvread('AGPCerrorTrans.csv'); %LeGoLOAMerrorRot AGPCerrorRot1
  AGPCSLAMTrans_error = AGPCSLAMTrans_Data(:,1);  
  AGPCSLAMTrans_epoch = AGPCSLAMTrans_error;
  AGPCSLAMTrans_epoch = 1:1:length(AGPCSLAMTrans_error);

  %%%%%%%%%%%% LeGOLOAMerrorTrans rot
  LeGoLOAMRot_Data = csvread('LeGoLOAMerrorRot.csv'); % AGPCerrorTrans  LeGoLOAMerrorRot
  LeGoLOAMRot_error = LeGoLOAMRot_Data(:,1);
  LeGoLOAMRot_epoch = LeGoLOAMRot_error;
  LeGoLOAMRot_epoch = 1:1:length(LeGoLOAMRot_error);

  %%%%%%%%%%%% AGPCerrorRot error
  AGPCSLAMRot_Data = csvread('AGPCerrorRot1.csv'); %LeGoLOAMerrorRot AGPCerrorRot1
  AGPCSLAMRot_error = AGPCSLAMRot_Data(:,1);
  AGPCSLAMRot_epoch = AGPCSLAMRot_error;
  AGPCSLAMRot_epoch = 1:1:length(AGPCSLAMRot_error);

  lineWidth = 2.9; 

  yyaxis left
  plot(LeGoLOAMTrans_epoch,LeGoLOAMTrans_error,'r-o','LineWidth',lineWidth )
  hold on;
  plot(AGPCSLAMTrans_epoch,AGPCSLAMTrans_error,'b-*','LineWidth',lineWidth )
  hold on;
  grid on;
  hold on;
  ax = gca;
  ax.FontSize = 30; 
  xlabel('epoch (second)');
  ylabel('translation error (meter)');
  % legend('\fontsize{25} LeGO-LOAM translation error','\fontsize{25} AGPC-SLAM translation error');

  yyaxis right
  plot(LeGoLOAMRot_epoch,LeGoLOAMRot_error,'g-o','LineWidth',lineWidth )
  hold on;
  plot(AGPCSLAMRot_epoch,AGPCSLAMRot_error,'c-*','LineWidth',lineWidth )
  hold on;
  grid on;
  hold on;
  ax = gca;
  ax.FontSize = 30; 
  % xlabel('epoch (second)');
  % ylabel('error (meter)');
  ylabel('rotation error (degree)');
  legend('\fontsize{25} LeGO-LOAM translation error','\fontsize{25} AGPC-SLAM translation error','\fontsize{25} LeGO-LOAM rotation error','\fontsize{25} AGPC-SLAM rotation error');
  ```


### Reference


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)