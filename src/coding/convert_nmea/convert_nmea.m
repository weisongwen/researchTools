% By NG Hoi-Fung, Ivan 
% Discription: Convert the NMEA sentence to time, llh data
% Output CSV format: gps time, latitude, longitude, MSL(m), Geoid(m), utc time(yyyymmddHHMMSS.FFF)
% csv file will output to the original path that nmea file directory

clc;
clear;
close all;

GPS_TIME_START = datetime(1980,1,6,0,0,0,0);
leap_second = 18;

nmea_path = 'E:\sparse and urban jiulongtang midundao Huawei VINS test\20200314\COM55_200314_093715.ubx'; % input nmea sentence directory
[nmea_dir, exp_name] = fileparts(nmea_path);

support_type = {'$G*GGA','$G*RMC','$G*GLL','$G*BWC'};

% GGA: UTC, Lat, NE, Lon, NE, Fix Quality, No Sat, HDOP, MSL(m), Geoid(m)
% RMC: UTC, Status, Lat, NE, Lon, NE, Speed, Track angle, Date, Magnetic, checksum
% GLL: Lat, NE, Lon, NE, UTC, Date active, checksum
% BWC: UTC, Lat, NE, Lon, NE,bearing(true), bearing(magnetic), dist to waypoint(NM), waypoint id, checksum
% RMB: data status, Cross-track error(NM), Origin waypoint ID, Destination waypoint ID

date_str = {};
S = textread(nmea_path,'%s');
nmea_pos = double.empty(0,4);

for i = 1:length(S)
    type = regexp(S{i}, regexptranslate('wildcard', support_type));
    sup = 0;
    sup = find(~cellfun(@isempty,type));
    k = strfind(S{i},'$GNRMC');
    if k
       disp(''); 
    end
    if ~isempty(sup)
        utc = nan; lat = nan; lon = nan; msl = nan; geoid = nan;
        lat_ = nan; lon_ = nan; date_ = nan;
        lat_factor = 1; lon_factor = 1;
        
        str = S{i};
        str = str(type{sup}:end)
        C = strsplit(str,',', 'CollapseDelimiters',false);
        %         disp('');
        switch(sup)
            case 1 % GGA
                utc = str2double(C{2});
                utc_ = C{2};
                try
                    lat_ = str2double(C{3});
                    if C{4}~='N'; lat_factor = -1; end
                catch
                end
                try
                    lon_ = str2double(C{5});
                    if C{6}~='E'; lon_factor = -1; end
                catch
                end
                try msl = str2double(C{10}); catch, end
                try geoid = str2double(C{12}); catch, end
            case 2 % RMC
                utc = str2double(C{2});
                utc_ = C{2};
                try
                    lat_ = str2double(C{4});
                    if C{5}~='N'; lat_factor = -1; end
                catch
                end
                try
                    lon_ = str2double(C{6});
                    if C{7}~='E'; lon_factor = -1; end
                catch
                end
                try
                    datetime(C{10},'InputFormat','ddMMyy');
                    if isnan(date_); date_ = C{10}; end
                catch
                end
            case 3 % GLL
                utc = str2double(C{6});
                utc_ = C{6};
                try
                    lat_ = str2double(C{2});
                    if C{3}~='N'; lat_factor = -1; end
                catch
                end
                try
                    lon_ = str2double(C{4});
                    if C{5}~='E'; lon_factor = -1; end
                catch
                end
            case 4 % BWC
                utc = str2double(C{2});
                utc_ = C{2};
                try
                    lat_ = str2double(C{3});
                    if C{4}~='N'; lat_factor = -1; end
                catch
                end
                try
                    lon_ = str2double(C{5});
                    if C{6}~='E'; lon_factor = -1; end
                catch
                end
        end
        
        try
            datetime(utc_,'InputFormat','HHmmss.S');
        catch
            utc_ = nan;
        end
        
        if ~isnan(lat_)
            degree = floor(lat_/100);
            minute = floor(lat_ - degree*100)/60;
            second = (lat_ - (degree*100 + floor(lat_ - degree*100)))*60/3600;
            lat = degree + minute + second;
        end
        
        if ~isnan(lon_)
            degree = floor(lon_/100);
            minute = floor(lon_ - degree*100)/60;
            second = (lon_ - (degree*100 + floor(lon_ - degree*100)))*60/3600;
            lon = degree + minute + second;
        end
        
        if isempty(nmea_pos)
            nmea_pos = [utc,nan,nan,nan,nan];
        end
        utc_idx = find(nmea_pos(:,1)==utc);
        if isempty(utc_idx)
            utc_idx = size(nmea_pos,1)+1;
            nmea_pos(utc_idx,:) = [utc,nan,nan,nan,nan];
        end
        if isnan(nmea_pos(utc_idx,2)) && ~isnan(lat); nmea_pos(utc_idx,2)=lat; end
        if isnan(nmea_pos(utc_idx,3)) && ~isnan(lon); nmea_pos(utc_idx,3)=lon; end
        if isnan(nmea_pos(utc_idx,4)) && ~isnan(msl); nmea_pos(utc_idx,4)=msl; end
        if isnan(nmea_pos(utc_idx,5)) && ~isnan(geoid); nmea_pos(utc_idx,5)=geoid; end
        if ~isnan(date_); date_str(utc_idx,1) = {date_}; end
        if ~isnan(utc_); date_str(utc_idx,2) = {utc_}; end
    end
end

empty_date_idx = find(cellfun(@isempty,date_str(:,1)));
date_idx = find(~cellfun(@isempty,date_str(:,1)));
for i = 1:size(empty_date_idx,1)
    [~, store_idx] = min(date_idx - empty_date_idx(i));
    date_str(empty_date_idx(i),1) = date_str(date_idx(store_idx),1);
end
validIdx = find(~isnan(nmea_pos(:,1)));
nmea_pos = nmea_pos(validIdx,:);
date_str = date_str(validIdx,:);
gps_time = zeros(size(date_str,1),1);
utc_time = zeros(size(date_str,1),1);
for i = 1:size(gps_time,1)
    tempt = datetime(strcat(date_str{i,:}),'InputFormat','ddMMyyHHmmss.S');
    gps_time(i) = seconds(tempt - GPS_TIME_START) + leap_second;
    utc_time(i) = str2double(datestr(tempt, 'yyyymmddHHMMSS.FFF'));
end
nmea_pos(:,1) = gps_time;
nmea_pos = [nmea_pos utc_time];

% if exist('plot_openstreetmap'); plot_openstreetmap(nmea_pos(:,[2:3])); end
if exist('plot_osm'); plot_osm(nmea_pos(:,[2:3])); end 

% Output format: gps time, latitude, longitude, MSL(m), Geoid(m), utc time(yyyymmddHHMMSS.FFF)
nmea_output = [nmea_dir,'\',exp_name,'_nmea.csv'];
fid = fopen(nmea_output,'w+');
for i = 1:size(nmea_pos,1)
    fprintf(fid,'%d,%.7f,%.7f,%.1f,%.1f,%.2f\n',nmea_pos(i,:));
end
fclose(fid);

ublox_data = csvread('COM55_200314_093715_nmea.csv');
% 
kmlwrite('COM55_200314_093715_nmea.kml',ublox_data(:,2),ublox_data(:,3),'Icon',...
    'http://maps.google.com/mapfiles/kml/shapes/shaded_dot.png','IconScale',0.5,'Color',[1,0,0],'Name','  ');


