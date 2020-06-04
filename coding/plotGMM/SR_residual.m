clc; close all; clear all; 
%%%%%% generate data. 
REAL_DATA = 1; 
histSize = 0.3;
if REAL_DATA
%     psr_error_beidou_res = importdata('psr_error_gps.csv');
    psr_error_beidou_res = importdata('error_im_all.csv');
else 
    psr_error_beidou_res = randn(50000, 1); 
    psr_error_beidou_res = [psr_error_beidou_res; randn(30000, 1)*0.5 + 2.0];
end
psr_error_beidou_res = psr_error_beidou_res(2:end);
%%%%%% fit GMM. 
K = 3;
gmm = fitgmdist(psr_error_beidou_res, K);
%%%%%%% compute empirical and theoretical histogram. 
if REAL_DATA
    est_edges  = floor(min(psr_error_beidou_res)) : histSize : ceil(max(psr_error_beidou_res)); %%%%%% divide to edges.
else
    est_edges  = linspace(min(psr_error_beidou_res), max(psr_error_beidou_res), 30); %%%%%% divide to edges. 
end
est_center = (est_edges(1:end-1) + est_edges(2:end)) / 2;  
data   = histcounts(psr_error_beidou_res, est_edges); 
est_prob = data / sum(data);  % this is empirical histogram. 
Mu     = gmm.mu; 
Sigma  = squeeze(gmm.Sigma);
Weight = gmm.ComponentProportion; 
grt_prob = []; 
grt_edges  = est_edges; linspace(min(psr_error_beidou_res), max(psr_error_beidou_res), 500); 
grt_center = (grt_edges(1:end-1) + grt_edges(2:end)) / 2;  
for i = 1 : 1 : length(Weight)
    val = Weight(i) * normcdf(grt_edges, Mu(i), sqrt(Sigma(i))); 
    if isempty(grt_prob)
        grt_prob = val; 
    else
        grt_prob = grt_prob + val; 
    end
end
grt_prob = grt_prob(2:end) - grt_prob(1:end-1);  % this is theoretical histogram.
sum(grt_prob)
figure; 
hold on; 
grid on; 
ax = gca;
ax.FontSize = 20; 
plot(est_center, est_prob, 'r.-'); 
plot(grt_center, grt_prob, 'b.-'); 

figure; 
hold on; 
grid on; 
ax = gca;
ax.FontSize = 30; 
per_hist = histogram(psr_error_beidou_res, est_edges); 
% hy_fusion = histogram(x1,30)
per_hist.BinWidth = histSize;
plot(grt_center, length(psr_error_beidou_res) * grt_prob, 'b.-', 'linewidth', 2); 
% legend({'error of point clouds', 'Gaussian mixture model'}, 'location', 'best', 'box', 'off', 'FontSize', 30); 
% ptCloud = pcread('GT.pcd');
% pcshow(ptCloud);