% Displays error histograms based on collected statistics (see visualize).
% Histograms are useful to visualize the distribution of behavior,
% because each run's performance is difference due to randomness in data.

% Input: statistics = [err_xtv, err_etv, err_xev, ang_xt, ang_et, ang_xe,
%                      err_xtm, err_etm, err_xem, num_xt, num_et, num_xe]
%        There are 4 triples of errors, for trajecotry position error,
%        trajectory angular error, landmark position error, and
%        number of landmarks detected respectively.
%        Each triple is a pairwise comparison between estimate and truth,
%        toolbox EKF and truth, estimate and toolbox EKF respectively.

function visualize_histogram(statistics)

    fprintf('Summary error statistics over %d trials\n', size(statistics, 1));
    fprintf('Average trajectory errors (average Euclidean and angular distances):\n');
    fprintf('-- between estimate and ground truth: \n   Pos %f \tOri %f\n', mean(statistics(:,1)), mean(statistics(:,4)));
    fprintf('Average landmark errors (average Euclidean distance for landmarks estimated):\n');
    fprintf('-- between estimate and ground truth (average %f landmarks): \n   Avg %f \tStd %f\n', mean(statistics(:,10)), mean(statistics(:,7)), std(statistics(:,7)));
    figure;
    subplot(2,2,1);
    histogram(statistics(:,1), 'BinWidth', 0.05, 'BinLimits', [0 max(max(statistics(:,1)), 1.5)]);
    title('Trajectory position errors');
    subplot(2,2,2);
    histogram(statistics(:,7), 'BinWidth', 0.05, 'BinLimits', [0 max(max(statistics(:,7)), 1.5)]);
    title('Landmark position errors');
    subplot(2,2,3);
    histogram(statistics(:,4), 'BinWidth', 0.02, 'BinLimits', [0 max(max(statistics(:,4)), 0.5)]);
    title('Trajectory angular errors');
    subplot(2,2,4);
    histogram(statistics(:,10), 'BinWidth', 1, 'BinLimits', [0 max(max(statistics(:,10)), 20)]);
    title('Number of landmarks detected');

end