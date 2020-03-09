%% Load equations
clear
Sp = load(fullfile( robodebRoot, 'subprojects', 'dynamic_id', 'save_data', 'Denso_results_maxDOF=6_2020-2-29_14-54.mat'));
Ss = load(fullfile( robodebRoot, 'subprojects', 'dynamic_id', 'save_data', 'Denso_sym_results_2020-2-29_20-41.mat'));

%%
n = 2:6;

f = figure(1);
clf;

semilogy(n, Sp.time(n), 'k-o', ...
    'markersize', 7, ...
    'MarkerFaceColor', [1 1 1], ...
    'linewidth', 1.2);
hold on;
semilogy(n, Ss.time(n), 'k-.o', ...
    'markersize', 7, ...
    'MarkerFaceColor', [1 1 1], ...
    'linewidth', 0.75);
hold off;
grid on;

legend({'PSDM', ...
    'Symbolic Derivation (Matlab)'}, ...
    'interpreter', 'latex', ...
    'fontsize', 12, ...
    'location', 'northwest')

ax = gca;
ax.GridColor = [1, 1, 1]*.1;
ylim([10^-3 10^4])
%yticks([0.001, 0.01, 0.1, 1, 10, 100, 1000, 10000]);
yticklabels({'$0.001$', '$0.01$', '$0.1$', '$1$', '$10$', '$100$', '$1000$', '$10,000$'});
ax.YAxis.FontSize = 12;
ax.YAxis.TickLabelInterpreter = 'latex';

xticks(n);

ax.XAxis.FontSize = 12;
ax.XAxis.TickLabelInterpreter = 'latex';

xlabel("Manipulator DOF, $n$", 'interpreter', 'latex', 'fontsize', 13);
ylabel("Derivation Time (s)", 'interpreter', 'latex','fontsize', 13);

f.PaperUnits = 'centimeter';
f.PaperPosition = [0 0 20 12]*0.9;


box off

dir = '/Users/steffanlloyd/Documents/School/00_Thesis/Papers/PSDM_iROS/img';
saveas(f, fullfile(dir, 'time.eps'), 'epsc');

