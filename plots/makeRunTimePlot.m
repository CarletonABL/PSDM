%% Load equations
clear
load(fullfile( robodebRoot, 'subprojects', 'dynamic_id', 'save_data', 'runtime_tests.mat'));

%%

f = figure(1);
clf
X = categorical({'PSDM (Optimized)','PSDM (Non-Optimized)', 'Symbolic (Regressor Form)', 'Symbolic (Canonical Form)',  'Newton-Euler Algorithm'});
X = reordercats(X, fliplr({'PSDM (Optimized)','PSDM (Non-Optimized)', 'Symbolic (Regressor Form)', 'Symbolic (Canonical Form)',  'Newton-Euler Algorithm'}));
Y = ([t5 t2 t6 t4 t3])*10^6;
b = barh(X, Y, ...
    'FaceColor', [1 1 1]*.7, ...
    'FaceAlpha', 0.8, ...
    'EdgeColor',[1 1 1]*.2, ...
    'LineWidth',0.9);

grid on;
ax = gca;
ax.XAxis.Scale = 'log';
ax.XAxis.TickLabelInterpreter = 'latex';
ax.YAxis.TickLabelInterpreter = 'latex';
ax.XAxis.FontSize = 12;
ax.YAxis.FontSize = 12;
ax.XColor = [0 0 0];
ax.YColor = [0 0 0];
ax.GridColor = [1, 1, 1]*.3;
ax.GridAlpha = 0.4;

xlabel('Mean Inverse Dynamics Evaluation Time ($\mu\rm s$)', 'interpreter', 'latex');
xticks([1 10 100 1000]);
xticklabels({'1', '10', '100', '1000'});
N = numel(Y);
xtips = b.YEndPoints*1.1;
ytips = b.XEndPoints;

for i = 1:N
    labels{i} = sprintf('$%.1f %s$', b.YData(i), ' \ \rm\mu s');
end

text(xtips,ytips,labels, ...
    'HorizontalAlignment','left',...
    'VerticalAlignment','middle', ...
    'interpreter', 'latex', ...
    'fontsize', 12, ...
    'color', [0 0 0])
box off;
f.PaperUnits = 'centimeter';
f.PaperPosition = [0 0 20 11]*0.9;

% Create textbox
annotation(f,'textbox',...
    [0 0.121428571428572 0.3615 0.0738095238095244],...
    'String','* Requires full inertial parameters $\mathbf{X}$',...
    'LineStyle','none',...
    'Interpreter','latex',...
    'HorizontalAlignment','right',...
    'FontSize',9,...
    'FitBoxToText','off');

% Create textbox
annotation(f,'textbox',...
    [0 0.27857142857143 0.3615 0.0738095238095249],...
    'String','* Requires full inertial parameters $\mathbf{X}$',...
    'LineStyle','none',...
    'Interpreter','latex',...
    'HorizontalAlignment','right',...
    'FontSize',9,...
    'FitBoxToText','off');

%%
f = gcf;
f.PaperPosition = [0 0 20 9]*0.9;
dir = '/Users/steffanlloyd/Documents/School/00_Thesis/Papers/PSDM_iROS/img';
saveas(f, fullfile(dir, 'run_time.eps'), 'epsc');

