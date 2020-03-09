n = 1:8;

size_m1 = 6.^n.*(n+1).*(n+2)./2;
size_m2 = 6.^n;
size_m3 = 5.^n;
size_g = 3.^n;
clf

f = figure(1);
[h11, h12] = plotnice(n, size_m1, '+', ':');
[h21, h22] = plotnice(n, size_m2, '^', ':');
[h31, h32] = plotnice(n, size_m3, 'x', '-');
[h41, h42] = plotnice(n, size_g, '.', '-');

legend([h12, h22, h32, h42], {'Full search space $6^n (n+1)(n+2)/2$', ...
    'Search space/accel.: $6^n$', ...
    'Search space/accel., w/o $\sin^2$:  $5^n$', ...
    'Search space, gravity accel., $3^n$'}, ...
    'interpreter', 'latex', ...
    'fontsize', 14, ...
    'location', 'northwest')

xlabel("Manipulator DOF, $n$", 'interpreter', 'latex', 'fontsize', 16);
ylabel("Maximum matrix inversion size", 'interpreter', 'latex','fontsize', 16);
set(gca, 'YScale', 'log')
grid on;

ax = gca;
ax.GridColor
ax.GridColor = [0.05, 0.05, 0.05];

f.PaperUnits = 'centimeter';
f.PaperPosition = [0 0 20 15];

dir = '/Users/steffanlloyd/Documents/School/00_Thesis/Papers/dynamic_id/img';

saveas(f, fullfile(dir, 'matsize.eps'), 'epsc');

function [h1, h2] = plotnice(x, y, symbol, linetype)
    hold on;
    h1 = semilogy(x, y, strcat('ko', linetype), 'linewidth', 1, 'MarkerFaceColor', [1 1 1], 'markersize', 10);
    h2 = semilogy(x, y, strcat('k', symbol), 'linewidth', 1);
	hold off;
end