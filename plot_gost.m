function plot_gost(x, y, x_label_text, y_label_text, filename)
figure('Color', [1 1 1], 'OuterPosition', [0 0 640 360]);
set(0,'DefaultAxesFontSize',14,'DefaultAxesFontName','Times New Roman');
set(0,'DefaultTextFontSize',14,'DefaultTextFontName','Times New Roman'); 

clf
x_max = 550;
y_max = 130;
% axis([0 x_max -30 y_max])
hold on;
grid on;

plot(x, y, 'LineWidth', 1.5); %, 'k-', 'LineWidth', 1);
xlabel(x_label_text);
ylabel(y_label_text);
legend('\gamma', '\psi', '\vartheta', 'Location', 'northeast', 'Orientation', 'horizontal');

saveas(gcf, filename, 'png');
% saveas(gcf, filename, 'svg')


% BX = get(gca, 'XTick');
% BY = get(gca, 'YTick');
% 
% xlabel(x_label_text, 'Rotation', 0, 'Position', [BX(size(BX,2)) BY(1)]);
% ylabel(y_label_text, 'Rotation', 0, 'Position', [BX(1) BY(size(BY,2))]);


end

