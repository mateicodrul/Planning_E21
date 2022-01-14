visible_volume = 10;
cost = 0:0.01:5;
lambda = [1e-1 0.25 5e-1 0.75 1 2];
figure
set(gcf,'units','centimeters','position',[0,0,21,12])
legend_cell = cell(length(lambda),1);
for i = 1:length(lambda)
    gain_increm = exp(-lambda(i) * cost);
    plot(cost, gain_increm, 'LineStyle', '-', 'LineWidth', 1.5, ...
        'Color', color(i,:))
    hold on
    legend_cell{i} = ['$\lambda = $ ', num2str(lambda(i))];
end
hold off
legend_handle = legend(legend_cell);
xlabel('$c(\sigma_{k,k+1})$','Interpreter','latex')
ylabel('$\Delta\gamma(\sigma_{k,k+1})$','Interpreter','latex')
set(gca,'FontSize',16,'TickLabelInterpreter','latex')
set(legend_handle,'Interpreter','Latex','Orientation','horizontal','Location','bestoutside','FontSize',14);

