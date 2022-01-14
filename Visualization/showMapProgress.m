function showMapProgress(time_vect, mapping_progress_vect)
    color = colororder;

    figure(4)
    set(gcf,'units','centimeters','position',[25,12.5,25,12.5])
    plot(time_vect, mapping_progress_vect(:,1),...
         'LineStyle', '-', 'LineWidth', 1.5, 'Color', color(1,:))
    hold on
    plot(time_vect, mapping_progress_vect(:,2),...
         'LineStyle', '-', 'LineWidth', 1.5, 'Color', color(2,:))
    plot(time_vect, mapping_progress_vect(:,3),...
         'LineStyle', '-', 'LineWidth', 1.5, 'Color', color(3,:))
    grid on
    set(gca,'FontSize',14,'TickLabelInterpreter','latex')
end

