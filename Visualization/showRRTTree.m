function showRRTTree(newParent, newNode, rrtInitializationCounter, rrt_tree, treeSection)
    color = colororder;

    figure(2)
    set(gcf,'units','centimeters','position',[1,14,20,12])

    % Choose plotting style for different tree sections
    if strcmp(treeSection, 'best')
        for nodeInBestBranchIdx = 1:size(rrt_tree.bestBranchStates, 1)-1
            state = rrt_tree.bestBranchStates(nodeInBestBranchIdx, :).';
            state_next = rrt_tree.bestBranchStates(nodeInBestBranchIdx + 1, :).';
            connectionRRT = [state(1:3).'; state_next(1:3).'];

            plot3(connectionRRT(:,1), connectionRRT(:,2), connectionRRT(:,3),...
                  'LineStyle', '-', 'LineWidth', 1.5, 'Color', color(2,:),...
                  'Marker', 'o',...
                  'MarkerFaceColor', color(3,:), 'MarkerEdgeColor', color(3,:))
            hold on
            title(['RRT-Tree of planning step ' num2str(rrtInitializationCounter)], ...
                   'Interpreter', 'Latex')
            set(gca,'FontSize',14,'TickLabelInterpreter','latex')
            axis equal
            grid on
            drawnow
        end
        hold off
    else
        if strcmp(treeSection, 'initialize')
            colorIdx = 4;
        end
        if strcmp(treeSection, 'iterate')
            colorIdx = 1;
        end
        connectionRRT = [newParent.state(1:3).'; newNode.state(1:3).'];
        plot3(connectionRRT(:,1), connectionRRT(:,2), connectionRRT(:,3),...
            'LineStyle', '-', 'LineWidth', 1.5, 'Color', color(colorIdx,:),...
            'Marker', 'o', ...
            'MarkerFaceColor', color(3,:), 'MarkerEdgeColor', color(3,:))
        hold on
        if norm(rrt_tree.get(1).state(1:3) - connectionRRT(1,:).') == 0
            plot3(connectionRRT(1,1), connectionRRT(1,2), connectionRRT(1,3),...
                'LineStyle', 'none', 'LineWidth', 1.5, 'Color', color(colorIdx,:),...
                'Marker', 'o', ...
                'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r')    
        end
        title(['RRT-Tree of planning step ' num2str(rrtInitializationCounter)], ...
               'Interpreter', 'Latex')
        set(gca,'FontSize',14,'TickLabelInterpreter','latex')
        axis equal
        grid on
        drawnow
    end 
end

