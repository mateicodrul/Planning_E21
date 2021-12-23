main;

%%% NBV planning %%%
% Create RRT tree object
rrt_tree = tree(struct('state', zeros(6,1), 'gain', 0, 'distance', 0));
% Counter for the number of plans executed
rrtInitializationCounter = 0;
% Inifnite loop for NBV planning
while rrtInitializationCounter >= 0
    % After the first run
    if rrtInitializationCounter
        % Erase all info from RRT tree, apart from bestBranch
        rrt_tree = rrt_tree.clearTree();
        % Clear old points in Kd-tree
        clear kd_tree_nodes
    end

    %%% RRT initialize function %%%
    % Add current pose to root of RRT tree
    [rrt_tree, ~] = rrt_tree.addnode(0, struct('state', pose, 'gain', 0, 'distance', 0));
    % Place position state of RRT tree root in Kd-Tree matrix
    kd_tree_nodes = pose(1:3).';
    % Increment initializationCount
    rrtInitializationCounter = rrtInitializationCounter + 1;
    disp(['Starting plan ' num2str(rrtInitializationCounter)])
    % Insert all nodes with states in bestBranchStates into the tree
    for nodeInBestBranchIdx = 1:size(rrt_tree.bestBranchStates, 1)
        newState = rrt_tree.bestBranchStates(nodeInBestBranchIdx, :).';
        % Create Kd-tree with the current nodes in the RRT tree, which are
        % stored in kd_tree_nodes
        clear kd_tree
        kd_tree = KDTreeSearcher(kd_tree_nodes);
        % Find newParent, the node whose position state is the nearest
        % neighbor for the position part of newState
        newParentIdx = knnsearch(kd_tree, newState(1:3).', 'k', 1);
        newParent = rrt_tree.get(newParentIdx);
        % Move a certain distance in the direction of newState
        origin = newParent.state(1:3);
        direction = newState(1:3) - origin;
        if norm(direction) > params.extensionRange
            direction = params.extensionRange * direction ./ norm(direction);
        end
        % Overwrite newState to be the new node we add to the tree
        newState(1:3) = origin + direction; 
        % Make sure we cross no obstacle on the way to newState. If we do,
        % we just skip the node
        collison_free_flag = checkCollisionAlongPath(occMap, origin, direction, params);
        if collison_free_flag
            % Make newNode 
            newNode = struct('state', newState, 'gain', 0, 'distance', 0);
            newNode.distance = newParent.distance + norm(direction);
            % Gain computation
            newNode.gain = newParent.gain + computeGain(occMap, newState, params) * exp(-params.degressiveCoeff * newNode.distance);
            % Insert newNode into RRT tree
            [rrt_tree, newNodeIdx] = rrt_tree.addnode(newParentIdx, newNode);
            % Add position state of newNode to the Kd-Tree matrix
            kd_tree_nodes = [kd_tree_nodes; newNode.state(1:3).'];
            % Update best gain and node
            if rrt_tree.bestGain < newNode.gain
                rrt_tree.bestGain = newNode.gain;
                rrt_tree.bestNodeIdx = newNodeIdx;
            end
            rrt_tree.iterationCounter = rrt_tree.iterationCounter + 1;
            connectionRRT = [newParent.state(1:3).'; newNode.state(1:3).'];
            figure(2)
            set(gcf,'units','centimeters','position',[81,14,20,12])
            plot3(connectionRRT(:,1), connectionRRT(:,2), connectionRRT(:,3),...
                'LineStyle', '-', 'LineWidth', 1.5, 'Color', color(4,:),...
                'Marker', 'o', ...
                'MarkerFaceColor', color(3,:), 'MarkerEdgeColor', color(3,:))
            hold on
            title(['RRT-Tree of planning step ' num2str(rrtInitializationCounter)], ...
               'Interpreter', 'Latex')
            set(gca,'FontSize',14,'TickLabelInterpreter','latex')
            axis equal
            grid on
            drawnow
        end
    end
    %%%
    
    % Loop for RRT tree construction
    loopCounter = 0;
    while ~(rrt_tree.bestGain > 0) || rrt_tree.iterationCounter < params.treeIterations
        % If maxIterations is reached, it means that rrt tree best gain stayed
        % 0, so the exploration is (hopefully) over
        if (rrt_tree.iterationCounter > params.maxIterations)
            disp(['nbvPlanner -- Best gain still 0 after ' ...
                  num2str(params.maxIterations) ...
                  ' iterations. Exploration terminated.']);
            % Exit the program here
            return;
        end
        % If we fail to add a new node to the tree for a certain number of 
        % iterations that depends on rrt_tree.iterationCounter, we should go
        % back to the previous pose and rebuild a tree from there
        if loopCounter > 1000 * (rrt_tree.iterationCounter + 1)
            disp(['nbvPlanner -- Exceeding maximum failed iterations during tree' ...
                  'construction. Return to previous point!'])
            %%% RRT getPathBackToPrevious function %%%
            
            % Exit the nbv function here
            return;
        end
    
        %%% RRT iterate function %%%
        % Grab tree root state
        rootNode = rrt_tree.get(1);
        % Select random state within the map
        diffXYZ = params.maxXYZ - params.minXYZ;
        newState = zeros(6,1);
        newState(1:3) = params.minXYZ + diffXYZ.*rand(3,1);
        % Create Kd-tree with the current nodes in the RRT tree, which are
        % stored in kd_tree_nodes
        clear kd_tree
        %disp('Nodes in Kd-tree:')
        %disp(kd_tree_nodes)
        kd_tree = KDTreeSearcher(kd_tree_nodes);
        % Find newParent, the node whose position state is the nearest
        % neighbor for the position part of newState
        newParentIdx = knnsearch(kd_tree, newState(1:3).', 'k', 1);
        newParent = rrt_tree.get(newParentIdx);
        % Move a certain distance in the direction of newState
        origin = newParent.state(1:3);
        direction = newState(1:3) - origin;
        if norm(direction) > params.extensionRange
            direction = params.extensionRange * direction ./ norm(direction);
        end
        % Overwrite newState to be the new node we add to the tree
        newState(1:3) = origin + direction;
        % Make sure we cross no obstacle on the way to newState. If we do,
        % we must go out of the function
        collison_free_flag = checkCollisionAlongPath(occMap, origin, direction, params);
        if collison_free_flag
            % Sample yaw orientation
            newState(6) = 2 * pi * (rand(1) - 0.5);
            % Make newNode 
            newNode = struct('state', newState, 'gain', 0, 'distance', 0);
            newNode.distance = newParent.distance + norm(direction);
            % Gain computation
            newNode.gain = newParent.gain + computeGain(occMap, newState, params) * exp(-params.degressiveCoeff * newNode.distance);
            % Insert newNode into RRT tree
            [rrt_tree, newNodeIdx] = rrt_tree.addnode(newParentIdx, newNode);
            % Add position state of newNode to the Kd-Tree matrix
            kd_tree_nodes = [kd_tree_nodes; newNode.state(1:3).'];
            % Update best gain and node
            if rrt_tree.bestGain < newNode.gain
                rrt_tree.bestGain = newNode.gain;
                rrt_tree.bestNodeIdx = newNodeIdx;
            end
            rrt_tree.iterationCounter = rrt_tree.iterationCounter + 1;
            
            connectionRRT = [newParent.state(1:3).'; newNode.state(1:3).'];
            figure(2)
            set(gcf,'units','centimeters','position',[81,14,20,12])
            plot3(connectionRRT(:,1), connectionRRT(:,2), connectionRRT(:,3),...
                'LineStyle', '-', 'LineWidth', 1.5, 'Color', color(1,:),...
                'Marker', 'o', ...
                'MarkerFaceColor', color(3,:), 'MarkerEdgeColor', color(3,:))
            hold on
            title(['RRT-Tree of planning step ' num2str(rrtInitializationCounter)], ...
               'Interpreter', 'Latex')
            set(gca,'FontSize',14,'TickLabelInterpreter','latex')
            axis equal
            grid on
            drawnow
            
            disp(['Iteration ' num2str(loopCounter)])
            disp(['Best node index is ' num2str(rrt_tree.bestNodeIdx)...
                  ' with gain ' num2str(rrt_tree.bestGain)])
        else
            %break;
        end
        %%%
        loopCounter = loopCounter + 1;
    end
    
    %%% RRT extractBaseBranch function %%%
    rrt_tree = rrt_tree.extractBestBranch();
    
    % Move to the new pose by executing the first edge of the best branch 
    % Vehicle dynamics should be included here, instead of pure assignment
    posePrev = pose;
    pose = rrt_tree.bestEdgeState;
    % Publish pose reference message and wait for the drone to move 
    gazeboMoveDrone(pubTrajectory, pose)
    % Decode pose messages to get initial pose of the drone in the world frame
    pose_msg = subPose.LatestMessage;
    poseQuat = [pose_msg.Position.X, pose_msg.Position.Y, pose_msg.Position.Z,...
                 pose_msg.Orientation.W, pose_msg.Orientation.X, pose_msg.Orientation.Y, pose_msg.Orientation.Z].';
    pose = poseQuat2Eul(poseQuat);
    % Pose with orientation part in quaternion form
    poseQuat = poseEul2Quat(pose);
    
    for nodeInBestBranchIdx = 1:size(rrt_tree.bestBranchStates, 1)-1
        state = rrt_tree.bestBranchStates(nodeInBestBranchIdx, :).';
        state_next = rrt_tree.bestBranchStates(nodeInBestBranchIdx + 1, :).';
        connectionRRT = [state(1:3).'; state_next(1:3).'];

        figure(2)
        set(gcf,'units','centimeters','position',[81,14,20,12])
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
    
    % Get scan in base frame of visible environment from Gazebo camera
    %tic
    PointsB = gazeboGetCameraPoints(subPointcloud, params);
    %toc
    % Add visible pointcloud in sensor coordinates to the occupancy map. Return
    % if there are no points to be added
    if ~isempty(PointsB)
        % Pose with orientation part in quaternion form
        poseQuat = poseEul2Quat(pose);
        insertPointCloud(occMap, poseQuat, PointsB, params.camMaxRange);
        
        %{
        % Set all the voxels within the sensor range and FOV to free if no
        % obstacles exist along the corresponding rays
        tic
        Points_uf = findUnmappedFree(occMap, pose, params);
        toc
        if ~isempty(Points_uf)
            tic
            setOccupancy(occMap, Points_uf, 0);
            toc
        else
            disp('No unmapped voxels to be made fee');
        end
        %}
    else
        disp('No visible points to be added to the voxel map');
    end
    
    % Plot explored map
    %showExploredMap(PointsB, occMap, pose, posePrev)

% Back to top to start new plan
end

% Shutdown ROS node
rosshutdown