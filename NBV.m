%%% NBV planning %%%

% Create RRT tree object
rrt_tree = tree(struct('state', zeros(6,1), 'gain', 0, 'distance', 0));
% Counter for the number of plans executed
rrtInitializationCounter = 0;
% Flag indicating that we must return to previous pose
backtoPosePrev = false;
% Inifnite loop for NBV planning
while rrtInitializationCounter >= 0
    % After the first run
    if rrtInitializationCounter
        % Erase all info from RRT tree, apart from best branch
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
        if ~collison_free_flag
            continue;
        end
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
        
        % Plot tree
        if params.showRRT
            showRRTTree(newParent, newNode, rrtInitializationCounter, rrt_tree, 'initialize');
        end
    end
    %%%
    
    % Loop for RRT tree construction
    loopCounter = 0;
    while ~(rrt_tree.bestGain > params.gainZero) || rrt_tree.iterationCounter < params.treeIterations
        % If maxIterations is reached, it means that rrt tree best gain stayed
        % 0, so the exploration is (hopefully) over. Terminate also if
        % the inmapped volume is below a certin percentage
        if rrt_tree.iterationCounter > params.maxIterations || mapping_progress_vect(end,3) <= params.minVUnmapped
            disp(['nbvPlanner -- Best gain still 0 after ' ...
                  num2str(params.maxIterations) ...
                  ' iterations or unmapped volume is very small. Exploration terminated.']);
            % Exit the program here
            return;
        end
        % If we fail to add a new node to the tree for a certain number of 
        % iterations that depends on rrt_tree.iterationCounter, we should go
        % back to the previous pose and rebuild a tree from there
        if loopCounter > 100 * (rrt_tree.iterationCounter + 1)
            disp(['nbvPlanner -- Exceeding maximum failed iterations during tree' ...
                  'construction. Return to previous point!'])
            %%% RRT getPathBackToPrevious function %%%
            backtoPosePrev = true;
            % Exit the nbv function here
            % return
            break;
        end
    
        %%% RRT iterate function %%%
        % Grab tree root
        rootNode = rrt_tree.get(1);
        % Select random state within the map
        diffXYZ = params.maxXYZ - params.minXYZ;
        newState = zeros(6,1);
        newState(1:3) = params.minXYZ + diffXYZ .* rand(3,1);
        % Create Kd-tree with the current nodes in the RRT tree, which are
        % stored in kd_tree_nodes
        clear kd_tree
        kd_tree = KDTreeSearcher(kd_tree_nodes);
        %disp('Nodes in Kd-tree:')
        %disp(kd_tree_nodes)
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
        if ~collison_free_flag
            loopCounter = loopCounter + 1;
            continue;
            %return;
        end
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
        
        % Plot tree
        if params.showRRT
            showRRTTree(newParent, newNode, rrtInitializationCounter, rrt_tree, 'iterate');
        end
        disp(['Iteration ' num2str(rrt_tree.iterationCounter)])
        disp(['Best node index is ' num2str(rrt_tree.bestNodeIdx)...
              ' with gain ' num2str(rrt_tree.bestGain)])
        %%%

        loopCounter = loopCounter + 1;
    end
    
    % Go back to previous pose
    if backtoPosePrev
        backtoPosePrev = false;
        poseRef = posePrev;
        % Publish pose reference message and wait for the drone to move
        gazeboMoveDroneSampled(pubTrajectory, pose, poseRef, params);
        % Decode pose messages to get pose of the drone in the world frame
        pose = gazeboGetPose(subPose, poseRef);
        pose_vect = [pose_vect; pose.'];
        continue;
    end

    %%% RRT extractBestBranch function %%%
    rrt_tree = rrt_tree.extractBestBranch();
    
    % Move to the new pose by executing the first edge of the best branch
    posePrev = pose;
    poseRef = rrt_tree.bestEdgeState;
    % Publish pose reference message and wait for the drone to move
    gazeboMoveDroneSampled(pubTrajectory, pose, poseRef, params);
    % Decode pose messages to get pose of the drone in the world frame
    pose = gazeboGetPose(subPose, poseRef);
    pose_vect = [pose_vect; pose.'];

    % Show best branch
    if params.showRRT
        showRRTTree(newParent, newNode, rrtInitializationCounter, rrt_tree, 'best');
    end

    % Get scan of visible environment from Gazebo camera
    PointsC = gazeboGetCameraPointsFromDepth(subDepth, params, 'depth');

    % Add visible pointcloud in camera coordinates to the occupancy map
    addPointsToMap(occMap, pose, PointsC, params);

    % Plot explored map
    if params.showMap
        showExploredMap(occMap, pose, posePrev, PointsC, params);
    end
    
    % Mapping progress
    mapping_progress_vect = [mapping_progress_vect; mappingProgress(occMap, voxels)];
    elapsed_time = toc;
    time_vect = [time_vect; elapsed_time];
    % Plot progress
    if params.showProgress
        showMapProgress(time_vect(end-1:end), mapping_progress_vect(end-1:end,:));
    end
    disp(['Unmapped (%): ', num2str(mapping_progress_vect(end,3))]);

    % Back to top to start new plan
end