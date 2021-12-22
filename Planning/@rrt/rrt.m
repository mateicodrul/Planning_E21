classdef rrt
    % Properties
    properties (SetAccess = private)
        
    end
    % Methods
    methods        
        % Constructor 
        function [obj, root_ID] = rrt()
           
        end
        
        % Functions defined in external files
        [obj, ID] = initialize(obj, parent, data)
        [obj, ID] = iterate(obj, parent, data)
        [obj, ID] = gain(obj, parent, data)
    end   
end
