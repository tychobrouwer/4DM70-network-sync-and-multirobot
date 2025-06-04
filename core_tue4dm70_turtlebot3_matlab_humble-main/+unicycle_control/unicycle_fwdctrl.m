% 

function [linvel, angvel] = unicycle_fwdctrl(position, orientation, goal, varargin)
%     
    lingain = 1.0;
    anggain = 1.0;
    tol = 1e-3;
    
    for k=1:2:length(varargin)
        switch lower(varargin{k})
            case 'lingain'
                lingain = varargin{k+1};
            case 'anggain'
                anggain = varargin{k+1};
            case 'tol'
                tol = varargin{k+1};
        end
    end
    
    position = reshape(position, 1, 2);
    goal = reshape(goal, 1, 2);
    
    if (norm(position - goal) < tol)
        linvel = 0;
        angvel = 0;
        return;
        end
    
    forwardDirection = [cos(orientation), sin(orientation)];
    normalDirection = [-sin(orientation), cos(orientation)];
    
    forwardError = sum((goal - position).*forwardDirection);
    normalError = sum((goal - position).*normalDirection);
    
    linvel = max(lingain*forwardError, 0.0);
    angvel = anggain*atan2(normalError, forwardError);
    
end