function [linvel, angvel] = control_governor(linvel, angvel)
    maxlinvel = 0.26;
    maxangvel = 1.82;
    
    linvel = max(min(linvel, maxlinvel), -maxlinvel);
    angvel = max(min(angvel, maxangvel), -maxangvel);
        
    scale = abs(linvel)/maxlinvel + abs(angvel)/maxangvel;
    if (scale > 1)
        linvel = linvel / scale;
        angvel = angvel / scale;
    end
        
end