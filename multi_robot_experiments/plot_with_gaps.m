function plot_with_gaps(traj, max_gap, color, displayName)
    if isempty(traj)
        return;
    end

    n = size(traj,1);
    idx_start = 1;
    hold on;

    % Plot segments without DisplayName
    for i = 2:n
        if norm(traj(i,:) - traj(i-1,:)) > max_gap
            plot(traj(idx_start:i-1,1), traj(idx_start:i-1,2), color, 'LineWidth', 2, 'HandleVisibility','off');
            idx_start = i;
        end
    end
    plot(traj(idx_start:end,1), traj(idx_start:end,2), color, 'LineWidth', 2, 'HandleVisibility','off');

    % Add one invisible plot for legend entry
    plot(nan, nan, color, 'LineWidth', 2, 'DisplayName', displayName);
end
