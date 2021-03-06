function virtualHorizonPlot(rp)

    % Calculated roll + pitch and scale by pi to get factors
    % instead of angles
    rpfactors = rp * 4 / pi;
    intercept = -rpfactors(2);
    
    % get affine transformation matrix
    roll = rp(1);
    R = [cos(roll), -sin(roll), 0;
         sin(roll), cos(roll), 0;
         0, 0, 1];
    T = [1, 0, 0;
         0, 1, intercept;
         0, 0, 1];
     A =T*R;

    % rotated corner vertices
    size = 4;
    topLeft     = A*[-size;  size; 1];
    topRight    = A*[ size;  size; 1];
    bottomLeft  = A*[-size; -size; 1];
    bottomRight = A*[ size; -size; 1];
    middleLeft  = 0.5*(topLeft+bottomLeft);
    middleRight  = 0.5*(topRight+bottomRight);
        
    % plot horizon
    hold off;
    
    % ground plot
    fill(...
        [middleLeft(1) middleRight(1) bottomRight(1) bottomLeft(1)], ...
        [middleLeft(2) middleRight(2) bottomRight(2) bottomLeft(2)], ...
        [0.41176 0.27059 0.17647] ...
        );
    hold on;
    
    % sky plot
    fill( ...
        [middleLeft(1) topLeft(1) topRight(1) middleRight(1)], ...
        [middleLeft(2) topLeft(2) topRight(2) middleRight(2)], ...
        [0.094118 0.48627 0.68235] ...
        );

    % Correct axes
    set(gca, ...
        'Visible', 'off' ...
        );

    axis square;
    xlim([-1 1]);
    ylim([-1 1]);
end