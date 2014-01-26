function plotOrientation(DCM, an, mn)

    global coordSysPlotHandle orientationPlotHandle
    
    % coordinate system and DCM are actually the same
    coordinateSystem = DCM;
    
    % prepare rotation matrices
    R = DCM';

    % prepare vertices
    vertices = [ 0.75  0     0.02;
                 0.25  0.5   0.02;
                 0.25  0.25  0.02;
                -0.75  0.25  0.02;
                -0.75 -0.25  0.02;
                 0.25 -0.25  0.02;
                 0.25 -0.5   0.02;
                 0.75  0     0.02
                ];
    N = size(vertices,1);
    vertices = [vertices; vertices];
    for vi=N+1:2*N
        vertices(vi,:) = vertices(vi,:) + [0 0 -0.04];
    end

    % transform vertices
    for vi=1:size(vertices,1)
        vx = vertices(vi,:);
        v = R*vertices(vi,:)';
        vertices(vi,:) = v';
    end    
    
    % Set data and draw
    for p=0:3
        set(orientationPlotHandle(p*2+1), 'XData', vertices(1:8,1), 'YData', vertices(1:8,2), 'ZData', vertices(1:8,3));
        set(orientationPlotHandle(p*2+2), 'XData', vertices((N+1):2*N,1), 'YData', vertices((N+1):2*N,2), 'ZData', vertices((N+1):2*N,3));
    end
    
    % Set coordinate system
    for ca=1:3
        set(coordSysPlotHandle(ca), 'XData', [0 coordinateSystem(ca,1)], ...
            'YData', [0 coordinateSystem(ca,2)], ...
            'ZData', [0 coordinateSystem(ca,3)]);
    end
    
    % accelerometer axis
    if exist('an', 'var')
        set(coordSysPlotHandle(4), 'XData', [0 an(1)*2], ...
                'YData', [0 an(2)*2], ...
                'ZData', [0 an(3)*2]);
    end
    
    % magnetometer
    if exist('mn', 'var')
        set(coordSysPlotHandle(5), 'XData', [0 mn(1)*2], ...
            'YData', [0 mn(2)*2], ...
            'ZData', [0 mn(3)*2]);
    end
    drawnow;
end