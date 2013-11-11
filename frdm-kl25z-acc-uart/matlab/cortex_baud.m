function [br, osr, baud_error] = cortex_baud(clock, baudrate, osr)

    clc; home;

    % for FRDM-KL25Z (Cortex-M0+)

    if ~exist('clock', 'var')
        disp('using default clock of PLL/2');
        clock = 24000000;
    end

    if ~exist('baudrate', 'var')
        disp('using default baud rate');
        baudrate = 115200;
    end
    
    if ~exist('osr', 'var')
        disp('using osr scan');
        osr = 3:31;
    end
    
    br = 1:8192;

    found_br = 0;
    found_osr = 0;
    found_error = 1;
    min_error = 1E20;
    
    % loop OSR
    for osr_value = osr
        
        % early exit
        if found_error == 0
            break;
        end
        
        % loop BR
        for br_value = br
            rate = clock/(br_value*(osr_value+1));
            if rate < 9600 
                continue 
            end
            
            % best match
            if rate == baudrate
                found_br = br_value;
                found_osr = osr_value;
                found_error = 0;
                break
            end
            
            % find smallest error
            error = (baudrate-rate)^2;
            if error < min_error
                min_error = error;
                found_br = br_value;
                found_osr = osr_value;
                found_error = min_error;
            end
        end
    end
    
    br = found_br;
    osr = found_osr;
    
    found = int32(clock)/int32((br*(1+osr)));
    baud_error = abs(baudrate-found);
    
    disp(sprintf('Target clock:       %u Hz', clock));
    disp(sprintf('Target baud rate:   %u', baudrate));
    disp(sprintf('Found baud rate:    %u', found));
    disp(sprintf('SBR:                %u', br));
    disp(sprintf('OSR (oversampling): %u', osr));
    disp(sprintf('Error:              %.2f (%.2f percent)', baud_error, (double(found)-baudrate) * 100 / baudrate));
end