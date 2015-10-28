function[error]=cost(Gp,param)
%--------------------------------------------------------------------------
%   Function: Cost Function.
%   Author: Fernando Martin Monar.
%   Date: November, 2013
%--------------------------------------------------------------------------
% -> Description:   
%   It calculates the cost function given a process Gp and a PID controller. 
%   The DE-based scan matching algorithm calls it to estimate the cost.
%   The PID Controller is defined by the following Expression:
%       PID(s)=Kp + Ki/s + Kd*s
%--------------------------------------------------------------------------
% -> Usage:
%       Inputs: Gp -> transfer function of the process to control
%               param = [Kp Ki Kd]; parameters of the PID
% Output:  error. The error is computed as the average squared error.
%--------------------------------------------------------------------------
% -> See also: Control_PID
%--------------------------------------------------------------------------

% Four different cost functions implemented:
%   option 1: squared error of the step response
%   option 2: robustness of variations in the gain of the plant, which
%   means a plane argument in the bode diagram around the phase margin frequency
%   option 3: like option 2, but discarding errors when negative phase
%   margin, inestable systems
%   option 4: mixture between step response and frecuency, giving different
%   weights to each component of the error function.

% Initialization parameters.
option=1;
warning('OFF');
PHASE=zeros(11,1);
fr_w=0.2;

%pid(Kp,Ki,Kd) 
%PID=pid(param(1),param(2),param(3));
PID=tf([param(2) param(3) param(1)],[1 0]);
M=feedback(PID*Gp,1);
[~,Pm,~,Wpm] = margin(PID*Gp);

if option==1
    %OPTION 1: step response
    [Y]=step(M,0:0.05:2);
    error=(Y-1)'*(Y-1);  % The squared error between the output and a unitary 
                         % step is computed
end
if option==2
    %OPTION 2: frequency response
    for i=1:11
        [~,phase_i] = bode(PID*Gp,Wpm+(i-6)); %phases around the phase margin
        PHASE(i)=abs(phase_i-(Pm-180));  %weighting the slope of the phases
    end
    error=PHASE'*PHASE;
end 
if option==3
    %OPTION 3: frequency response: phase margin + plain
    % How to include the phase margin in the cost function
    if Pm>30
        if Wpm>15 && Wpm<20
            for i=1:11
                Wint=Wpm+(i-6)/2;
                if Wint>0
                    [~,phase_i] = bode(PID*Gp,Wint); %phases around the phase margin
                    PHASE(i)=abs(phase_i-(Pm-180));  %weighting the slope of the phases
                else
                    PHASE(i)=10000;
                end
            end
            error=PHASE'*PHASE;
        else
            error=100000000000;
        end
    else
        error=100000000000;
    end
end 
if option==4
    %OPTION 4: mixture between step response and phase response
    [Y]=step(M);
    size_Y=size(Y,1);
    error_st=((Y-1)'*(Y-1))/(size_Y);
    
    if Pm>30
        if Wpm>10 && Wpm<15
            for i=1:11
                Wint=Wpm+(i-6)/10;
                if Wint>0
                    [~,phase_i] = bode(PID*Gp,Wint); %phases around the phase margin
                    PHASE(i)=abs(phase_i-(Pm-180));  %weighting the slope of the phases
                else
                    PHASE(i)=10000;
                end
            end
            error_fr=PHASE'*PHASE/11;
        else
            error_fr=100000000000;
        end
    else
        error_fr=100000000000;
    end
    
    error=(1-fr_w)*error_st+fr_w*error_fr;
end

end