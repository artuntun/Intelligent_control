function F = tracklsq(pid,y)

%% Par?metros del PID
Kp = pid(1);
Ki = pid(2);
Kd = pid(3);

%% Funci?n de transferencia del sistema en bucle cerrado
Gc = tf([Kd, Kp, Ki],[1,0]);      
Gp = tf(6,[1, 5, 6]);       
Gba = Gc*Gp;                
Gbc = feedback(Gba,1);      

%% Respuesta del sistema ante entrada escal?n unitario
[Y] = step(Gbc);

%% Funci?n objetivo
%% Opci?n 1: Suma de los errores en valor absoluto
% error = Y - 1;
% F = sum(abs(error));

%% Opci?n 2: Error cuadr?tico
% error = Y - 1;
% F = error' * error;

%% Opci?n 3: Suma ponderada del factor de sobreoscilaci?n y el tiempo de establecimiento
% vf = Y(length(Y));
% sys = stepinfo(Gbc);
% ts = sys.SettlingTime;
% Mp = sys.Overshoot;
% F = 0.3*ts+0.4*Mp;

%% Opci?n 4: Suma ponderada del factor de sobreoscilaci?n, el tiempo de establecimiento y el error en r?gimen permanente
% vf = Y(length(Y));
% sys = stepinfo(Gbc);
% ts = sys.SettlingTime;
% Mp = sys.Overshoot;
% F = 0.3*ts + 0.4*Mp + 0.3*(abs(1-vf));

%% Opci?n 5: 
 vf = Y(length(Y));
 error = Y - 1;
 sys = stepinfo(Gbc);
 ts = sys.SettlingTime;
 tr = sys.RiseTime;
 Mp = sys.Overshoot;
 F = 0.4*tr + 0.3*Mp + 0.3*(abs(1-vf));

