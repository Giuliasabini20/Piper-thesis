% parametri_piper.m
% Parametri del Piper X: tabella DH (standard), offset dei giunti, limiti dei giunti.


% Parametri DH modificati [alpha, a, d, theta_offset]
dh = [
    0,       0,        0.123,   pi;
    pi/2,    0,        0,       0.1357866158;
    0,       0.28503,  0,       2.8380798967;
    0,       0.27364,  0,       0.0806342114;
    -pi/2,    0.07466,  0,       pi/2;
    pi/2,    0,        0.03526, 0
    ];
% angoli DH = angoli del robot + offset
offset = dh(:, 4)';

% limiti dei giunti in angoli del ROBOT (q): [min max] per ogni giunto

limits = [
    -154/180*pi,  154/180*pi;   % Giunto 1
    0,          195/180*pi;   % Giunto 2
    -175/180*pi,    0;           % Giunto 3
    -102/180*pi,  102/180*pi;   % Giunto 4
    -75/180*pi,   75/180*pi;   % Giunto 5
    -120/180*pi,  120/180*pi    % Giunto 6
    ];
