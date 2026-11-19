function [roll, pitch, yaw] = rot2rpy(R)
% Matrice di rotazione -> angoli RPY. Eq. (2.23), soluzione con pitch in [-pi/2, pi/2]
c = sqrt(R(3,2)^2 + R(3,3)^2);          % = |cos(pitch)|
if c > 1e-6                              % caso regolare
    yaw   = atan2(R(2,1), R(1,1));
    pitch = atan2(-R(3,1), c);
    roll  = atan2(R(3,2), R(3,3));
else                                     % singolarita: pitch = +-90 gradi
    yaw   = 0;                           % si fissa yaw e si ricava solo roll
    pitch = atan2(-R(3,1), c);
    if R(3,1) < 0                        % pitch = +90
        roll = atan2(R(1,2), R(2,2));
    else                                 % pitch = -90
        roll = atan2(-R(1,2), R(2,2));
    end
end
end
