function R = rpy2rot(roll, pitch, yaw)
% Angoli RPY (assi fissi XYZ) -> matrice di rotazione. Eq. (2.22): R = Rz(yaw)*Ry(pitch)*Rx(roll)
Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
R = Rz * Ry * Rx;
end
