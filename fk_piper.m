function [T, p, R, rpy] = fk_piper(q, dh)
% FK_PIPER  Cinematica diretta del Piper.
%   q  : vettore con i 6 angoli del robot, in RADIANTI (riga o colonna)
%   dh : (facoltativo) tabella [alpha a d offset]; se manca la carica da parametri_piper
%   T  : matrice 4x4 della flangia rispetto alla base
%   p  : posizione (3x1, metri)      R : rotazione (3x3)
%   rpy: [roll pitch yaw] in radianti

if nargin < 2
    parametri_piper;                         % crea dh (e offset, lim) dentro la funzione
end
q = q(:)';                                   % forza q a vettore riga
if numel(q) ~= 6
    error('q deve avere 6 elementi (uno per giunto)');
end

T = eye(4);
for i = 1:6
    T = T * matrice_dh(dh(i,1), dh(i,2), dh(i,3),q(i) + dh(i,4));
end
p = T(1:3,4);
R = T(1:3,1:3);
[ro, pit, ya] = rot2rpy(R);
rpy = [ro, pit, ya];
end
