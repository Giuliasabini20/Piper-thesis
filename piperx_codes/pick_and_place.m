clear;
clc;
close all;

% Caricamento parametri robot
run('parametri_piper.m');

% Configurazione iniziale dei 6 giunti
q_initial = zeros(1,6);
% Posa iniziale A
A = [0.30; 0.10; 0.20];   % XYZ in metri
RPY_A = [0; 0; 0];        % RPY in gradi

% Posa finale B
B = [0.40; 0.20; 0.30];   % XYZ in metri
RPY_B = [0; 0; 0];       % RPY in gradi
Tf = 4;       % Durata del movimento [s]
N = 100;      % Numero di campioni

t = linspace(0,Tf,N);

tau = t/Tf;

% Legge temporale cubica
s = 3*tau.^2 - 2*tau.^3;
P = zeros(3,N);
RPY = zeros(3,N);

for i = 1:N

    % Interpolazione posizione
    P(:,i) = A + s(i)*(B-A);

    % Interpolazione orientamento
    RPY(:,i) = RPY_A + s(i)*(RPY_B-RPY_A);

end
% Legge temporale cubica
s = 3*tau.^2 - 2*tau.^3;

T = zeros(4,4,N);

for i = 1:N

    roll  = deg2rad(RPY(1,i));
    pitch = deg2rad(RPY(2,i));
    yaw   = deg2rad(RPY(3,i));

    Rx = [1 0 0;
        0 cos(roll) -sin(roll);
        0 sin(roll) cos(roll)];

    Ry = [cos(pitch) 0 sin(pitch);
        0 1 0;
        -sin(pitch) 0 cos(pitch)];

    Rz = [cos(yaw) -sin(yaw) 0;
        sin(yaw) cos(yaw) 0;
        0 0 1];

    R = Rz*Ry*Rx;

    T(:,:,i) = [R P(:,i);
        0 0 0 1];

end
Q = zeros(6,N);

q_start = q_initial;

for i = 1:N

    [q, final_error] = ik_piper2( ...
        q_start, T(:,:,i), dh, limits, false);

    Q(:,i) = q(:);

    % La soluzione precedente diventa
    % la configurazione iniziale del punto successivo
    q_start = q;

end
dt = t(2)-t(1);

% Velocità articolari
Qd = zeros(6,N);

Qd(:,1) = (Q(:,2)-Q(:,1))/dt;
Qd(:,N) = (Q(:,N)-Q(:,N-1))/dt;

for i = 2:N-1
    Qd(:,i) = (Q(:,i+1)-Q(:,i-1))/(2*dt);
end
% Verifica dei limiti di velocità

qd_reale_max = max(abs(Qd), [], 2);

disp('Velocita massime raggiunte dai giunti:');
disp(qd_reale_max);

if any(isnan(qd_max))
    warning('Inserire i limiti di velocita reali del Piper.');
else
    rapporto = qd_reale_max ./ qd_max;

    for j = 1:6
        fprintf('Giunto %d: %.3f / %.3f rad/s\n', ...
            j, qd_reale_max(j), qd_max(j));
    end

    if any(rapporto > 1)
        warning('La traiettoria supera i limiti di velocita!');
    else
        disp('Tutti i limiti di velocita sono rispettati.');
    end
end

% Accelerazioni articolari
Qdd = zeros(6,N);

Qdd(:,1) = (Qd(:,2)-Qd(:,1))/dt;
Qdd(:,N) = (Qd(:,N)-Qd(:,N-1))/dt;

for i = 2:N-1
    Qdd(:,i) = (Qd(:,i+1)-Qd(:,i-1))/(2*dt);
end
% Posizione articolare
figure;
plot(t,Q');
title('Posizione dei giunti');
xlabel('Tempo [s]');
ylabel('Angolo [rad]');
legend('J1','J2','J3','J4','J5','J6');
grid on;

% Velocità articolare
figure;
plot(t,Qd');
title('Velocità dei giunti');
xlabel('Tempo [s]');
ylabel('Velocità [rad/s]');
legend('J1','J2','J3','J4','J5','J6');
grid on;

% Accelerazione articolare
figure;
plot(t,Qdd');
title('Accelerazione dei giunti');
xlabel('Tempo [s]');
ylabel('Accelerazione [rad/s^2]');
legend('J1','J2','J3','J4','J5','J6');
grid on;

% Stampa tempo di trasferimento
fprintf('Tempo di trasferimento: %.2f secondi\n',Tf);
