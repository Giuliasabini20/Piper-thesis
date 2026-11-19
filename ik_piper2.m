function [q, final_error] = ik_piper2(q_initial, T_target, dh, limits, verbose)
% COMPUTE_IK IK iterativa smorzata per 6 giunti con parametri DH modificati.
%
% Input:
%   q_initial : configurazione iniziale [1x6], rad
%   T_target  : posa target [4x4]
%   dh        : tabella [6x4], colonne [alpha, a, d, theta_offset]
%   limits    : limiti articolari [6x2], colonne [min, max], rad
%   verbose   : true per stampare l'errore a ogni iterazione (opzionale)
%
% Output:
%   q          : configurazione trovata [1x6], rad
%   final_error: errore residuo [6x1], posizione [m] e orientamento [rad]
%
% Convenzione della cinematica diretta:
%   A_i = Rx(alpha_i) * Tx(a_i) * Rz(q_i + offset_i) * Tz(d_i)

    if nargin < 5
        verbose = false;
    end

    q = q_initial(:).';

    if numel(q) ~= 6
        error("q_initial deve contenere 6 valori.");
    end
    if ~isequal(size(T_target), [4 4])
        error("T_target deve essere una matrice 4x4.");
    end
    if ~isequal(size(dh), [6 4])
        error("dh deve essere una matrice 6x4 [alpha, a, d, offset].");
    end
    if ~isequal(size(limits), [6 2])
        error("limits deve essere una matrice 6x2 [min, max].");
    end

    % Parametri dell'algoritmo
    max_iterations = 300;
    position_tolerance = 1e-4;       % 0.1 mm
    orientation_tolerance = 1e-3;    % circa 0.057 gradi
    lambda = 0.05;                    % smorzamento
    epsilon = 1e-6;                   % passo Jacobiano numerico

    % Mantieni la configurazione iniziale nei limiti
    q = min(max(q, limits(:,1).'), limits(:,2).');

    success = false;

    for iter = 1:max_iterations
        T_current = fk_mdh(q, dh);
        error_pose = pose_error(T_current, T_target);

        pos_error_norm = norm(error_pose(1:3));
        ori_error_norm = norm(error_pose(4:6));

        if verbose
            fprintf("Iterazione %d: errore posizione %.6g m, orientamento %.6g rad\n", ...
                iter, pos_error_norm, ori_error_norm);
        end

        if pos_error_norm < position_tolerance && ...
           ori_error_norm < orientation_tolerance
            success = true;
            break;
        end

        % Jacobiano numerico 6x6
        J = zeros(6, 6);

        for j = 1:6
            q_plus = q;
            q_plus(j) = q_plus(j) + epsilon;

            T_plus = fk_mdh(q_plus, dh);

            % Derivata della posizione
            J(1:3, j) = ...
                (T_plus(1:3,4) - T_current(1:3,4)) / epsilon;

            % Derivata dell'orientamento nello stesso riferimento
            R_delta = T_plus(1:3,1:3) * T_current(1:3,1:3).';
            J(4:6, j) = rotmat_to_rotvec(R_delta) / epsilon;
        end

        % Damped Least Squares
        Jt = J.';
        delta_q = Jt * ((J * Jt + lambda^2 * eye(6)) \ error_pose);

        q = q + delta_q.';
        q = min(max(q, limits(:,1).'), limits(:,2).');
    end

    % Calcola l'errore finale
    T_current = fk_mdh(q, dh);
    final_error = pose_error(T_current, T_target);

    if ~success
        error("IK non convergente: posizione %.6g m, orientamento %.6g rad.", ...
            norm(final_error(1:3)), norm(final_error(4:6)));
    end
end


function T = fk_mdh(q, dh)
% Cinematica diretta secondo la convenzione DH modificata indicata sopra.

    T = eye(4);

    for i = 1:6
        alpha = dh(i,1);
        a     = dh(i,2);
        d     = dh(i,3);
        theta = q(i) + dh(i,4);

        Rx = [1, 0, 0, 0;
              0, cos(alpha), -sin(alpha), 0;
              0, sin(alpha),  cos(alpha), 0;
              0, 0, 0, 1];

        Tx = [1, 0, 0, a;
              0, 1, 0, 0;
              0, 0, 1, 0;
              0, 0, 0, 1];

        Rz = [cos(theta), -sin(theta), 0, 0;
              sin(theta),  cos(theta), 0, 0;
              0, 0, 1, 0;
              0, 0, 0, 1];

        Tz = [1, 0, 0, 0;
              0, 1, 0, 0;
              0, 0, 1, d;
              0, 0, 0, 1];

        T = T * Rx * Tx * Rz * Tz;
    end
end


function e = pose_error(T_current, T_target)
% Errore: posizione target-corrente e rotazione target rispetto a corrente.

    e_pos = T_target(1:3,4) - T_current(1:3,4);

    R_delta = T_target(1:3,1:3) * T_current(1:3,1:3).';
    e_ori = rotmat_to_rotvec(R_delta);

    e = [e_pos; e_ori];
end


function v = rotmat_to_rotvec(R)
% Converte una matrice di rotazione in vettore di rotazione [rad].

    L = logm(R);
    L = real(L);  % rimuove eventuali residui numerici immaginari
    v = [L(3,2); L(1,3); L(2,1)];
end
