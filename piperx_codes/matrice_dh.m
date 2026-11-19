function A = matrice_dh(alpha, a, d, theta)
% Trasformazione DH modificata:
% A = Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)

A = [
    cos(theta),             -sin(theta),              0,          a;
    cos(alpha)*sin(theta),   cos(alpha)*cos(theta),  -sin(alpha), -sin(alpha)*d;
    sin(alpha)*sin(theta),   sin(alpha)*cos(theta),   cos(alpha),  cos(alpha)*d;
    0,                       0,                       0,          1
    ];
end
%È la funzione che usi nel ciclo della cinematica diretta, chiamandola 6 volte con i parametri della riga i della tabella DH
%praticamnete usa i valori che sono del nostro robot all interno della
%funzione matrice_dh, USA MDH QUINDI DH MODIFICATA 
