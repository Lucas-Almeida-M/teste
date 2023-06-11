function M2 = Rz2D(M1 , th)
% Matriz de rota��o no eixo-z com duas dimens�es e coord. generalizadas.
Rz = [cos(th) -sin(th) 0
sin(th) cos(th) 0
0 0 1];
% Aplicando a rota��o
M2 = Rz * M1;
end