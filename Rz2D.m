function M2 = Rz2D(M1 , th)
% Matriz de rotação no eixo-z com duas dimensões e coord. generalizadas.
Rz = [cos(th) -sin(th) 0
sin(th) cos(th) 0
0 0 1];
% Aplicando a rotação
M2 = Rz * M1;
end