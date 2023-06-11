clear all;
clc;

%% Diferencial
clear all;
close all;
clc;

Corpod = [100 , 227.5 , 227.5 , 100 , -200 , -227.5 , -227.5 , -200

-190.5 , -50 , 50 , 190.5 , 190.5 , 163 , -163 , -190.5]/1000;
Corpod = [Corpod ; [1 1 1 1 1 1 1 1]]; % linha de 1s para transf. homogenea.
% Roda esquerda
RodaEd = [ 97.5 97.5 -97.5 -97.5
170.5 210.5 210.5 170.5]/1000;

RodaEd = [RodaEd; [1 1 1 1]];
% Roda direita
RodaDd = [ 97.5 97.5 -97.5 -97.5
-170.5 -210.5 -210.5 -170.5]/1000;

RodaDd = [RodaDd; [1 1 1 1]];

rd = 1/2 * (195/1000); % [m]
% Metade do comprimento do eixo das rodas (l) [m]
ld = 1/2 * (381/1000); % [m]

dtd = 0.1; % [s]
%dtd = 69/7000;

Pd = [0;0;0];
G = [10;10;0];
limitV = 100;
limitW = 100;
erroP = 0.01;
erroAlpha = 0.01;
kp = 0.1;
kalpha = 0.8;
Rd = Pd;
dX =  G(1) - Pd(1);
dY =  G(2) - Pd(2);
dTheta = ajustaAngulo(Pd(3) - G(3));
p = sqrt(dX^2 + dY^2);
gamma = atan2(dY,dX);
alpha = ajustaAngulo(gamma - Pd(3));
vd = 0;
wd = 0;
c = 0;
while(c < 800 && ( p > erroP))
 
    c = c+1;
    dX     = G(1) - Pd(1);
    dY     = G(2) - Pd(2);
    dTheta = ajustaAngulo(Pd(3) - G(3));
    p      = sqrt(dX^2 + dY^2);
    gamma  = atan2(dY,dX);
    alpha  = ajustaAngulo(gamma - Pd(3));
    vd     = min(kp*p , limitV);
    wd     = min(kalpha * alpha , limitW);
    
    dPd = [vd * cos(Pd(3)); vd * sin(Pd(3)); wd];

    Pd = Pd + dPd * dtd;

    Rd = [Rd , Pd];
    
    %-------------------------------------------------
    
     RoboCd = T2D(Rz2D(Corpod , G(3)) , G(1) , G(2));
    % Roda Esquerda do robô
    RoboEd = T2D(Rz2D(RodaEd , G(3)) , G(1) , G(2));
    % Roda Direita do robô
    RoboDd = T2D(Rz2D(RodaDd , G(3)) , G(1) , G(2));
    % Plot do corpo do robô
    fill(RoboCd(1,:) , RoboCd(2,:) , 'c')
    hold on;
    % Plot roda esquerda
    fill(RoboEd(1,:) , RoboEd(2,:) , 'c')
    % Plot roda direita
    fill(RoboDd(1,:) , RoboDd(2,:) , 'c')
    %-------------------------------------------------
    plot(G(1),G(2),'--ro' ,'linewidth' , 2 ,'markersize' , 25);
    RoboCd = T2D(Rz2D(Corpod , Pd(3)) , Pd(1) , Pd(2));
    % Roda Esquerda do robô
    RoboEd = T2D(Rz2D(RodaEd , Pd(3)) , Pd(1) , Pd(2));
    % Roda Direita do robô
    RoboDd = T2D(Rz2D(RodaDd , Pd(3)) , Pd(1) , Pd(2));
    % Plot do corpo do robô
    fill(RoboCd(1,:) , RoboCd(2,:) , 'y')
    hold on;
    % Plot roda esquerda
    fill(RoboEd(1,:) , RoboEd(2,:) , 'y')
    % Plot roda direita
    fill(RoboDd(1,:) , RoboDd(2,:) , 'y')
   
    % Histórico de posições: o 'rastro' do robô, por onde ele passou.
    plot(Rd(1,:) , Rd(2,:) , 'b' , 'linewidth' , 2);

    % Posição atual do robô: onde ele está no momento atual 't'
    plot(Pd(1) , Pd(2) , 'or' , 'linewidth' , 2 , 'markersize' , 15)
    % Orientação atual do robô: para onde ele aponta no momento atual 't'
    plot([Pd(1) Pd(1)+0.1*cos(Pd(3))] , [Pd(2) Pd(2)+0.1*sin(Pd(3))] , 'r' , 'linewidth' , 2)
    % Eixo das rodas
    plot([Pd(1) Pd(1)+ld*cos(Pd(3)+pi/2)] , [Pd(2) Pd(2)+ld*sin(Pd(3)+pi/2)] , 'k' , 'linewidth' , 2)
    plot([Pd(1) Pd(1)+ld*cos(Pd(3)-pi/2)] , [Pd(2) Pd(2)+ld*sin(Pd(3)-pi/2)] , 'k' , 'linewidth' , 2)
    hold off;axis equal; xlabel('x [m]'); ylabel('y [m]');grid on;
    %title(['Tempo: t = ' num2str(t)])
    drawnow
end