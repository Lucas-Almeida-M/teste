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
% Letra L
% points = [0,2.5,-pi/2;0,0,-pi/5;2.5,0,0];
% np = 3;
%Letra A
% np = 5;
% points = [2.5,5,-1.1071;5,0,-1.1071;2.5+2.5/2,2.5,pi;2.5/2,2.5,pi];
%Letra C
% points = [2.5+1.77,2.5-1.77,atan2(-(2.5-1.77),2.5-(2.5+1.77));
%           2.5,0,atan2(2.5-1.77,2.5-1.77-2.5);
%           2.5-1.77,2.5-1.77,atan2(2.5-(2.5-1.77),0-(2.5-1.77));
%           0,2.5,atan2(2.5+1.77-2.5,2.5-1.77-0);
%           2.5-1.77,2.5+1.77,atan2(5-(2.5+1.77),2.5-(2.5-1.77));
%           2.5,5,atan2(2.5+1.77-(5),2.5+1.77-2.5);
%           2.5+1.77,2.5+1.77,atan2(2.5-1.77-(2.5+1.77),2.5+1.77-(2.5+1.77))];
%  QUADRADO
points = [0,0,0;
          5,0,pi/2;
          5,5,pi;
          0,5,-pi/2;
          0,0,0]
% theta = 3*pi/4;
% points = [0,0,0;
%           3*cos(theta),3*sin(theta),theta];
np = size(points(:,1));
np = np(1);
dtd = 0.1; % [s]
%dtd = 69/7000;
limitV = 0.5 ;
limitW = deg2rad(180);
krho = 1;
kp = 10.1747;
ki = 0.0;%0.05;
kd = 2e-03;%0.5;
erroP = 0.1;
erroBeta = 0.1;
erroAlpha = 0.1;
Pd = points(1,:)';
Rd = Pd;
for a = 2:np
    G = points(a,:);

    dX =  G(1) - Pd(1);
    dY =  G(2) - Pd(2);
    dTheta = ajustaAngulo(Pd(3) - G(3));
    p = sqrt(dX^2 + dY^2);
    gamma = atan2(dY,dX);
    alpha = ajustaAngulo(gamma - Pd(3));
    vd = 0;
    wd = 0;
    c = 0;
    somaAlpha = 0;
    somaDiferenca = 0;
    
    while(c < 1000 && ( p > erroP))

        c = c+1;
    %     r = [-cos(alpha) 0;sin(alpha)/p -1;-sin(alpha)/p 0] * [vd;wd];
    %     p = r(1,1);
    %     alpha = r(2,1);
    %     beta = r(3,1);
    %     vd = kp*p;
    %     wd = kalpha * alpha + kbeta * beta;

        dX     = G(1) - Pd(1);
        dY     = G(2) - Pd(2);
        dTheta = ajustaAngulo(Pd(3) - G(3));
        p      = sqrt(dX^2 + dY^2);
        gamma  = atan2(dY,dX);
        somaDiferenca = alpha;
        alpha  = ajustaAngulo(gamma - Pd(3));
        somaDiferenca = alpha - somaDiferenca;
        somaAlpha = somaAlpha + alpha;
        if a<np
            vd = limitV;
        else
             vd     = min(kp*p,limitV);
        end
        
        wd = kp*alpha + ki*somaAlpha + kd*somaDiferenca;
        wd = sign(wd)*min(abs(wd),limitW);
%         AJUSTA DESLOCAMENTO PARA FRENTE E PARA TRAZ
%         if ~(alpha <= -pi/2 && alpha > -pi) || (alpha >= pi/2 && alpha < pi)
%             beta   = ajustaAngulo(G(3) - gamma);
%             vd     = limita(kp*p,limitV);
%             wd     = limita(kalpha * alpha + kbeta * beta,limitW);
%         else 
%             alpha  = ajustaAngulo(gamma - Pd(3)+pi);
%             beta   = ajustaAngulo(G(3) - gamma+pi);
%             vd     = - limita(kp*p,limitV);
%             wd     = limita(kalpha * alpha + kbeta * beta,limitW);
%         end

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
        line([0,points(2,1)],[0,points(2,2)],'color','r','linewidth',3);
        hold on
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
        plot(Rd(1,:) , Rd(2,:) , 'b' , 'linewidth' , 5);

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
end