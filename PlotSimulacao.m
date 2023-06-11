function sim = PlotSimulacao(Pd,G,Rd,Corpod,RodaEd,RodaDd,areas)
    % Definir a matriz de áreas

% Exemplo de matriz de áreas

% Definir tamanho do quadrado em centímetros
tamQuadrado = 1;

% Definir tamanho da área segmentada em quadrados
tamArea = size(areas, 1);

% Calcular tamanho total da área em centímetros
tamTotal = tamQuadrado * tamArea;

%Criar figura



    rd = 1/2 * (195/1000); % [m]
    % Metade do comprimento do eixo das rodas (l) [m]
    ld = 1/2 * (381/1000); % [m]
    RoboCd = T2D(Rz2D(Corpod , Pd(3)) , Pd(1) , Pd(2));
    % Roda Esquerda do robô
    RoboEd = T2D(Rz2D(RodaEd , Pd(3)) , Pd(1) , Pd(2));
    % Roda Direita do robô
    RoboDd = T2D(Rz2D(RodaDd , Pd(3)) , Pd(1) , Pd(2));
    % Plot do corpo do robô
    fill(RoboCd(1,:) , RoboCd(2,:) , 'c')
    hold on;
    % Plot roda esquerda
    fill(RoboEd(1,:) , RoboEd(2,:) , 'c')
    % Plot roda direita
    fill(RoboDd(1,:) , RoboDd(2,:) , 'c')
    %-------------------------------------------------
    
%     plot(G(1),G(2),'--ro' ,'linewidth' , 2 ,'markersize' , 25);
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
    plot(Rd(1,:) , Rd(2,:) , 'b' , 'linewidth' , 1);

    % Posição atual do robô: onde ele está no momento atual 't'
%     plot(Pd(1) , Pd(2) , 'or' , 'linewidth' , 2 , 'markersize' , 15)
%     % Orientação atual do robô: para onde ele aponta no momento atual 't'
%     plot([Pd(1) Pd(1)+0.1*cos(Pd(3))] , [Pd(2) Pd(2)+0.1*sin(Pd(3))] , 'r' , 'linewidth' , 2)
%     % Eixo das rodas
%     plot([Pd(1) Pd(1)+ld*cos(Pd(3)+pi/2)] , [Pd(2) Pd(2)+ld*sin(Pd(3)+pi/2)] , 'k' , 'linewidth' , 2)
%     plot([Pd(1) Pd(1)+ld*cos(Pd(3)-pi/2)] , [Pd(2) Pd(2)+ld*sin(Pd(3)-pi/2)] , 'k' , 'linewidth' , 2)
    axis equal; xlabel('x [m]'); ylabel('y [m]');grid on;
    %title(['Tempo: t = ' num2str(t)])
    % grid de plot
% Percorrer a matriz de áreas e plotar os quadrados coloridos
for i = 1:tamArea
    for j = 1:tamArea
        x = (j - 1) * tamQuadrado;
        y = tamTotal - (i * tamQuadrado);
        
        % Obter o valor do quadrado atual
        valor = areas(i, j);
        
        % Definir a cor com base no valor do quadrado
        if valor == -1
            cor = [0 0 0]; % Branco
        elseif valor == 0
            cor = [0 1 0]; % Verde
        elseif valor == 1
            cor = [0 0 1]; % Azul
        else
            cor = []; % Sem cor
        end
        
        if ~isempty(cor)
            % Plotar o quadrado com a cor correspondente
            rectangle('Position', [x, y, tamQuadrado, tamQuadrado], 'FaceColor', cor);
            hold on;
        end
    end
end
% % Configurar os eixos
axis equal;
    rd = 1/2 * (195/1000); % [m]
    % Metade do comprimento do eixo das rodas (l) [m]
    ld = 1/2 * (381/1000); % [m]
    RoboCd = T2D(Rz2D(Corpod , Pd(3)) , Pd(1) , Pd(2));
    % Roda Esquerda do robô
    RoboEd = T2D(Rz2D(RodaEd , Pd(3)) , Pd(1) , Pd(2));
    % Roda Direita do robô
    RoboDd = T2D(Rz2D(RodaDd , Pd(3)) , Pd(1) , Pd(2));
    % Plot do corpo do robô
    fill(RoboCd(1,:) , RoboCd(2,:) , 'c')
    hold on;
    % Plot roda esquerda
    fill(RoboEd(1,:) , RoboEd(2,:) , 'c')
    % Plot roda direita
    fill(RoboDd(1,:) , RoboDd(2,:) , 'c')
    %-------------------------------------------------
    x = 0:1:30;
    y = 0:1:30;
    hold on
    for n = 1:numel(x); %// loop over vertical lines
    plot([x(n) x(n)], [y(1) y(end)], 'k-'); %// change 'k-' to whatever you need
    end
    hold on
    for n = 1:numel(y); %// loop over horizontal lines
    plot([x(1) x(end)], [y(n) y(n)], 'k-'); %// change 'k-' to whatever you need
    end
    hold on;

%     plot(G(1),G(2),'--ro' ,'linewidth' , 2 ,'markersize' , 25);
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
    plot(Rd(1,:) , Rd(2,:) , 'b' , 'linewidth' , 3);

    % Posição atual do robô: onde ele está no momento atual 't'
%     plot(Pd(1) , Pd(2) , 'or' , 'linewidth' , 2 , 'markersize' , 15)
%     % Orientação atual do robô: para onde ele aponta no momento atual 't'
%     plot([Pd(1) Pd(1)+0.1*cos(Pd(3))] , [Pd(2) Pd(2)+0.1*sin(Pd(3))] , 'r' , 'linewidth' , 2)
%     % Eixo das rodas
%     plot([Pd(1) Pd(1)+ld*cos(Pd(3)+pi/2)] , [Pd(2) Pd(2)+ld*sin(Pd(3)+pi/2)] , 'k' , 'linewidth' , 2)
%     plot([Pd(1) Pd(1)+ld*cos(Pd(3)-pi/2)] , [Pd(2) Pd(2)+ld*sin(Pd(3)-pi/2)] , 'k' , 'linewidth' , 2)
    axis equal; xlabel('x [m]'); ylabel('y [m]');grid on;
    
    hold off;
    drawnow
end


