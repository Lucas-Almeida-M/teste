function [DeltaErro,ErroReta] = SimulaProcesso(K, plot_sim)

    % Desenho do robô
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

    %Velocidades limite do robô
    limitV = 0.5;
    limitW = deg2rad(180);

    %Ganhos do robô
    krho = 1;
    kp = K(1);
    ki = K(2);
    kd = K(3);

    %Posição Inicial
    P = [0;0;0];

    %Histórico de posições
    R = P;

    Pinicial = P;
    Raio = 3;
    Angulo = deg2rad(45);

    G = [Pinicial(1)+Raio*cos(Angulo);Pinicial(2)+Raio*sin(Angulo);];

    %Reta entre os pontos 
    a = G(2) - Pinicial(2);
    b = Pinicial(1) - G(1);
    c = G(1)*Pinicial(2)-Pinicial(1)*G(1);

    % Erro entre o robô e o percurso calculado
    Erro = abs(a*P(1)+b*P(2) + c)/sqrt(a^2+b^2);
    ErroReta = [Erro,0];

    Integral = 0;
    Derivada = 0;

    %Tempo inicial
    t = 0;
    
    %Tempo máximo
    tmax = 10*sqrt((G(1)-Pinicial(1))^2+(G(2)-Pinicial(2))^2)/limitV;

    %Período de amostragem
    dt = 0.1;

    %Parâmetro de erro aceitável
    erroP = 0.1;

    % Simulação do controle PID aplicado ao robô na trajetória estipulada
    dX =  G(1) - P(1);
    dY =  G(2) - P(2);
    p = sqrt(dX^2 + dY^2);
    gamma = ajustaAngulo(atan2(dY,dX));
    alpha = ajustaAngulo(gamma - P(3));
    oldAlpha = alpha;
    h = 0;
    while (p>erroP) && (t<=tmax)
        t = t+dt;
        h = h+1;
        dX =  G(1) - P(1);
        dY =  G(2) - P(2);
        p = sqrt(dX^2 + dY^2);
        gamma = ajustaAngulo(atan2(dY,dX));
        alpha = ajustaAngulo(gamma - P(3));

        Integral = Integral + alpha;
        Derivada = alpha - oldAlpha; oldAlpha = alpha;

        v = min(p*krho,limitV);
        %Ponto atrás:
        if abs(alpha)>pi/2
            v = -v;
            alpha = ajustaAngulo(alpha+pi);
        end

        w = kp*alpha + ki*Integral + kd*Derivada;
        w = sign(w)*min(abs(w),limitW);

        dP = [v * cos(P(3)); v * sin(P(3)); w];
        P = P + dP * dt;
        P(3) = ajustaAngulo(P(3));

        R = [R , P];   
        % Calculo do erro para ser utilizado no processo de otimização dos
        % ganhos do controlador 
        Erro = [Erro;abs(w)+abs(a*P(1)+b*P(2) + c)/sqrt(a^2+b^2)];
        ErroReta = [ErroReta;abs(a*P(1)+b*P(2) + c)/sqrt(a^2+b^2),t];
        if plot_sim == 1
            PlotSimulacao(P,G,R,Corpod,RodaEd,RodaDd);
        end
    end
    DeltaErro = mean(Erro);
end


