clear all
close all 
clc

% Chutes iniciais para os ganhos do controlador
K = [0,0,0];

% Passo para mudanças no vetor de ganhos K
dK = [1,1,1];

% Ajuste do peso do vetor dK
ksi = 1/100;

% Limite da soma do vetor dK (incremento mínimo)
delta = 0.001;

% Simulação do processo com os valores de ganho iniciais
[ErroBest, ErroReta] = SimulaProcesso(K,0);

k = 0;

% Implementação do algoritimo Twiddle

while sum(dK) > delta;
    k = k + 1;
    for i = 1:length(K)
        K(i) = K(i) + dK(i);
        [Erro, ErroReta] = SimulaProcesso(K,0);
        if Erro < ErroBest
            ErroBest = Erro;
            dK(i) = dK(i)*(1+ksi);
        else
            K(i) = K(i) - 2*dK(i);
            [Erro, ErroReta] = SimulaProcesso(K,0);
            if Erro < ErroBest
                ErroBest = Erro;
                dK(i) = dK(i)*(1+ksi);
            else
                K(i) = K(i) + dK(i);
                dK(i) = dK(i)*(1-ksi);
            end
        end
    end
    fprintf('Rodada = %i: Melhor erro = %.4f, soma(dk) = %.6f\n',k, ErroBest, sum(dK));
end

% Print dos parâmetros ótimos finais
fprintf('Parâmetros: P = %.4f, I = %.4f, D = %4.d\n',K(1),K(2),K(3));

%Simulação do processo com os parâmetros ótimos alcançados
[Erro, ErroReta] = SimulaProcesso(K,1);

% Plot do erro entre o robô e o percurso estipulado
figure(2);
plot(ErroReta(:,2),ErroReta(:,1));

xlabel('Tempo [s]'); ylabel('Erro [m]');