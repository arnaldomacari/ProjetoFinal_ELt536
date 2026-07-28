clearvars
close all
clc

% Looking for and adding the root directory
FolderCurrent = which(mfilename);
FolderKey = '/AuRoRA';
FolderRootId = strfind(FolderCurrent,FolderKey);
FolderRoot = FolderCurrent(1:FolderRootId(end)+numel(FolderKey)-1);
addpath(genpath(FolderRoot));

P = Pioneer3DX;
P.pPar.a  = 0.10;
P.pPar.Ts = 1/10; 

% Postura Inicial do Robô
P.pPos.X([1 2 6]) = [0; -5; deg2rad(90)];
P.pPos.Xc = P.pPos.X;

% Obstáculos
Obst = {
    -3.1, -2.0, 3.0, 2.4, 0.3, [0.50 0 0], "Obstáculo A" ;
    -3.1,  3.0, 3.0, 2.4, 0.3, [0 0.50 0], "Obstáculo B" ;
     2.5,  3.0, 3.0, 2.4, 0.3, [0 0 0.50], "Obstáculo C" ;
     2.5, -2.0, 3.0, 2.4, 0.3, [0 0.50 0.50], "Obstáculo D" ;
};

%Baias
Baias = { 
     0.0, -5.0, deg2rad(90),  0.8, 'seta2.png',"Home"  , -0.20,  -0.35;
    -3.0,  0.0, deg2rad(-90), 0.8, 'seta1.png',"Baia 1", -0.20 ,  0.35;
    -5.0,  3.0, deg2rad(0),   0.8, 'seta1.png',"Baia 2", -0.20 ,  0.0 ;
     4.5,  3.0, deg2rad(180), 0.8, 'seta1.png',"Baia 3", -1.00 ,  0.0 ;
     2.5, -4.0, deg2rad(90),  0.8, 'seta1.png',"Baia 4", -0.20 , -0.35;
};


% Waypoints
Wp_ = [   0.0  -5.0  NaN            0  0; 
        -0.5  -0.5  NaN            0  1;   
        -3.0   0.0  deg2rad(-90)   1  0;   % >>> BAIA A
        -3.0   0.0  deg2rad(150)   1  1; 
        -5.0   1.0  NaN            0  1;   
        -5.0   3.0  deg2rad(0)     1  0;   % >>> BAIA B
        -5.0   3.0  deg2rad(130)   1  1; 
        -5.0   4.7  NaN            0  1;   
         4.5   5.0  NaN            0  1;   
         4.5   3.0  deg2rad(180)   1  0;   % >>> BAIA C
         4.5   3.0  deg2rad(330)   1  1; 
         5    -4.0  NaN            0  1;   
         2.5  -4.0  NaN            0  0;   
         2.5  -4.0  deg2rad( 90)   1  0;   % >>> BAIA D
         2.5  -4.0  deg2rad( 230)  1  1;
         0.0  -5.0  NaN            0  0;   
         0.0  -5.0  deg2rad( 90)   1  0];  % >>> HOME (base de recarga)


% Waypoints
Wp = [  0.0  -5.0  NaN            0  0; 
        -0.5  -0.5  NaN            0  0;   
        -3.0   0.0  deg2rad(-90)   1  0;   % >>> BAIA A
        -3.0   0.0  deg2rad(150)   1  0; 
        -5.0   1.0  NaN            0  0;   
        -5.0   3.0  deg2rad(0)     1  0;   % >>> BAIA B
        -5.0   3.0  deg2rad(130)   1  0; 
        -5.0   4.7  NaN            0  0;   
         4.5   5.0  NaN            0  0;   
         4.5   3.0  deg2rad(180)   1  0;   % >>> BAIA C
         4.5   3.0  deg2rad(330)   1  0; 
         5.0  -4.0  NaN            0  0;   
         2.5  -4.0  NaN            0  0;   
         2.5  -4.0  deg2rad( 90)   1  0;   % >>> BAIA D
         2.5  -4.0  deg2rad( 230)  1  0;
         0.0  -5.0  NaN            0  0;   
         0.0  -5.0  deg2rad( 90)   1  0];  % >>> HOME (base de recarga)


ehBaia  = Wp(:,4);
ehDireto  = Wp(:,5);
nWp     = size(Wp,1);
%rotBaia = {'A','B','C','D',''}; 

% Ganhos
Ku   = 1.80; 
Kw   = 1.50;   
Kori = 1.5;
wmax = 1.20;

% Tolerâncias
tolBaia = 0.05;
tolVia  = 0.15; 
tolOri  = deg2rad(2);

% Figura
figure('Position',[100 100 1280 720]);                        
hold on, grid on, axis equal
P.mCADplot  % desenha o robo na postura inicial
axis([-7 7 -7 7 0 2])
xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]')
view(0,60)
%view(55, 24);
title('Projeto Final ELT536 - Navegacao com desvio de obstaculos')


% Desenha os 4 obstaculos (blocos pretos, alinhados aos eixos)
for k = 1:size(Obst,1)
    desenhaObstaculo(Obst{k,:});
end

for k = 1:size(Baias,1)
    desenhaBaias(Baias{k,:});
end

% Rota planejada (tracejada) e handles atualizados no laco
plot3(Wp(:,1),Wp(:,2),zeros(nWp,1),':','Color',[0.5 0.5 0.5]); % rota
hTraj = plot3(P.pPos.X(1),P.pPos.X(2),0,'b-','LineWidth',1.6); % rastro real
hAlvo = plot3(Wp(1,1),Wp(1,2),0,'gp','MarkerSize',14,'MarkerFaceColor','g');

drawnow
%disp('Pressione qualquer tecla para iniciar a simulacao...')
pause(2)

% Armazenamento de dados e tempo máximo
tmax = 300;                                   % tempo maximo de seguranca [s]
sinData = zeros(round(tmax/P.pPar.Ts), 27);   % pre-alocacao

j = 1;


% Loop de controle

idx = 1; % waypoint atual
fase = 'POS';        % 'POS' = percurso ; 'ORI' = orientacao
missaoCompleta = false;

t  = tic;
tc = tic;
% toc(t) < tmax &&
while ~missaoCompleta
    if toc(tc) > P.pPar.Ts
        tc = tic;
    
        P.rGetSensorData;  
    
        P.pPos.Xd([1 2]) = Wp(idx,1:2)';
        P.pPos.Xd([7 8]) = [0; 0];
        psi_d = Wp(idx,3);
        
        % Erros
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        rho = norm(P.pPos.Xtil([1 2]));
    
        % Lei de controle
        switch fase
            case 'POS'   % ---------- CONTROLE DE PERCURSO (POLAR) -------
                % r, theta, alfa (modelo polar; alvo fixo)
                r     = rho;
                theta = atan2(P.pPos.Xtil(2), P.pPos.Xtil(1));
                alfa  = atan2(sin(theta - P.pPos.X(6)), cos(theta - P.pPos.X(6)));
                
                if ( ehDireto(idx))
                    u = Ku*cos(alfa);          % velocidade linear para pontos intermediários
                else
                    u = Ku*tanh(r)*cos(alfa);          % velocidade linear saturada
                end 
                w = Kw*alfa + u*sin(alfa)/max(r,1e-3); % velocidade angular
                P.pSC.Ud = [u; w];
    
                if ehBaia(idx),  tol = tolBaia;  else,  tol = tolVia;  end
                if rho < tol
                    if ehBaia(idx)
                        fase = 'ORI';           % chegou na baia -> alinhar
                    else
                        [idx,missaoCompleta] = proximo(idx,nWp);  % via-point
                    end
                end
    
            case 'ORI'   % ---------- CONTROLE DE ORIENTACAO -------------
                psi_til = psi_d - P.pPos.X(6);
                psi_til = atan2(sin(psi_til),cos(psi_til)); % [-pi,pi]
                if ( ehDireto(idx))
                   P.pSC.Ud = [0; 2*Kori*psi_til];   % gira parado (u = 0)
                else
                   P.pSC.Ud = [0; Kori*psi_til];   % gira parado (u = 0)        
                end 
                
    
                if abs(psi_til) < tolOri
                    [idx,missaoCompleta] = proximo(idx,nWp);
                    if ~ehBaia(idx)
                    fase = 'POS';
                    end
                end
        end
    
        P.pSC.Ud(2) = max(min(P.pSC.Ud(2), wmax), -wmax);
    
        % ----- Envia e desenha ------------------------------------------
        if ~missaoCompleta
            hAlvo.XData = Wp(idx,1);
            hAlvo.YData = Wp(idx,2);
        end
    
        P.rSendControlSignals
        P.mCADplot
    
        hTraj.XData = [hTraj.XData P.pPos.X(1)];
        hTraj.YData = [hTraj.YData P.pPos.X(2)];
        hTraj.ZData = [hTraj.ZData 0];
    
        title(sprintf('waypoint %d/%d  |  fase: %s  |  rho = %.2f m', ...
            idx, nWp, fase, rho));
        drawnow
    
        % ----- Armazena -------------------------------------------------
        sinData(j,:) = [P.pPos.Xd' P.pPos.X' P.pSC.Ud' toc(t)];
        j = j + 1;

    end
end
toc(t)
j = j - 1; % elimina o numero da ultima linha que não recebeu dados
P.pSC.Ud = [0; 0];
P.rSendControlSignals


%% ========================================================================
%  ANALISE GRAFICA
% ========================================================================
sinDataLimpa = sinData(1:j,:);
tempo = sinDataLimpa(:,27);

% (a) Evolucao temporal de X e Y
figure(2)
clf
tiledlayout(3,1)

nexttile
plot(tempo, sinDataLimpa(:,1),'g--','LineWidth',1.4)
hold on
plot(tempo, sinDataLimpa(:,13),'b-','LineWidth',1.4)
grid on
ylabel('x [m]')
legend('x_d','x','Location','best')
title('Evolução temporal da posição')

nexttile
plot(tempo, sinDataLimpa(:,2),'g--','LineWidth',1.4)
hold on
plot(tempo, sinDataLimpa(:,14),'b-','LineWidth',1.4)
grid on
ylabel('y [m]')
legend('y_d','y','Location','best')

nexttile
plot(tempo, sinDataLimpa(:,18),'b-','LineWidth',1.4)
hold on
%plot(tempo, sinDataLimpa(:,6),'g--','LineWidth',1.4)
grid on
xlabel('tempo [s]')
ylabel('\psi [graus]')
legend('\psi','Location','best')

% (b) Erros
figure(3)
erroPos = sqrt((sinDataLimpa(:,1)-sinDataLimpa(:,13)).^2 + (sinDataLimpa(:,2)-sinDataLimpa(:,14)).^2);
subplot(2,1,1)
plot(tempo, erroPos,'g-','LineWidth',1.4), grid on
xlabel('tempo [s]'), ylabel('\rho [m]')
title('Erro de posicao (distancia ate o waypoint atual)')
erroOri = rad2deg(atan2(sin(sinDataLimpa(:,6)-sinDataLimpa(:,18)), cos(sinDataLimpa(:,6)-sinDataLimpa(:,18))));
subplot(2,1,2)
plot(tempo, erroOri,'b-','LineWidth',1.4), grid on
xlabel('tempo [s]'), ylabel('erro \psi [graus]'), title('Erro de orientacao')

% (c) Sinais de controle
figure(4)
subplot(2,1,1)
plot(tempo, sinDataLimpa(:,25),'g-','LineWidth',1.4), grid on
xlabel('tempo [s]'), ylabel('u [m/s]'), title('Sinais de controle enviados')
subplot(2,1,2)
plot(tempo, sinDataLimpa(:,26),'b-','LineWidth',1.4), grid on
xlabel('tempo [s]'), ylabel('\omega [rad/s]')


%% Funções Locais
function [idx,fim] = proximo(idx,nWp)
% Avanca para o proximo waypoint; sinaliza fim se acabou a lista
fim = false;
if idx < nWp
    idx = idx + 1;
else
    fim = true;
end
end


function h = desenhaObstaculo(cx, cy, sx, sy, H, cor, rotulo)
% Desenha um obstaculo como bloco 3D alinhado aos eixos, centrado em (cx,cy)
    x0=cx-sx/2; x1=cx+sx/2; y0=cy-sy/2; y1=cy+sy/2;
    v = [x0 y0 0; x1 y0 0; x1 y1 0; x0 y1 0; ...
        x0 y0 H; x1 y0 H; x1 y1 H; x0 y1 H];
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    h = patch('Vertices',v,'Faces',faces, ...
        'FaceColor',cor,'FaceAlpha',1.0,'EdgeColor',[0.45 0.45 0.45]);
    if nargin >= 7 && ~isempty(rotulo)
        text(cx, cy, H+0.15, rotulo, 'HorizontalAlignment','center', ...
            'FontWeight','bold','Color','w');
    end
end





function h = desenhaBaias(x, y, theta, L, arquivo, nome, xt, yt)

% Carrega a imagem
[img, ~, alpha] = imread(arquivo);

% Grupo de transformação
hg = hgtransform;

% Desenha a imagem centrada na origem
h.imagem = image( [-L/2 L/2], [-L/2 L/2], img, 'Parent', hg);

if ~isempty(alpha)
    set(h.imagem, 'AlphaData', alpha);
end

% Nome colocado na base da seta
% A seta original deve apontar para +X
h.nome = text(-L/2 + xt,yt, nome,'Parent',hg,'HorizontalAlignment', ...
  'right','VerticalAlignment','middle','FontWeight','bold','Color','k');

% Rotação e translação do conjunto
T = makehgtform('translate', [x y 0]) * makehgtform('zrotate', theta);
set(hg, 'Matrix', T);
h.grupo = hg;
axis xy;
end