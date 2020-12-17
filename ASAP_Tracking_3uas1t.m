function ASAP_Tracking_3uas1t
% Cycling pursuit of 1 threat using 3 uas

clear all;
close all;
clc;

global n

% define simulation parameters
T = 25;                % simulation period
dt = 0.02;             % sampling time
n = 4;                 % number of uas_sensor

% Case study 1: Initial (baseline) simulation based on CDC'16 submission -
% 3 agents only

% p1 = [1 8 0.5]';    % position of agent 1 
% p2 = [-2.5 -1 0]'; % position of agent 2
% p3 = [-1 2 1]';    % position of agent 3
% 
% pt = [0 0 2]';         % position of threat

% Case study 2: ASAP simulation

p1 = [5 1 7]';    % position of agent 1 
p2 = [3 0 6]'; % position of agent 2
p3 = [0 4 8]';    % position of agent 3
p4 = [0 10 0]';    % position of agent 4
p5 = [10 0 0]';    % position of agent 5

pt = [3 3 7]';         % position of threat


%a = [1 0 0]';          % vector of x_plane of motion
%a = [0 1 0]';          % vector of y_plane of motion
a = [0 0 1]';           % vector of z_plane of motion


rho = 2;               % desired relative distance bet agent and target

k0=1.4; k1=0.5; k2=1; k3=2.5;    % control gains  

# A = [0 1 1;          % (Adjacency matrix) Ring topology for 3 agents
#      1 0 1;
#      1 1 0];

 A = [0 1 0 1;          % (Adjacency matrix) Ring topology for 4 agents
      1 0 1 0;
      0 1 0 1;
      1 0 1 0];

%A = [0 1 0 0 0 1;       % Ring for 6 agents
%     1 0 1 0 0 0;
%     0 1 0 1 0 0;
%     0 0 1 0 1 0;
%     0 0 0 1 0 1;
%     1 0 0 0 1 0];

thetaij = 2*pi/n;           % desired phasing angle between agents (thetaij may be different from thetaik)
dij = 2*rho*sin(thetaij/2); % desired inter-agent distance
D = dij*A;                  % matrix of inter-agent distances 

% simulate using ode45
yinit = [p1;p2;p3;p4;pt];
tspan = 0:dt:T;
options=odeset('AbsTol',1e-8,'RelTol',1e-8);

[t,y] = ode45(@EOM,tspan,yinit,options,k0,k1,k2,k3,a,rho,A,D);

% plot simulated variables
% color options
blue  = [0,0,255]/256;
darkgreen = [40,140,60]/256;
red = [250,50,50]/256;
blue2 = [30,144,255]/256;
green2 = [80,205,50]/256;
red2 = [250,128,114]/256;
gold = [184,134,11]/256;
slate = [132,112,255]/256;
magenta = [255,0,255]/256;
black = [0,0,0];
darkbrown = [139,69,19]/256;
prettyblue = [0,200,255]/256;
orangered = [255,69,0]/256;
coral = [255,127,80]/256;
color = [coral; green2; prettyblue; gold; magenta; slate];

P = zeros(length(t),n,3);
for i=1:length(t)
    pt(i,1:3) = y(i,end-2:end); % target position
    k=1;
    for j=1:n
        % Agent positions
        P(i,j,1:3) = y(i,k:k+2);   % P[i,j,1:3] = position of agent j at time t=i
        k = k+3; 
    end
end

Pjt = zeros(length(t),n);
Prel = zeros(length(t),n,3);
Pjl = zeros(n,n,length(t));
Pja = zeros(length(t),n);
for i=1:length(t)
    for j=1:n
        % Rel distance to target
        pj(1,1:3) = P(i,j,1:3);
        Pjt(i,j) = norm(pj-pt(i,1:3)); % Pit[i,j]=rel dist of agent j wrt target at time t=i 
        
        % Rel agent position
        Prel(i,j,1:3) = pj(1,1:3)-pt(i,1:3);  % relative to target position
        
        % Rel inter-agent distance
        if j<n 
            for l=j+1:n
                ajl = A(j,l);
                pl(1,1:3) = P(i,l,1:3);
                Pjl(j,l,i) = ajl*norm(pj-pl);  % Pjl[j,l,i]=dis between agents j and (neighbor) l at time i
                % note: if l is not a neighbor of j, then Pjl[j,l,i]=0
                % also, we only evaluate edges (1,2),(1,4) etc. and not (2,1),(4,1)
                % since they are equal quantities
            end
        end
        
        % target plane condition 
        Pja(i,j) = a'*(pt(i,1:3)'-pj');
    end
end

% save('agent_pos.mat')    % save workspace to a file

%%%------- FIG 1: 3-D Positions of Agents and Target -------------%%%%
figure(1);
for j=1:n
    plot3(P(:,j,1),P(:,j,2),P(:,j,3),'color',color(j,:),'Linewidth',1.3);
    hold on;
    plot3(P(1,j,1),P(1,j,2),P(1,j,3),'o','MarkerEdgeColor','k',...
                       'MarkerFaceColor','g',...
                       'MarkerSize',10);
    plot3(P(end,j,1),P(end,j,2),P(end,j,3),'d','MarkerEdgeColor','k',...
                       'MarkerFaceColor','m',...
                       'MarkerSize',10);    
    plot3(P(ceil(length(t)/2),j,1),P(ceil(length(t)/2),j,2),...
        P(ceil(length(t)/2),j,3),'s','MarkerEdgeColor','k',...
                       'MarkerFaceColor','c',...
                       'MarkerSize',10);                  
end
plot3(pt(:,1),pt(:,2),pt(:,3),'-.k','Linewidth',1.5)
plot3(pt(1,1),pt(1,2),pt(1,3),'*','MarkerEdgeColor','k',...
                       'MarkerFaceColor','k',...
                       'MarkerSize',10);
plot3(pt(end,1),pt(end,2),pt(end,3),'*','MarkerEdgeColor','k',...
                       'MarkerFaceColor','k',...
                       'MarkerSize',10);
plot3(pt(ceil(length(t)/2),1),pt(ceil(length(t)/2),2),...
    pt(ceil(length(t)/2),3),'*','MarkerEdgeColor','k',...
                       'MarkerFaceColor','k',...
                       'MarkerSize',10);                   
    
%axis ([-1 15 -1 15 0 8])
xlabel('$p_{ix}$ (m)','Interpreter','LaTex')
ylabel('$p_{iy}$ (m)','Interpreter','Latex')
zlabel('$p_{iz}$ (m)','Interpreter','Latex')
axis('equal')
%set(gca,'FontSize',20)
set(gca,'FontSize',14)
grid;
