%% by Bellila Ahmed Nassim, student at ENST
%On exchange at Université Laval
%Course UL GEL-7029 Prictive control under supervision of Prof. André Desbiens

%implementation of MPC using the MPC toolbox
%see Readme.txt and the simulink simulator to understand more the process


%%
clc;

%% Operating points
u_op = [24.3;30.2];
y_op = [52.5;51;27.55];
Hp = 20;
Hc = 1;


%% Model

G32 = tf(-0.0476,[18.48 1],'inputDelay',3.5);
G31 = tf(0.013237,[10.2 1],'inputDelay',2);
G22 = tf(2.32,[1.536 1],'inputDelay',3);
G21 = tf(0.74,[9.21 1],'inputDelay',0.5);
G12 = tf(0.8,[8 1],'inputDelay',1);
G11 = tf(2.64,[5.9 1],'inputDelay',2.5);
Gg = [G11 G12; G21 G22; G31 G32];
Gss = ss(Gg);
Ts = 0.5; % Sampling period    
model = c2d(Gss,Ts);
model = absorbDelay(model); 
n=length(model.A);
nu=size(model.B,2);
ny=size(model.C,1);


%% MPC

mpcverbosity off;
model = setmpcsignals(model,'MV',1:2); 
model = setmpcsignals(model,'MO',1:3);
mpcobj = mpc(model,Ts);
% model=minreal(model);
mpcobj.model.Nominal.U=u_op;
mpcobj.model.Nominal.Y=y_op;
mpcobj.PredictionHorizon = Hp;
mpcobj.ControlHorizon = Hc;
%Plages d'opération
mpcobj.MV = struct('Min',{15,15},'Max',{50,50}, ... 
                   'RateMin',{-35,-35},'RateMax',{35,35}, ... 
                   'RateMinECR',{0,0},'RateMaxECR',{0,0}, ... 
                   'MinECR',{0,0},'MaxECR',{0,0}, ... 
                   'ScaleFactor',{35,35});
               
mpcobj.OV = struct('Min',{10,10,27},'Max',{90,90,28}, ... 
                   'MinECR',{0,0,0.1},'MaxECR',{0,0,0.1}, ...
                   'ScaleFactor',{80,80,1});
               
%weights
mpcobj.Weights.OutputVariables = [1 1 0]; 
mpcobj.Weights.ManipulatedVariablesRate = [.1 2];
% mpcobj.Weights.ECR = 1e5;

%observer 
setoutdist(mpcobj,'integrators'); 


%%
%Error
% sy=[mpcobj.OV(1).ScaleFactor,mpcobj.OV(2).ScaleFactor,mpcobj.OV(3).ScaleFactor];
% Aaug=[model.A zeros(n,ny); zeros(ny,n) eye(ny)]; % augmented model with integrators
% Baug=[model.B; zeros(ny,nu)];
% Caug=[model.C diag(sy)*eye(ny)];
% Daug=[zeros(nu,nu)];
% poles_obs=zeros(1,n+ny);
% for i=1:n+ny
%     poles_obs(i)=0.5-i*0.02; 
% end
% Kpred = place(Aaug',Caug',poles_obs)';
% setEstimator(mpcobj,Kpred);

