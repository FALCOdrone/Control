close all
clear all

%IMPORTANT our blades/proppellors are different from those in the table (17*6.5), probabily we should do some test
%17*5.8
y=[0.5,0.55,0.6,0.65,0.75,0.85,1];
x=[1095,1320,1625,1875,2363,2871,3716];
%18*6.1
y1=[0.5,0.55,0.6,0.65,0.75,0.85,1];
x1=[1318,1612,1901,2259,2835,3477,4355];


%% regressione esponenziale XY
fig = figure('name','esempi motore');
f = fit(x',y','exp2');
plot(f,x,y);
hold on;
%      Coefficients (with 95% confidence bounds):
       a = f.a ;
       b = f.b ;
       c = f.c   ;
       d = f.d;
 T=100;%thrust
 throttle= a*exp(b*T) + c*exp(d*T)
 
  %% regressione esponenziale X1Y1
% f1 = fit(x1',y1','exp2');
% plot(f1,x1,y1);
% %      Coefficients (with 95% confidence bounds):
%        a = f1.a ;
%        b = f1.b ;
%        c = f1.c   ;
%        d = f1.d;
%  T=100;%thrust
%  throttle= a*exp(b*T) + c*exp(d*T)
%% regressione lineare XY
  p = polyfit(x,y,1);
  yfit =  p(1)* x + p(2)
  plot(x,yfit,'color','black');
  %% regressione lineare X1Y1
  p1= polyfit(x1,y1,1);
  yfit1 =  p(1)* x + p(2)
  plot(x1,yfit1,'color','black');

 %% PASSAGGIO DA 0 A 0.5 CON ESPONENZIALE
 figure('name','0-0.5 curve')
 x50=[0.01,0.5,0.55, 0.6];
 y50=[0.01,1095,1320,1625];
 f50= fit(x50',y50','exp1');
 plot(f50,x50,y50);

 syms e q u
 v = e*exp(q*u);
 g  = finverse(v,u)

x_=0.001:25:1625
f50_ =log((x_)/f50.a)/f50.b;
figure(fig);
plot(x_,f50_);
% TRY TO START FROM (0,0)
f50_ =log((x_+100)/f50.a)/f50.b;
figure(fig);
plot(x_,f50_);

%% best conclusion FOR MOTOR dynamics THRUST TO THOTTLE (X,Y)

figure('name','best result')
x_=0.001:25:500
f50_ =log((x_+100)/f50.a)/f50.b;
plot(x_,f50_)
;
hold on
y=[0.374,0.5,0.55,0.6,0.65,0.75,0.85,1];
x=[475,1095,1320,1625,1875,2363,2871,3716];
plot(f,x,y);
legend('low throttle','test point','high throttle,LINEAR')



