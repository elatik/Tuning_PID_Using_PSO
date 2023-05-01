clc; 
clear;
close all;

ns=[1];
ds=[1 2 3];
G=tf(ns,ds);
Gf=feedback(G,1);
step(Gf);
hold on


% Param PSO
c=2; w=0.7; particles=20; iteration=50; Var=3;

% search space
a=0;
b=30;

% optimization steps
c_cf=0;

% Initialization
for m=1:particles
    for n=1:Var
        v(m,n)=0;
        x(m,n)=a+rand*(b-a);
        xp(m,n)=x(m,n);
    end
%Model param
kp=x(m,1);
ki=x(m,2);
kd=x(m,3);

 %Model de simulation
Gc=pid(kp,ki,kd);
Gcf=feedback(Gc*G,1);
y=step(Gcf);

% TIAE (fonction on=bjectif)
ff1=0;
sim1=size(y);
for m1=1:sim1
    ff1=ff1+((abs(y(m1)-1))*m1);
end
ITAE(m)=ff1;
end

% find best value
[Best_performance,location]=min(ITAE);
fg=Best_performance;
xg(1)=x(location,1);
xg(2)=x(location,2);
xg(3)=x(location,3);



for i=1:iteration

for m=1:particles

for n=1:Var
   v(m,n)=(w*v(m,n))+(c*rand*(xp(m,n)-x(m,n)))+(c*rand*(xg(n)-x(m,n)));
   x(m,n)=x(m,n)+v(m,n);
end

% chek bound
for n=1:Var
    if x(m,n)<a
    x(m,n)=a;
    
    end
 if x(m,n)>b
    x(m,n)=b;
 end
end
%Model param
kp=x(m,1);
ki=x(m,2);
kd=x(m,3);

%Model de simulation
Gc=pid(kp,ki,kd);
Gcf=feedback(Gc*G,1);
y=step(Gcf);


% TIAE (fonction on=bjectif)
ff1=0;
sim1=size(y);
for m1=1:sim1
    ff1=ff1+((abs(y(m1)-1))*m1);
end
ITAEp(m)=ff1;


% compare Local
if ITAEp(m)<ITAE(m)
    ITAE(m)=ITAEp(m);
    xp(m,1)=x(m,1);
    xp(m,2)=x(m,2);
    xp(m,3)=x(m,3);
end

end 

 [B_fg,location]=min(ITAE);
 
%  compare global
if B_fg<fg
    fg=B_fg;
    xg(1)=xp(location,1);
     xg(2)=xp(location,2);
      xg(3)=xp(location,3);
end
c_cf=c_cf+1;
best_cf_ac(c_cf)=fg;

end

Min_ITAE=fg;
kp=xg(1)
ki=xg(2)
kd=xg(3)

Gc=pid(kp,ki,kd);
Gcf=feedback(Gc*G,1);
y=step(Gcf);

t_cf=1:c_cf;

figure
plot(t_cf,best_cf_ac,'r--','LineWidth',2),xlabel('iteration'),ylabel('cost function(TIAE)')
legend('ITAE for PSO-PID')
title('ITAE with each iteration')