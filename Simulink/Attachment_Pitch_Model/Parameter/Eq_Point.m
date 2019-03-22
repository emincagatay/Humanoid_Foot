syms alpha beta Ta Tb Tf

betab=beta;
betaf=beta/n;
lambda=(kf*(cf+ betaf*rf - sin(alpha)*df)*rf -n*kb*(cb+ betab*rb - sin(alpha)*db)*rb + n*Tb -Tf)/(cos(alpha)*(kf*df*rf + n*kb*db*rb));
 
f = (cos(alpha)*m*g*h + kb*(cb + betab*rb - sin(alpha)*db)*cos(alpha)*db + kf*(cf+ betaf*rf - sin(alpha)*df)*cos(alpha)*df ...
            + lambda*(-sin(alpha)*(kf*df*(cf +betaf*rf ) - kb*db*(cb + betab* rb))+2*cos(2*alpha)*(kb*db^2 - kf*df^2)) +Ta);
        
f = simplify(f);

fbeta=solve(f==0,beta);

alpha=10/180*pi;
Ta=0;


