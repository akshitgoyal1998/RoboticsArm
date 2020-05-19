%%function thetaval =  quasiNewton()
x_t = rand(3,1);
L = length(x_t);
a = 1;
M = L;
c = rand(M*L/a,1 );
c_mn=reshape(c,[M,L/a]);
J = @(theta) objfuntc(x_t,c_mn,theta );
theta0 = 10 ;
options = optimoptions('fminunc','Algorithm','quasi-newton ');
[theta, thetaval] = fminunc(J,theta0,options);
%%end
function f = objfuntc(x_t,c_mn,theta )
L = length(x_t );
[M,N] =size(c_mn);
f = 0 ;
for t = 1:L
    xs = 0 ;
    for n = 1:N
        for m=1:M
            xs = xs + c_mn(m,n)*sin(2*pi*m*(t - n -2) + theta);
        end
    end
    f = f + (x_t(t) - xs)*conj(x_t(t) - xs) ;
end
end