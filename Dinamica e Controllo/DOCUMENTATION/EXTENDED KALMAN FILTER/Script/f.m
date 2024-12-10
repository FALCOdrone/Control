function dx = f(x,u,w0,L,zeta,m)

dx = zeros(2,1);

dx(1) = - 2*zeta*w0*x(1) - w0^2*sin(x(2)) + u/m/L^2;
dx(2) = x(1);
