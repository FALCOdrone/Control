function Out = fx(x,u,w0,L,zeta,m)

Out = [ - 2*zeta*w0, - w0^2*cos(x(1));
                  1,               0];