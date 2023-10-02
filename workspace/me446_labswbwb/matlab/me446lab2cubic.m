function thetas = cubic(t)
if (t<1)
    theta = 1.5*t^2 - t^3;
    theta_dot = 3*t - 3*t^2;
    theta_dd = 3 - 6*t;
end
if ((t>=1)&&(t<2))
    theta = -2+6*t-4.5*t^2+t^3;
    theta_dot = 6-9*t+3*t^2;
    theta_dd = -9+6*t;
end

thetas = [theta; theta_dot; theta_dd];   % vector of theta , theta_dot, and theta_doubledot
end
