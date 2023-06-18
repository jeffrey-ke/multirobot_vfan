function y = inv_jacob(r1, r2, r3, c_dot)
% function y = inv_jacob(u)

syms x1 y1 th1 x2 y2 th2 x3 y3 th3

x_c = (x1 + x2 + x3)/3.0;
y_c = (y1 + y2 + y3)/3.0;
theta_c = atan2( (2.0/3.0) * x1 - (1.0/3.0) * (x2 + x3), (2.0/3.0) * y1 - (1.0/3.0) * (y2 + y3));
phi_1 = th1 + theta_c;
phi_2 = th2 + theta_c;
phi_3 = th3 + theta_c;
p = sqrt( (x1 - x2)^2 + (y1 - y2)^2 );
q = sqrt( (x3 - x1)^2 + (y3 - y1)^2 );
beta = acos( (p^2 + q^2 - (x3 - x2)^2 - (y3 - y2)^2 ) / (2 * p * q) );

F = [x_c, y_c, theta_c, phi_1, phi_2, phi_3, p, q, beta];
X = [x1, y1, th1, x2, y2, th2, x3, y3, th3];

J = jacobian(F, X);
J_with_current_values = subs(J, [x1, y1, th1, x2, y2, th2, x3, y3, th3], [r1(1), r1(2), r1(3), r2(1), r2(2), r2(3), r3(1), r3(2), r3(3)]);
J_with_current_values = double(J_with_current_values);
% J_inv = inv(J_with_current_values); Matlab has told me not to find the
% inverse of J but instead to use their "matrix division"
% 
R_dot = J_with_current_values \ [c_dot(1); c_dot(2); c_dot(3); c_dot(4); c_dot(5); c_dot(6); c_dot(7); c_dot(8); c_dot(9)];
y = R_dot;
end