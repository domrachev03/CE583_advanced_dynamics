syms theta1 theta2 theta3
syms dtheta1 dtheta2 dtheta3

I1_body = [2541.211, -82.979, -190.902;
      -82.979, 2090.928, -577.713;
      -190.902, -577.713, 1347.574];
I2_body = [191.411, 0, 17.871;
      0, 332.670, 0;
      17.871, 0, 170.245];
I3_body = [186.052, 69.115, -53.174;
      69.115,1718.842, 8.788;
      -53.174, 8.788, 1582.435];
I4_body = [4.0300, 0, 0;
      0, 400.314, 0;
      0, 0, 398.071];
I5_body = [0.335, -0.115, 0;
      -0.115, 61.954, 0;
      0, 0, 61.955];

m1 = 0.530;
m2 = 0.192;
m3 = 0.284;
m4 = 0.058;
m5 = 0.025;

T_01 = [cos(theta1) -sin(theta1) 0 0;
        sin(theta1) cos(theta1) 0 0;
        0 0 1 101;
        0 0 0 1];
T_12 = [cos(theta2-pi/2) -sin(theta2-pi/2) 0 0;
        0 0 1 0;
        -sin(theta2-pi/2) -cos(theta2-pi/2) 0 131;
        0 0 0 1];
T_13 = [cos(theta3) -sin(theta3) 0 0;
        0 0 1 0;
        -sin(theta3) -cos(theta3) 0 131;
        0 0 0 1];
T_24 = [cos(pi/2-theta2+theta3) -sin(pi/2-theta2+theta3) 0 -33;
        sin(pi/2-theta2+theta3) cos(pi/2-theta2+theta3) 0 0;
        0 0 1 0;
        0 0 0 1];
T_35 = [cos(pi/2+theta2-theta3) -sin(pi/2+theta2-theta3) 0 210;
        sin(pi/2+theta2-theta3) cos(pi/2+theta2-theta3) 0 0;
        0 0 1 0;
        0 0 0 1];

T_1com = [eye(3), [-9.604; 15.159; 59.074];
          0 0 0 1];
T_2com = [eye(3), [57.307; 0; -12.270];
          0 0 0 1];
T_3com = [eye(3), [-42.296; 8.638; -10.508];
          0 0 0 1];
T_4com = [eye(3), [108.424; 0; 0];
          0 0 0 1];
T_5com = [eye(3), [65.274; -0.00; 0];
          0 0 0 1];

T_01com = T_01*T_1com;
T_02com = T_01*T_12*T_2com;
T_03com = T_01*T_13*T_3com;
T_04com = T_01*T_12*T_24*T_4com;
T_05com = T_01*T_13*T_35*T_5com;

I1_world = T_01com(1:3,1:3)*I1_body*transpose(T_01com(1:3,1:3));
I2_world = T_02com(1:3,1:3)*I2_body*transpose(T_02com(1:3,1:3));
I3_world = T_03com(1:3,1:3)*I3_body*transpose(T_03com(1:3,1:3));
I4_world = T_04com(1:3,1:3)*I4_body*transpose(T_04com(1:3,1:3));
I5_world = T_05com(1:3,1:3)*I5_body*transpose(T_05com(1:3,1:3));

Pc1 = T_01com(1:3,4);
Pc2 = T_02com(1:3,4);
Pc3 = T_03com(1:3,4);
Pc4 = T_04com(1:3,4);
Pc5 = T_05com(1:3,4);

Jvc1 = [diff(Pc1, theta1) diff(Pc1, theta2) diff(Pc1, theta3)];
Jvc2 = [diff(Pc2, theta1) diff(Pc2, theta2) diff(Pc2, theta3)];
Jvc3 = [diff(Pc3, theta1) diff(Pc3, theta2) diff(Pc3, theta3)];
Jvc4 = [diff(Pc4, theta1) diff(Pc4, theta2) diff(Pc4, theta3)];
Jvc5 = [diff(Pc5, theta1) diff(Pc5, theta2) diff(Pc5, theta3)];

Jw1 = [T_01com(1:3,3) zeros(3,1) zeros(3,1)];
Jw2 = [T_01com(1:3,3) T_02com(1:3,3) zeros(3,1)];
Jw3 = [T_01com(1:3,3) zeros(3,1) T_03com(1:3,3)];
Jw4 = [T_01com(1:3,3) zeros(3,1) T_03com(1:3,3)];
Jw5 = [T_01com(1:3,3) T_02com(1:3,3) zeros(3,1)];

D = m1*transpose(Jvc1)*Jvc1 +m2*transpose(Jvc2)*Jvc2 + m3*transpose(Jvc3)*Jvc3 + m4*transpose(Jvc4)*Jvc4 + m5*transpose(Jvc5)*Jvc5 + transpose(Jw1)*I1_world*Jw1 + transpose(Jw2)*I2_world*Jw2 + transpose(Jw3)*I3_world*Jw3 + transpose(Jw4)*I4_world*Jw4 + transpose(Jw5)*I5_world*Jw5;
D = simplify(D)


C = sym(zeros(3, 3));
theta = [theta1; theta2; theta3];
dtheta = [dtheta1; dtheta2; dtheta3];

for i = 1:3
    for j = 1:3
        for k = 1:3
            C(j, k) = C(j, k) + (diff(D(k, j), theta(i)) + diff(D(k, i), theta(j)) - diff(D(i, j), theta(k))) / 2 * dtheta(i);
        end
    end
end

C = simplify(C);

G = sym(zeros(3, 1));
m = [m1; m2; m3; m4; m5];
P = [Pc1, Pc2, Pc3, Pc4, Pc5];
g = [0; 0; -9.81];
for i = 1:5
   for k = 1:3
        G(k) = G(k) + diff(m(i) * dot(P(:, i), g), theta(k));
   end
end
G = simplify(G);
G

syms tau_ext1 tau_ext2 tau_ext3
tau_ext = [tau_ext1; tau_ext2; tau_ext3];
inv_dyn = D \ (tau_ext - C * dtheta  - G);
matlabFunction(inv_dyn, 'File', 'invDyn', 'Vars', {theta, dtheta, tau_ext});
Ddot = sym(zeros(3, 3));
for i = 1:3
    Ddot = Ddot + diff(D, theta(i)) * dtheta(i);
end

S = simplify(Ddot - 2*C);
is_skew = simplify(S + S.');
S % supposed to be all zeros