function J = jacobian_calc (q)
% Calculation of Jacobian Matrix for a 3-DoF Manipulator
% Mapping from jointspace (3*1 Dimension) to workspace (2*1 Dimension)
% Input:    q == joint angle of Manipulator with 3*1 Dimension [q1;q2;q3]
% Output:   J == Jacobian Matrix with 3*2 Dimension
l1 = 0.30;
l2 = 0.24;
l3 = 0.34;
s1 = sin(q(1));
c1 = cos(q(1));
s12 = sin(q(1) + q(2));
c12 = cos(q(1) + q(2));
s123 = sin(q(1) + q(2) +q(3));
c123 = cos(q(1) + q(2) +q(3));

j_11 = - l1 * s1 - l2 * s12 - l3 * s123;
j_12 = - l2 * s12 - l3 * s123;
j_13 = - l3 * s123;

j_21 = l1 * c1 + l2 * c12 + l3 * c123;
j_22 = l2 * c12 + l3 * c123;
j_23 = l3 * c123;

J = [j_11, j_12, j_13;
     j_21, j_22, j_23];

end