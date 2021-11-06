function J_pinv = pseudo_inverse_calc(q)
% Calculation of pseudo inverse of Jacobian Matrix J with 2*3 Dimension
% Dimension 2<3, used ''right inverse'' ---> J_pinv = J'*(J*J')^-1

% Input:     q      == joint Angle with Dimension 3*1
% Output:    J_pinv == pseudo inverse of J with Dimension 3*2, Mapping from
%                      workspace (2*1) to jointspace (3*1)

l1 = 0.30;
l2 = 0.24;
l3 = 0.34;
tolo_11 = 0.001;
tolo_22 = 0.001;
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

% term == 2*2 matrix for (J*J')^(-1), term_coeff == 1/det(term)
term_11 = j_21^2 + j_22^2 + j_23^2 + tolo_11;
term_12 = - j_11 * j_21 - j_12 * j_22 - j_13 * j_23;
term_21 = term_12;
term_22 = j_11^2 + j_12^2 + j_13^2 + tolo_22;

term_coeff = (term_11 * term_22 - term_12 * term_21)^(-1);

J_pinv_11 = term_coeff*(j_11 * term_11 + j_21 * term_21);
J_pinv_21 = term_coeff*(j_12 * term_11 + j_22 * term_21);
J_pinv_31 = term_coeff*(j_13 * term_11 + j_23 * term_21);
J_pinv_12 = term_coeff*(j_11 * term_12 + j_21 * term_22);
J_pinv_22 = term_coeff*(j_12 * term_12 + j_22 * term_22);
J_pinv_32 = term_coeff*(j_13 * term_12 + j_23 * term_22);

% n = 2 < m = 3 ---> right peudo inverse
%J_pinv = J' * (J * J')^(-1);
J_pinv = [J_pinv_11, J_pinv_12;
          J_pinv_21, J_pinv_22;
          J_pinv_31, J_pinv_32];


end