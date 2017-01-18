function [w,z] = LCPLinEqu(A,b,z)
z= (z~=0);
dim_var = length(z);

T = eye(dim_var);
[z_sort,index_sort] = sort(z,'descend');
T = T(index_sort,:);

num_nonzero = (find(z_sort==0));
num_nonzero = num_nonzero(1)-1;

A_new = T * A * inv(T);
b_new = T * b;

ret = double(z_sort);
ret(1:num_nonzero) = A_new(1:num_nonzero,1:num_nonzero) \ -b_new(1:num_nonzero);
ret(num_nonzero+1:end) = zeros(dim_var-num_nonzero,1);

z=T \ ret;
w=A*z + b;

end