function vert = feassol(A, b)
% Basic feasible solutions vert to the system of constraints
% Ax = b, x >= 0.
% They are stored in columns of the matrix vert.
[m, n] = size(A);
warning off
b = b(:);
vert = [];
if (n >= m)
    t = nchoosek(1:n,m);
    nv = nchoosek(n,m);
    for i=1:nv
        y = zeros(n,1);
        x = A(:,t(i,:))\b;
        if all(x >= 0 & (x ~= inf & x ~= -inf))
            y(t(i,:)) = x;
            vert = [vert y];
        end
    end
else
    error('Number of equations is greater than the number of variables.')
end
if ~isempty(vert)
    vert = delcols(vert);
else
    vert = [];
end

function e = vr(m,i)
% The ith coordinate vector e in the m-dimensional Euclidean space.
e = zeros(m,1);
e(i) = 1;
function d = delcols(d)
% Delete duplicated columns of the matrix d.
d = union(d',d','rows')';
n = size(d,2);
j = [];
for k =1:n
    c = d(:,k);
    for l=k+1:n
        if norm(c - d(:,l),'inf') <= 100*eps
            j = [j l];
        end
    end
end
if ~isempty(j)
    j = sort(j);
    d(:,j) = [ ];
end

function [row, mi] = MRT(a, b)
% The Minimum Ratio Test (MRT) performed on vectors a and b.
% Output parameters:
% row ? index of the pivot row
% mi ? value of the smallest ratio.
m = length(a);
c = 1:m;
a = a(:);
b = b(:);
l = c(b > 0);
[mi, row] = min(a(l)./b(l));
row = l(row);
function col = MRTD(a, b)
% The Maximum Ratio Test performed on vectors a and b.
% This function is called from within the function dsimplex.
% Output parameter:
% col - index of the pivot column.
m = length(a);
c = 1:m;
a = a(:);
b = b(:);
l = c(b < 0);
[mi, col] = max(a(l)./b(l));
col = l(col);
function [m, j] = Br(d)
% Implementation of the Bland's rule applied to the array d.
% This function is called from within the following functions:
% simplex2p, dsimplex, addconstr, simplex and cpa.
% Output parameters:
% m - first negative number in the array d
% j - index of the entry m.
ind = find(d < 0);
if ~isempty(ind)
    j = ind(1);
    m = d(j);
else
    m = [];
    j = [];
end