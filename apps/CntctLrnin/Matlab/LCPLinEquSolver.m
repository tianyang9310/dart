A=[            3.1360,   -2.0370,   0.9723,   0.1096,  -2.0370,   0.9723,
           -2.0370,    3.7820,   0.8302,  -0.0257,   2.4730,   0.0105,
            0.9723,    0.8302,   5.1250,  -2.2390,  -1.9120,   3.4080,
            0.1096,   -0.0257,  -2.2390,   3.1010,  -0.0257,  -2.2390,
           -2.0370,    2.4730,  -1.9120,  -0.0257,   5.4870,  -0.0242,
            0.9723,    0.0105,   3.4080,  -2.2390,  -0.0242,   3.3860];
        
b=[            0.1649,
           -0.0025,
           -0.0904,
           -0.0093,
           -0.0000,
           -0.0889];
       
z=zeros(1,6);


% index = 0;
ret=[];
truth = [0,1,0,1,1,1];

for i1=0:1
   z(1) = i1;
   for i2=0:1
       z(2) = i2;
       for i3=0:1
            z(3) = i3;
            for i4=0:1
                z(4)=i4;
                for i5=0:1
                    z(5)=i5;
                    for i6=0:1
                        z(6)=i6
%                         x=[x;z]
                        [w,x]=LCPLinEqu(A,b,z');
                        if z==truth
                            % pass
                        end
                        if (all(abs(w.*x)<1e-6) && all(x>=-1e-10) &&  all(w>-1e-10))
                            ret=[ret;x']
                        end
                    end
                end
            end
       end
   end
end


