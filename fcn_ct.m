function [ct] = fcn_ct(J,rpm,prop)

%CT as a function of J (V/(nD)) and rpm


if strcmp(prop,'11x7')
    x = J;
    y = rpm;
    p00 =     0.05945;
    p10 =    -0.01384;
    p01 =   1.239e-05;
    p20 =     -0.1057;
    p11 =  -3.204e-06;
    p02 =  -9.234e-10;

    ct = p00 + p10.*x + p01.*y + p20.*x.^2 + p11.*x.*y + p02.*y.^2;

elseif strcmp(prop,'18x8')
    x = J;
    p1 =     0.03442;
    p2 =     -0.1123 ;
    p3 =   -0.004359  ;
    p4 =      0.0767  ;

    ct = p1.*x.^3 + p2.*x.^2 + p3.*x + p4;


end
%force ct to zero when rpm is small
ct(rpm<2500) = 0;
% if rpm<1000
%     ct=0;
% end
end

