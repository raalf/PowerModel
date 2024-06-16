function [ct] = fcn_ct(J,rpm)
%CT as a function of J (V/(nD)) and rpm

        x = J;
        y = rpm;
        p00 =     0.05945;
        p10 =    -0.01384;
        p01 =   1.239e-05;
        p20 =     -0.1057;
        p11 =  -3.204e-06;
        p02 =  -9.234e-10;

        ct = p00 + p10.*x + p01.*y + p20.*x.^2 + p11.*x.*y + p02.*y.^2;
        
        %force ct to zero when rpm is small
        ct(rpm<1500) = 0;
        % if rpm<1000
        %     ct=0;
        % end
end

