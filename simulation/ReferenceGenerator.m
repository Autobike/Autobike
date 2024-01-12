function [Xref,Yref,Psiref] = ReferenceGenerator(type,ref_dis,N,scale,angle)

    switch type          
        case 'line'
            t = (0:(N-1));
            xref = t*ref_dis;
            yref = 0*ones(1,N);
            psiref=atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1)); 

        case 'sharp_turn'
            t = (0:(N-1))*ref_dis;
            xref = t;
            yref = [zeros(1,N/2) (t(N/2+1:end)-t(N/2))*tan(deg2rad(angle))];
            psiref=atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1)); 
            
        case 'smooth_curve'
%             t = (0:(N-1))*ref_dis;
%             xref = (300-t).*sin(0.15*t);  OLD
%             yref= -(300-t).*cos(0.15*t)+300;
            t = (0:(N-1))*ref_dis;
            xref = t;
            yref = [zeros(1,N/2) (t(N/2+1:end)-t(N/2))*tan(deg2rad(angle))];
            test = [yref(N/2-20:5:N/2-10),yref(N/2+10:5:N/2+20)]; % used to interpolate the smoothed curve
            testx = [xref(N/2-20:5:N/2-10),xref(N/2+10:5:N/2+20)];
            yi = interp1(testx,test,xref(N/2-20:N/2+20),'spline');
            inter = find(~yi,1,'last');
            yi(1:inter) = 0;
            yref(N/2-20:N/2+20) = yi;
            psiref=atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1)); 
            
        case 'circle'
            t = (0:(N-1))*ref_dis;
            xref = scale*sin(0.15*t);
            yref= -scale*cos(0.15*t)+30;
            psiref=atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1)); 
            
        case 'infinite'
            t = (0:(N-1))*ref_dis;
            xref = scale*cos(t);
            yref = scale*sin(2*t) / 2;
            psiref =atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1)); 
            
        case 'ascent_sin'
            t = (0:(N-1))*ref_dis;
            xref = t*ref_dis;
            yref = 8*sin(0.02*t+0.0004*t.*t);
            psiref=atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1)); 
        case 'wiggle'
            Amp = 10;
            fre = 0.01;
            t = (0:(N-1));
            xref = t*ref_dis;
            yref = Amp*sin(fre*t);
            psiref=atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1));
        case 'test'
            t = (0:(N-1))*ref_dis;
            dis = 1;
            y= 1;
            xref = dis*[1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20];
            yref = [0 0 0 0 0 0 0 0 0.333*y 0.666*y y y y y y y y y y y];
            psiref=atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1)); 

    end
    
    Xref=xref';%change row into column
    Yref=yref';%change row into column
    Psiref=psiref';%change row into column
    
    %Duplicate first value for first iteration in simulink (For proper delay value)
    Xref = [Xref(1); Xref];
    Yref = [Yref(1); Yref];
    Psiref = [Psiref(1); Psiref(1); Psiref];
end