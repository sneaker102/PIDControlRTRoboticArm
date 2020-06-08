classdef RTrobot
    %RTROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        q1 = 0; %rad
        q2 = 0; %m
        razaCuplaRot = 0.02 %m;
        l1 = 5; %m
        l2=0.2; %m
        maxQ1 = 210*pi/180; %rad
        minQ1 = -60*pi/180; %rad
        maxQ2 = 3.5; %m
        minQ2 = 0; %m
    end
    
    methods
        function obj = RTrobot(q1,q2)
            obj.q1 = q1*pi/180;
            obj.q2 = q2;
            obj.l1 = 5 + obj.razaCuplaRot;
        end
        function drawRobot(obj, flag)
            cuplaRot= [obj.razaCuplaRot*cos(0:0.1:2*pi);...
            obj.razaCuplaRot*sin(0:0.1:2*pi);...
            zeros(1,length(0:0.1:2*pi))];
       
            L1 = [obj.razaCuplaRot, obj.l1*cos(obj.q1);...
                  0, obj.razaCuplaRot+(obj.l1*sin(obj.q1));...
                  0, 0];
            cuplaTrans = [L1(1,2)-(obj.maxQ2)*cos(obj.q1), L1(1,2);...
                          L1(2,2)-(obj.maxQ2)*sin(obj.q1),L1(2,2);...
                          0,     0];
            L2 = [L1(1,2), L1(1,2)+(obj.l2+obj.q2)*cos(obj.q1);...
                  L1(2,2), L1(2,2)+(obj.l2+obj.q2)*sin(obj.q1);...
                  0,0];
           cap = L2(:,2) + [obj.l2*cos(0:0.1:2*pi);...
                            obj.l2*sin(0:0.1:2*pi);...
                            zeros(1,length(0:0.1:2*pi))];
          if (flag == 1)
           axis 'equal'
           view (32,14)
           hold
           grid
          end
           plot3(cuplaRot(1,:),cuplaRot(2,:),cuplaRot(3,:),'*r')
           plot3(L1(1,:),L1(2,:),L1(3,:),'linewidth', 2,'color','b')
           plot3(cuplaTrans(1,:),cuplaTrans(2,:),cuplaTrans(3,:),'linewidth', 4,'color','r')
           plot3(L2(1,:),L2(2,:),L2(3,:),'linewidth', 2,'color','b')
           plot3([cap(1,:);L2(1,2)*ones(1,length(0:0.1:2*pi))],[cap(2,:);L2(2,2)*ones(1,length(0:0.1:2*pi))],...
           [cap(3,:);-0.1+zeros(1,length(0:0.1:2*pi))],'r')
           fill3([0,obj.l2,obj.l2,0],[0,0,obj.l2,obj.l2],[0,0,0,0]-obj.l2,'w')
        
         
        end
        function dQs = getDqs(obj, Qs)
            dQ = [];
            for i = 1:length(Qs)
                         
            speedX = -obj.l1*sin(Qs(1,i));
            speedX = speedX * speedX;
            speedY = obj.l1*cos(Qs(1,i));
            speedY = speedY * speedY;
            dQ1 = sqrt(speedX+speedY);
            
            speedX = -obj.l1*sin(Qs(1,i)) -(obj.l2+Qs(2,i))*sin(Qs(1,i));
            speedX = speedX * speedX;
            speedY = obj.razaCuplaRot+(obj.l1*cos(Qs(1,i)))+(obj.l2+Qs(2,i))*cos(Qs(1,i));
            speedY = speedY * speedY;
            dQ2 =  sqrt(speedX+speedY);
            dQes = [ dQ1;...
                   dQ2]; 
            dQ = cat(2,dQ,dQes);
            end
            dQs = dQ;
        end
        function d2Qs = getD2qs(obj,Qs)
            d2Qes = [];
            for i = 1:length(Qs)
                         
            accX = -obj.l1*cos(Qs(1,i));
            accX = accX * accX;
            accY = -obj.l1*sin(Qs(1,i));
            accY = accY * accY;
            d2Q1 = sqrt(accX+accY);
            
            accX = -obj.l1*cos(Qs(1,i)) -(obj.l2+Qs(2,i))*cos(Qs(1,i));
            accX = accX * accX;
            accY = obj.razaCuplaRot-(obj.l1*sin(Qs(1,i)))-(obj.l2+Qs(2,i))*sin(Qs(1,i));
            accY = accY * accY;
            d2Q2 =  sqrt(accX+accY);
            d2Q = [ d2Q1;...
                   d2Q2]; 
            d2Qes = cat(2,d2Qes,d2Q);
            end
            d2Qs = d2Qes;
        end
        function tooltipPoint = getTooltipPoint(obj)
            tooltipPoint = [ cos(obj.q1)*obj.l1+cos(obj.q1)*(obj.l2 + obj.q2);...
                                sin(obj.q1)*obj.l1+sin(obj.q1)*(obj.l2 + obj.q2)];
        end
        
        function plotTooltipSpeed(obj,lengthQs,q1Init,q2Init,q1Final,q2Final)
            qsPath = obj.jointPlanning(lengthQs,q1Init,q1Final,q2Init,q2Final,0,0,0,0);
             pathSize = size(qsPath);
           
            obj.q1 = q1Init;
            obj.q2 = q2Init;
            dx = [];
            dy = [];
            for i = 1:pathSize(2)
                Vend = [
                    (-sin(qsPath(1,i))*obj.l1-sin(qsPath(1,i))*(obj.l2+qsPath(2,i)))*qsPath(3,i) + cos(qsPath(1,i))*qsPath(4,i);...
                    (cos(qsPath(1,i))*obj.l1+cos(qsPath(1,i))*(obj.l2+qsPath(2,i)))*qsPath(3,i) + sin(qsPath(1,i))*qsPath(4,i)
                    ];
                dx = cat(1,dx,Vend(1));
                dy = cat(1,dy,Vend(2));
            end
            t=linspace(0,3,length(dx));
              figure
              plot(t,dx,'b',t,dy,'r')
              xlabel ' dx [m/s]'
              ylabel ' dy [m/s]'
              grid
        end
        
        function drawLine(obj, Ex, Ey)
           tPoint = obj.getTooltipPoint;
             if (tPoint(1) == 0)
                tPoint(1) = 0.0001;
            end
            if (tPoint(2) == 0)
                tPoint(2) = 0.0001;
            end
           % line points
           xInTime = linspace(tPoint(1,1),Ex,15);
           yInTime = linspace(tPoint(2,1),Ey,15);
           lengthQs = 0;
           for i = 1:length(xInTime) 
            if (xInTime(i) == 0)
                xInTime(i) = 0.00001;
            end
            if (yInTime(i) == 0)
                yInTime(i) = 0.00001;
            end
            neededPoint = [xInTime(i);...
                          yInTime(i)];    
            deltaPoint = neededPoint  - tPoint;
            if (obj.q1 == 0)
                obj.q1 = 0.0001;
            end
            if (obj.q2 == 0)
                obj.q2 = 0.0001;
            end
            tQs = [obj.q1;...
                  obj.q2];
            initQs = tQs;
            lengthQs = lengthQs + 1;
            tQs = tQs + obj.calculateQsByIK(deltaPoint);
            obj.q1 = tQs(1);
            obj.q2 = tQs(2);
           
            tPoint = obj.getTooltipPoint;
            deltaSignal = abs(tPoint - neededPoint);
            percentageDifference = deltaSignal ./ tPoint; % Percent by element.
            err = mean(percentageDifference); % Average percentage over all elements.
            while ( abs(err) > 0.0001)
                deltaPoint = neededPoint  - obj.getTooltipPoint;
                tQs = tQs + obj.calculateQsByIK(deltaPoint);
                lengthQs = lengthQs + 1;
                if (tQs(1) == 0)
                    tQs(1) = 0.0001;
                end
                if (tQs(2) == 0)
                    tQs(2) = 0.0001;
                end
                obj.q1 = tQs(1);
                obj.q2 = tQs(2);
             
                cla;
                obj.drawRobot(0);
                pause(0.09); 
                tPoint = obj.getTooltipPoint;
                if (tPoint(1) == 0)
                    tPoint(1) = 0.0001;
                end
                if (tPoint(2) == 0)
                    tPoint(2) = 0.0001;
                end
                deltaSignal = abs(tPoint - neededPoint);
                percentageDifference = deltaSignal ./ tPoint; % Percent by element.
                err = mean(percentageDifference); % Average percentage over all elements.
            end
           
           end
           obj.plotTooltipSpeed(lengthQs,initQs(1),obj.q1, initQs(2),obj.q2);
        end
        
        function path = jointPlanning(obj,interpolationNr,q1init,q1final,q2init,q2final,dq1init,dq1final,dq2init,dq2final)
            Tf=3; %intervalul  de timp in care se realizeaza taskul [s]
            %aij , i stands for joint, j for a parameter from q.i(T) = a.i.0 + a.i.1 * T + a.i.2 * (T)^2 + a.i.3 * (T)^3;
            a10 = q1init;
            a20 = q2init;
            a11 = dq1init;
            a21 = dq2init;
            a12 = (3/Tf.^2)*(q1final-q1init)-(1/Tf)*(dq1final + 2*dq1init);
            a22 = (3/Tf.^2)*(q2final-q2init)-(1/Tf)*(dq2final + 2*dq2init);
            a13 = (2/Tf.^3)*(q1init - q1final) + (1/Tf.^2)*(dq1final + dq1init);
            a23 = (2/Tf.^3)*(q2init - q2final) + (1/Tf.^2)*(dq2final + dq2init);
            t=linspace(0,Tf,interpolationNr);
            q1InTime = polyval([a10,a11,a12,a13],t);
            q2InTime = polyval([a20,a21,a22,a23],t);
            dq1InTime = polyval([a11,a12,a13],t);
            dq2InTime = polyval([a21,a22,a23],t);
            d2q1InTime = polyval([a12,a13],t);
            d2q2InTime = polyval([a22,a23],t);
            path = [q1InTime;...
                    q2InTime;...
                    dq1InTime;...
                    dq2InTime;...
                    d2q1InTime;...
                    d2q2InTime];
        end
        function dotQs =  calculateQsByIK(obj, deltaPoint)
            adjJacob = [sin(obj.q1),                                    -cos(obj.q1);...
                        -cos(obj.q1)*obj.l1-cos(obj.q1)*(obj.l2+obj.q2), -sin(obj.q1)*obj.l1-sin(obj.q1)*(obj.l2+obj.q2)];
            invJacob = 1/(((-sin(obj.q1).^2)*obj.l1-sin(obj.q1).^2*(obj.l2+obj.q2))-(cos(obj.q1).^2*obj.l1+cos(obj.q1).^2*(obj.l2+obj.q2)));
            invJacob = invJacob * adjJacob;
            dotQs = invJacob * deltaPoint;
        end
        
    end
end

