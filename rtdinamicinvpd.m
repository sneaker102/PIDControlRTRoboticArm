clear
clc

myRobot =  RTrobot(0,0);
tPoint = myRobot.getTooltipPoint;
initQs = tPoint;
Ex = 7.0;
Ey = 3.0;
xInTime = linspace(tPoint(1,1),Ex,70);
yInTime = linspace(tPoint(2,1),Ey,70);
dQs = [0;...
       0];
tQs = [myRobot.q1;...
        myRobot.q2];
Qs = [myRobot.q1;...
        myRobot.q2];
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
    dQs = cat(2,dQs,deltaPoint);
    if (myRobot.q1 == 0)
        myRobot.q1 = 0.0001;
    end
    if (myRobot.q2 == 0)
        myRobot.q2 = 0.0001;
    end
    tQs = tQs + myRobot.calculateQsByIK(deltaPoint);
    Qs = cat(2,Qs,tQs);
    myRobot.q1 = tQs(1);
    myRobot.q2 = tQs(2);
    
    tPoint = myRobot.getTooltipPoint;
    deltaSignal = abs(tPoint - neededPoint);
    percentageDifference = deltaSignal ./ tPoint; % Percent by element.
    err = mean(percentageDifference); % Average percentage over all elements.
    while ( abs(err) > 0.001)
        deltaPoint = neededPoint  - myRobot.getTooltipPoint;
        dQs = cat(2,dQs,deltaPoint);
        tQs = tQs + myRobot.calculateQsByIK(deltaPoint);
        Qs = cat(2,Qs,tQs);
        if (tQs(1) == 0)
            tQs(1) = 0.0001;
        end
        if (tQs(2) == 0)
            tQs(2) = 0.0001;
        end
        myRobot.q1 = tQs(1);
        myRobot.q2 = tQs(2);
        
       
        tPoint = myRobot.getTooltipPoint;
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
 
 t=linspace(0,3,length(Qs));
 
 jointPath = myRobot.jointPlanning(length(Qs),initQs(1),tQs(1),initQs(1),tQs(2),0,0,0,0); 
 
 dQs = jointPath(3:4,:).*dQs;
 
 d2Qs = jointPath(5:6,:);
 

 poz_q1=[t;Qs(1,:)];
 poz_q2=[t;Qs(2,:)];
 vit_q1=[t;dQs(1,:)];
 vit_q2=[t;dQs(2,:)];
 acc_q1=[t;d2Qs(1,:)];
 acc_q2=[t;d2Qs(2,:)];
 
 save pozdorq1.mat poz_q1
 save pozdorq2.mat poz_q2
 save vitdorq1.mat vit_q1
 save vitdorq2.mat vit_q2
 save accdorq1.mat acc_q1
 save accdorq2.mat acc_q2



sim('RTDinamicInvPD.slx')
t = linspace(0,3,length(q1out.Data));
q1Dor = Qs(1,:);
q2Dor = Qs(2,:);


figure
myRobot.q1 = 0.001;
myRobot.q2 = 0.001;
myRobot.drawRobot(1);
p = [myRobot.getTooltipPoint;
      -0.1];
for i = 1 : length(q1out.Data)
    if (abs(q1out.Data(i)) > 0.005 || abs(q2out.Data(i)) > 0.005)
     cla;
     myRobot.q1 = q1out.Data(i);
     myRobot.q2 = q2out.Data(i);
     myRobot.drawRobot(0);
     p = cat(3,p,[myRobot.getTooltipPoint;-0.1]);
     pause(0.009);
     end
 end
 pause(1);
 
figure
hold
plot(p(1,:),p(2,:),'linewidth', 1,'color','r')
grid
xlabel 'x'
ylabel 'y'

figure 
hold
plot(q1out.Data,q2out.Data)
plot(q1Dor,q2Dor,'.')
grid
legend('Rezultat', 'Dorit')
xlabel 'q1'
ylabel 'q2'
%axis([0,1,0,1])
axis 'equal'

figure 
hold
plot(t,q1out.Data)
plot(t,q2out.Data)
grid
legend('q1', 'q2')
xlabel 't'
ylabel 'q1/2'

figure 
hold
plot(t,dq1out.Data)
plot(t,dq2out.Data)
grid
legend('dq1', 'dq2')
xlabel 't'
ylabel 'dq1/2'

figure 
hold
plot(t,d2q1out.Data)
plot(t,d2q2out.Data)
grid
legend('d2q1', 'd2q2')
xlabel 't'
ylabel 'd2q1/2'






