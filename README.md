# PIDControlRTRoboticArm





Proiect Sisteme de Conducere în Robotică (SCR)



Conducerea unui robot RT








Student: Ilie Razvan-Mihai
Specializare: Robotica
Grupa: 4LF871
Anul: 2020









1. Introducere
	Un brat robotic este utilizat pentru a usura procesul de mutare a materialelor, miscare necesara in procesele de fabricatie. Procesul de transfer al materialelor este de obicei implinit folosind puterea omului, acesta poate provoca rani operatorului. Bratul robotic previne posibilele accidente ale omului si  ajuta la cresterea eficientei productiei prin reducerea erorii umane.
	Cei mai multi roboti manipulatori gasiti in operatiile industriale sunt controlati prin algoritmi de tipul PID (instrumentul derivat proportional-integral ). Acest tip de control are o structura simpla pentru cele trei castiguri ale sale.
	Exista multe tehnici utilizate pentru controlul bratului robotului. Cele mai cunoscute tehnici de control sunt control PID, control adaptiv, control optim si controlul robust. 
	Intrucat obiectivul final este proiectarea si fabricarea de roboti adevarati, este necesar sa avem simularea dinaintea investigatiilor cu roboti reali.



2. Modelarea geometrica a structurii

	Bratul robotic are doua grade de libertate si este constituit dintr-o cupla de rotatie si una de translatie (RT) avand 2 laturi si 2 greutati ce prin animarea structurii degaja 2 forte:  momentul motor (T), forta motoare (F).
 
		m1=1kg;
		l1=5.02m;
		m2=1kg;
		l2=0.2m;

	Vom folosi principiul coordonatelor generalizate pentru abstractizarea variabilelor cuplelor:
i - indicele cuplei;
epsilon.i = {0-rot;1-trans; !epsilon.i=1 - epsilon.i;}
q.i = !epsilon.i * theta.i + epsilon.i * x.i;
Prin urmare miscarile din cuple devin:
q1= (1-0)*theta + 0*x=theta;
q2=(1-1)*theta + 1*x=x;
	Cu ajutorul principiului reperului translatat construim matricile de transformare 01T, 12T, 2ET. Matricia de transformare de la originie la end effector:
0ET=01T*12T*2ET;
ijT=[  ijR(3x3), jiD(3x1),
	0(1x3),    1(1x1)  ];
01T = [ Rz(q1)(3x3), 10D(3x1),
	    0(1x3),           1(1x1)   ];
Rz(q1) = [ cos(q1), -sin(q1), 0,
	        sin(q1), cons(q1), 0,  
		0,         0,              1 ];
10D=[ 0,
	 0,
	 0 ];
12T=[ I(3x3), 21D(3x1),
           0(1x3),   1(1x1) ];
I = [ 	1,0,0,
	0,1,0,
	0,0,1 ];
21D = [ l1,
	    0,
	    0 ];
2ET = [ I(3x3),  E2D(3x1),
	    0(1x3),  1(1x1) ];
E2D = [ l2+q2,
	     0,
	     0 ];
0ET = [ cos(q1), -sin(q1), 0, cos(q1)*l1+ cos(q1)*(l2 + q2),
	    sin(q1), cos(q1), 0, sin(q1)*l1+sin(q1)*(l2+q2),
	     0,         0,           1,       0,
	      0,        0,            0,       1     ];
de unde rezulta punctul end effectorului:
E(x,y)=[ cos(q1)*l1+ cos(q1)*(l2 + q2), sin(q1)*l1+sin(q1)*(l2+q2) ];

 



3. Modelarea cinematica a structurii
3.1 Cinematica directa
	Modelarea cinematica presupune cunoasterea valorilor finale si initiale ale cuplelor din care rezulta incrementarea (schimbarea) la un moment de timp de la valoarea initiala la valoare urmatoare (dq=qfinal-qinit) a unei cuple, cu ajutorul acestora putem formula Jacobian-ului in cinematica directa pentru a afla viteza end effectorului la fiecare miscare. 
Folosind formula Jacobianului avem:
[ Vx,   = J * [ dq1,
  Vy ]             dq2 ];
Unde J este matricea Jacobianului de forma:
J = [ dp(f1(q))/dp(q1), dp(f1(q))/dp(q2),
        dp(f2(q))/dp(q1), dp(f2(q))/dp(q2) ];
Unde f1 si f2 sunt functii ce primesc ca input valorile finale ale cuplelor si returneaza propietatea X si respectiv Y a punctului end effectorului.
f1(q)= cos(q1)*l1+cos(q1)*(l2+q2);
f2(q)= sin(q1)*l1+sin(q1)*(l2+q2);
dp(f1(q))/dp(q1)=-sin(q1)*l1-sin(q1)*(l2+q2);
dp(f1(q))/dp(q2)=cos(q1);
dp(f2(q))/dp(q1)=cos(q1)*l1+cos(q1)*(l2+q2);
dp(f2(q))/dp(q2)=sin(q1);
Din acestea rezulta:
[ -sin(q1)*l1-sin(q1)*(l2+q2), cos(q1),    *  [ dq1,    =   [ Vx,
  cos(q1)*l1+cos(q1)*(l2+q2), sin(q1) ]       dq2 ]          Vy ];
Vx = (-sin(q1)*l1-sin(q1)*(l2+q2))*dq1 + cos(q1)*dq2;
Vy = (cos(q1)*l1+cos(q1)*(l2+q2))*dq1 + sin(q1)*dq2;
V = sqrt((Vx)^2+(Vy)^2); 
q1,q2 fiind functii ce depind de timp.

3.2 Cinematica inversa
	Presupune cunoasterea vitezelor si a valorilor initiale si finale din cuple pentru deductia noilor valorilor din acestea.Cinematica inversa pentru controlul manipulatoarelor se foloseste intr-o bucla inchisa pentru a putea ajunge la valorile dorite din cuple (xd).
 
Formularea Jacobian-ului in cinematica inversa:
[ dq1,   =  (J)^(-1) * [ Vx,
  dq2 ]                       Vy ];
(J)^(-1) = 1/det(J) * adj(J);
det(J) != 0 (daca det(J)=0 inseamna ca avem un punct de singularitate si suntem inafara volumului de lucru al robotului, prin urmare nu se poate construi inversa Jacobianului);
det(J) = ( -sin^2(q1) * l1 - sin^2(q1) * (l2+q2) ) - ( cos^2(q1) * l1 
+ cos^2(q1)*(l2+q2));
adj(J)= [ sin(q1), 			               -cos(q1),
	     -cos(q1)*l1-cos(q1)*(l2+q2), -sin(q1)*l1-sin(q1)*(l2+q2) ];
de unde rezulta:
(J)^(-1) = 1/( (-sin^2(q1) * l1 - sin^2(q1) * (l2+q2) ) - ( cos^2(q1) * l1+ cos^2(q1) * (l2+q2) ) ) * adj(J);

3.4 Definirea traiectorilor
	Se defineste traiectoria din spatiul task ca fiind o linie dreapta intre doua puncte P0(x0,y0) si Pf(xf,yf), iar pentru traiectorile din spatiul articular se defineste o functie polinomiala de gradul 3 in functie de timp pentru fiecare cupla i (1 sau 2):
q.i(T) = a.i.0 + a.i.1 * T + a.i.2 * (T)^2 + a.i.3 * (T)^3;
dq.i(T) = a.i.1 + a.i.2 * T  + a.i.3 * (T)^2;
d2q.i(T) = a.i.2 + a.i.3 * T;
Unde se cunosc valorile si vitezele din cuple la momentul initial si la cel final (q1init, q1final, q2init, q2final, dq1init, dq1final, dq2init, dq2final).
La T = 0 avem:
q.i(0) = a.i.0, din care rezulta a.i.0 = q.iinit;
dq.i(0) = a.i.1, din care rezulta a.i.1 = dq.iinit;
La T = tfinal avem:
q.i(tfinal) = q.ifinal;
dq.i(tfinal) = dq.ifinal;
Din acestea rezulta:
a.i.2 = (3/(tfinal)^2)*(q.ifinal - q.iinit) - (1/tfinal)*(dq.ifinal + 2*dq.iinit);
a.i.3 = (2/(tfinal)^3)*(q.iinit - q.ifinal) + (1/(tfinal)^2)*(dq.ifinal + dq.iinit);
 


 


4. Modelarea dinamica a structurii
	Fortele ce afecteaza structura sunt: T (momentul motor), F (forta motoare). Generalizand pe acestea avem Q1,2.
Folosind formularea Lagrangian-ului:
L(q,dq)=Ec(q,dq)-Ep(q);
Forta gravitationala G fiind perpendiculara pe planul in care opereaza robotul se considera energia potentiala egala cu 0 (Ep=0) si rezulta:
L(q,dq)=Ec(q,dq);
L=( 1/2 * m1 * (V1)^2 + 1/2 * MI1 * (dq1)^2 ) + ( 1/2 * m2 * (V2)^2 + 1/2 * MI2 * (dq2)^2 );
MI1=m1*(l1)^2 - Momentul de inertie 1;
MI2=m2*(l2+q2)^2 - Momentul de inertie 2;
(V1)^2=(l1)^2*(dq1)^2 - viteza corp 1;
(V2)^2=(dq2)^2+(dq1)^2*(q2)^2  -viteza corp 2;
Rezulta:
L = m1 * (l1)^2 * (dq1)^2 + (m2/2) * ( (dq2)^2 + (dq1)^2 * (q2)^2+ (dq2)^2 * ( (l2)^2 + 2*l2*q2 + (q2)^2) );
d/dt(dp(L)/dp(dq1,2)) - dp(L)/dp(q1,2) = Q1,2;
dp(L)/dp(dq1)=dq1*(2*m1*(l1)^2+m2*(q2)^2);
d/dt( dp(L) / dp(dq1) ) = d2q1*(2 * m1 * (l1)^2 + m2 * (q2)^ 2) 
				 + 2 * dq1 * m2 * q2;
dp(L)/dp(q1) = 0;
din acestea rezulta:
T= d2q1 * ( 2 * m1 * (l1)^2 + m2 * (q2)^2) +   2 * dq1 * m2 * q2;
dp(L)/dp(dq2)=dq2*(m2+m2*( (l2)^2 + 2*l2*q2 + (q2)^2 ) );
d/dt(dp(L)/dp(dq2))=d2q2*(m2 + m2*( (l2)^2 + 2*l2*q2 + (q2)^2));
dp(L)/dp(q2)=m2*( (dq1)^2 * q2 + (dq2)^2 * l2 + (dq2)^2 * q2);
din acestea rezulta:
F= d2q2 * ( m2 + m2 * ( (l2)^2 + 2*l2*q2 + (q2)^2 ) ) 
     - m2 * ( (dq1)^2 * q2 + (dq2)^2 * l2 + (dq2)^2 * q2);


Simulink: robot RT model dinamic direct
 

Simulink: robot RT model dinamic invers

 
 
Simulink: robot RT model dinamic invers, termenul K1
 
Simulink: robot RT model dinamic invers, termenul K2
 

5. Controlul sistemului robotic
5.1 Etape parcurse
Task plan - obiectivul robotului este de a desena o linie dreapta din pozitia q1=0, q2=0 pana la Ex=7, Ey=3.
Action plan - se genereaza prin interpolarea liniei un numar de 70 puncte ce se afla pe dreapta.
Path plan - pentru fiecare punct de pe dreapta se calculeaza prin modelul cinematic invers pozitile si vitezele articulatilor.
Trajectory plan - pentru definirea spatiului articular se folosesc ca si pozitii, viteze si acceleratii  dorite functile cubice polynomiale de gradul 3 cu viteze si acceleratii initiale si finale inexistente.
Controller - pentru controller-ul motorului a fost folosit un regulator de tip PD (proportional-derivative controller) ce modifica semnalul de iesire in functie de eroarea pozitiei si a vitezei.
Robot - este reprezentat de modelul matematic al motorului de cc + reductor + modelul dinamic direct al structurii. 
Sensor - simulink

Etape parcurse
 

5.2. Regulatorul PID
	Regulatorul PID este un mecanism bucla de control care foloseste principiul feedback ce este utilizat pe scara larga in sistemele de control industriale. Un controler PID calculează continuu o valoare de eroare e(t) ca diferenta intre o valoare dorita si o variabila de proces masurata si aplica o corectie bazata pe proportionare, integrare si derivare, de unde și numele.
Comportamentul dinamic al sistemelor liniare de ordin 2:
m*d2x + b*dx + K*x = V;
avand forma canonica:
d2x+2*gama*w*dx+w^2*x = u;
Aplicand transformata Laplace avem functia de transfer:
G(s)=1/(s^2+2*gama*w*s+w^2);
unde:
gama=b/2*sqrt(K*m) = 1;
w=sqrt(K/m) = 0.2;
Polii s1 si s2 dorim sa fie inclusi in multimea numerelor reale, s1=s2 si s2<0 pentru a avea un sistem critic amortizat.

Sistem critic-amortizat
 

Parametrii de acord Kv > -b si Kp > 0 ii aflam cu ajutorul relatilor lui Viete:
s1+s2 = -(Kv+b);
s1*s2 = Kp+K;
din care rezulta:
Kv = b - (s1+s2);
Kp = s1*s2-K;
De unde putem scrie:
u = Kv * dx + Kp * x + u*;
Se alege s1 = -100, s2 = -100, din care rezulta:
Kv = 0.4 - (-200) = 200.4;(termenul de derivare din regulatorul PID)
Kp = 10000 - 0.04 = 10000.04;(termenul de proportionare din regulatorul PID)
Astfel putem construi un controller PID pentru robotul nostru.

Simulink: regulator PD
 
Simulink: regulator PID
 

5.3. Controlul modelului dinamic holist
	Controlul modelul dinamic holist presupune utilizare unui regulator PID/PD pentru controlul motorului de curent continuu ce este legat prin transmisie la modelul dinamic direct al robotului.

Schema bloc structura motor cc
 

	Motorul este definit atat electric cat si mecanic in relatiile urmatoare:
V1/2=Ra*i1/2+L*((di1/2)/dt)+E1/2;
J*d2qm1/2=Km*i1/2-b*dq-T1/2;

Ecuatii motorc cc
 
qm1/2 = i1/2 * q1/2;
T1/2 = 1/(i1/2) * Q1/2;
Q1/2 - fortele generalizate de afecteaza structura (momentul motor + forta motoare);
	Motorul ales are un diametru de 32 mm,o putere mecanica de 80 watt si poate fi maxim alimentat la 48V. Reductorul ales este GP 32 BZ de diametru 32 mm si reactie medie de 0.4.
iR = 35; (raportul de transmisie)
J = 20 g(cm)^2 = 20*(10)^(-7) kg(m)^2; (momentul de inertie)
Km = 40 mNm/A = 40*(10)^(-3) Nm/A; (constanta cuplului)
Kb = 325 rpm/V = 60/325*2*pi V/(rad/s); (constanta vitezei)
Ra = 5.42 ohm; (rezistenta armaturii)
b = 0.4;(reactie medie)
m1,2 = 1 kg;
Putem deduce pentru modelul motorului:
Km*V1 = (J*iR+m2/iR)*d2q1m+iR*(b+Kb*Km/Ra)*dq1m+P1;
Km*V2 = (J*iR+(m1+m2)/iR)*d2q2m+iR*(b+(Kb*Km)/Ra)*dq2m  +P2;
(J*iR+m2/iR)/Km = 0.71575;
(iR*(b+Kb*Km/Ra))/Km = 350.1892;
(J*iR+(m1+m2)/iR)/Km = 1.43032;
P1,P2 - perturbarea motorului1, respectiv2
din care rezulta:
V1 = 0.71575*d2q1m + 350.1892*dq1m + P1;
V2 = 1.43032*d2q2m + 350.1892*dq2m + P2;
Se va tine cont de alimentarea maxima de 48V.


