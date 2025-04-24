c=1;v=1;d=2;
lambda=0;
C=[0.6 0.8];
A=[-2 0; -1 -2];
Ad=[-0.2 -0.5; 0.5 -0.2];
B=[-0.2; -0.3];
Gamma=eye(size(A));
A1=A+c*lambda*Gamma;

setlmis([]);

P=lmivar(1,[2 1]);
Q=lmivar(1,[2 1]);
X=lmivar(1,[2 1]);
Y=lmivar(1,[2 1]);
Z=lmivar(1,[2 1]);

lmiterm([1 1 1 P],1,A1,'s');
lmiterm([1 1 1 Y],1,1,'s');
lmiterm([1 1 1 X],d,1);
lmiterm([1 1 1 Q],1,1);

lmiterm([1 1 2 P],1,Ad);
lmiterm([1 1 2 Y],-1,1);
lmiterm([1 1 3 P],-1,B);
lmiterm([1 1 3 0],-C'*v');
lmiterm([1 1 4 Z],d*A1,1);
lmiterm([1 2 2 Q],-1,1);
lmiterm([1 2 4 Z],d*Ad',1);
lmiterm([1 3 3 0],-2);
lmiterm([1 3 4 Z],-d*B',1);
lmiterm([1 4 4 Z],-d,1);

lmiterm([-2 1 1 X],1,1);
lmiterm([-2 1 2 Y],1,1);
lmiterm([-2 2 2 Z],1,1);

lmiterm([-3 1 1 P],1,1);

lmisys=getlmis;
[tmin,xfeas]=feasp(lmisys);

PP=dec2mat(lmisys,xfeas,P);
QQ=dec2mat(lmisys,xfeas,Q);
XX=dec2mat(lmisys,xfeas,X);
YY=dec2mat(lmisys,xfeas,Y);
ZZ=dec2mat(lmisys,xfeas,Z);

