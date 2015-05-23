%  此程式用來分析eight-bar的動作
%  常數: r2 r3 r4 r5 r6 r7 r8 r9 r10 r11 th1 al be ga
%  變數: r1 th2 th3 th4 th5 th th7 th8 th9 th10 th11  (單位 cm)
%  equation :10
%  dof : 1
%  r2 is crank; thet2 can be turn 360 dergree
clear all

% 弧度角度互換
d2r = pi/180;
r2d = 180/pi;

%輸入常數
r2 = input('請輸入 r2 長度 [23.5]:');
if isempty(r2); r2 = 23.5; end
r3 = input('請輸入 r3 長度 [135]:');
if isempty(r3); r3 = 135; end
r4 = input('請輸入 r4 長度 [30.5]:');
if isempty(r4); r4 = 30.5; end
r5 = input('請輸入 r5 長度 [95]:');
if isempty(r5); r5 = 95; end
r6 = input('請輸入 r6 長度 [90]:');
if isempty(r6); r6 = 90; end
r7 = input('請輸入 r7 長度 [18]:');
if isempty(r7); r7 = 18; end
r8 = input('請輸入 r8 長度 [10]:');
if isempty(r8); r8 = 10; end
r9 = input('請輸入 r9 長度 [80]:');
if isempty(r9); r9 = 80; end
r10 = input('請輸入 r10 長度 [74.5]:');
if isempty(r10); r10 = 74.5; end
r11 = input('請輸入 r11 長度 [73]:');
if isempty(r11); r11 = 73; end
theta1 = input('請輸入 theta1 角度 [0]:');
if isempty(theta1); theta1 = 0 ; end
alpha = input('請輸入 alpha 角度 [0]:');
if isempty(alpha); alpha = 0 ; end
beta = input('請輸入 beta 角度 [2]:');
if isempty(beta); beta = 2 ; end
gamme = input('請輸入 gamme 角度 [7]:');
if isempty(gamme); gamme = 7 ; end

%角度換弧度
th1 = theta1*d2r;
al = alpha*d2r;
be = beta*d2r;
ga = gamme*d2r;

%預設轉一圈
run = 'y';
run2 = '-';
nn=1; %theta2 角度間隔
while run == 'y'
   
    theta2 = 0:nn:359;
    n = numel(theta2);
    th2 = theta2.*d2r;
    
    %開始計算 eightbar 變數
    var = zeros(n,12);
    X = zeros(1,n); Y = zeros(1,n);
    for i = 1:n
        %equation 1
        th4 = th1 - (d2r*90);
        A = 2*r4*(cos(th1)*cos(th4)+sin(th1)*sin(th4))-2*r2*(cos(th1)*cos(th2(i))+sin(th1)*sin(th2(i)));
        B = (r2^2)+(r4^2)-(r3^2)-2*r2*r4*(cos(th2(i))*cos(th4)+sin(th2(i))*sin(th4));
        sig = 1;
        r1 = (-A+sig*sqrt((A^2)-4*B))/2 ;
        th3 = atan2(r1*sin(th1)+r4*sin(th4)-r2*sin(th2(i)),r1*cos(th1)+r4*cos(th4)-r2*cos(th2(i)));  
        th9 = th3 - al; th11 = th3 + ga;
        var(i,1)=r1 ;var(i,2)=th2(i); var(i,3)=th3; var(i,4)=th4; var(i,9)=th9; var(i,11)=th11;
        %EQUATION 2
        theta13 = 0; theta14 = theta13 - (90); th12 = th3+(180*d2r); th13 = theta13*d2r; th14 = theta14*d2r;
        r12 = 55; r13 = (r1+6) ; r14 = r4+59;  var(i,12) = th12;
        
        C1 = r12*cos(th12)+r13*cos(th13)+r14*cos(th14);
        C2 = r12*sin(th12)+r13*sin(th13)+r14*sin(th14);
        C3 = (r5^2)-(C1^2)-(C2^2)-(r10^2)-2*r10*C1;
        C4 = 4*C2*r10;
        C5 = (r5^2)-(C1^2)-(C2^2)-(r10^2)+2*C1*r10;
        sig = -1;
        det = sqrt((C4^2)-4*C3*C5);
        th10 = 2*atan2((-C4+sig*det),(2*C3));
        th5 = atan2((C2-r10*sin(th10)),(C1-r10*cos(th10)));
        th6 = th10 + be;
        var(i,5) = th5; var(i,10) = th10; var(i,6)= th6;
        
%         %equation3
         r15 = 59.5; th15 = (86-180)*d2r;
%         C1 = r5*cos(th5)+r6*cos(th6)-r2*cos(th2(i))-r11*cos(th11)-r15*cos(th15);
%         C2 = r5*sin(th5)+r6*sin(th6)-r2*sin(th2(i))-r11*sin(th11)-r15*sin(th15);
%         C3 = (r8^2)-(C1^2)-(C2^2)-(r7^2)-2*r7*C1;
%         C4 = 4*C2*r7;
%         C5 = (r8^2)-(C1^2)-(C2^2)-(r7^2)+2*C1*r7;
%         sig = -1;
%         det = sqrt((C4^2)-4*C3*C5);
%         th7 = 2*atan2((-C4+sig*det),(2*C3));
%         th8 = atan2((C2-r7*sin(th7)),(C1-r7*cos(th7)));
%         var(i,7)= th7; var(i,8)= th8;
        %equation 4
        C1 = (r1*cos(th1)+r4*cos(th4)+r12*cos(th12)+r6*cos(th6))-(r2*cos(th2(i))+r10*cos(th10)+r11*cos(th11));
        C2 = (r1*sin(th1)+r4*sin(th4)+r12*sin(th12)+r6*sin(th6))-(r2*sin(th2(i))+r10*sin(th10)+r11*sin(th11));
        C3 = (r8^2)-(C1^2)-(C2^2)-(r7^2)-2*C1*r7;
        C4 = 4*C2*r7;
        C5 = (r8^2)-(C1^2)-(C2^2)-(r7^2)+2*C1*r7;
        sig = -1;
        det = sqrt((C4^2)-4*C3*C5);
        th7 = 2*atan2((-C4+sig*det),(2*C3));
        th8 = atan2((C2-r7*sin(th7)),(C1-r7*cos(th7)));
        var(i,7)= th7; var(i,8)= th8;
    end
    %動畫
         
    for i=1:1
        for i=1:n
            JOx = 0; JOy = 0;
            JAx = r2*cos(var(i,2)); JAy = r2*sin(var(i,2));
            JBx = r2*cos(var(i,2))+r3*cos(var(i,3)); JBy = r2*sin(var(i,2))+r3*sin(var(i,3));
            JEx = JBx+r12*cos(var(i,12)); JEy = JBy+r12*sin(var(i,12));          
            JDx = JEx-r10*cos(var(i,10)); JDy = JEy-r10*sin(var(i,10));
            JCx = JDx-r5*cos(var(i,5)); JCy = JDy-r5*sin(var(i,5));
            JFx = JDx+r6*cos(var(i,6)); JFy = JDy+r6*sin(var(i,6));
            JHx = JAx+r11*cos(var(i,11)); JHy = JAy+r11*sin(var(i,11));
            JGx = JHx+r7*cos(var(i,7)); JGy = JHy+r7*sin(var(i,7));
            JPx = JHx+0.5*r7*cos(var(i,7)); JPy = JHy+0.5*r7*sin(var(i,7));
            x1 = [JOx JAx JBx]; y1 = [JOy JAy JBy];
            x2 = [JCx JDx JEx]; y2 = [JCy JDy JEy];
            x3 = [JAx JHx JGx JFx JEx]; y3 = [JAy JHy JGy JFy JEy]; 
            x4 = JPx; y4 = JPy;
            X(i) = JPx; Y(i) = JPy;
            h = plot(x1, y1,'b', x2, y2,'g', x3, y3,'g', x4, y4,'ro', 'erasemode', 'none');
            axis equal
            axis([-30,170,-40,70]);
            drawnow
        end
    end
    
    % 判斷是否要再看動畫
    run = input('是否要是否要再跑一次動畫 y/n ? [y]:','s');
    if isempty(run); run = 'y'; end
    
    if run ~= 'y'
        break
    end
    
    %判斷加減速
    run2 = input('是否要改變速度 +/-/= [-]:','s');
    if isempty(run2); run2 = '-'; end
    
    if run2 == '-'
        nn = nn/5;
    elseif run2 == '+'
        nn = nn*5;
    elseif run2 == '='
        continue
    else
        break
    end %endif
    
end %end while

% picture of point p

figure
plot(X,Y)
axis equal
xlabel('X-axis'); ylabel('Y-axis');

%找出X,Y最大最小值
theta = var(:,2:12).*r2d;
E = zeros(1,10); %紀錄 (Xmax, Xmin, Ymax, Ymin, Xd, Yd, X/Y, Rmax, Rmin, Rran )
Xmax = max(X) , Xmin = min(X), Ymax = max(Y), Ymin = min(Y), Xd = Xmax-Xmin, Yd = Ymax-Ymin, XYratio = Xd/Yd, Rmax = max(theta(:,4)), Rmin = min(theta(:,4)), Rran = Rmax-Rmin

E(1,:) = [Xmax, Xmin, Ymax, Ymin, Xd, Yd, XYratio, Rmax, Rmin, Rran]
