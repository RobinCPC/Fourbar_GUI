%期末報告 靈敏度分析用  by rb     06 6/11
%6/13 修正力分析部份 V1.1
%6/15 修正 擺角值  V1.2
%先給初始值
clear all
r1 = input('請輸入 r1 長度 [73]:');
if isempty(r1); r1 = 73; end
r2 = input('請輸入 r2 長度 [16]:');
if isempty(r2); r2 = 16; end
r3 = input('請輸入 r3 長度 [60]:');
if isempty(r3); r3 = 60; end
r4 = input('請輸入 r4 長度 [58]:');
if isempty(r4); r4 = 58; end
theta1 = input('請輸入 theta1 角度 [35]:');
if isempty(theta1); theta1 = 35 ; end
beta = input('請輸入 beta 角度 [0]:');
if isempty(beta); beta = 0; end
r6 = input('請輸入 r6 長度 [18]');
if isempty(r6); r6= 18; end
mr = input('請輸入 momentun 值(N*m) [3]');
if isempty(mr); mr= 3; end

%set the direction of rotate
nn=1;
a1 = 180:-nn:0;
a2 = -1:-nn:-179;
theta2=[a1 a2];
n = numel(theta2);

% calculate loop equation

rm = r2; rj = r3;
thetam = theta2;
A = 2*r1*r4*cos(theta1*(pi/180))-2*rm*r4*cos(thetam.*(pi/180));
B = 2*r1*r4*sin(theta1*(pi/180))-2*rm*r4*sin(thetam.*(pi/180));
C = (r1^2)+(rm^2)+(r4^2)-(rj^2)-2*r1*rm*(cos(theta1*(pi/180))*cos(thetam.*(pi/180))+sin(theta1*(pi/180))*sin(thetam.*(pi/180)));

det = sqrt((B.^2)+(A.^2)-(C.^2));

if det > 0
    fprintf('有兩相異實根, 請自行判斷哪個解是你要的^^||. \n');
    theta41 = 2* atan2((-B+det),(C-A))*(180/pi);
    for i = 1:n
        if (theta41(i) > 0)
            theta41(i) = theta41(i)-360;
        end
    end
    theta42 = 2* atan2((-B-det),(C-A))*(180/pi);
     for i = 1:n
        if (theta42(i) > 0)
            theta42(i) = theta42(i)-360;
        end
    end
    thetaj1 = atan2((r1*sin(theta1*(pi/180))+r4*sin(theta41*(pi/180))-rm*sin(thetam.*(pi/180))),...
        (r1*cos(theta1*(pi/180))+r4*cos(theta41*(pi/180))-rm*cos(thetam.*(pi/180))));
    
    thetaj2 = atan2((r1*sin(theta1*(pi/180))+r4*sin(theta42*(pi/180))-rm*sin(thetam.*(pi/180))),...
        (r1*cos(theta1*(pi/180))+r4*cos(theta42*(pi/180))-rm*cos(thetam.*(pi/180))));
    
    theta31 = thetaj1*(180/pi);
    
    theta32 = thetaj2*(180/pi);
    
elseif det == 0
    fprintf('重根\n');
    theta4 = 2* atan2((-B),(C-A))*(180/pi);
    
    thetaj = atan2((r1*sin(theta1*(pi/180))+r4*sin(theta4*(pi/180))-rm*sin(thetam.*(pi/180))),...
        (r1*cos(theta1*(pi/180))+r4*cos(theta4*(pi/180))-rm*cos(thetam.*(pi/180))));
    
    theta3 = thetaj*(180/pi);
    
else 
    fprintf('無實數解,請確認你的值有沒有帶錯,不然就是你的四連桿斷拉XD\n');
    break
end

%先找出P點的軌跡
X = zeros(1,n); Y = zeros(1,n); %for 儲存P點的位移
M = zeros(1,n); N = zeros(1,n); %for 儲存B點的位移
    for i = 1:n
    JBx = r1*cos(theta1*(pi/180))+r4*cos(theta41(i)*(pi/180)); JBy = r1*sin(theta1*(pi/180))+r4*sin(theta41(i)*(pi/180));
    theta5 =atan2((r1*sin(theta1*(pi/180))+r4*sin(theta41(i)*(pi/180))-r2*sin(theta2(i)*(pi/180))),(r1*cos(theta1*(pi/180))+r4*cos(theta41(i)*(pi/180))-r2*cos(theta2(i)*(pi/180))))*(180/pi);
    theta6 = beta + theta5;
    JPx = r2*cos(theta2(i)*(pi/180))+r6*cos(theta6*(pi/180)); JPy = r2*sin(theta2(i)*(pi/180))+r6*sin(theta6*(pi/180));
        M(i)=JBx; N(i)=JBy;
        X(i)=JPx; Y(i)=JPy;
    end
%加上four bar 的位置的運動圖  以及上一點的位移
    Flen =zeros(1,n);
    F = zeros(1,n);
    for i = 1:n
    
        JAx = r2*cos(theta2(i)*(pi/180)); JAy = r2*sin(theta2(i)*(pi/180));
        JBx = r1*cos(theta1*(pi/180))+r4*cos(theta41(i)*(pi/180)); JBy = r1*sin(theta1*(pi/180))+r4*sin(theta41(i)*(pi/180));
        JCx = r1*cos(theta1*(pi/180)); JCy = r1*sin(theta1*(pi/180));
        JOx = 0; JOy = 0;
        x1 = [JOx JAx JBx JCx]; y1 = [JOy JAy JBy JCy];

        theta5 =atan2((r1*sin(theta1*(pi/180))+r4*sin(theta41(i)*(pi/180))-r2*sin(theta2(i)*(pi/180))),(r1*cos(theta1*(pi/180))+r4*cos(theta41(i)*(pi/180))-r2*cos(theta2(i)*(pi/180))))*(180/pi);
        theta6 = beta + theta5;
        JPx = r2*cos(theta2(i)*(pi/180))+r6*cos(theta6*(pi/180)); JPy = r2*sin(theta2(i)*(pi/180))+r6*sin(theta6*(pi/180));

            x2 = JPx ; y2 = JPy;

        h = plot(x1, y1,x2,y2,'go', X, Y,'r:', M, N,'m:','erasemode', 'none');

        %set(h,'linewidth',3)

%          hold 
%          g = plot(x2,y2, 'go', 'erasemode', 'normal');
        % 
        axis([-20,80,-20,60]);
        % 
        % title('four-bar linkage')
        % legend('linkage','locus of P')
        % hold  
        drawnow

        %求力臂
        dOP = sqrt((abs(JOx-JPx)^2)+(abs(JOy-JPy)^2));
        alpha = acos(((r2^2)+(dOP^2)-(r6^2))/(2*r2*dOP))*(180/pi);
       % if alpha < 90*(pi/180)
            gamme = (abs(theta2(i))+theta31(i)-alpha)*(pi/180);
        %else
            %gamme = abs(theta2(i)*(pi/180))-abs(theta31(i)*(pi/180))-alpha;
       % end

        Flen(i) = dOP*cos(gamme);
        F(i) = mr/Flen(i);
    end
% plot 力臂比crank角度

figure
plot(theta2,Flen)
xlabel('crank角度'); ylabel('力臂');  

% plot  施力比crank角度
figure
plot(theta2 , F)
xlabel('crank角度'); ylabel('最小施力'); 
% picture of point p

figure
plot(X,Y)
axis equal
xlabel('X-axis'); ylabel('Y-axis');

%找出X,Y最大最小值
E = zeros(1,13); %紀錄 (Xmax, Xmin, Ymax, Ymin, Xd, Yd, X/Y, Rmax, Rmin, Rran, th3max, th3min, th3ran)
Xmax = max(X) ; Xmin = min(X); Ymax = max(Y); Ymin = min(Y); Xd = Xmax-Xmin; Yd = Ymax-Ymin; XYratio = Xd/Yd; Rmax = max(theta41); Rmin = min(theta41); Rran = Rmax-Rmin;
th3max = max(theta31); th3min = min(theta31); th3ran = th3max - th3min;

E(1,:) = [Xmax, Xmin, Ymax, Ymin, Xd, Yd, XYratio, Rmax, Rmin, Rran, th3max, th3min, th3ran];



