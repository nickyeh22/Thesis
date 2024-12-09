%Example on the use of AStar Algorithm in an occupancy grid. 
clear all;
close all;
clc;
tic
%%% Generating a MAP
%1 represent an object that the path cannot penetrate, zero is a free path
% MAP=int8(zeros(128,140));
% MAP(1:64,1)=1;
% MAP(120,3:100)=1;
% MAP(125:128,40:60)=1;
% MAP(120:128,100:120)=1;
% MAP(126,100:118)=0;
% MAP(120:126,118)=0;
% MAP(100:120,100)=1;
% MAP(114:124,112:118)=0;
% MAP(1,1:128)=1;
% MAP(128,1:128)=1;
% MAP(100,1:130)=1;
% MAP(50,28:128)=1;
% MAP(20:30,50)=1;
% MAP(1:128,1)=1;
% MAP(1:65,128)=1;
% MAP(1,1:128)=1;
% MAP(128,1:128)=1;
% MAP(10,1:50)=1;
% MAP(25,1:50)=1;
% MAP(40,40:50)=1;
% MAP(40,40:45)=1;
% MAP(80,20:40)=1;
% MAP(80:100,40)=1;
% MAP(80:100,120)=1;
% MAP(120:122,120:122)=1;
% MAP(120:122,20:25)=1;
% MAP(120:122,10:11)=1;
% MAP(125:128,10:11)=1;
% MAP(100:110,30:40)=1;
% MAP(1:20,100:128)=1;
% MAP(10:20,80:128)=1;
% MAP(20:40,80:90)=1;
% MAP(1:40,90:90)=1;
% MAP(100:105,70:80)=1;
obs_num = 3;
%case 1
obs_pos(:,:,1) = [113; 92];
obs_pos(:,:,2) =  [106; 141];
obs_pos(:,:,3) =  [60; 112];
%case 2
obs_pos = [112; 90];
obs_pos(:,:,2) =  [74; 62];
obs_pos(:,:,3) =  [55; 112];    %可通過狹縫
% obs_pos(:,:,3) =  [59; 109];    %無法通過狹縫(繞開


% obs_pos(:,:,3) =  [152; 22];
% obs_pos(:,:,4) =  [50; 95];
% obs_pos(:,:,5) =  [10; 30];
% obs_sz = [22 15 22 20 33]; 
%case 1
obs_sz = [17 15 22]; 
%case 2
obs_sz = [22 15 17]; 

%vehicle coefficeint setting
v = 10;             %載具速度
psi = 20;           %載具最大轉角(degree)
psi_rate = 10;      %載具rate of sheering angle(degree/sec)
L_f = 5; L_r = 5;   %載具前後輪軸距
L = L_f+L_r;
L_c = L_f*2;
r_v = 5*sqrt(2);
rw=4.5/2;  %輪子半徑


%pre-calculation
R = cotd(abs(psi))*sqrt((L_r*tand(abs(psi))).^2 + L.^2);
beta = atan(L_r/L*tand(psi));
psi_rate = psi_rate*pi/180;

% MAP = zeros(200);
% for i = 1:1:200
%     for j = 1:1:200
%         MAP(j,i) = 0;
%         for k = 1:1:obs_num
%             if sqrt((i-obs_pos(1,1,k)).^2 + (j-obs_pos(2,1,k)).^2) < obs_sz(k) + 9
%                 MAP(j,i) = 1;
%             end
%         end
%     end
% end
for i = 1:1:200
    for j = 1:1:200
        MAP(j,i) = 0;
        for k = 1:obs_num
            if sqrt((i-obs_pos(1,1,k)).^2 + (j-obs_pos(2,1,k)).^2) < obs_sz(k) + r_v + 1.6
                MAP(j,i) = 1;
            end
        end
    end
end
     
%Start Positions
% StartX=100;
% StartY=1;
%case 1
StartX=20;
StartY=80;
%case 2
StartX=35;
StartY=35;

%Generating goal nodes, which is represented by a matrix. Several goals can be speciefied, in which case the pathfinder will find the closest goal. 
%a cell with the value 1 represent a goal cell
GoalRegister=int8(zeros(200,200));

GoalRegister(120,140)=1;


%Number of Neighboors one wants to investigate from each cell. A larger
%number of nodes means that the path can be alligned in more directions. 
%Connecting_Distance=1-> Path can  be alligned along 8 different direction.
%Connecting_Distance=2-> Path can be alligned along 16 different direction.
%Connecting_Distance=3-> Path can be alligned along 32 different direction.
%Connecting_Distance=4-> Path can be alligned along 56 different direction.
%ETC......

Connecting_Distance=8; %Avoid to high values Connecting_Distances for reasonable runtimes. 

turning_coeff = [r_v psi_rate v L L_r R beta];
% Running PathFinder
OptimalPath=ASTARPATH(StartX,StartY,MAP,GoalRegister,Connecting_Distance, obs_pos, obs_sz, turning_coeff);
% ORG_path = ASTARPATH_ORG(StartX,StartY,MAP,GoalRegister,Connecting_Distance, obs_pos, obs_sz, r_v);
% End. 

%%
%b-spline路徑平滑 (modified_path
X=OptimalPath(:,2);
X=X';
X=[X(1),X(1),X,X(end),X(end)];
Y=OptimalPath(:,1);
Y=Y';
Y=[Y(1),Y(1),Y,Y(end),Y(end)];
B = [
        [1,4,1,0];
        [-3,0,3,0];
        [3,-6,3,0];
        [-1,3,-3,1];
    ];
A = (1/6)*B;
C_1 = [];
N = size(X,2);
for i=2:N-2
    for t=0:0.3:1
        D = [1,t,t^2,t^3]*A*[[X(i-1),Y(i-1)];[X(i),Y(i)];[X(i+1),Y(i+1)];[X(i+2),Y(i+2)]];
        C_1 = [C_1;D];
        
        
    end
end

% %b-spline路徑平滑 (ORG_path
% X=ORG_path(:,2);
% X=X';
% X=[X(1),X(1),X,X(end),X(end)];
% Y=ORG_path(:,1);
% Y=Y';
% Y=[Y(1),Y(1),Y,Y(end),Y(end)];
% B = [
%         [1,4,1,0];
%         [-3,0,3,0];
%         [3,-6,3,0];
%         [-1,3,-3,1];
%     ];
% A = (1/6)*B;
% C_2 = [];
% N = size(X,2);
% for i=2:N-2
%     for t=0:0.3:1
%         D = [1,t,t^2,t^3]*A*[[X(i-1),Y(i-1)];[X(i),Y(i)];[X(i+1),Y(i+1)];[X(i+2),Y(i+2)]];
%         C_2 = [C_2;D];
%         
%         
%     end
% end


%%
%pre-plot

% for i = 1:1:200
%     for j = 1:1:200
%         MAP(j,i) = 0;
%         for k = 1:1:obs_num
%             if sqrt((i-obs_pos(1,1,k)).^2 + (j-obs_pos(2,1,k)).^2) < obs_sz(k) + r_v
%                 MAP(j,i) = 1;
%             end
%         end
%     end
% end
% if size(OptimalPath,2)>1
figure(1)
for i = 1:obs_num
    m = circle(obs_pos(1,i), obs_pos(2,i), obs_sz(i), 'k');
end
% imagesc((MAP))
%     colormap(flipud(gray));
%     set(gca,'YDir','normal')

hold on
% plot(OptimalPath(1,2),OptimalPath(1,1),'o','color','k')
scatter(C_1(1,1),C_1(1,2),45,"red", "filled")
% plot(OptimalPath(end,2),OptimalPath(end,1),'o','color','b')
scatter(C_1(end,1),C_1(end,2),45,"blue", "filled")

% plot(C_2(:,1),C_2(:,2),'b')     %ORG_path
plot(C_1(:,1),C_1(:,2),'r')     %modified_path
axis([0 200 0 200],'square'); 
legend('Modified A* Path', 'Tracking Trajectory');
% legend('off')

% h1 = circle(StartX, StartY, r_v)
% h2 = circle(90, 83, r_v)
% else 
%      pause(1);
%  h=msgbox('Sorry, No path exists to the Target!','warn');
%  uiwait(h,5);
%  end
toc

% h = circle(C_1(end,1), C_1(end,2), r_v, "#A2142F");
%%
%fuzzy
%載具控制
NB=-2;NS=-1;ZO=0;PS=1;PM=2;PB=3;PVB=4;  %建立隸屬函數
%訂定模糊區間
drmf_paras=[3 3 6 , 3 6 9 , 6 9 12 , 9 12 15 , 12 15 15];
%drmf_paras=[5 5 10 , 5 10 15 , 10 15 20 , 15 20 25 , 20 25 25];
armf_paras=[-pi/2 -pi/2 -pi/4 , -pi/2 -pi/4 0 , -pi/4 0 pi/4 , 0 pi/4 pi/2 , pi/4 pi/2 pi/2];
% %錄影----------------------------------
% writerObj=VideoWriter('fuzzy2.avi'); 
% open(writerObj); 
%%
%程式主迴圈
h = [];
E_1=[]
for i = length(C_1):-1:1
    E_1=[E_1;C_1(i,:)];
end
actual=[StartX,StartY]; %起始點  
preStartPose=actual + E_1(1,:) - E_1(2,:); %用來決定起始heading
theta = atan2((actual(2) - preStartPose(2)), (actual(1) - preStartPose(1)));    %車子與x軸的夾角
if (actual(2) - preStartPose(2)) < 0   %第三四象限角
    theta = 2*pi + theta;
elseif (actual(1) - preStartPose(1)) == 0 && (actual(2) - preStartPose(2)) > 0
    theta = pi/2;
elseif (actual(1) - preStartPose(1)) == 0 && (actual(2) - preStartPose(2)) < 0
    theta = 3*pi/4;
end

myepoch=2;

for epoch=1:10000
%      target=E(ceil(epoch/4),:)  %計算每個時刻的目標點
    if myepoch > size(E_1,1)
        break
    end
    for check=1:100
%         disp(E(ceil(myepoch)-check,:))
%         disp(E(ceil(myepoch),:))
        if E_1(ceil(myepoch)-check,1) ~=  E_1(ceil(myepoch),1) || E_1(ceil(myepoch)-check,2) ~=  E_1(ceil(myepoch),2)
             old_target=E_1(ceil(myepoch)-check,:);
             break
        end          
    end    
    target=E_1(ceil(myepoch),:);  %計算每個時刻的目標點
    heading_ang=[target(1)-old_target(1),target(2)-old_target(2)];
    dr=sqrt((target(1)-actual(1))^2+(target(2)-actual(2))^2); %計算距離誤差
    ar=angletest(preStartPose,actual,target);   %計算角度誤差
    heading_ar=angletest(preStartPose,actual,actual+target-old_target);
    ar=ar+(1/(0.2*dr))*heading_ar;
% disp(target)
    preStartPose=actual;
    count=1;m=1;n=1;
    inputDis_list = zeros(2, 2);
    inputAng_list = zeros(2, 2);
    while count<15 
        %[disweigh:距離隸屬度 dismf:距離隸屬函數]
        %[angweigh:角度隸屬度 angmf:角度隸屬函數]
        if count==1
            [disweigh,dismf]=disTrapL(dr,drmf_paras(count+1),drmf_paras(count+2),NB ,NS ,ZO ,PS ,PM ,PB ,PVB );
        elseif count==13
            [disweigh,dismf]=disTrapR(dr,drmf_paras(count),drmf_paras(count+1),NB ,NS ,ZO ,PS ,PM ,PB ,PVB );
        else
            [disweigh,dismf]=disTri(dr,drmf_paras(count),drmf_paras(count+1),drmf_paras(count+2),NB ,NS ,ZO ,PS ,PM ,PB ,PVB );
        end
        if count==1
            [angweigh,angmf]=angTrapL(ar,armf_paras(count+1),armf_paras(count+2),NB ,NS ,ZO ,PS ,PM ,PB ,PVB );
        elseif count==13
            [angweigh,angmf]=angTrapR(ar,armf_paras(count),armf_paras(count+1),NB ,NS ,ZO ,PS ,PM ,PB ,PVB );
        else
            [angweigh,angmf]=angTri(ar,armf_paras(count),armf_paras(count+1),armf_paras(count+2),NB ,NS ,ZO ,PS ,PM ,PB ,PVB );
        end
            
        if disweigh ~= 0
            inputDis_list(m,1:2) = [disweigh,dismf];
            m=m+1;
        end
        if angweigh ~= 0
            inputAng_list(n,1:2) =  [angweigh,angmf];
            n=n+1;
        end
        count=count+3;
    end
    [dr ar];
    inputDis_list;
    inputAng_list;
    %------------------------------------------------------------------
    %跑逐條規則
    accR=0;accL=0;accEpoch=0;
    for i=1:2
        for j=1:2
            yweigh = min(inputDis_list(i,1),inputAng_list(j,1));
            [Rmf,Lmf,time]=rule(inputDis_list(i,2),inputAng_list(j,2)  ,NB ,NS ,ZO ,PS ,PM ,PB ,PVB);
            [accL,accR,accEpoch]=defuzzy(accR,accL,accEpoch,yweigh,Rmf,Lmf,time);   %聚集輸出並解模糊化
        end
    end
    round(accEpoch)
    myepoch=myepoch+round(accEpoch);
    [dr ar];
    [accL,accR,round(accEpoch)];
    %neural fuzzy------------------------------
    if dr>15
        dr=15;
    end
    if ar>pi/2
        ar=pi/2;
    elseif ar<-pi/2
        ar=-pi/2;
    end
%     xa=[dr dr dr dr dr]; xb=[ar ar ar ar ar];
%     ya2=exp(-(((xa-m_dr_old)./std_dr_old).^2));  % 注意 點 指令
%     yb2=exp(-(((xb-m_ar_old)./std_ar_old).^2));
%     for kk=1:rule_n
%        Ro(kk)=ya2(r_f_connect(kk,1))*yb2(r_f_connect(kk,2));
%     end
%     Ro_sum=sum(Ro);
%     % O/P Layer
%     accR=(w_R_old*Ro')/Ro_sum   
%     accL=(w_L_old*Ro')/Ro_sum  
    %------------------------------------------
    pause(0.01);
    %將輸出的PWM回推下一時刻的位置
    %ackermann model
    xdot=rw/2*cos(theta)*(accL+accR) ;
    ydot=rw/2*sin(theta)*(accL+accR);
    thetadot=rw/L_c*(accL-accR);
    v = sqrt(xdot.^2 + ydot.^2);
    psi = atan2((L*thetadot), v);

    if abs(psi) > 20*pi/180
        if psi > 0
            psi = 20*pi/180;
        else
            psi = -20*pi/180;
        end
    end
    beta = atan2((L_r*tan(psi)), L);
    theta=theta+thetadot*0.2;
    x_c_dot = v*cos(theta+beta);
    y_c_dot = v*sin(theta+beta);
    
%     xdot=rw/2*cos(theta)*(accL+accR) ;
%     ydot=rw/2*sin(theta)*(accL+accR);
    x=0.2*x_c_dot;
    y=0.2*y_c_dot;
%     theta=theta+thetadot*0.2;
    actual(1)=actual(1)+x;
    actual(2)=actual(2)+y;
    pause(0.01);
    feasible = collisionChecking(actual, r_v, obs_num, obs_pos, obs_sz);
    plot(actual(1), actual(2), 'go', 'MarkerSize',1, 'MarkerFaceColor','g', 'HandleVisibility','off');
    %plot(target(1), target(2), 'ro', 'MarkerSize',6, 'MarkerFaceColor','r');
%     hold on;
    %錄影
%     frame = getframe(figure(1));            
%     writeVideo(writerObj,frame); 
%actual=.....................

    delete(h);
    h = circle(actual(1), actual(2), r_v, "#A2142F");
    if feasible == false
        break;
    end
    drawnow;
%     hold off
end

% pause(5);

%%
%程式主迴圈 (ORG_path
% h=[];
% E_2=[]
% for i = length(C_2):-1:1
%     E_2=[E_2;C_2(i,:)];
% end
% actual=[StartX,StartY]; %起始點  
% preStartPose=actual + E_2(1,:) - E_2(2,:); %用來決定起始heading
% theta = atan2((actual(2) - preStartPose(2)), (actual(1) - preStartPose(1)));    %車子與x軸的夾角
% if (actual(2) - preStartPose(2)) < 0   %第三四象限角
%     theta = 2*pi + theta;
% elseif (actual(1) - preStartPose(1)) == 0 && (actual(2) - preStartPose(2)) > 0
%     theta = pi/2;
% elseif (actual(1) - preStartPose(1)) == 0 && (actual(2) - preStartPose(2)) < 0
%     theta = 3*pi/4;
% end
% 
% myepoch=2;
% 
% while 1
% %      target=E(ceil(epoch/4),:)  %計算每個時刻的目標點
%     if myepoch > size(E_2,1)
%         break
%     end
%     for check=1:100
% %         disp(E(ceil(myepoch)-check,:))
% %         disp(E(ceil(myepoch),:))
%         if E_2(ceil(myepoch)-check,1) ~=  E_2(ceil(myepoch),1) || E_2(ceil(myepoch)-check,2) ~=  E_2(ceil(myepoch),2)
%              old_target=E_2(ceil(myepoch)-check,:);
%              break
%         end          
%     end    
%     target=E_2(ceil(myepoch),:);  %計算每個時刻的目標點
%     heading_ang=[target(1)-old_target(1),target(2)-old_target(2)];
%     dr=sqrt((target(1)-actual(1))^2+(target(2)-actual(2))^2); %計算距離誤差
%     ar=angletest(preStartPose,actual,target);   %計算角度誤差
%     heading_ar=angletest(preStartPose,actual,actual+target-old_target);
%     ar=ar+(1/(0.2*dr))*heading_ar;
% % disp(target)
%     preStartPose=actual;
%     count=1;m=1;n=1;
%     inputDis_list = zeros(2, 2);
%     inputAng_list = zeros(2, 2);
%     while count<15 
%         %[disweigh:距離隸屬度 dismf:距離隸屬函數]
%         %[angweigh:角度隸屬度 angmf:角度隸屬函數]
%         if count==1
%             [disweigh,dismf]=disTrapL(dr,drmf_paras(count+1),drmf_paras(count+2),NB ,NS ,ZO ,PS ,PM ,PB ,PVB );
%         elseif count==13
%             [disweigh,dismf]=disTrapR(dr,drmf_paras(count),drmf_paras(count+1),NB ,NS ,ZO ,PS ,PM ,PB ,PVB );
%         else
%             [disweigh,dismf]=disTri(dr,drmf_paras(count),drmf_paras(count+1),drmf_paras(count+2),NB ,NS ,ZO ,PS ,PM ,PB ,PVB );
%         end
%         if count==1
%             [angweigh,angmf]=angTrapL(ar,armf_paras(count+1),armf_paras(count+2),NB ,NS ,ZO ,PS ,PM ,PB ,PVB );
%         elseif count==13
%             [angweigh,angmf]=angTrapR(ar,armf_paras(count),armf_paras(count+1),NB ,NS ,ZO ,PS ,PM ,PB ,PVB );
%         else
%             [angweigh,angmf]=angTri(ar,armf_paras(count),armf_paras(count+1),armf_paras(count+2),NB ,NS ,ZO ,PS ,PM ,PB ,PVB );
%         end
%             
%         if disweigh ~= 0
%             inputDis_list(m,1:2) = [disweigh,dismf];
%             m=m+1;
%         end
%         if angweigh ~= 0
%             inputAng_list(n,1:2) =  [angweigh,angmf];
%             n=n+1;
%         end
%         count=count+3;
%     end
%     [dr ar];
%     inputDis_list;
%     inputAng_list;
%     %------------------------------------------------------------------
%     %跑逐條規則
%     accR=0;accL=0;accEpoch=0;
%     for i=1:2
%         for j=1:2
%             yweigh = min(inputDis_list(i,1),inputAng_list(j,1));
%             [Rmf,Lmf,time]=rule(inputDis_list(i,2),inputAng_list(j,2)  ,NB ,NS ,ZO ,PS ,PM ,PB ,PVB);
%             [accL,accR,accEpoch]=defuzzy(accR,accL,accEpoch,yweigh,Rmf,Lmf,time);   %聚集輸出並解模糊化
%         end
%     end
%     round(accEpoch)
%     myepoch=myepoch+round(accEpoch);
%     [dr ar];
%     [accL,accR,round(accEpoch)];
%     %neural fuzzy------------------------------
%     if dr>15
%         dr=15;
%     end
%     if ar>pi/2
%         ar=pi/2;
%     elseif ar<-pi/2
%         ar=-pi/2;
%     end
% %     xa=[dr dr dr dr dr]; xb=[ar ar ar ar ar];
% %     ya2=exp(-(((xa-m_dr_old)./std_dr_old).^2));  % 注意 點 指令
% %     yb2=exp(-(((xb-m_ar_old)./std_ar_old).^2));
% %     for kk=1:rule_n
% %        Ro(kk)=ya2(r_f_connect(kk,1))*yb2(r_f_connect(kk,2));
% %     end
% %     Ro_sum=sum(Ro);
% %     % O/P Layer
% %     accR=(w_R_old*Ro')/Ro_sum   
% %     accL=(w_L_old*Ro')/Ro_sum  
%     %------------------------------------------
%     pause(0.01);
%     %將輸出的PWM回推下一時刻的位置
% 
%     %ackermann model
%     xdot=rw/2*cos(theta)*(accL+accR) ;
%     ydot=rw/2*sin(theta)*(accL+accR);
%     thetadot=rw/L_c*(accL-accR);
% 
%     v = sqrt(xdot.^2 + ydot.^2);
%     psi = atan2((L*thetadot), v);
%     beta = atan2((L_r*tan(psi)), L);
%     theta=theta+thetadot*0.2;
%     x_c_dot = v*cos(theta+beta);
%     y_c_dot = v*sin(theta+beta);
%     
% %     xdot=rw/2*cos(theta)*(accL+accR) ;
% %     ydot=rw/2*sin(theta)*(accL+accR);
%     x=0.2*x_c_dot;
%     y=0.2*y_c_dot;
% %     theta=theta+thetadot*0.2;
%     actual(1)=actual(1)+x;
%     actual(2)=actual(2)+y;
%     pause(0.01);
%     feasible = collisionChecking(actual, r_v, obs_num, obs_pos, obs_sz);
%     plot(actual(1), actual(2), 'go', 'MarkerSize',1, 'MarkerFaceColor','g', 'HandleVisibility','off');
%     %plot(target(1), target(2), 'ro', 'MarkerSize',6, 'MarkerFaceColor','r');
%     %錄影
% %     frame = getframe(figure(1));            
% %     writeVideo(writerObj,frame); 
% %actual=.....................
% 
%     delete(h);
% 
%     h = circle(actual(1), actual(2), r_v, "#A2142F");
%     if feasible == false
%         break;
%     end
%     drawnow;
% end
%%
function y = func(t, obs_pos, vhc_hd, c1, c2, del_x, del_y, beta, R, r_v, r_o, vhc_hd_new, turning)
    if abs(tan(vhc_hd)) < tan(pi/4)
        x_pos = t + del_x;
        y_pos = (tan(vhc_hd)*t+c1) + del_y;
    else
        x_pos = (cot(vhc_hd)*t+c2) + del_x;
        y_pos = t + del_y;
    end

    phi = atan2((obs_pos(2) - y_pos), (obs_pos(1) - x_pos));
    if (obs_pos(2) - y_pos) < 0   %第三四象限角
        phi = 2*pi + phi;
    elseif (obs_pos(1) - x_pos) == 0 && (obs_pos(2) - y_pos) > 0
        phi = pi/2;
    elseif (obs_pos(1) - x_pos) == 0 && (obs_pos(2) - y_pos) < 0
        phi = 3*pi/4;
    end

    if turning == true      %右轉
        alpha = pi/2 + beta + (phi-vhc_hd_new); 
    else                    %左轉
        alpha = pi/2 + beta - (phi-vhc_hd_new);     
    end

    y = sqrt((obs_pos(2) - y_pos)^2 + (obs_pos(1) - x_pos)^2)... 
        - (sqrt((R*cos(alpha)).^2 + 2*R*(r_v+r_o) + (r_v+r_o).^2) + R*cos(alpha));
end

function ext_r = obs_ext_radi(vhc_pos, vhc_hd, obs_idx, obs_pos, obs_sz, turning_coeff)
    obs_pos = obs_pos(:,:,obs_idx);
    r_o = obs_sz(obs_idx);
    r_v = turning_coeff(1);
    psi_rate = turning_coeff(2);
    v = turning_coeff(3);
    L = turning_coeff(4);
    L_r = turning_coeff(5);
    R = turning_coeff(6);
    beta= turning_coeff(7);

    %計算開始轉彎位置
    vhc_hd_r = -(v/psi_rate/L)*log(abs(sec(-psi_rate*2))) + vhc_hd;      %載具右轉達最大steering angle時的heading
    vhc_hd_l = (v/psi_rate/L)*log(abs(sec(psi_rate*2))) + vhc_hd;       %載具左轉達最大steering angle時的heading

    del_x_r = 0;
    del_y_r = 0;
    del_x_l = 0;
    del_y_l = 0;
    
    for t=0:0.001:2
        x_diff_r = v*cos(atan(L_r/L*tan(-psi_rate*t)) + v/(-psi_rate)/L*cos(L_r/L*tan(-psi_rate*t))*log(abs(sec(-psi_rate*t))) + vhc_hd);
        y_diff_r = v*sin(atan(L_r/L*tan(-psi_rate*t)) + v/(-psi_rate)/L*cos(L_r/L*tan(-psi_rate*t))*log(abs(sec(-psi_rate*t))) + vhc_hd);
        x_diff_l = v*cos(atan(L_r/L*tan(psi_rate*t)) + v/(psi_rate)/L*cos(L_r/L*tan(psi_rate*t))*log(abs(sec(psi_rate*t))) + vhc_hd);
        y_diff_l = v*sin(atan(L_r/L*tan(psi_rate*t)) + v/(psi_rate)/L*cos(L_r/L*tan(psi_rate*t))*log(abs(sec(psi_rate*t))) + vhc_hd);
    
        del_x_r = del_x_r + x_diff_r*0.001;
        del_y_r = del_y_r + y_diff_r*0.001;
        del_x_l = del_x_l + x_diff_l*0.001;
        del_y_l = del_y_l + y_diff_l*0.001;
    end
    
    c1 = vhc_pos(2) - tan(vhc_hd)*vhc_pos(1);
    c2 = vhc_pos(1) - cot(vhc_hd)*vhc_pos(2);

    phi = atan2((obs_pos(2) - vhc_pos(2)), (obs_pos(1) - vhc_pos(1)));
    if (obs_pos(2) - vhc_pos(2)) < 0   %第三四象限角
        phi = 2*pi + phi;
    elseif (obs_pos(1) - vhc_pos(1)) == 0 && (obs_pos(2) - vhc_pos(2)) > 0
        phi = pi/2;
    elseif (obs_pos(1) - vhc_pos(1)) == 0 && (obs_pos(2) - vhc_pos(2)) < 0
        phi = 3*pi/4;
    end

    if abs(vhc_hd) < abs(phi)
        % 檢查初始條件--右轉
        if abs(tan(vhc_hd)) < tan(pi/4)
            a = vhc_pos(1);
            if vhc_pos(1) < obs_pos(1)
                b = obs_pos(1)+r_o;
            else
                b = obs_pos(1)-r_o;
            end
        else
             a = vhc_pos(2);
            if vhc_pos(2) < obs_pos(2)
                b = obs_pos(2)+r_o;
            else
                b = obs_pos(2)-r_o;
            end
        end
        err_r = false;
        f_a = func(a, obs_pos, vhc_hd, c1, c2, del_x_r, del_y_r, beta, R, r_v, r_o, vhc_hd_r, true);
        f_b = func(b, obs_pos, vhc_hd, c1, c2, del_x_r, del_y_r, beta, R, r_v, r_o, vhc_hd_r, true);
        
        if sign(f_a) == sign(f_b)
            err_r = true;
        end
        
        if err_r ~= true
            % 初始化
            t_0_r = (a + b) / 2;
            
            % 開始迭代
            while abs(func(t_0_r, obs_pos, vhc_hd, c1, c2, del_x_r, del_y_r, beta, R, r_v, r_o, vhc_hd_r, true)) > 1e-6
                if sign(func(t_0_r, obs_pos, vhc_hd, c1, c2, del_x_r, del_y_r, beta, R, r_v, r_o, vhc_hd_r, true))... 
                   == sign(func(a, obs_pos, vhc_hd, c1, c2, del_x_r, del_y_r, beta, R, r_v, r_o, vhc_hd_r, true))    % 根在 [root, b] 區間    
                    a = t_0_r;
                else                                    % 根在 [a, root] 區間   
                    b = t_0_r;
                end
              
                t_0_r = (a + b) / 2;
            end
            % disp(x_0_r);

            if abs(tan(vhc_hd)) < tan(pi/4)
                tur_pos = [t_0_r tan(vhc_hd)*t_0_r+c1];
            else
                tur_pos = [cot(vhc_hd)*t_0_r+c2 t_0_r];
            end
            ext_r = sqrt((obs_pos(1)-tur_pos(1)).^2 + (obs_pos(2)-tur_pos(2)).^2);
        else
            % 檢查是擦邊還是轉不過去(把起始點沿heading方向往後拉56.209)
            if abs(tan(vhc_hd)) < tan(pi/4)
                a = vhc_pos(1) - 56.209*cos(vhc_hd);
                if vhc_pos(1) < obs_pos(1)
                    b = obs_pos(1)+r_o;
                else
                    b = obs_pos(1)-r_o;
                end
            else
                 a = vhc_pos(2) - 56.209*sin(vhc_hd);
                if vhc_pos(2) < obs_pos(2)
                    b = obs_pos(2)+r_o;
                else
                    b = obs_pos(2)-r_o;
                end
            end
            f_a = func(a, obs_pos, vhc_hd, c1, c2, del_x_r, del_y_r, beta, R, r_v, r_o, vhc_hd_r, true);
            f_b = func(b, obs_pos, vhc_hd, c1, c2, del_x_r, del_y_r, beta, R, r_v, r_o, vhc_hd_r, true);
            
            if sign(f_a) == sign(f_b)
                ext_r = -1;
            else
                ext_r = -2;
            end
        end
   
    else       
        %檢查初始條件--左轉
        if abs(tan(vhc_hd)) < tan(pi/4)
            a = vhc_pos(1);
            if vhc_pos(1) < obs_pos(1)
                b = obs_pos(1)+r_o;
            else
                b = obs_pos(1)-r_o;
            end
        else
             a = vhc_pos(2);
            if vhc_pos(2) < obs_pos(2)
                b = obs_pos(2)+r_o;
            else
                b = obs_pos(2)-r_o;
            end
        end
        err_l = false;
        f_a = func(a, obs_pos, vhc_hd, c1, c2, del_x_l, del_y_l, beta, R, r_v, r_o, vhc_hd_l, false);
        f_b = func(b, obs_pos, vhc_hd, c1, c2, del_x_l, del_y_l, beta, R, r_v, r_o, vhc_hd_l, false);
        
        if sign(f_a) == sign(f_b)
            err_l = true;
        end
        
        if err_l ~= true
            % 初始化
            t_0_l = (a + b) / 2;
            
            % 開始迭代
            while abs(func(t_0_l, obs_pos, vhc_hd, c1, c2, del_x_l, del_y_l, beta, R, r_v, r_o, vhc_hd_l, false)) > 1e-6
                if sign(func(t_0_l, obs_pos, vhc_hd, c1, c2, del_x_l, del_y_l, beta, R, r_v, r_o, vhc_hd_l, false))... 
                   == sign(func(a, obs_pos, vhc_hd, c1, c2, del_x_l, del_y_l, beta, R, r_v, r_o, vhc_hd_l, false))    % 根在 [root, b] 區間    
                    a = t_0_l;
                else                                    % 根在 [a, root] 區間   
                    b = t_0_l;
                end
            
                t_0_l = (a + b) / 2;
            end
        
            if abs(tan(vhc_hd)) < tan(pi/4)
                tur_pos = [t_0_l tan(vhc_hd)*t_0_l+c1];
            else
                tur_pos = [cot(vhc_hd)*t_0_l+c2 t_0_l];
            end
            ext_r = sqrt((obs_pos(1)-tur_pos(1)).^2 + (obs_pos(2)-tur_pos(2)).^2);
        else
            %檢查是擦邊還是轉不過去(把起始點沿heading方向往後拉56.209)
            if abs(tan(vhc_hd)) < tan(pi/4)
                a = vhc_pos(1) - 56.209*cos(vhc_hd);
                if vhc_pos(1) < obs_pos(1)
                    b = obs_pos(1)+r_o;
                else
                    b = obs_pos(1)-r_o;
                end
            else
                 a = vhc_pos(2) - 56.209*sin(vhc_hd);
                if vhc_pos(2) < obs_pos(2)
                    b = obs_pos(2)+r_o;
                else
                    b = obs_pos(2)-r_o;
                end
            end
            
            f_a = func(a, obs_pos, vhc_hd, c1, c2, del_x_l, del_y_l, beta, R, r_v, r_o, vhc_hd_l, false);
            f_b = func(b, obs_pos, vhc_hd, c1, c2, del_x_l, del_y_l, beta, R, r_v, r_o, vhc_hd_l, false);
    
            if sign(f_a) == sign(f_b)
                ext_r = -1;
            else
                ext_r = -2;
            end
        end

    end

end

%%
function OptimalPath=ASTARPATH(StartX,StartY,MAP,GoalRegister,Connecting_Distance, obs_pos, obs_sz, turning_coeff)
%Version 1.0
% By Einar Ueland 2nd of May, 2016

%FINDING ASTAR PATH IN AN OCCUPANCY GRID


%nNeighboor=3;
% Preallocation of Matrices
[Height,Width]=size(MAP); %Height and width of matrix
GScore=zeros(Height,Width);           %Matrix keeping track of G-scores 
FScore=single(inf(Height,Width));     %Matrix keeping track of F-scores (only open list) 
Hn=single(zeros(Height,Width));       %Heuristic matrix
OpenMAT=int8(zeros(Height,Width));    %Matrix keeping of open grid cells
ClosedMAT=int8(zeros(Height,Width));  %Matrix keeping track of closed grid cells
ClosedMAT(MAP==1)=1;                  %Adding object-cells to closed matrix
ParentX=int16(zeros(Height,Width));   %Matrix keeping track of X position of parent
ParentY=int16(zeros(Height,Width));   %Matrix keeping track of Y position of parent
ct = 1;

%%% Setting up matrices representing neighboors to be investigated
NeighboorCheck=ones(2*Connecting_Distance+1);
Dummy=2*Connecting_Distance+2;
Mid=Connecting_Distance+1;
for i=1:Connecting_Distance-1
NeighboorCheck(i,i)=0;
NeighboorCheck(Dummy-i,i)=0;
NeighboorCheck(i,Dummy-i)=0;
NeighboorCheck(Dummy-i,Dummy-i)=0;
NeighboorCheck(Mid,i)=0;
NeighboorCheck(Mid,Dummy-i)=0;
NeighboorCheck(i,Mid)=0;
NeighboorCheck(Dummy-i,Mid)=0;
end
NeighboorCheck(Mid,Mid)=0;

[row, col]=find(NeighboorCheck==1);
Neighboors=[row col]-(Connecting_Distance+1);
N_Neighboors=size(col,1);
%%% End of setting up matrices representing neighboors to be investigated


%%%%%%%%% Creating Heuristic-matrix based on distance to nearest  goal node
[col, row]=find(GoalRegister==1);
RegisteredGoals=[row col];
Nodesfound=size(RegisteredGoals,1);


for k=1:size(GoalRegister,1)
    for j=1:size(GoalRegister,2)
        if MAP(k,j)==0
            Mat=RegisteredGoals-(repmat([j k],(Nodesfound),1));
            Distance=(min(sqrt(sum(abs(Mat).^2,2))));
            Hn(k,j)=Distance;
        end
    end
end
%End of creating Heuristic-matrix. 

%Note: If Hn values is set to zero the method will reduce to the Dijkstras method.

%Initializign start node with FValue and opening first node.
FScore(StartY,StartX)=Hn(StartY,StartX);         
OpenMAT(StartY,StartX)=1;   
ParentX(StartY,StartX) = 0;
ParentY(StartY,StartX) = 0;

% ct=1
while 1==1 %Code will break when path found or when no path exist
    MINopenFSCORE=min(min(FScore));
    if MINopenFSCORE==inf
    %Failuere!
    OptimalPath=[inf];
    RECONSTRUCTPATH=0;
    break
    end
    [CurrentY,CurrentX]=find(FScore==MINopenFSCORE);
    CurrentY=CurrentY(1);
    CurrentX=CurrentX(1);
    if GoalRegister(CurrentY,CurrentX)==1
    %GOAL!!
        RECONSTRUCTPATH=1;
        break
    end
    
  %Remobing node from OpenList to ClosedList  
    OpenMAT(CurrentY,CurrentX)=0;
    FScore(CurrentY,CurrentX)=inf;
    ClosedMAT(CurrentY,CurrentX)=1;

%     obs_idx = 1;

    vhc_hd = atan2((CurrentY - double(ParentY(CurrentY,CurrentX))), (CurrentX - double(ParentX(CurrentY,CurrentX))));
    if (CurrentY - ParentY(CurrentY,CurrentX)) < 0   %第三四象限角
        vhc_hd = 2*pi + vhc_hd;
    elseif (CurrentX - ParentX(CurrentY,CurrentX)) == 0 && (CurrentY - ParentY(CurrentY,CurrentX)) > 0
        vhc_hd = pi/2;
    elseif (CurrentX - ParentX(CurrentY,CurrentX)) == 0 && (CurrentY - ParentY(CurrentY,CurrentX)) < 0
        vhc_hd = 3*pi/4;
    end

    obs_num = size(obs_sz, 2);
    for obs_idx = 1:obs_num
        if sqrt((CurrentY - obs_pos(2,obs_idx))^2 + ((CurrentX - obs_pos(1,obs_idx))^2)) < obs_sz(obs_idx) + 2*turning_coeff(6) + turning_coeff(1)  %2R + r_v

            if CurrentX == StartX && CurrentY == StartY
                vhc_hd = atan2((CurrentY - obs_pos(2,obs_idx)), (CurrentX - obs_pos(1,obs_idx)));
                if (CurrentY - obs_pos(2,obs_idx)) < 0   %第三四象限角
                    vhc_hd = 2*pi + vhc_hd;
                elseif (CurrentX - obs_pos(1,obs_idx)) == 0 && (CurrentY - obs_pos(2,obs_idx)) > 0
                    vhc_hd = pi/2;
                elseif (CurrentX - obs_pos(1,obs_idx)) == 0 && (CurrentY - obs_pos(2,obs_idx)) < 0
                    vhc_hd = 3*pi/4;
                end
            end

            obs_adj_r(obs_idx) = obs_ext_radi([CurrentX CurrentY], vhc_hd, obs_idx, obs_pos, obs_sz, turning_coeff);
% disp([CurrentX CurrentY obs_idx obs_adj_r(obs_idx)]);
% pause(0.001)
            if obs_adj_r(obs_idx) == -1
                obs_adj_r(obs_idx) = obs_sz(obs_idx) + turning_coeff(1);
            elseif obs_adj_r(obs_idx) == -2
                dist = sqrt((CurrentY - obs_pos(2,obs_idx))^2 + ((CurrentX - obs_pos(1,obs_idx))^2));
                if dist > obs_sz(obs_idx) + turning_coeff(1)
                    obs_adj_r(obs_idx) = dist;
                else
                    obs_adj_r(obs_idx) = obs_sz(obs_idx) + turning_coeff(1);
                end
            end
        else
            obs_adj_r(obs_idx) = obs_sz(obs_idx) + turning_coeff(1);
        end
    end
% for i=1:3
%     disp([CurrentX CurrentY obs_adj_r(i)]);    
% end
% for p=1:N_Neighboors
%     disp([CurrentX CurrentY Neighboors(p,1) Neighboors(p,2)]);    
% end
% pause(1000)
    for p=1:N_Neighboors
        i=Neighboors(p,1); %Y
        j=Neighboors(p,2); %X
        if CurrentY+i<1||CurrentY+i>Height||CurrentX+j<1||CurrentX+j>Width
            continue
        end

%         if CurrentX ~= StartX && CurrentY ~= StartY
%             vhc_hd_new = atan2(i, j);
%             if i < 0   %第三四象限角
%                 vhc_hd_new = 2*pi + vhc_hd_new;
%             elseif j == 0 && i > 0
%                 vhc_hd_new = pi/2;
%             elseif j == 0 && i < 0
%                 vhc_hd_new = 3*pi/4;
%             end
%     
%             if abs(vhc_hd_new - vhc_hd) >     %轉角大於10度
%                 continue;
%             end
%         end
%         
        if(ClosedMAT(CurrentY+i,CurrentX+j)==0) %Neiboor is open; 
            % Need to check that the path does not pass an object
%                 JumpCells=2*max(abs(i),abs(j))-1;   
            Flag = 1;
%                 flag_2 = 1;
% disp([CurrentX CurrentY j i]);
% pause(0.001)
            for obs_idx=1:obs_num
                ax = CurrentX - obs_pos(1,obs_idx);
                ay = CurrentY - obs_pos(2,obs_idx);
                bx = CurrentX+j - obs_pos(1,obs_idx);
                by = CurrentY+i - obs_pos(2,obs_idx);
                a = (bx - ax)^2 + (by - ay)^2;
                b = 2*(ax*(bx - ax) + ay*(by - ay));
                c = ax^2 + ay^2 - obs_adj_r(obs_idx).^2;
                dit = b^2 - 4*a*c;
% disp([CurrentX CurrentY j i obs_adj_r(obs_idx)]);
% disp(dit);
% pause(0.001)
                if dit == 0      %交於一點
                    t1 = (-b + sqrt(dit))/(2*a);
                    if (0 < t1) && (t1 <= 1)
                        Flag = 0;
                        break;
                    end
                elseif dit > 0
                    t1 = (-b + sqrt(dit))/(2*a);
                    t2 = (-b - sqrt(dit))/(2*a);
% if CurrentX == 24 && CurrentY == 81 && j == 8 i == -1
% disp([CurrentX CurrentY j i t1 t2]);

% end
                    if (t1 > 0) && (t2 <= 1)    %(t1>t2)  %無穿越:t1<=0 || t2>1
                        Flag = 0;
                        break;
                    end
                end

%                     for K=1:JumpCells
%                         YPOS=round(K*i/JumpCells);
%                         XPOS=round(K*j/JumpCells);
%                 
%                         if sqrt((CurrentY+YPOS - obs_pos(2,obs_idx))^2 + ((CurrentX+XPOS - obs_pos(1,obs_idx))^2)) < obs_adj_r(obs_idx)
%                             flag_2 = flag_2*0;
%                         end
%                     end
% 
%                     if Flag ~= flag_2
%                         disp([CurrentX CurrentY]);
%                         disp([CurrentX+XPOS CurrentY+YPOS]);
%                         disp([Flag flag_2]);
%                         disp([obs_idx dist obs_adj_r(obs_idx)])
%                         error('不一樣');
%                     end
            end
            
             %End of  checking that the path does not pass an object

            if Flag==1     
                tentative_gScore = GScore(CurrentY,CurrentX) + sqrt(i^2+j^2);
                if OpenMAT(CurrentY+i,CurrentX+j)==0
                    OpenMAT(CurrentY+i,CurrentX+j)=1;                    
                elseif tentative_gScore >= GScore(CurrentY+i,CurrentX+j)
                    continue
                end
                ParentX(CurrentY+i,CurrentX+j)=CurrentX;
                ParentY(CurrentY+i,CurrentX+j)=CurrentY;
                GScore(CurrentY+i,CurrentX+j)=tentative_gScore;
                FScore(CurrentY+i,CurrentX+j)= tentative_gScore+Hn(CurrentY+i,CurrentX+j);
% disp([CurrentX CurrentY j i FScore(CurrentY+i,CurrentX+j)]);
% pause(0.001)
            end
        end
    end
% disp(ct);
% ct = ct+1;
end

k=2;
if RECONSTRUCTPATH
    OptimalPath(1,:)=[CurrentY CurrentX];
    while RECONSTRUCTPATH
        CurrentXDummy=ParentX(CurrentY,CurrentX);
        CurrentY=ParentY(CurrentY,CurrentX);
        CurrentX=CurrentXDummy;
        OptimalPath(k,:)=[CurrentY CurrentX];
        k=k+1;
        if (((CurrentX== StartX)) &&(CurrentY==StartY))
            break
        end
    end
end
% disp(ct)
end

function OptimalPath=ASTARPATH_ORG(StartX,StartY,MAP,GoalRegister,Connecting_Distance, obs_pos, obs_sz, r_v)
%Version 1.0
% By Einar Ueland 2nd of May, 2016

%FINDING ASTAR PATH IN AN OCCUPANCY GRID


%nNeighboor=3;
% Preallocation of Matrices
[Height,Width]=size(MAP); %Height and width of matrix
GScore=zeros(Height,Width);           %Matrix keeping track of G-scores 
FScore=single(inf(Height,Width));     %Matrix keeping track of F-scores (only open list) 
Hn=single(zeros(Height,Width));       %Heuristic matrix
OpenMAT=int8(zeros(Height,Width));    %Matrix keeping of open grid cells
ClosedMAT=int8(zeros(Height,Width));  %Matrix keeping track of closed grid cells
ClosedMAT(MAP==1)=1;                  %Adding object-cells to closed matrix
ParentX=int16(zeros(Height,Width));   %Matrix keeping track of X position of parent
ParentY=int16(zeros(Height,Width));   %Matrix keeping track of Y position of parent


%%% Setting up matrices representing neighboors to be investigated
NeighboorCheck=ones(2*Connecting_Distance+1);
Dummy=2*Connecting_Distance+2;
Mid=Connecting_Distance+1;
for i=1:Connecting_Distance-1
NeighboorCheck(i,i)=0;
NeighboorCheck(Dummy-i,i)=0;
NeighboorCheck(i,Dummy-i)=0;
NeighboorCheck(Dummy-i,Dummy-i)=0;
NeighboorCheck(Mid,i)=0;
NeighboorCheck(Mid,Dummy-i)=0;
NeighboorCheck(i,Mid)=0;
NeighboorCheck(Dummy-i,Mid)=0;
end
NeighboorCheck(Mid,Mid)=0;

[row, col]=find(NeighboorCheck==1);
Neighboors=[row col]-(Connecting_Distance+1);
N_Neighboors=size(col,1);
%%% End of setting up matrices representing neighboors to be investigated


%%%%%%%%% Creating Heuristic-matrix based on distance to nearest  goal node
[col, row]=find(GoalRegister==1);
RegisteredGoals=[row col];
Nodesfound=size(RegisteredGoals,1);

for k=1:size(GoalRegister,1)
    for j=1:size(GoalRegister,2)
        if MAP(k,j)==0
            Mat=RegisteredGoals-(repmat([j k],(Nodesfound),1));
            Distance=(min(sqrt(sum(abs(Mat).^2,2))));
            Hn(k,j)=Distance;
        end
    end
end
%End of creating Heuristic-matrix. 

%Note: If Hn values is set to zero the method will reduce to the Dijkstras method.

%Initializign start node with FValue and opening first node.
FScore(StartY,StartX)=Hn(StartY,StartX);         
OpenMAT(StartY,StartX)=1;   




while 1==1 %Code will break when path found or when no path exist
    MINopenFSCORE=min(min(FScore));
    if MINopenFSCORE==inf;
    %Failuere!
    OptimalPath=[inf];
    RECONSTRUCTPATH=0;
     break
    end
    [CurrentY,CurrentX]=find(FScore==MINopenFSCORE);
    CurrentY=CurrentY(1);
    CurrentX=CurrentX(1);

    if GoalRegister(CurrentY,CurrentX)==1
    %GOAL!!
        RECONSTRUCTPATH=1;
        break
    end
    
  %Remobing node from OpenList to ClosedList  
    OpenMAT(CurrentY,CurrentX)=0;
    FScore(CurrentY,CurrentX)=inf;
    ClosedMAT(CurrentY,CurrentX)=1;
    obs_num = size(obs_sz, 2);
   
    for p=1:N_Neighboors
        i=Neighboors(p,1); %Y
        j=Neighboors(p,2); %X
        if CurrentY+i<1||CurrentY+i>Height||CurrentX+j<1||CurrentX+j>Width
            continue
        end

        if(ClosedMAT(CurrentY+i,CurrentX+j)==0) %Neiboor is open;

            % Need to check that the path does not pass an object
            Flag=1;
%                 flag_2 = 1;
            for obs_idx=1:obs_num
                ax = CurrentX - obs_pos(1,obs_idx);
                ay = CurrentY - obs_pos(2,obs_idx);
                bx = CurrentX+j - obs_pos(1,obs_idx);
                by = CurrentY+i - obs_pos(2,obs_idx);
                a = (bx - ax)^2 + (by - ay)^2;
                b = 2*(ax*(bx - ax) + ay*(by - ay));
                c = ax^2 + ay^2 - (obs_sz(obs_idx)+r_v)^2;
                dit = b^2 - 4*a*c;
                if dit == 0      %交於一點
                    t1 = (-b + sqrt(dit))/(2*a);
                    if (0 < t1) && (t1 <= 1)
                        Flag = 0;
                        break;
                    end
                elseif dit > 0
                    t1 = (-b + sqrt(dit))/(2*a);
                    t2 = (-b - sqrt(dit))/(2*a);
                    if (t1 > 0) && (t2 <= 1) 
                        Flag = 0;
                        break;
                    end
                end
            end

             %End of  checking that the path does not pass an object

            if Flag==1;           
                tentative_gScore = GScore(CurrentY,CurrentX) + sqrt(i^2+j^2);
                if OpenMAT(CurrentY+i,CurrentX+j)==0
                    OpenMAT(CurrentY+i,CurrentX+j)=1;                    
                elseif tentative_gScore >= GScore(CurrentY+i,CurrentX+j)
                    continue
                end
                ParentX(CurrentY+i,CurrentX+j)=CurrentX;
                ParentY(CurrentY+i,CurrentX+j)=CurrentY;
                GScore(CurrentY+i,CurrentX+j)=tentative_gScore;
                FScore(CurrentY+i,CurrentX+j)= tentative_gScore+Hn(CurrentY+i,CurrentX+j);
            end
        end
    end
end

k=2;
if RECONSTRUCTPATH
    OptimalPath(1,:)=[CurrentY CurrentX];
    while RECONSTRUCTPATH
        CurrentXDummy=ParentX(CurrentY,CurrentX);
        CurrentY=ParentY(CurrentY,CurrentX);
        CurrentX=CurrentXDummy;
        OptimalPath(k,:)=[CurrentY CurrentX];
        k=k+1;
        if (((CurrentX== StartX)) &&(CurrentY==StartY))
            break
        end
    end
end


end
    
%{
line 93 -106 is using brute force to check if the path passes an object. For large connecting_distances
this uses alot of CPU time. If you have any more efficient way to add this effect in the code, pleas contact me and 
I will edit the code.  
%}

%%
%main function
%求兩向量間的夾角 (Vector2-Vector1)
function angle = angletest(preStartPose,startPose,goalPose)
    vector1=[(startPose(2)-preStartPose(2)),(startPose(1)-preStartPose(1))];
    vector2=[(goalPose(2)-startPose(2)),(goalPose(1)-startPose(1))];
    
    angle=mod(atan2d(det([vector2;vector1]),dot(vector1,vector2)),360);
    angle=angle/360*2*pi;
    if angle>pi
        angle=angle-2*pi;
    end
end

function h = circle(x,y,r,color)
    hold on
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    h = plot(xunit, yunit, 'color', color, 'HandleVisibility','off');
end

function feasible = collisionChecking(cur_pos, r_v, obs_num, obs_pos, obs_sz)
    for i = 1:obs_num
        d = sqrt((cur_pos(2)-obs_pos(2,i)).^2 + (cur_pos(1)-obs_pos(1,i)).^2);
        if d <= obs_sz(i) + r_v
            feasible = false;
            return
        end
    end
    feasible = true;
end
%%
%fuzzy function
%distance error 隸屬度與隸屬域
function  [newX,mf] = disTri(x,a,b,c,NB ,NS ,ZO ,PS ,PM ,PB ,PVB )  %得到該輸入在此隸屬函數中的隸屬度與隸屬域
    if x>=a && x<=b
        newX=(x-a)/(b-a);
    elseif x>b && x<=c
        newX=(c-x)/(c-b);
    else
        newX=0;
    end
    if b==6 
        mf=PS;
    elseif b==9
        mf=PM;
    elseif b==12
        mf=PB;
    end
end
function  [newX,mf] = disTrapR(x,a,b,NB ,NS ,ZO ,PS ,PM ,PB ,PVB )  %得到該輸入在此隸屬函數中的隸屬度與隸屬域
    if x>=a && x<=b
        newX=(x-a)/(b-a);
    elseif x>b
        newX=1;
    else
        newX=0;
    end
    mf=PVB;
end
function  [newX,mf] = disTrapL(x,a,b,NB ,NS ,ZO ,PS ,PM ,PB ,PVB )  %得到該輸入在此隸屬函數中的隸屬度與隸屬域
    if x>=a && x<=b
        newX=(b-x)/(b-a);
    elseif x<a
        newX=1;
    else
        newX=0;
    end
    mf=ZO;
end

function  [newX,mf] = angTri(x,a,b,c, NB, NS,ZO, PS, PM, PB,PVB )  %得到該輸入在此隸屬函數中的隸屬度與隸屬域
    if x>=a && x<=b
        newX=(x-a)/(b-a);
    elseif x>b && x<=c
        newX=(c-x)/(c-b);
    else
        newX=0;
    end
    if b==-pi/4 
        mf=NS;
    elseif b==0
        mf=ZO;
    elseif b==pi/4
        mf=PS;

    end
end
function  [newX,mf] = angTrapR(x,a,b,NB ,NS ,ZO ,PS ,PM ,PB ,PVB )  %得到該輸入在此隸屬函數中的隸屬度與隸屬域
    if x>=a && x<=b
        newX=(x-a)/(b-a);
    elseif x>b
        newX=1;
    else
        newX=0;
    end
    mf=PB;
end
function  [newX,mf] = angTrapL(x,a,b,NB ,NS ,ZO ,PS ,PM ,PB ,PVB )  %得到該輸入在此隸屬函數中的隸屬度與隸屬域
    if x>=a && x<=b
        newX=(b-x)/(b-a);
    elseif x<a
        newX=1;
    else
        newX=0;
    end
    mf=NB;
end

%建立模糊規則
function   [LMF RMF time]=rule(drMf,arMf,NB ,NS ,ZO ,PS ,PM ,PB ,PVB) %輸入 : 距離Mf，角度Mf  輸出 : 右輪Mf、左輪Mf   
    if drMf==ZO
        if arMf==NB
            LMF=PS;
            RMF=ZO;
            time=PS;
        elseif arMf==NS
            LMF=PS;
            RMF=ZO;
            time=PM;
        elseif arMf==ZO
            LMF=PS;
            RMF=PS;
            time=PM;
        elseif arMf==PS
            LMF=ZO;
            RMF=PS;
            time=PM;
        elseif arMf==PB
            LMF=ZO;
            RMF=PS;
            time=PS;
        end
    elseif drMf==PS
        if arMf==NB
            LMF=PM;
            RMF=ZO;
            time=PS;
        elseif arMf==NS
            LMF=PM;
            RMF=PS;
            time=PS;
        elseif arMf==ZO
            LMF=PM;
            RMF=PM;
            time=PM;
        elseif arMf==PS
            LMF=PS;
            RMF=PM;
            time=PS;
        elseif arMf==PB
            LMF=ZO;
            RMF=PM;
            time=PS;
        end       
    elseif drMf==PM
        if arMf==NB
            LMF=PB;
            RMF=ZO;
            time=PS;
        elseif arMf==NS
            LMF=PB;
            RMF=PS;
            time=PS;
        elseif arMf==ZO
            LMF=PB;
            RMF=PB;
            time=PS;
        elseif arMf==PS
            LMF=PS;
            RMF=PB;
            time=PS;
        elseif arMf==PB
            LMF=ZO;
            RMF=PB;
            time=PS;
        end       
    elseif drMf==PB
        if arMf==NB
            LMF=PVB;
            RMF=PS;
            time=PM;
        elseif arMf==NS
            LMF=PVB;
            RMF=PM;
            time=ZO;
        elseif arMf==ZO
            LMF=PVB;
            RMF=PVB;
            time=ZO;
        elseif arMf==PS
            LMF=PM;
            RMF=PVB;
            time=ZO;
        elseif arMf==PB
            LMF=PS;
            RMF=PVB;
            time=PM;
        end
    elseif drMf==PVB
        if arMf==NB
            LMF=PVB;
            RMF=PM;
            time=PM;
        elseif arMf==NS
            LMF=PVB;
            RMF=PB;
            time=ZO;
        elseif arMf==ZO
            LMF=PVB;
            RMF=PVB;
            time=ZO;
        elseif arMf==PS
            LMF=PB;
            RMF=PVB;
            time=ZO;
        elseif arMf==PB
            LMF=PM;
            RMF=PVB;
            time=PM;
        end
    end
end


%解模糊化
function [LPWM RPWM newEpoch] = defuzzy(accR,accL,accEpoch,yweigh,Rmf,Lmf,time)
    LPWM=accL+Lmf*yweigh*0.5;
    RPWM=accR+Rmf*yweigh*0.5;
    newEpoch=accEpoch+time*yweigh*0.5;
end