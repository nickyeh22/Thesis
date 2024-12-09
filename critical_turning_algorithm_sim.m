clc;
clear all;
close all;
tic
%%
%coefficeint setting
obs_pos = [113 92];
r_o = 17;

vhc_hd = 0; 
% vhc_hd = 40.2364;
% vhc_hd = 70;
% disp(sqrt((90-50.2543)^2 + (90-50.2543)^2));
vhc_pos = [28 92];  %剛好
% vhc_pos = obs_pos - [56.209*cosd(vhc_hd) 56.209*sind(vhc_hd)];
% disp(vhc_pos);

% vhc_pos = [80 31.1]; 

v = 10;             %載具速度
psi = 20;           %載具最大轉角(degree)
psi_rate = 10;      %載具rate of sheering angle(degree/sec)
L_f = 5; L_r = 7;   %載具前後輪軸距
L = L_f+L_r;
r_v = 5*sqrt(2);

%%
%計算開始轉彎位置
vhc_hd = vhc_hd*pi/180;
psi_rate = psi_rate*pi/180;
vhc_hd_r = -(v/psi_rate/L)*log(abs(sec(-psi_rate*2))) + vhc_hd;      %載具右轉達最大steering angle時的heading
vhc_hd_l = (v/psi_rate/L)*log(abs(sec(psi_rate*2))) + vhc_hd;       %載具左轉達最大steering angle時的heading


%計算d_0
% disp(sqrt((obs_pos(1)-vhc_pos(1)).^2 + (obs_pos(2)-vhc_pos(2)).^2) - r_o)

R = cotd(abs(psi))*sqrt((L_r*tand(abs(psi))).^2 + L.^2);
beta = atan(L_r/L*tand(psi));

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

disp([R beta])
disp([c1 c2])
disp([del_x_r del_y_r vhc_hd_r])

phi = atan2((obs_pos(2) - vhc_pos(2)), (obs_pos(1) - vhc_pos(1)));
if (obs_pos(2) - vhc_pos(2)) < 0   %第三四象限角
    phi = 2*pi + phi;
elseif (obs_pos(1) - vhc_pos(1)) == 0 && (obs_pos(2) - vhc_pos(2)) > 0
    phi = pi/2;
elseif (obs_pos(1) - vhc_pos(1)) == 0 && (obs_pos(2) - vhc_pos(2)) < 0
    phi = 3*pi/4;
end
% disp(phi)

if tan(vhc_hd) < tan(phi)
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
    end

    if err_r ~= true
        if abs(tan(vhc_hd)) < tan(pi/4)
            tur_pos = [t_0_r tan(vhc_hd)*t_0_r+c1];
        else
            tur_pos = [cot(vhc_hd)*t_0_r+c2 t_0_r];
        end
        psi = -psi;
        beta = -beta;
        disp(tur_pos);
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
            error('擦邊');
        else
            error('逃不出去');
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
    
    f_a = func(a, obs_pos, vhc_hd, c1, c2, del_x_l, del_y_l, beta, R, r_v, r_o, vhc_hd_l, false);
    f_b = func(b, obs_pos, vhc_hd, c1, c2, del_x_l, del_y_l, beta, R, r_v, r_o, vhc_hd_l, false);
    err_l = false;
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
    end

    if err_l ~= true
        if abs(tan(vhc_hd)) < tan(pi/4)
            tur_pos = [t_0_l tan(vhc_hd)*t_0_l+c1];
        else
            tur_pos = [cot(vhc_hd)*t_0_l+c2 t_0_l];
        end
        disp(tur_pos);
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
            error('擦邊');
        else
            error('逃不出去');
        end
    end


end
toc
% disp(x_0_l);

% if err_r ~= true
%     if abs(tan(vhc_hd)) < tan(pi/4)
%         tur_pos_r = [t_0_r tan(vhc_hd)*t_0_r+c1];
%     else
%         tur_pos_r = [cot(vhc_hd)*t_0_r+c2 t_0_r];
%     end
% else
%     tur_pos_r = [inf inf];
% end
% 
% if err_l ~= true
%     if abs(tan(vhc_hd)) < tan(pi/4)
%         tur_pos_l = [t_0_l tan(vhc_hd)*t_0_l+c1];
%     else
%         tur_pos_l = [cot(vhc_hd)*t_0_l+c2 t_0_l];
%     end
% else
%     tur_pos_l = [inf inf];
% end
% 
% d_0_r = sqrt((obs_pos(1)-tur_pos_r(1)).^2 + (obs_pos(2)-tur_pos_r(2)).^2);
% d_0_l = sqrt((obs_pos(1)-tur_pos_l(1)).^2 + (obs_pos(2)-tur_pos_l(2)).^2);
% disp(err_r)
% if err_r ~= true || err_l ~= true  
%     if d_0_r < d_0_l
%         psi = -psi;
%         beta = -beta;
%         disp(tur_pos_r);
%     else
%         disp(tur_pos_l);
%     end
% else
%     error('no solution');
% end

%%
%plot
h = circle(obs_pos(1), obs_pos(2), r_o);  
plot(vhc_pos(1),vhc_pos(2),'*b', obs_pos(1),obs_pos(2),'*r');   
axis([0 150 -25 125],'square'); 

lined = vhc_pos+r_v*[cos(vhc_hd) sin(vhc_hd)];
ln = line([vhc_pos(1) lined(1)], [vhc_pos(2) lined(2)],'color','r');
delete(ln)

% if psi < 0
%     t_0 = sqrt((tur_pos_r(1)-vhc_pos(1)).^2 + (tur_pos_r(2)-vhc_pos(2)).^2)/v;
% else
%     t_0 = sqrt((tur_pos_l(1)-vhc_pos(1)).^2 + (tur_pos_l(2)-vhc_pos(2)).^2)/v;
% end
t_0 = sqrt((tur_pos(1)-vhc_pos(1)).^2 + (tur_pos(2)-vhc_pos(2)).^2)/v;

for t = 0:t_0/250:t_0
    x = vhc_pos(1) + v*cos(vhc_hd)*t;
    y = vhc_pos(2) + v*sin(vhc_hd)*t;
%     x_nxt = vhc_pos(1) + v*cos(vhc_hd)*(t+500/25000);
%     y_nxt = vhc_pos(2) + v*sin(vhc_hd)*(t+500/25000);
    plot(x,y,'.b');  
    lined = [x y]+r_v*[cos(vhc_hd) sin(vhc_hd)];    
    ln = line([x lined(1)], [y lined(2)],'color','r');
    h = circle(x,y,r_v);
    drawnow;
%     d = sqrt((obs_pos(1)-x).^2 + (obs_pos(2)-y).^2);
%     d_nxt = sqrt((obs_pos(1)-x_nxt).^2 + (obs_pos(2)-y_nxt).^2);

%     if d_nxt > d
%         return;
%         break; 
%     end
    %     if t < t_0
        delete(h);
        delete(ln);
%     end

end

for t = 0:0.001:2
    if psi < 0
        vhc_hd_cur = -(v/psi_rate/L)*log(abs(sec(-psi_rate*t))) + vhc_hd;
        x_diff = v*cos(atan(L_r/L*tan(-psi_rate*t)) + v/(-psi_rate)/L*log(abs(sec(-psi_rate*t))) + vhc_hd);
        y_diff = v*sin(atan(L_r/L*tan(-psi_rate*t)) + v/(-psi_rate)/L*log(abs(sec(-psi_rate*t))) + vhc_hd);
    else
        vhc_hd_cur = (v/psi_rate/L)*log(abs(sec(psi_rate*t))) + vhc_hd; 
        x_diff = v*cos(atan(L_r/L*tan(psi_rate*t)) + v/(psi_rate)/L*log(abs(sec(psi_rate*t))) + vhc_hd);
        y_diff = v*sin(atan(L_r/L*tan(psi_rate*t)) + v/(psi_rate)/L*log(abs(sec(psi_rate*t))) + vhc_hd);
    end

    x = x + x_diff*0.001;
    y = y + y_diff*0.001;

    if floor(t*100) == t*100
        plot(x,y,'.b');  
        lined = [x y]+r_v*[cos(vhc_hd_cur) sin(vhc_hd_cur)];    
        ln = line([x lined(1)], [y lined(2)],'color','r');
        h = circle(x,y,r_v);
        drawnow;
    end
%     if t < 2
        delete(h);
        delete(ln);
%     end
end

x_0 = x;
y_0 = y;

for t = 0:0.01:3
    del_theta = (v/L)*tand(psi)*t;
    del_theta_nxt = (v/L)*tand(psi)*(t+0.01);
%     disp(del_theta);
    x = L/tand(psi)*sin(beta+vhc_hd_cur+del_theta) + (x_0-sin(beta+vhc_hd_cur)*L/tand(psi));
    y = -L/tand(psi)*cos(beta+vhc_hd_cur+del_theta) + (y_0+cos(beta+vhc_hd_cur)*L/tand(psi));
    x_nxt = L/tand(psi)*sin(beta+vhc_hd_cur+del_theta_nxt) + (x_0-sin(beta+vhc_hd_cur)*L/tand(psi));
    y_nxt = -L/tand(psi)*cos(beta+vhc_hd_cur+del_theta_nxt) + (y_0+cos(beta+vhc_hd_cur)*L/tand(psi));
%     hold on;
    plot(x,y,'.b');  
    lined = [x y]+r_v*[cos(vhc_hd_cur+del_theta) sin(vhc_hd_cur+del_theta)];    
    ln = line([x lined(1)], [y lined(2)],'color','r');
    h = circle(x,y,r_v);
    drawnow;
    d = sqrt((obs_pos(1)-x).^2 + (obs_pos(2)-y).^2);
    d_nxt = sqrt((obs_pos(1)-x_nxt).^2 + (obs_pos(2)-y_nxt).^2);
%     if d_nxt > d
%         break;
%     end
    if t < 3
        delete(h);
        delete(ln);
    end
end
 

%%
%function
function h = circle(x,y,r)
    hold on
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    h = plot(xunit, yunit, 'k');
end

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

% disp(sqrt((R*cos(alpha)).^2 + 2*R*(r_v+r_o) + (r_v+r_o).^2) + R*cos(alpha));
    y = sqrt((obs_pos(2) - y_pos)^2 + (obs_pos(1) - x_pos)^2)... 
        - (sqrt((R*cos(alpha)).^2 + 2*R*(r_v+r_o) + (r_v+r_o).^2) + R*cos(alpha));
% a = sqrt((obs_pos(2) - y_pos)^2 + (obs_pos(1) - x_pos)^2);
% b = sqrt((R*cos(alpha)).^2 + 2*R*(r_v+r_o) + (r_v+r_o).^2) + R*cos(alpha);
% disp([t y a b]);
end

