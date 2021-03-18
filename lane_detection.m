%%
close all;
clear;
clc;
fclose(instrfindall);
%%Define computer-specific variables
ipA = 'localhost';   portA = 9090;   % Modify these values to be those of your first computer.
ipB = 'localhost';  portB = 9091;  % Modify these values to be those of your second computer.
%%Create UDP Object
udpA = udp('localhost',portB,'localPort',portA);
%%Connect to UDP Object
fopen(udpA);

%%
n = 0
mov = VideoReader('Clockwise.mp4');
rlim = 5;
glim = 20;
blim = 100;
while hasFrame(mov)
 %% Data input from the sign detection script.  
    while udpA.BytesAvailable <= 0 
    end
        if udpA.BytesAvailable > 0
            data = fread(udpA, udpA.BytesAvailable);
            
            if data == 0
                color = 'green';

            elseif data == 1
                color = 'red';

            elseif data == 2
                color = 'red';
            end
        end
    

%%  Image Processing
    picture = readFrame(mov);
%     pic = rgb2hsv(picture);
    pic = picture;
    shape = size(pic);
%     gray_pic = rgb2gray(pic);
%     gray_pic = imgaussfilt(gray_pic, 0.2);
    gray_pic = pic(:, :, 3) + 70;
    edge_pic = edge(gray_pic, 'canny', [0.15, 0.35]);
%     imshow(edge_pic)
    

    a = [shape(2)*0.4, shape(2)*0.6, shape(2)*0.95, shape(2)*0.05];
    b = [shape(1)*0.6, shape(1)*0.6, shape(1),shape(1)];
    bw=roipoly(pic,a,b);
    BW=(edge_pic(:,:,1)&bw);
    

%% Hough Transform
    [H, T, R] = hough(BW);
    p = houghpeaks(H, 3);
    lines = houghlines(BW, T, R, p, 'FillGap', 50, 'MinLength', 10);
     
    anglethres=0.01; %separate left/right by orientation threshold
    leftlines=[];rightlines=[]; %Two group of lines
    for k = 1:length(lines)
        x1=lines(k).point1(1);y1=lines(k).point1(2);
        x2=lines(k).point2(1);y2=lines(k).point2(2);
        if (x2 >= shape(2)/2) && ((y2-y1)/(x2-x1)>anglethres)
            rightlines=[rightlines;x1,y1;x2,y2];
        elseif (x2<=shape(2)/2) && ((y2-y1)/(x2-x1)<(-1*anglethres))
            leftlines=[leftlines;x1,y1;x2,y2];
        end
    end
    
    j = size(leftlines) ~= [0 0];
    k = size(rightlines) ~= [0 0];
    
    if  j & k
        draw_y=[shape(1)*0.6,shape(1)]; %two row coordinates
        PL=polyfit(leftlines(:,2),leftlines(:,1),1);
        draw_lx=polyval(PL,draw_y); %two col coordinates of left line
        PR=polyfit(rightlines(:,2),rightlines(:,1),1);
        draw_rx=polyval(PR,draw_y); %two col coordinates of right line        
    end
    % Inverse Projection
    left_img_upper = [draw_lx(1), draw_y(1)];
    left_img_lower = [draw_lx(2), draw_y(2)];
    right_img_upper = [draw_rx(1), draw_y(1)];
    right_img_lower = [draw_rx(2), draw_y(2)];
    
    %Intrinsic Parameters
    Fu = 728.2899;
    Fv = 729.1500;
    u0 = 623.3829;
    v0 = 474.5165;
    Yc = 210;
    
    % Left Line upper coordinates
    left_cam_upper = InverseProjection(draw_lx(1), draw_y(1), Fu, Fv, u0, v0, Yc);
    left_cam_lower = InverseProjection(draw_lx(2), draw_y(2), Fu, Fv, u0, v0, Yc);
    right_cam_upper = InverseProjection(draw_rx(1), draw_y(1), Fu, Fv, u0, v0, Yc);
    right_cam_lower = InverseProjection(draw_rx(2), draw_y(2), Fu, Fv, u0, v0, Yc);
   
    % Stanley Calculations
    Xcf = (left_cam_upper(1) + right_cam_upper(1))/2;
    Xcn = (left_cam_lower(1) + right_cam_upper(1))/2;
    Zcf = (left_cam_upper(2) + right_cam_upper(2))/2;
    Zcn= (left_cam_lower(2) + right_cam_lower(2))/2;
    Cpos = 60;
    
    departure_angle = -atan2d((Zcf - Zcn),(Xcf - Xcn)) - 90;                % theta_e
    departure_distance = Xcn + (Xcf - Xcn)/(Zcf - Zcn)*(Zcn - Cpos);        % efa
    k1 = 0.006;
    k2 = 0.005;  % Departure angle and distance values when steering angle (phi) is zero.
    phi = -k1*departure_angle - k2*departure_distance + 0.7275; % offset added.
    
%%    % HMI
    % Signs    

    % Steering Angle
    figure(1)
    text_str2 = ['Steering Angle: ' num2str(phi) , char(176)];
    steer = insertText(pic, [5 100], text_str2, 'FontSize', 45, 'BoxColor', 'red', 'BoxOpacity', 0.4, 'TextColor', 'white');
    pos_triangle = [1550 600 1650 450 1750 600];
    RGB = insertShape(steer, 'FilledPolygon', pos_triangle, 'Color', color, 'Opacity', 0.8);
    imshow(RGB);
 
    hold on;
    plot(draw_lx,draw_y,'LineWidth', 10,'Color','red');
    hold on;
    plot(draw_rx,draw_y,'LineWidth',10,'Color','red');
    hold on;
    fwrite(udpA, 'This should work')
     
end

function coordinates = InverseProjection(x, z, Fu, Fv, u0, v0, Yc)
    Zc = Fv*Yc/(z - v0);
    Xc = Zc*(x - u0)/Fu;
    coordinates = [Xc, Zc];
end



    
    


   

    
       

    
