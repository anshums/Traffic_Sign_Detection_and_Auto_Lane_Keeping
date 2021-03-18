clear;
clc;
fclose(instrfindall);
% Computer-specific parameteres
ipA = 'localhost'; portA = 9090;
ipB = 'localhost'; portB = 9091;

udpB = udp('localhost', portA, 'localPort', portB);
fopen(udpB);


%% Stop Sign Detection
load 'var.mat';
mov = VideoReader('Clockwise.mp4');
sign = 0;
while hasFrame(mov)
    test = readFrame(mov);
    testImage = test(1:600, 1:650, :);
    testImage = imresize(testImage, 0.35);
    [bboxes,score,label] = detect(rcnn,testImage,'MiniBatchSize',128);
    [score, idx] = max(score);
    bbox = bboxes(idx, :);
    annotation = sprintf('%s: (Confidence = %f)', label(idx), score);
    outputImage = insertObjectAnnotation(testImage, 'rectangle', bbox, annotation);
%     figure(1)
%     imshow(outputImage)
    if score > 0.999 & label == 'stop_sign'
        sign = 1
    elseif score > 0.998 & label == 'school_sign'
        sign = 2
    else
        sign = 0
    end
    fwrite(udpB,sign)
    while udpB.BytesAvailable <= 0
    end
    flushinput(udpB);
end

% fclose(instrfindall)