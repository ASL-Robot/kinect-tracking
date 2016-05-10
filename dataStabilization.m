clear all;
[body,txt,raw] = xlsread('Hello.xlsx',1); % replace "Hello" with your file name 
[face,txtf,rawf] = xlsread('Hello.xlsx',2); % replace "Hello" with your file name 


dataLenB = 15;
dataLenF = 9;

BODY = zeros(length(body), 3); % 3 columns
for k = 1:length(body)/dataLenB;
    origin = (body((k-1)*dataLenB + 3,:) + body((k-1)*dataLenB + 7,:))/2; % origin is the average of the shoulders
    datBlokB = body((k-1)*dataLenB+1:k*dataLenB,:);
    BODY((k-1)*dataLenB+1:k*dataLenB,:) = bsxfun(@minus,datBlokB, origin);
    
end

originSH = (body(3:dataLenB:length(body),:) + body(7:dataLenB:length(body),:))/2 ;
originSHAvg = mean(originSH);
face = bsxfun(@minus,face, originSHAvg);

% changing (z,x,y) to (x,y,z)
xdata = BODY(:,3);
ydata = BODY(:,1);
zdata = BODY(:,2);
BODY = [xdata ydata zdata]; % data matrix with adjusted coordinates

% xdata = face(:,3);
% ydata = face(:,1);
% zdata = face(:,2);
% FACE = [xdata, ydata, zdata];

% position array for each joint of the body
neck = BODY(1:dataLenB:length(BODY),:);
head = BODY(2:dataLenB:length(BODY),:);
shoulderL = BODY(3:dataLenB:length(BODY),:);
elbowL = BODY(4:dataLenB:length(BODY),:);
wristL = BODY(5:dataLenB:length(BODY),:);
handL = BODY(6:dataLenB:length(BODY),:);
shoulderR = BODY(7:dataLenB:length(BODY),:);
elbowR = BODY(8:dataLenB:length(BODY),:);
wristR = BODY(9:dataLenB:length(BODY),:);
handR = BODY(10:dataLenB:length(BODY),:);
spineSH = BODY(11:dataLenB:length(BODY),:);
handTipL = BODY(12:dataLenB:length(BODY),:);
thumbL = BODY(13:dataLenB:length(BODY),:);
handTipR = BODY(14:dataLenB:length(BODY),:);
thumbR = BODY(15:dataLenB:length(BODY),:);

% % position array for each facial feature
% eyeInL = FACE(2:9:length(FACE),:);
% eyeOutL = FACE(3:9:length(FACE),:);
% eyeInR = FACE(4:9:length(FACE),:);
% eyeOutR = FACE(5:9:length(FACE),:);
% nose = FACE(6:9:length(FACE),:);
% mouthL = FACE(7:9:length(FACE),:);
% mouthR = FACE(8:9:length(FACE),:);

%
figure
plot3(shoulderR(:,1),shoulderR(:,2),shoulderR(:,3),'linewidth',4)
hold on
plot3(shoulderL(:,1),shoulderL(:,2),shoulderL(:,3),'linewidth',4)
plot3(elbowR(:,1),elbowR(:,2),elbowR(:,3),'linewidth',3)
plot3(wristR(:,1),wristR(:,2),wristR(:,3),'linewidth',2)
plot3(handTipR(:,1),handTipR(:,2),handTipR(:,3),'linewidth',2)
grid on
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
legend('Right shoulder','Left shoulder','Right elbow','Right wrist','Right hand tip')
title('Motion obtained from the Kinect (Hello)')

%% calculating shoulder length
shoulderLenArr = [];
for k = 1:length(BODY)/dataLenB
    deltaX_LSH = shoulderL(k,1);  % delta x of Left Shoulder
    deltaY_LSH = shoulderL(k,2);  % delta y of Left Shoulder
    deltaZ_LSH = shoulderL(k,3);  % delta z of Left Shoulder

    leftShoulderLen = sqrt(deltaX_LSH^2 + deltaY_LSH^2 + deltaZ_LSH^2); 
    
    deltaX_RSH = shoulderR(k,1);  % delta x of right Shoulder
    deltaY_RSH = shoulderR(k,2);  % delta y of right Shoulder
    deltaZ_RSH = shoulderR(k,3);  % delta z of right Shoulder

    rightShoulderLen = sqrt(deltaX_RSH^2 + deltaY_RSH^2 + deltaZ_RSH^2);
    
    shoulderLenArr = [shoulderLenArr leftShoulderLen rightShoulderLen];
end

shoulderLen = mean(shoulderLenArr);

%% getting arm length
armLenArr = [];
leftArmDelta = shoulderL - elbowL; % delta x, y, and z for left arm
rightArmDelta = shoulderR - elbowR; % delta x, y, and z for right arm

for k = 1:length(BODY)/dataLenB
    leftArmLen = sqrt(leftArmDelta(k,1)^2 + leftArmDelta(k,2)^2 + leftArmDelta(k,3)^2); 
    
    rightArmLen = sqrt(rightArmDelta(k,1)^2 + rightArmDelta(k,2)^2 + rightArmDelta(k,3)^2); 
    
    armLenArr = [armLenArr leftArmLen rightArmLen];
end

armLen = mean(armLenArr);

%% getting forearm length
foreArmLenArr = [];
leftForeArmDelta = elbowL - wristL; % delta x, y, and z for left arm
rightForeArmDelta = elbowR - wristR; % delta x, y, and z for right arm

for k = 1:length(BODY)/dataLenB
    leftForeArmLen = sqrt(leftForeArmDelta(k,1)^2 + leftForeArmDelta(k,2)^2 + leftForeArmDelta(k,3)^2); 
    
    rightForeArmLen = sqrt(rightForeArmDelta(k,1)^2 + rightForeArmDelta(k,2)^2 + rightForeArmDelta(k,3)^2); 
    
    foreArmLenArr = [foreArmLenArr leftForeArmLen rightForeArmLen];
end

foreArmLen = mean(foreArmLenArr);

%% getting hand length
handLenArr = [];
leftHandDelta = handL - wristL; % delta x, y, and z for left arm
rightHandDelta = handR - wristR; % delta x, y, and z for right arm

for k = 1:length(BODY)/dataLenB
    leftHandLen = sqrt(leftHandDelta(k,1)^2 + leftHandDelta(k,2)^2 + leftHandDelta(k,3)^2); 
    
    rightHandLen = sqrt(rightHandDelta(k,1)^2 + rightHandDelta(k,2)^2 + rightHandDelta(k,3)^2); 
    
    handLenArr = [handLenArr leftHandLen rightHandLen];
end

handLen = mean(handLenArr);

%% getting hand length
handTipLenArr = [];
leftHandTipDelta = handTipL - handL; % delta x, y, and z for left arm
rightHandTipDelta = handTipR - handR; % delta x, y, and z for right arm

for k = 1:length(BODY)/dataLenB
    leftHandTipLen = sqrt(leftHandTipDelta(k,1)^2 + leftHandTipDelta(k,2)^2 + leftHandTipDelta(k,3)^2); 
    
    rightHandTipLen = sqrt(rightHandTipDelta(k,1)^2 + rightHandTipDelta(k,2)^2 + rightHandTipDelta(k,3)^2); 
    
    handTipLenArr = [handTipLenArr leftHandTipLen rightHandTipLen];
end

handTipLen = mean(handTipLenArr);

%% shoulder position fixing by using average position of the shoulder
BODYA = BODY;
shoulderPosLx = mean(shoulderL(:,1))*ones(length(shoulderL),1);
shoulderPosLy = mean(shoulderL(:,2))*ones(length(shoulderL),1);
shoulderPosLz = mean(shoulderL(:,3))*ones(length(shoulderL),1);
shoulderPosL = [shoulderPosLx shoulderPosLy shoulderPosLz];

shoulderPosRx = mean(shoulderR(:,1))*ones(length(shoulderR),1);
shoulderPosRy = mean(shoulderR(:,2))*ones(length(shoulderR),1);
shoulderPosRz = mean(shoulderR(:,3))*ones(length(shoulderR),1);
shoulderPosR = [shoulderPosRx shoulderPosRy shoulderPosRz];

BODYA(3:dataLenB:length(BODYA),:) = shoulderPosL;
BODYA(7:dataLenB:length(BODYA),:) = shoulderPosR;

%% std calculating
elbowR_std = std(elbowR);
elbowL_std = std(elbowL);
wristR_std = std(wristR);
wristL_std = std(wristL);
handR_std = std(handR);
handL_std = std(handL);


%% elbow position fixing
elbowPosLx = mean(elbowL(:,1))*ones(length(elbowL),1);
elbowPosLy = mean(elbowL(:,2))*ones(length(elbowL),1);
elbowPosLz = mean(elbowL(:,3))*ones(length(elbowL),1);
elbowPosL = [elbowPosLx elbowPosLy elbowPosLz];

elbowPosRx = mean(elbowR(:,1))*ones(length(elbowR),1);
elbowPosRy = mean(elbowR(:,2))*ones(length(elbowR),1);
elbowPosRz = mean(elbowR(:,3))*ones(length(elbowR),1);
elbowPosR = [elbowPosRx elbowPosRy elbowPosRz];

if (elbowR_std(2) <= 0.03) && (elbowR_std(3) <= 0.03)
    BODYA(4:dataLenB:length(BODYA),:) = elbowPosL;
    BODYA(8:dataLenB:length(BODYA),:) = elbowPosR;
else
    newElbowPosR = [];
    for k = 1: length(BODYA)/dataLenB
        lengthR = sqrt(rightArmDelta(k,1)^2 + rightArmDelta(k,2)^2 + rightArmDelta(k,3)^2);
        unitR = rightArmDelta(k,:)/lengthR; % getting the unit vector
        rR = shoulderPosR(k,:) + armLen*unitR; % r = r0 + t*unit vector
        newElbowPosR = [newElbowPosR; rR];   % new elbow positions     
    end
    BODYA(8:dataLenB:length(BODYA),:) = newElbowPosR;

    newElbowPosL = [];
    for k = 1: length(BODYA)/dataLenB
        lengthL = sqrt(leftArmDelta(k,1)^2 + leftArmDelta(k,2)^2 + leftArmDelta(k,3)^2);
        unitL = leftArmDelta(k,:)/lengthL;
        rL = shoulderPosL(k,:) + armLen*unitL;
        newElbowPosL = [newElbowPosL; rL];        
    end
    BODYA(4:dataLenB:length(BODYA),:) = newElbowPosL;
   
end


%% wrist position fixing
elbowPosRA = BODYA(8:dataLenB:length(BODYA),:);
elbowPosLA = BODYA(4:dataLenB:length(BODYA),:);
rightForeArmDeltaA = wristR - elbowPosRA;
leftForeArmDeltaA = wristL - elbowPosLA;

wristPosLx = mean(wristL(:,1))*ones(length(wristL),1);
wristPosLy = mean(wristL(:,2))*ones(length(wristL),1);
wristPosLz = mean(wristL(:,3))*ones(length(wristL),1);
wristPosL = [wristPosLx wristPosLy wristPosLz];

wristPosRx = mean(wristR(:,1))*ones(length(wristR),1);
wristPosRy = mean(wristR(:,2))*ones(length(wristR),1);
wristPosRz = mean(wristR(:,3))*ones(length(wristR),1);
elbowPosR = [wristPosRx wristPosRy wristPosRz];

if (wristR_std(2) <= 0.02) && (wristR_std(3) <= 0.02)
    BODYA(5:dataLenB:length(BODYA),:) = wristPosL;
    BODYA(9:dataLenB:length(BODYA),:) = elbowPosR;
else
    newWristPosR = [];
    for k = 1: length(BODYA)/dataLenB
        lengthR = sqrt(rightForeArmDeltaA(k,1)^2 + rightForeArmDeltaA(k,2)^2 + rightForeArmDeltaA(k,3)^2);
        unitR = rightForeArmDeltaA(k,:)/lengthR; % getting the unit vector
        rR = elbowPosRA(k,:) + foreArmLen*unitR; % r = r0 + t*unit vector
        newWristPosR = [newWristPosR; rR];   % new elbow positions     
    end
    BODYA(9:dataLenB:length(BODYA),:) = newWristPosR;

    newWristPosL = [];
    for k = 1: length(BODYA)/dataLenB
        lengthL = sqrt(leftForeArmDeltaA(k,1)^2 + leftForeArmDeltaA(k,2)^2 + leftForeArmDeltaA(k,3)^2);
        unitL = leftForeArmDeltaA(k,:)/lengthL;
        rL = elbowPosLA(k,:) + foreArmLen*unitL;
        newWristPosL = [newWristPosL; rL];        
    end
    BODYA(5:dataLenB:length(BODYA),:) = newWristPosL;
end

%% hand position fixing
wristPosRA = BODYA(9:dataLenB:length(BODYA),:);
wristPosLA = BODYA(5:dataLenB:length(BODYA),:);

rightHandDeltaA = handR - wristPosRA;
leftHandDeltaA = handL - wristPosLA;

handPosLx = mean(handL(:,1))*ones(length(handL),1);
handPosLy = mean(handL(:,2))*ones(length(handL),1);
handPosLz = mean(handL(:,3))*ones(length(handL),1);
handPosL = [handPosLx handPosLy handPosLz];

handPosRx = mean(handR(:,1))*ones(length(handR),1);
handPosRy = mean(handR(:,2))*ones(length(handR),1);
handPosRz = mean(handR(:,3))*ones(length(handR),1);
handPosR = [handPosRx handPosRy handPosRz];

if (handR_std(2) <= 0.02) && (handR_std(3) <= 0.02)
    BODYA(6:dataLenB:length(BODYA),:) = handPosL;
    BODYA(10:dataLenB:length(BODYA),:) = handPosR;
else
    newHandPosR = [];
    for k = 1: length(BODYA)/dataLenB
        lengthR = sqrt(rightHandDeltaA(k,1)^2 + rightHandDeltaA(k,2)^2 + rightHandDeltaA(k,3)^2);
        unitR = rightHandDeltaA(k,:)/lengthR; % getting the unit vector
        rR = wristPosRA(k,:) + handLen*unitR; % r = r0 + t*unit vector
        newHandPosR = [newHandPosR; rR];   % new elbow positions     
    end
    BODYA(10:dataLenB:length(BODYA),:) = newHandPosR;

    newHandPosL = [];
    for k = 1: length(BODYA)/dataLenB
        lengthL = sqrt(leftHandDeltaA(k,1)^2 + leftHandDeltaA(k,2)^2 + leftHandDeltaA(k,3)^2);
        unitL = leftHandDeltaA(k,:)/lengthL;
        rL = wristPosLA(k,:) + handLen*unitL;
        newHandPosL = [newHandPosL; rL];        
    end
    BODYA(6:dataLenB:length(BODYA),:) = newHandPosL;
end

%% hand tip position fixing
handPosRA = BODYA(10:dataLenB:length(BODYA),:);
handPosLA = BODYA(6:dataLenB:length(BODYA),:);

rightHandTipDeltaA = handTipR - handPosRA;
leftHandTipDeltaA = handTipL - handPosLA;

handTipPosLx = mean(handTipL(:,1))*ones(length(handTipL),1);
handTipPosLy = mean(handTipL(:,2))*ones(length(handTipL),1);
handTipPosLz = mean(handTipL(:,3))*ones(length(handTipL),1);
handTipPosL = [handTipPosLx handTipPosLy handTipPosLz];

handTipPosRx = mean(handTipR(:,1))*ones(length(handTipR),1);
handTipPosRy = mean(handTipR(:,2))*ones(length(handTipR),1);
handTipPosRz = mean(handTipR(:,3))*ones(length(handTipR),1);
handTipPosR = [handTipPosRx handTipPosRy handTipPosRz];

newHandTipPosR = [];
for k = 1: length(BODYA)/dataLenB
    lengthR = sqrt(rightHandTipDeltaA(k,1)^2 + rightHandTipDeltaA(k,2)^2 + rightHandTipDeltaA(k,3)^2);
    unitR = rightHandTipDeltaA(k,:)/lengthR; % getting the unit vector
    rR = handPosRA(k,:) + handTipLen*unitR; % r = r0 + t*unit vector
    newHandTipPosR = [newHandTipPosR; rR];   % new elbow positions     
end
BODYA(14:dataLenB:length(BODYA),:) = newHandTipPosR;

newHandTipPosL = [];
for k = 1: length(BODYA)/dataLenB
    lengthL = sqrt(leftHandTipDeltaA(k,1)^2 + leftHandTipDeltaA(k,2)^2 + leftHandTipDeltaA(k,3)^2);
    unitL = leftHandTipDeltaA(k,:)/lengthL;
    rL = handPosLA(k,:) + handTipLen*unitL;
    newHandTipPosL = [newHandTipPosL; rL];        
end
BODYA(12:dataLenB:length(BODYA),:) = newHandTipPosL;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FACE %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% getting facial feature positions
originSH = (BODYA(3:dataLenB:length(BODYA),:) + BODYA(7:dataLenB:length(BODYA),:))/2 ;
originSHAvg = mean(originSH);
faceA = bsxfun(@minus,face, originSHAvg);
xdata = faceA(:,3);
ydata = faceA(:,1);
zdata = faceA(:,2);
FACE = [xdata, ydata, zdata];

% position array for each facial feature
eyeInL = FACE(2:9:length(FACE),:);
eyeOutL = FACE(3:9:length(FACE),:);
eyeInR = FACE(4:9:length(FACE),:);
eyeOutR = FACE(5:9:length(FACE),:);
nose = FACE(6:9:length(FACE),:);
mouthL = FACE(7:9:length(FACE),:);
mouthR = FACE(8:9:length(FACE),:);

% getting positions of facial features
eyeL = (eyeInL + eyeOutL)/2;
eyeR = (eyeInR + eyeOutR)/2;
eyeL = mean(eyeL);
eyeR = mean(eyeR);

noseTip = mean(nose);
mouthL = mean(mouthL);
mouthR = mean(mouthR);

FACE = [eyeL; eyeR; noseTip; mouthL; mouthR];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5%%%
%%%%%%%%%%%%%%%%%% EXPORTING STABLIZED DATA%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%xlswrite('HelloB_stabilized.xls', BODYA)
%xlswrite('HelloF_stabilized.xls', FACE)

% position array for each joint of the body
shoulderRA = BODYA(7:dataLenB:length(BODY),:);
shoulderLA = BODYA(3:dataLenB:length(BODY),:);
elbowRA = BODYA(8:dataLenB:length(BODY),:);
wristRA = BODYA(9:dataLenB:length(BODY),:);
handRA = BODYA(10:dataLenB:length(BODY),:);
handTipRA = BODYA(14:dataLenB:length(BODY),:);
thumbRA = BODYA(15:dataLenB:length(BODY),:);

figure
scatter3(shoulderRA(:,1),shoulderRA(:,2),shoulderRA(:,3),'linewidth',2)
hold on
scatter3(shoulderLA(:,1),shoulderLA(:,2),shoulderLA(:,3),'linewidth',2)
scatter3(elbowRA(:,1),elbowRA(:,2),elbowRA(:,3),'linewidth',2)
scatter3(wristRA(:,1),wristRA(:,2),wristRA(:,3),'linewidth',2)
scatter3(handRA(:,1),handRA(:,2),handRA(:,3))
scatter3(handTipRA(:,1),handTipRA(:,2),handTipRA(:,3),'linewidth',2)
grid on
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
legend('Right shoulder','Left shoulder','Right elbow','Right wrist','right hand','Right hand tip')
title('Stabilized motion (Hello)')


scatter3(eyeL(1), eyeL(2), eyeL(3))
scatter3(eyeR(1), eyeR(2), eyeR(3))
scatter3(mouthL(1), mouthL(2), mouthL(3))
scatter3(mouthR(1), mouthR(2), mouthR(3))
scatter3(noseTip(1), noseTip(2), noseTip(3))

save('HelloB.mat','BODYA') % body data for "Hello"
save('HelloF.mat','FACE') % face data for "Hello"