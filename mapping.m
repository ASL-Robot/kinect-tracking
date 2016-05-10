clear all;
load HelloB.mat
load HelloF.mat
dataLenB = 15;
dataLenF = 9;

BODYA = BODYA*100; % converting m to cm
FACE = FACE*100; % converting m to cm

%%%%%%%%%%%%%%%% stabilized data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
shoulderLA = BODYA(3:dataLenB:length(BODYA),:);
elbowLA = BODYA(4:dataLenB:length(BODYA),:);
wristLA = BODYA(5:dataLenB:length(BODYA),:);
handLA = BODYA(6:dataLenB:length(BODYA),:);
handTipLA = BODYA(12:dataLenB:length(BODYA),:);
thumbLA = BODYA(13:dataLenB:length(BODYA),:);

shoulderRA = BODYA(7:dataLenB:length(BODYA),:);
elbowRA = BODYA(8:dataLenB:length(BODYA),:);
wristRA = BODYA(9:dataLenB:length(BODYA),:);
handRA = BODYA(10:dataLenB:length(BODYA),:);
handTipRA = BODYA(14:dataLenB:length(BODYA),:);
thumbRA = BODYA(15:dataLenB:length(BODYA),:);

eyeL = FACE(1,:);
eyeR = FACE(2,:);
nose = FACE(3,:);
mouthL = FACE(4,:);
mouthR = FACE(5,:);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5%%



%%%left arm angle calculation to determine if left arm moves in the sign%%%
armL = shoulderLA - elbowLA;
foreArmL = wristLA - elbowLA;

armLenL = mean(sqrt(armL(:,1).^2 + armL(:,2).^2 + armL(:,3).^2)); % arm length in every frame is the same
foreArmLenL = mean(sqrt(foreArmL(:,1).^2 + foreArmL(:,2).^2 + foreArmL(:,3).^2));% same arm length in every frame

angle = [];
for k = 1:length(armL)
    anglet = acos(dot(armL(k,:),foreArmL(k,:))/(armLenL*foreArmLenL))*180/pi;
    angle = [angle anglet];
end
angle_mu = mean(angle);
angle_std = std(angle);

if angle_mu > 130 && angle_std < 10
    armLFlag = 0; % left arm not moving
else
    armLFlag = 1; % left arm moving
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%% Robot's measurements %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robotShoulderLen = 25.7;
robotArmLen = 24.331;
robotForeArmLen = 24.617;
robotHandLen = 9;
robotHandTipLen = 11.113 + 6.075;

robotEyeL = [-3.9 -5.06 35.39];
robotEyeR = [-3.9 5.06 35.39];
robotNose = [-3.9 0 28.23];
robotMouthL = [-3.9 -5 24.395];
robotMouthR = [-3.9 5 24.395];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%% Initializing matrix for joints and face position %%%%%%%%%%%%%%%%
robotHandTipR = zeros(length(handTipRA),3);
robotHandR = zeros(length(handRA),3);
robotWristR = zeros(length(wristRA),3);
robotElbowR = zeros(length(elbowRA),3);

robotHandTipL = zeros(length(handTipLA),3);
robotHandL = zeros(length(handLA),3);
robotWristL = zeros(length(wristLA),3);
robotElbowL = zeros(length(elbowLA),3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% changing position of the robot's shoulder
BODYRo = BODYA;
shX = 0;
shY = robotShoulderLen;
shZ = 0;

shoulderPosLx = shX*ones(length(shoulderLA),1);
shoulderPosLy = -shY*ones(length(shoulderLA),1);
shoulderPosLz = shZ*ones(length(shoulderLA),1);
shoulderPosL = [shoulderPosLx shoulderPosLy shoulderPosLz];

shoulderPosRx = shX*ones(length(shoulderRA),1);
shoulderPosRy = shY*ones(length(shoulderRA),1);
shoulderPosRz = shZ*ones(length(shoulderRA),1);
shoulderPosR = [shoulderPosRx shoulderPosRy shoulderPosRz];

BODYRo(3:dataLenB:length(BODYRo),:) = shoulderPosL;
BODYRo(7:dataLenB:length(BODYRo),:) = shoulderPosR;
robotShoulderR = BODYRo(7:dataLenB:length(BODYRo),:);
robotShoulderL = BODYRo(3:dataLenB:length(BODYRo),:);

%% %****** Getting initial position of the hand tip***********
if handTipRA(3)>=0
    tip_eyeDelRR = handTipRA(1,:) - eyeR; % right hand tip with right eye
    tip_eyeDelRL = handTipRA(1,:) - eyeL; % right hand tip with left eye
    tip_noseDelR = handTipRA(1,:) - nose; % right hand tip with nose
    tip_mouthDelRR = handTipRA(1,:) - mouthR; % right hand tip with right mouth corner
    tip_mouthDelRL = handTipRA(1,:) - mouthL; % right hand tip with left mouth corner

    tip_eyeDelLR = handTipLA(1,:) - eyeR; % left hand tip with right eye
    tip_eyeDelLL = handTipLA(1,:) - eyeL; % left hand tip with left eye
    tip_noseDelL = handTipLA(1,:) - nose; % left hand tip with nose
    tip_mouthDelLR = handTipLA(1,:) - mouthR; % left hand tip with right mouth corner
    tip_mouthDelLL = handTipLA(1,:) - mouthL; % left hand tip with left mouth corner

    tip_eyeRR = sqrt(tip_eyeDelRR(1,1)^2 + tip_eyeDelRR(1,2)^2 + tip_eyeDelRR(1,3)^2); 
    tip_eyeRL = sqrt(tip_eyeDelRL(1,1)^2 + tip_eyeDelRL(1,2)^2 + tip_eyeDelRL(1,3)^2); 
    tip_noseR = sqrt(tip_noseDelR(1,1)^2 + tip_noseDelR(1,2)^2 + tip_noseDelR(1,3)^2);
    tip_mouthRR = sqrt(tip_mouthDelRR(1,1)^2 + tip_mouthDelRR(1,2)^2 + tip_mouthDelRR(1,3)^2);
    tip_mouthRL = sqrt(tip_mouthDelRL(1,1)^2 + tip_mouthDelRL(1,2)^2 + tip_mouthDelRL(1,3)^2);

    tip_eyeLR = sqrt(tip_eyeDelLR(1,1)^2 + tip_eyeDelLR(1,2)^2 + tip_eyeDelLR(1,3)^2); 
    tip_eyeLL = sqrt(tip_eyeDelLL(1,1)^2 + tip_eyeDelLL(1,2)^2 + tip_eyeDelLL(1,3)^2); 
    tip_noseL = sqrt(tip_noseDelL(1,1)^2 + tip_noseDelL(1,2)^2 + tip_noseDelL(1,3)^2);
    tip_mouthLR = sqrt(tip_mouthDelLR(1,1)^2 + tip_mouthDelLR(1,2)^2 + tip_mouthDelLR(1,3)^2);
    tip_mouthLL = sqrt(tip_mouthDelLL(1,1)^2 + tip_mouthDelLL(1,2)^2 + tip_mouthDelLL(1,3)^2);

    face_distance = [tip_eyeRR tip_eyeRL tip_noseR tip_mouthRR tip_mouthRL ];
    face_distance = [face_distance  tip_eyeLR tip_eyeLL tip_noseL  tip_mouthLR tip_mouthLL];

    [dis_minF, min_indexF] = min(face_distance);

    if min_indexF == 1
        eyeDel = abs(eyeL(2) - eyeR(2)); % distance between eyes
        eyeRatio = tip_eyeDelRR(2)/eyeDel; % raito of the y distance of (tip to right eye) to the eye distance
        robotEyeDel = abs(robotEyeL(2) - robotEyeR(2)); % get the eye distance of the robot
        robotHandTipR(1,:) = robotEyeR + tip_eyeDelRR; 
        robotHandTipR(1,2) = robotEyeR(2) + robotEyeDel * eyeRatio; % get the correct y component for hand tip
        robotHandTipR(1,3) = robotHandTipR(1,3) + 2; % adding 2 to get the actual hand tip position since the kinect hand tip is shorter
    end

    
    if min_indexF == 2
        eyeDel = abs(eyeL(2) - eyeR(2)); % distance between eyes
        eyeRatio = tip_eyeDelRL(2)/eyeDel; % raito of the y distance of (tip to right eye) to the eye distance
        robotEyeDel = abs(robotEyeL(2) - robotEyeR(2)); % get the eye distance of the robot
        robotHandTipR(1,:) = robotEyeL + tip_eyeDelRL; 
        robotHandTipR(1,2) = robotEyeL(2) + robotEyeDel * eyeRatio; % get the correct y component for hand tip
        robotHandTipR(1,3) = robotHandTipR(1,3) + 2; % adding 2 to get the actual hand tip position since the kinect hand tip is shorter
    end

    if min_indexF == 3
        robotHandTipR(1,:) = robotNose + tip_noseDelR; 
        robotHandTipR(1,3) = robotHandTipR(1,3) + 2; % adding 2 to get the actual hand tip position since the kinect hand tip is shorter
    end

    if min_indexF == 4
        mouthDel = abs(mouthL(2) - mouthR(2)); % distance between human mouth corners
        mouthRatio = tip_mouthDelRR(2)/mouthDel;
        robotMouthDel = abs(robotMouthL(2) - robotMouthR(2));
        robotHandTipR(1,:) = robotMouthR + tip_mouthDelRR;
        robotHandTipR(1,2) = robotMouthR(2) + robotMouthDel * mouthRatio;
        robotHandTipR(1,3) = robotHandTipR(1,3) + 2; % adding 2 to get the actual hand tip position since the kinect hand tip is shorter
    end

    if min_indexF == 5
        mouthDel = abs(mouthL(2) - mouthR(2)); % distance between human mouth corners
        mouthRatio = tip_mouthDelRL(2)/mouthDel;
        robotMouthDel = abs(robotMouthL(2) - robotMouthR(2));
        robotHandTipR(1,:) = robotMouthL + tip_mouthDelRL;
        robotHandTipR(1,2) = robotMouthL(2) + robotMouthDel * mouthRatio;
        robotHandTipR(1,3) = robotHandTipR(1,3) + 2; % adding 2 to get the actual hand tip position since the kinect hand tip is shorter
    end

    if min_indexF == 6
        eyeDel = abs(eyeL(2) - eyeR(2)); % distance between eyes
        eyeRatio = tip_eyeDelLR(2)/eyeDel; % raito of the y distance of (tip to right eye) to the eye distance
        robotEyeDel = abs(robotEyeL(2) - robotEyeR(2)); % get the eye distance of the robot
        robotHandTipL(1,:) = robotEyeR + tip_eyeDelLR; 
        robotHandTipL(1,2) = robotEyeR(2) + robotEyeDel * eyeRatio; % get the correct y component for hand tip
        robotHandTipL(1,3) = robotHandTipL(1,3) + 2; % adding 2 to get the actual hand tip position since the kinect hand tip is shorter
    end

    if min_indexF == 7
        eyeDel = abs(eyeL(2) - eyeR(2)); % distance between eyes
        eyeRatio = tip_eyeDelLL(2)/eyeDel; % raito of the y distance of (tip to right eye) to the eye distance
        robotEyeDel = abs(robotEyeL(2) - robotEyeR(2)); % get the eye distance of the robot
        robotHandTipL(1,:) = robotEyeL + tip_eyeDelLL; 
        robotHandTipL(1,2) = robotEyeL(2) + robotEyeDel * eyeRatio; % get the correct y component for hand tip
        robotHandTipL(1,3) = robotHandTipL(1,3) + 2; % adding 2 to get the actual hand tip position since the kinect hand tip is shorter
    end

    if min_indexF == 8
        robotHandTipL(1,:) = robotNose + tip_noseDelL; 
        robotHandTipL(1,3) = robotHandTipL(1,3) + 2; % adding 2 to get the actual hand tip position since the kinect hand tip is shorter
    end

    if min_indexF == 9
        mouthDel = abs(mouthL(2) - mouthR(2)); % distance between human mouth corners
        mouthRatio = tip_mouthDelLR(2)/mouthDel;
        robotMouthDel = abs(robotMouthL(2) - robotMouthR(2));
        robotHandTipL(1,:) = robotMouthR + tip_mouthDelLR;
        robotHandTipL(1,2) = robotMouthR(2) + robotMouthDel * mouthRatio;
        robotHandTipL(1,3) = robotHandTipL(1,3) + 2; % adding 2 to get the actual hand tip position since the kinect hand tip is shorter
    end

    if min_indexF == 10
        mouthDel = abs(mouthL(2) - mouthR(2)); % distance between human mouth corners
        mouthRatio = tip_mouthDelLL(2)/mouthDel;
        robotMouthDel = abs(robotMouthL(2) - robotMouthR(2));
        robotHandTipL(1,:) = robotMouthL + tip_mouthDelLL;
        robotHandTipL(1,2) = robotMouthL(2) + robotMouthDel * mouthRatio;
        robotHandTipL(1,3) = robotHandTipL(1,3) + 2; % adding 2 to get the actual hand tip position since the kinect hand tip is shorter
    end
    
else
   tip_shoulderDelRR = handTipRA(1,:) - shoulderRA(1,:); % distabce from right hand tip to right shoulder
   tip_shoulderDelRL = handTipRA(1,:) - shoulderLA(1,:); % distabce from right hand tip to left shoulder
   tip_shoulderDelLR = handTipLA(1,:) - shoulderRA(1,:); % distabce from LEFT hand tip to RIGHT shoulder
   tip_shoulderDelLL = handTipLA(1,:) - shoulderLA(1,:); % distabce from LEFT hand tip to LEFT shoulder
   
   tip_shoulderRR = sqrt(tip_shoulderDelRR(1)^2 + tip_shoulderDelRR(2)^2 + tip_shoulderDelRR(3)^2);
   tip_shoulderRL = sqrt(tip_shoulderDelRL(1)^2 + tip_shoulderDelRL(2)^2 + tip_shoulderDelRL(3)^2);
   tip_shoulderLR = sqrt(tip_shoulderDelLR(1)^2 + tip_shoulderDelLR(2)^2 + tip_shoulderDelLR(3)^2);
   tip_shoulderLL = sqrt(tip_shoulderDelLL(1)^2 + tip_shoulderDelLL(2)^2 + tip_shoulderDelLL(3)^2);
   
   tip_shoulder = [tip_shoulderRR tip_shoulderRL tip_shoulderLR tip_shoulderLL];
   
   [dis_minSH, min_indexSH] = min(tip_shoulder);
   
   if min_indexSH == 1
       shoulderDel = abs(shoulderRA(1,2) - shoulderLA(1,2));
       tip_shoulderRatioY = tip_shoulderDelRR(2)/shoulderDel;
       tip_shoulderRatioZ = tip_shoulderDelRR(3)/shoulderDel;
       
       robotShoulderDel = abs(robotShoulderR(1,2) - robotShoulderL(1,2)); % length of robot shoulder
       robotHandTipR(1,:) = robotShoulderR(1,:) + tip_shoulderDelRR(1,:);
       robotHandTipR(1,2) = robotShoulderR(1,2) + robotShoulderDel*tip_shoulderRatioY;
       robotHandTipR(1,3) = robotShoulderR(1,3) + robotShoulderDel*tip_shoulderRatioZ; 
   end
    
   if min_indexSH == 2
       shoulderDel = abs(shoulderRA(1,2) - shoulderLA(1,2));
       tip_shoulderRatioY = tip_shoulderDelRL(2)/shoulderDel;
       tip_shoulderRatioZ = tip_shoulderDelRL(3)/shoulderDel;
       
       robotShoulderDel = abs(robotShoulderR(1,2) - robotShoulderL(1,2)); % length of robot shoulder
       robotHandTipR(1,1) = robotShoulderL(1,1) + tip_shoulderDelRL(1,1);
       robotHandTipR(1,2) = robotShoulderL(1,2) + robotShoulderDel*tip_shoulderRatioY;
       robotHandTipR(1,3) = robotShoulderL(1,3) + robotShoulderDel*tip_shoulderRatioZ; 
   end

   if min_indexSH == 3
       shoulderDel = abs(shoulderRA(1,2) - shoulderLA(1,2));
       tip_shoulderRatioY = tip_shoulderDelLR(2)/shoulderDel;
       tip_shoulderRatioZ = tip_shoulderDelLR(3)/shoulderDel;
       
       robotShoulderDel = abs(robotShoulderR(1,2) - robotShoulderL(1,2)); % length of robot shoulder
       robotHandTipL(1,1) = robotShoulderR(1,1) + tip_shoulderDelLR(1,1);
       robotHandTipL(1,2) = robotShoulderR(1,2) + robotShoulderDel*tip_shoulderRatioY;
       robotHandTipL(1,3) = robotShoulderR(1,3) + robotShoulderDel*tip_shoulderRatioZ; 
   end
   
   if min_indexSH == 4
       shoulderDel = abs(shoulderRA(1,2) - shoulderLA(1,2));
       tip_shoulderRatioY = tip_shoulderDelLL(2)/shoulderDel;
       tip_shoulderRatioZ = tip_shoulderDelLL(3)/shoulderDel;
       
       robotShoulderDel = abs(robotShoulderR(1,2) - robotShoulderL(1,2)); % length of robot shoulder
       robotHandTipL(1,1) = robotShoulderL(1,1) + tip_shoulderDelLL(1,1);
       robotHandTipL(1,2) = robotShoulderL(1,2) + robotShoulderDel*tip_shoulderRatioY;
       robotHandTipL(1,3) = robotShoulderL(1,3) + robotShoulderDel*tip_shoulderRatioZ; 
   end
end
%***********************************************************


%% Right arm and hand
% computing fingertip movement difference between frames right
handTipDeltaR = diff(handTipRA);
%robotHandTipR(1,:) = [-3.8, 0, 20.73 - 11];
for k = 1:length(robotHandTipR)-1
   robotHandTipR(k+1,:) = robotHandTipR(k,:) + handTipDeltaR(k,:); 
end

% computing hand movement based on figertip movement
handDeltaR = diff(handRA);
tip_handUnit = (handTipRA - handRA); % unit vector from Tip to Hand, this is parrallel to that of the robot
tip_handUnit = tip_handUnit/mean(sqrt(tip_handUnit(:,1).^2 + tip_handUnit(:,2).^2 + tip_handUnit(:,3).^2));
robotHandR(1,:) = robotHandTipR(1,:) - robotHandTipLen*tip_handUnit(1,:);
for k = 1:length(robotHandR)-1
   robotHandR(k+1,:) = robotHandR(k,:) + handDeltaR(k,:); 
end

% computing wrist movement based on hand movement
wristDeltaR = diff(wristRA);
hand_wristUnitR = (handRA - wristRA); % unit vector from Tip to Hand, this is parrallel to that of the robot
hand_wristUnitR = hand_wristUnitR/mean(sqrt(hand_wristUnitR(:,1).^2 + hand_wristUnitR(:,2).^2 + hand_wristUnitR(:,3).^2));
robotWristR(1,:) = robotHandR(1,:) - robotHandLen*hand_wristUnitR(1,:);
for k = 1:length(robotWristR)-1
   robotWristR(k+1,:) = robotWristR(k,:) + wristDeltaR(k,:); 
end

% computing temp elbow movement
elbowDeltaR = diff(elbowRA);
wrist_elbowUnitR = (wristRA - elbowRA); % unit vector from Tip to Hand, this is parrallel to that of the robot
wrist_elbowUnitR = wrist_elbowUnitR/mean(sqrt(wrist_elbowUnitR(:,1).^2 + wrist_elbowUnitR(:,2).^2 + wrist_elbowUnitR(:,3).^2));
robotElbowR_temp(1,:) = robotWristR(1,:) - robotForeArmLen*wrist_elbowUnitR(1,:);
for k = 1:length(robotElbowR)-1
   robotElbowR_temp(k+1,:) = robotElbowR_temp(k,:) + elbowDeltaR(k,:); 
end
% %computing elbow movement from the shoulder
% elbow_shoulderUnit = (elbowRA - shoulderRA); % unit vector from elbow to shoulder, this is parrallel to that of the robot
% elbow_shoulderUnit = elbow_shoulderUnit/mean(sqrt(elbow_shoulderUnit(:,1).^2 + elbow_shoulderUnit(:,2).^2 + elbow_shoulderUnit(:,3).^2));
% robotElbowR = robotShoulderR + robotArmLen*elbow_shoulderUnit;
% 
% % calculating the change in elbow real and temp
% elbowDelR = robotElbowR - robotElbowR_temp;
% 
% for k =1:length(elbowDelR)
%    robotWristR(k,:) = robotWristR(k,:) + elbowDelR(k,:); 
%    robotHandR(k,:) = robotHandR(k,:) + elbowDelR(k,:); 
%    robotHandTipR(k,:) = robotHandTipR(k,:) + elbowDelR(k,:); 
%    
% end
% 
% 
% 
% 
% 
% elbow_wrist = robotWristR - robotElbowR;
% ewUnit = [];
% for k = 1: length(elbow_wrist)
%     ewUnitt = elbow_wrist(k,:)/sqrt(elbow_wrist(k,1).^2 + elbow_wrist(k,2).^2 + elbow_wrist(k,3).^2 );
%     ewUnit = [ewUnit; ewUnitt];
% end
% robotWristNewR = zeros(length(robotWristR),3);
% robotWristNewR(1,:) = robotElbowR(1,:) + robotArmLen*ewUnit(1,:);
% 
% wristDelR = diff(robotWristR);
% for k = 1:length(robotWristR)-1
%    robotWristNewR(k+1,:) = robotWristNewR(k,:) + wristDelR(k,:); 
% end
% 
% wrist_hand = robotHandR - robotWristR;
% whUnit = [];
% for k = 1: length(elbow_wrist)
%     whUnitt = wrist_hand(k,:)/sqrt(wrist_hand(k,1).^2 + wrist_hand(k,2).^2 + wrist_hand(k,3).^2 );
%     whUnit = [whUnit; whUnitt];
% end
% robotHandNewR = robotWristNewR + robotHandLen*whUnit;
% 
% hand_tip = robotHandTipR - robotHandR;
% htUnit = [];
% for k = 1: length(hand_tip)
%     htUnitt = hand_tip(k,:)/sqrt(hand_tip(k,1).^2 + hand_tip(k,2).^2 + hand_tip(k,3).^2 );
%     htUnit = [htUnit; htUnitt];
% end
% robotHandTipNewR = robotHandNewR + robotHandTipLen*htUnit;
% 
% 
% figure
% scatter3(robotElbowR(:,1),robotElbowR(:,2),robotElbowR(:,3),'linewidth',2)
% hold on
% plot3(robotWristNewR(:,1),robotWristNewR(:,2),robotWristNewR(:,3),'linewidth',2)
% plot3(robotHandNewR(:,1),robotHandNewR(:,2),robotHandNewR(:,3))
% plot3(robotHandTipNewR(:,1),robotHandTipNewR(:,2),robotHandTipNewR(:,3),'linewidth',2)
% 
% % 


%% elbow position
syms x y z
for k = 1:length(robotShoulderR)
    shldR = robotShoulderR(k,:);
    wrR = robotWristR(k,:);
    wrist_elbow = -(robotElbowR_temp(k,:) - robotWristR(k,:));
    shoulder_elbow = -(robotElbowR_temp(k,:) - robotShoulderR(k,:));

    c1 = cross(wrist_elbow,shoulder_elbow);

    plane = c1(1)*(x - wrR(1)) + c1(2)*(y - wrR(2)) + c1(3)*(z- wrR(3));
    eq1 = ((wrR(1) - x)^2 + ( wrR(2) - y)^2 + ( wrR(3) - z)^2) - robotForeArmLen^2;
    eq2 = ((x-shldR(1))^2 + (y - shldR(2))^2 + (z - shldR(3))^2) - robotArmLen^2;
    f = solve([plane == 0, eq1 == 0, eq2 == 0], [x y z]);

    xE = double(f.x);
    yE = double(f.y);
    zE = double(f.z);

    p1 = [xE(1) yE(1) zE(1)];
    p2 = [xE(2) yE(2) zE(2)];

    E_p1 = robotElbowR_temp(k,:) - p1;
    E_p2 = robotElbowR_temp(k,:) - p2;
    
    S_p1 = real(robotShoulderR(k,:) - p1);
    W_p1 = real(robotWristR(k,:) - p1);

    E_p1Dis = sqrt(E_p1(1)^2 + E_p1(2)^2 + E_p1(3)^2);
    E_p2Dis = sqrt(E_p2(1)^2 + E_p2(2)^2 + E_p2(3)^2);


    S_p1Dis = sqrt(S_p1(1)^2 + S_p1(2)^2 + S_p1(3)^2);
    W_p2Dis = sqrt(W_p1(1)^2 + W_p1(2)^2 + W_p1(3)^2);

    [minElDist, Ind] = min([E_p1Dis E_p2Dis]);

    if Ind == 1
        robotElbowR(k,:) = p1;
    else
        robotElbowR(k,:) = p2;
    end
    

end

%%%%% adjusting elbow position to make sure the elbow is not pushed behind
%%%%% the back


maxInd = find(robotElbowR(:,1)>=0);
maxVal = max(robotElbowR(maxInd,1));

if isempty(maxInd) == 0
    adjVal = (maxVal + 5)*ones(length(robotElbowR),1); %% 5 is safety factor
    robotElbowR(:,1) = robotElbowR(:,1) - adjVal; 
    robotWristR(:,1) = robotWristR(:,1) - adjVal;
    robotHandR(:,1) = robotHandR(:,1) - adjVal;
    robotHandTipR(:,1) = robotHandTipR(:,1) - adjVal;
    
% recalculating position of the robot elbow again to make sure length of
% arm is the same
    syms x y z
    for k = 1:length(robotShoulderR)
        shldR = robotShoulderR(k,:);
        wrR = robotWristR(k,:);
        wrist_elbow = -(robotElbowR_temp(k,:) - robotWristR(k,:));
        shoulder_elbow = -(robotElbowR_temp(k,:) - robotShoulderR(k,:));

        c1 = cross(wrist_elbow,shoulder_elbow);

        plane = c1(1)*(x - wrR(1)) + c1(2)*(y - wrR(2)) + c1(3)*(z- wrR(3));
        eq1 = ((wrR(1) - x)^2 + ( wrR(2) - y)^2 + ( wrR(3) - z)^2) - robotForeArmLen^2;
        eq2 = ((x-shldR(1))^2 + (y - shldR(2))^2 + (z - shldR(3))^2) - robotArmLen^2;
        f = solve([plane == 0, eq1 == 0, eq2 == 0], [x y z]);

        xE = double(f.x);
        yE = double(f.y);
        zE = double(f.z);

        p1 = [xE(1) yE(1) zE(1)];
        p2 = [xE(2) yE(2) zE(2)];

        E_p1 = robotElbowR_temp(k,:) - p1;
        E_p2 = robotElbowR_temp(k,:) - p2;

        S_p1 = real(robotShoulderR(k,:) - p1);
        W_p1 = real(robotWristR(k,:) - p1);

        E_p1Dis = sqrt(E_p1(1)^2 + E_p1(2)^2 + E_p1(3)^2);
        E_p2Dis = sqrt(E_p2(1)^2 + E_p2(2)^2 + E_p2(3)^2);


        S_p1Dis = sqrt(S_p1(1)^2 + S_p1(2)^2 + S_p1(3)^2);
        W_p2Dis = sqrt(W_p1(1)^2 + W_p1(2)^2 + W_p1(3)^2);

        [minElDist, Ind] = min([E_p1Dis E_p2Dis]);

        if Ind == 1
            robotElbowR(k,:) = p1;
        else
            robotElbowR(k,:) = p2;
        end


    end
end


%%%%%% recalculating wrist position 
%elbow_wristDelR = robotWristR - robotElbowR;
%% Left arm and hand
if armLFlag == 0 
    robotElbowL = repmat([0 -robotShoulderLen -robotArmLen], length(robotShoulderL),1);
    robotWristL = repmat([0 -robotShoulderLen (-robotArmLen - robotForeArmLen)], length(robotShoulderL),1);
    robotHandL = repmat([0 -robotShoulderLen (-robotArmLen - robotForeArmLen - robotHandLen)], length(robotShoulderL),1);
    robotHandTipL = repmat([0 -robotShoulderLen (-robotArmLen - robotForeArmLen - robotHandLen - robotHandTipLen)], length(robotShoulderL),1);
else
    handTipDeltaL = diff(handTipLA);
    %robotHandTipL(1,:) = [-3.83, 11, 31.73]; % ***********************************************
    for k = 1:length(robotHandTipL)-1
       robotHandTipL(k+1,:) = robotHandTipL(k,:) + handTipDeltaL(k,:); 
    end


    % computing hand movement based on figertip movement
    handDeltaL = diff(handLA);
    tip_handUnitL = (handTipLA - handLA); % unit vector from Tip to Hand, this is parrallel to that of the robot
    tip_handUnitL = tip_handUnitL/mean(sqrt(tip_handUnitL(:,1).^2 + tip_handUnitL(:,2).^2 + tip_handUnitL(:,3).^2));
    robotHandL(1,:) = robotHandTipL(1,:) - robotHandTipLen*tip_handUnitL(1,:);
    for k = 1:length(robotHandL)-1
       robotHandL(k+1,:) = robotHandL(k,:) + handDeltaL(k,:); 
    end

    % computing wrist movement based on hand movement
    wristDeltaL = diff(wristLA);
    hand_wristUnitL = (handLA - wristLA); % unit vector from Tip to Hand, this is parrallel to that of the robot
    hand_wristUnitL = hand_wristUnitL/mean(sqrt(hand_wristUnitL(:,1).^2 + hand_wristUnitL(:,2).^2 + hand_wristUnitL(:,3).^2));
    robotWristL(1,:) = robotHandL(1,:) - robotHandLen*hand_wristUnitL(1,:);
    for k = 1:length(robotWristL)-1
       robotWristL(k+1,:) = robotWristL(k,:) + wristDeltaL(k,:); 
    end

    % computing temp elbow movement
    elbowDeltaL = diff(elbowLA);
    wrist_elbowUnitL = (wristLA - elbowLA); % unit vector from Tip to Hand, this is parrallel to that of the robot
    wrist_elbowUnitL = wrist_elbowUnitL/mean(sqrt(wrist_elbowUnitL(:,1).^2 + wrist_elbowUnitL(:,2).^2 + wrist_elbowUnitL(:,3).^2));
    robotElbowL_temp(1,:) = robotWristL(1,:) - robotForeArmLen*wrist_elbowUnitL(1,:);
    for k = 1:length(robotElbowL)-1
       robotElbowL_temp(k+1,:) = robotElbowL_temp(k,:) + elbowDeltaL(k,:); 
    end
    %computing elbow movement from the shoulder
    elbow_shoulderUnitL = (elbowLA - shoulderLA); % unit vector from elbow to shoulder, this is parrallel to that of the robot
    elbow_shoulderUnitL = elbow_shoulderUnitL/mean(sqrt(elbow_shoulderUnitL(:,1).^2 + elbow_shoulderUnitL(:,2).^2 + elbow_shoulderUnitL(:,3).^2));
    robotElbowL = robotShoulderL + robotArmLen*elbow_shoulderUnitL;

    % calculating the change in elbow real and temp
    elbowDelL = robotElbowL - robotElbowL_temp;

    for k =1:length(elbowDelL)
       robotWristL(k,:) = robotWristL(k,:) + elbowDelL(k,:); 
       robotHandL(k,:) = robotHandL(k,:) + elbowDelL(k,:); 
       robotHandTipL(k,:) = robotHandTipL(k,:) + elbowDelL(k,:); 

    end
end

%% plotting
figure
scatter3(robotShoulderR(:,1),robotShoulderR(:,2),robotShoulderR(:,3),'linewidth',2)
hold on
scatter3(robotShoulderL(:,1),robotShoulderL(:,2),robotShoulderL(:,3),'linewidth',2)
scatter3(robotElbowR(:,1),robotElbowR(:,2),robotElbowR(:,3),'linewidth',2)
plot3(robotWristR(:,1),robotWristR(:,2),robotWristR(:,3),'linewidth',2)
plot3(robotHandR(:,1),robotHandR(:,2),robotHandR(:,3))
plot3(robotHandTipR(:,1),robotHandTipR(:,2),robotHandTipR(:,3),'linewidth',2)


scatter3(robotElbowL(:,1),robotElbowL(:,2),robotElbowL(:,3),'linewidth',2)
scatter3(robotWristL(:,1),robotWristL(:,2),robotWristL(:,3),'linewidth',2)
scatter3(robotHandL(:,1),robotHandL(:,2),robotHandL(:,3))
scatter3(robotHandTipL(:,1),robotHandTipL(:,2),robotHandTipL(:,3),'linewidth',2)


grid on
xlabel('x (cm)')
ylabel('y (cm)')
zlabel('z (cm)')
legend('Right shoulder','Left shoulder','Right elbow','Right wrist','right hand','Right handtip','Left elbow','Left wrist','left hand','left handtip')
title('Mapped motion to the robot (Thank you)')


%% writing to body matrix
BODYRo(8:dataLenB:length(BODYRo),:) = robotElbowR;
BODYRo(9:dataLenB:length(BODYRo),:) = robotWristR;
BODYRo(10:dataLenB:length(BODYRo),:) = robotHandR;
BODYRo(14:dataLenB:length(BODYRo),:) = robotHandTipR;

BODYRo(4:dataLenB:length(BODYRo),:) = robotElbowL;
BODYRo(5:dataLenB:length(BODYRo),:) = robotWristL;
BODYRo(6:dataLenB:length(BODYRo),:) = robotHandL;
BODYRo(12:dataLenB:length(BODYRo),:) = robotHandTipL;

%% bone length testing
we = robotWristR - robotElbowR;
sqrt(we(:,1).^2 + we(:,2).^2 + we(:,3).^2)

we = robotWristR - robotHandR;
sqrt(we(:,1).^2 + we(:,2).^2 + we(:,3).^2);
we = robotShoulderR - robotElbowR;
sqrt(we(:,1).^2 + we(:,2).^2 + we(:,3).^2)

%% exporting
xlswrite('Hello_Mapped.xlsx', BODYRo)
