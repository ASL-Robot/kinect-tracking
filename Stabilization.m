%[num,txt,raw] = xlsread('Data.xlsx');
[num,txt,raw] = xlsread('data.xlsx',3);

Nan = [NaN NaN NaN];
num = [Nan; Nan; num];

dataLen = 34;
newData = zeros(length(num), 3); % 3 columns
for k = 1:length(num)/dataLen;
    origin = num((k-1)*dataLen+21,:);
    datBlok = num((k-1)*dataLen+1:k*dataLen,:);
    newData((k-1)*dataLen+1:k*dataLen,:) = bsxfun(@minus,datBlok, origin);
    
end
 
xdata = newData(:,3);
ydata = newData(:,1);
zdata = newData(:,2);

newDataA = [xdata ydata zdata]; % data matrix with adjusted coordinates

%% getting positions of facial features

%%
% joints coordinates
spineSh = newDataA(21:34:length(num),:); % shoulder spine
leftSH = newDataA(5:34:length(num),:); % shoulder left
rSH =  newDataA(9:34:length(num),:); % shoulder right
rEl =  newDataA(10:34:length(num),:); % elbow right
rWr =  newDataA(11:34:length(num),:); % wrist right
rHa =  newDataA(12:34:length(num),:); % hand right
rTip =  newDataA(24:34:length(num),:); % right hand tip
nose = newDataA(31:34:length(newDataA),:);
leftMo = newDataA(32:34:length(newDataA),:);
rightMo = newDataA(33:34:length(newDataA),:);

std(rEl)
% rTip =  newDataA(24:34:length(num),:); % right hand tip
% rTip = smooth(rTip,'rloess');
% scatter3(rTip(1:41,1),rTip(42:82,1),rTip(83:123,1))
% plot3(rTip(1:41,1),rTip(42:82,1),rTip(83:123,1))


rightHanfDelta = rHa - rWr;
handLenArr =[];
for k = 1:length(newDataA)/dataLen
    %leftHandLen = sqrt(leftHandDelta(k,1)^2 + leftHandDelta(k,2)^2 + leftHandDelta(k,3)^2); 
    
    rightHandLen = sqrt(rightHanfDelta(k,1)^2 + rightHanfDelta(k,2)^2 + rightHanfDelta(k,3)^2); 
    
    handLenArr = [handLenArr rightHandLen];
end

handLen = mean(handLenArr);


plot3(rWr(:,1), rWr(:,2), rWr(:,3))
hold on
plot3(rTip(:,1),rTip(:,2),rTip(:,3))
plot3(rHa(:,1),rHa(:,2),rHa(:,3))

grid on

%% hand Tip stabilizing
nl = 15;
handTipp = newDataA(24:34:length(newDataA),:);
handTip = newDataA(24:34:34*nl,:);
dat = [];
for k = 1:length(handTip)-1
    n = (handTip(k,2) + handTip(k+1,2))/2;
    dat = [dat; n];
end
dat = [dat ;handTip(length(handTip),2)];
dat2 = dat;    
for i = 1:30
    dat1 = [];
    for k = 1:length(handTip)-1
        n = (dat2(k) + dat2(k+1))/2;
        dat1 = [dat1; n];
    end
    dat1 = [dat1 ;dat(length(handTip))];
    dat2 = dat1;
end
datyyTip = dat2;

% x
datx1 = [];
for k = 1:length(handTip)-1
    n = (handTip(k,1) + handTip(k+1,1))/2;
    datx1 = [datx1; n];
end
datx1 = [datx1 ;handTip(length(handTip),1)];
dat2x = datx1;    

for i = 1:10
    dat1x = [];
    for l = 1:length(handTip)-1
        n = (dat2x(l) + dat2x(l+1))/2;
        dat1x = [dat1x; n];
    end
    dat1x = [datx1 ];
    dat2x = dat1x;
end
datxxTip = dat2x;

% stabilizing good portion
handTipG = newDataA(24+34*nl:34:length(newDataA),:);
dat = [];
for k = 1:length(handTipG)-1
    n = (handTipG(k,2) + handTipG(k+1,2))/2;
    dat = [dat; n];
end
dat = [dat ;handTipG(length(handTipG),2)];
dat2 = dat;    
for i = 1
    dat1 = [];
    for k = 1:length(handTipG)-1
        n = (dat2(k) + dat2(k+1))/2;
        dat1 = [dat1; n];
    end
    dat1 = [dat1 ;dat(length(handTipG))];
    dat2 = dat1;
end
datyyTipG = dat2;

% x
datx1 = [];
for k = 1:length(handTipG)-1
    n = (handTipG(k,1) + handTipG(k+1,1))/2;
    datx1 = [datx1; n];
end
datx1 = [datx1 ;handTipG(length(handTipG),1)];
dat2x = datx1;    

for i = 1:10
    dat1x = [];
    for l = 1:length(handTipG)-1
        n = (dat2x(l) + dat2x(l+1))/2;
        dat1x = [dat1x; n];
    end
    dat1x = [datx1 ];
    dat2x = dat1x;
end
datxxTipG = dat2x;
%
handTipNewa = [[datxxTip datyyTip handTip(:,3)] ; [datxxTipG datyyTipG handTipG(:,3)]];

plot3(handTipp(:,1), handTipp(:,2), handTipp(:,3))
hold on
plot3(handTipNewa(:,1), handTipNewa(:,2), handTipNewa(:,3),'--r')
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
grid on
title('Hand tip movement smoothing (Thank you)(Top View)')

plot3(nose(:,1), nose(:,2), nose(:,3),'c')
plot3(leftMo(:,1), leftMo(:,2), leftMo(:,3))
plot3(rightMo(:,1), rightMo(:,2), rightMo(:,3))

legend('Original motion','Smoothed motion','nose','left mouth corner','right mouth corner')

% %% wrist stabilizing
% % joints coordinates
% spineSh = newDataA(21:34:length(num),:);
% leftSH = newDataA(5:34:length(num),:); 
% rSH =  newDataA(9:34:length(num),:);
% rEl =  newDataA(10:34:length(num),:);
% rWr =  newDataA(11:34:length(num),:);
% rHa =  newDataA(12:34:length(num),:);
% rTip =  newDataA(24:34:length(num),:);
% 
% rWr =  newDataA(11:34:length(num),:);
% dat = [];
% for k = 1:length(rWr)-1
%     n = (rWr(k,2) + rWr(k+1,2))/2;
%     dat = [dat; n];
% end
% dat = [dat ;rWr(length(rWr),2)];
% dat2 = dat;    
% for i = 1:30
%     dat1 = [];
%     for k = 1:length(rWr)-1
%         n = (dat2(k) + dat2(k+1))/2;
%         dat1 = [dat1; n];
%     end
%     dat1 = [dat1 ;dat(length(rWr))];
%     dat2 = dat1;
% end
% plot3(rWr(:,1),dat2, rWr(:,3))
% datyyWr = dat2;
% 
% % x
% datx1 = [];
% for k = 1:length(rWr)-1
%     n = (rWr(k,1) + rWr(k+1,1))/2;
%     datx1 = [datx1; n];
% end
% datx1 = [datx1 ;rWr(length(rWr),1)];
% dat2x = datx1;    
% 
% for i = 1:30
%     dat1x = [];
%     for l = 1:length(rWr)-1
%         n = (dat2x(l) + dat2x(l+1))/2;
%         dat1x = [dat1x; n];
%     end
%     dat1x = [datx1 ];
%     dat2x = dat1x;
% end
% datxxWr = dat2x;
% %scatter3(datxxWr,datyyWr, rWr(:,3))
% plot3(datxxWr,datyyWr, rWr(:,3))
% %plot3(rWr(:,1), rWr(:,2), rWr(:,3))
% hold on
% xlabel('x')
% ylabel('y')
% 
