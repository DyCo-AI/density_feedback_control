function plan_filt = filter_ee_plan()
%% get parameters
addpath(genpath('saved_data'))
load('planarKINOVA_ee_plan.mat');
x = ee_plan.x;
X = x(:,1); Y = x(:,2);
L = length(X);
Fs = 100; % sampting freq = 1/euler_stepsize

%% do FFT
% X_fft = fft(X); Y_fft = fft(Y);
% f = Fs*(0:(L/2))/L;
% 
% P2X = abs(X_fft/L);
% P1X = P2X(1:L/2+1);
% P1X(2:end-1) = 2*P1X(2:end-1);
% 
% P2Y = abs(Y_fft/L);
% P1Y = P2Y(1:L/2+1);
% P1Y(2:end-1) = 2*P1Y(2:end-1);
% 
% %plot FFT
% %figure()
% %plot(f,P1X) 
% %title("Single-Sided Amplitude Spectrum of X(t)")
% %xlabel("f (Hz)")
% %ylabel("|P1X(f)|")
% %figure()
% %plot(f,P1Y) 
% %title("Single-Sided Amplitude Spectrum of Y(t)")
% %xlabel("f (Hz)")
% %ylabel("|P1Y(f)|")
% 
% % plot filtered signal
% X_filt = lowpass(X,2,Fs);
% Y_filt = lowpass(Y,2,Fs);
% 
% figure()
% plot(X); plot(Y);
% plot(X_filt); plot(Y_filt);

%% do moving average
planarKINOVA = 1;
if(planarKINOVA)
    wts = [1/100; repmat(1/5,5,1); 1/100];
else
    wts = [1/100; repmat(1/5,5,1/5); 1/100];
end

X_movAvg = conv(X,wts,'valid');
Y_movAvg = conv(Y,wts,'valid');

% plots
% figure()
% hold on;
% plot(X); plot(Y);
% %plot(X_filt); plot(Y_filt);
% plot(X_movAvg);plot(Y_movAvg);
% legend('x','y','x filt','y filt')
% xlabel('time'); ylabel('position x,y')

plan_filt.x = X_movAvg;
plan_filt.y = Y_movAvg;
end
