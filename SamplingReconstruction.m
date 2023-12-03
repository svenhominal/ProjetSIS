% Sampling/reconstruction/aliasing/filtering demo
%
%

function SamplingReconstruction
close all;
%Temporal limits for the example
limit=1;
dataFrequency=1000;
t=-limit:1/dataFrequency:limit;


%Definition of sinusoid components (amplitude + frequency)
%-------------------
%%%%%%%%%%%%%%%%%%%%
%Parameters to modify
%%%%%%%%%%%%%%%%%%%%
f(1)=8 ;a(1)=1;
%f(2)=7;a(2)=1;
%%%%%%%%%%%%%%%%%%%%
%-------------------

%Create the final signal from the different components
clear('y');
y=zeros(1,length(t));
for i=1:length(f)
    y=y+sin(2*pi*f(i)*t)*a(i);
end
%Plot Original Signal
temp=figure('Name','Original signal','NumberTitle','off');
resizeFigure(temp);
subplot(2,2,1); grid on;
plot(t,y); grid on;
xlabel('Time [s]');
title('Original signal');

%Compute Fourier Transform and plot it
%-------------------
%%%%%%%%%%%%%%%%%%%%
%Parameters to modify
%%%%%%%%%%%%%%%%%%%%
%Maximal frequency to display
Fmax=25;
%%%%%%%%%%%%%%%%%%%%
%-------------------
subplot(2,2,3); grid on;
fty=abs(fft(y)/length(y));
f=dataFrequency/2*linspace(0,1,round(length(y)/2));
fty=fty(1:length(f));
maxIndex=find(f>Fmax,1);
plot([-f(maxIndex:-1:2) f(1:maxIndex)], [fty(maxIndex:-1:2) fty(1:maxIndex)]); grid on;
xlabel('Frequency [Hz]');
title('FFT');

%Filter signal
%-------------------
%%%%%%%%%%%%%%%%%%%%
%Parameters to modify
%%%%%%%%%%%%%%%%%%%%
% filterType can be:
%   none: no filter
%   simplelow: first order low pass filter
%   bwlow: Butterworth low pass filter
%   bwhigh: Butterworth high pass filter
% filterType='simplelow';
% filterType='bwlow';
% filterType='bwhigh';
filterType='none';
Fc=3; %Filter cutoff frequency
filterOrder=3; % Only for Butterworth filter
%%%%%%%%%%%%%%%%%%%%
%-------------------

%Compute filter;
switch(filterType)
    case 'simplelow'
        %Simple first order low-pass filter
        % http://preview.web-ee.com/tutorials/digital_filters/1st_Order_Digital_Filter_Designer/
        k=exp(-Fc*pi/dataFrequency);
        b=[(1-k)];
        a=[1 -k];
        wintitle=sprintf('Bode plot: First order filter, cutoff %.2f Hz', Fc);
    case 'bwlow'
        [b,a]=butter(filterOrder,Fc/dataFrequency,'low');
        wintitle=sprintf('Bode plot: Butterworth low pass filter order %d, cutoff %.2f Hz', filterOrder, Fc);
    case 'bwhigh'
        [b,a]=butter(filterOrder,Fc/dataFrequency,'high');
        wintitle=sprintf('Bode plot: Butterworth high pass filter order %d, cutoff %.2f Hz', filterOrder, Fc);
    case 'none'
        %do nothing
    otherwise
        error('The filter must be chosen between ''none'',''simple'' and ''butterworth''');
end

%Display filter response
if (not(strcmp(filterType,'none')))
    figure('Name',wintitle,'NumberTitle','off');
    freqz(b,a,512,dataFrequency);
    ax = findall(gcf, 'Type', 'axes');
    set(ax, 'XScale', 'log');
end



%Filter the signal if a filter is used
if (not(strcmp(filterType,'none')))
    y=filter(b,a,y);
    %Display Filtered Signal
    figure(temp);
    subplot(2,2,2); 
    plot(t,y); grid on;
    xlabel('Time [s]');
    title('Filtered signal');
    %Compute Fourrier Transform of Filtered Signal
    subplot(2,2,4);
    fty=abs(fft(y)/length(y));
    f=dataFrequency/2*linspace(0,1,round(length(y)/2));
    fty=fty(1:length(f));
    maxIndex=find(f>Fmax,1);
    plot([-f(maxIndex:-1:2) f(1:maxIndex)], [fty(maxIndex:-1:2) fty(1:maxIndex)]); grid on;
    %A=axis;
    %A(1)=-1;
    %axis(A);
    xlabel('Frequency [Hz]');
    title('FFT');
end


%Array of the tested sampling frequencies
%-------------------
%%%%%%%%%%%%%%%%%%%%
%Parameters to modify
%%%%%%%%%%%%%%%%%%%%
fs=[10 20]; %You can specify multiple sampling frequencies to observe their influence
%%%%%%%%%%%%%%%%%%%%
%-------------------
%Loop for different sampling frequencies
for i=1:length(fs)
    wintitle=sprintf('Sampling frequency %d Hz',fs(i));
    figure('Name',wintitle,'NumberTitle','off');
    resizeFigure(gcf);    %Create a wider plot
       
    %Resample the signal at the desired frequency
    [ys,ts]=sampling(t,y,fs(i));
    %Plot signal on the left;
    subplot(2,2,1);      
    stem(ts,ys,'o');    grid on;
    title(['Sampling frequency ' num2str(fs(i)) ' Hz']);  
    xlabel('Time [s]');
       
    %Signal reconstruction
    tr=t;
    
    %%%%%%%%%%%%%%%%%%%%
    %Parameters to modify
    %%%%%%%%%%%%%%%%%%%%
    % can be either linearSignalReconstruction or wsSignalReconstruction
    yr=wsSignalReconstruction(tr,ts,ys);
    % yr=linearSignalReconstruction(tr,ts,ys);
    %%%%%%%%%%%%%%%%%%%%
    %-------------------
    
    %Plot reconstructed signal on the right    
    subplot(2,2,2);        
    plot(tr,yr); grid on;
    title(['Reconstructed signal @ ' num2str(fs(i)) ' Hz']);    
    xlabel('Time [s]');   
    %Compute Fourrier Transform of Filtered Signal
    subplot(2,2,4);
    fty=abs(fft(yr)/length(yr));
    f=dataFrequency/2*linspace(0,1,round(length(yr)/2));
    fty=fty(1:length(f));
    maxIndex=find(f>Fmax,1);   
    plot([-f(maxIndex:-1:2) f(1:maxIndex)], [fty(maxIndex:-1:2) fty(1:maxIndex)]); grid on;
    xlabel('Frequency [Hz]');
    title('FFT');
    
end




%Function to resample the signal
function [ys,ts]=sampling(t,y,Fs)
originalFrequency=1/(t(2)-t(1));
if(5*Fs>originalFrequency)
    error('Sampling frequency must be at least 5 times smaller than the original frequency');
end
step=round(originalFrequency/Fs);
ts=t(1:step:end);
ys=y(1:step:end);

%Function to reconstruct the signal
function yr=wsSignalReconstruction(tr,ts,ys)
%tr      Resulting timeline
%ts     Sampled timeline
%ys     Sampled signal
if(length(tr)<5*length(ts))
    error('Reconstructed signal must have a frequency 5 times bigger than the sampling frequency');
end
Ts=(ts(2)-ts(1));
yr=zeros(1,length(tr));
for j=1:length(tr)
    for k=1:length(ys)        
        shift=tr(j)-ts(k);
        yr(j)=yr(j)+sinc(shift/Ts)*ys(k);
    end
end

%Function to reconstruct the signal
function yr=linearSignalReconstruction(tr,ts,ys)
%tr      Resulting timeline
%ts     Sampled timeline
%ys     Sampled signal
if(length(tr)<5*length(ts))
    error('Reconstructed signal must have a frequency 5 times bigger than the sampling frequency');
end
k=0;
Ts=(ts(2)-ts(1));
Tr=(tr(2)-tr(1));
yr=zeros(1,length(tr));
for j=1:length(ts)-1
    for k=k+1:k+round(Ts/Tr)
        yr(k)=ys(j)+(ys(j+1)-ys(j))*(tr(k)-ts(j))/Ts;
    end
end

%First order low pass filter
%http://preview.web-ee.com/tutorials/digital_filters/1st_Order_Digital_Filter_Designer/
function yf=lowPassFilter(y,dataFrequency,Fc)
k=exp(-Fc*pi/dataFrequency);
yf(1)=(1-k)*y(1);
for i=2:length(y)
    yf(i)=(1-k)*y(i)+k*yf(i-1);
end

function resizeFigure(h)
position=get(h,'Position');
position(1)=100;
position(2)=200;
position(3)=position(3)*1.2;
position(4)=position(4)*1.2;
set(h,'Position',position);



        
