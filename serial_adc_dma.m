clear all
clc
close all
delete(instrfind);
arduino=serial('COM7','BaudRate',9600);
 
fopen(arduino);
 
x=linspace(1,1000,1000);

%fprintf(arduino,'%s','nu')

k = 1;
for i=1:length(x)
    y1(i) = fscanf(arduino,'%f , %f , %f , %f');
    y2(i) = fscanf(arduino,'%f , %f , %f , %f');
    y3(i) = fscanf(arduino,'%f , %f , %f , %f');
    tiempo(i) = fscanf(arduino,'%f , %f , %f , %f');
    xx(i) = x(i);
    plot(tiempo,y1);
    hold on
    plot(tiempo,y2);
    plot(tiempo,y3);
    ylim([0 260])
    xlim_inf = tiempo(i)-1;
    xlim_sup = tiempo(i)+1;
    xlim([xlim_inf xlim_sup])
    title('data acquisition from STM32F407VG , 2 ADC and 1 digital counter')
    drawnow
end
	
fclose(arduino);
disp('making plot..')
figure(2)
plot(tiempo,y1);
hold on
plot(tiempo,y2);
hold on
plot(tiempo,y3);
legend('adcValueX','adcValueX','Amp')
instrfind