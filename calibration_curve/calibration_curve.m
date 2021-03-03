
%% Выгружаем данные из файла
file_name = 'log2.mat';
load(file_name, 'data_x')
% data_x = dlmread(file_name,';', 0, 0);
% save(file_name,'data_x','-v7.3');
pressure = data_x(:,1);
adc = data_x(:,2);
pressure_temp = [];
adc_temp = [];
for i=2:size(adc, 1)
    if adc(i) < 0
        adc(i) = 32767 + (32767 + adc(i));
    end
    %if not(pressure(i) == 0.0) && (pressure(i) < 103000) && not(pressure(i) == pressure(i-1))
    if not(pressure(i) == 0.0) && (pressure(i) < 103000)
        pressure_temp = [pressure_temp; pressure(i)];
        adc_temp = [adc_temp; adc(i)];
    end
end
return;
pressure = sort(pressure_temp);
adc = sort(adc_temp);
pressure_temp = [];
adc_temp = [];
for i=2:size(adc, 1)
    if not(pressure(i) == pressure(i-1))
        pressure_temp = [pressure_temp; pressure(i)];
        adc_temp = [adc_temp; adc(i)];
    end
end
pressure = pressure_temp;
adc = adc_temp;
plot(pressure, adc);
