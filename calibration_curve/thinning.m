pressure_temp = [];
adc_temp = [];

%����������� �����
cp_1 = 0; %�� 10 ��
cp_2 = 0; %�� 100 ��
cp_3 = 0; %�� 200 ��
cp_4 = 0; %�� 1000 ��
cp_5 = 0; %�� 10000 ��
cp_6 = 0; %�� 70000 ��
for i=1:size(pressure,1)
    if (pressure(i) >= 10) && (cp_1 == 0)
        cp_1 = i;
    end
    if (pressure(i) >= 100) && (cp_2 == 0)
        cp_2 = i;
    end
    if (pressure(i) >= 200) && (cp_3 == 0)
        cp_3 = i;
    end
    if (pressure(i) >= 1000) && (cp_4 == 0)
        cp_4 = i;
    end
    if (pressure(i) >= 10000) && (cp_5 == 0)
        cp_5 = i;
    end
    if (pressure(i) >= 70000) && (cp_6 == 0)
        cp_6 = i;
    end
end
for i=1:20:cp_1-1
    pressure_temp = [pressure_temp; pressure(i)];
    adc_temp = [adc_temp; adc(i)];
end
for i=cp_1:17:cp_2-1
    pressure_temp = [pressure_temp; pressure(i)];
    adc_temp = [adc_temp; adc(i)];
end
for i=cp_2:15:cp_3-1
    pressure_temp = [pressure_temp; pressure(i)];
    adc_temp = [adc_temp; adc(i)];
end
for i=cp_3:12:cp_4-1
    pressure_temp = [pressure_temp; pressure(i)];
    adc_temp = [adc_temp; adc(i)];
end
for i=cp_4:9:cp_5-1
    pressure_temp = [pressure_temp; pressure(i)];
    adc_temp = [adc_temp; adc(i)];
end
for i=cp_5:1:cp_6-1
    pressure_temp = [pressure_temp; pressure(i)];
    adc_temp = [adc_temp; adc(i)];
end
for i=cp_6:1:size(pressure,1)
    pressure_temp = [pressure_temp; pressure(i)];
    adc_temp = [adc_temp; adc(i)];
end
plot(adc_temp, pressure_temp);