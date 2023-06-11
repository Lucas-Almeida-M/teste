clear all
clc
close all
figure
x = 0:1:30;
y = 0:1:30;
hold on
for n = 1:numel(x); %// loop over vertical lines
    plot([x(n) x(n)], [y(1) y(end)], 'k-'); %// change 'k-' to whatever you need
end
hold on
for n = 1:numel(y); %// loop over horizontal lines
    plot([x(1) x(end)], [y(n) y(n)], 'k-'); %// change 'k-' to whatever you need
end
hold on;
