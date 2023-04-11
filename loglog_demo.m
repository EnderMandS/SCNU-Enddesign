clear

% y = a * x^b
% log(y) = log(a) + b * log(x)

a=5; %截距
b=1; %斜率

x = linspace(0.01,1000);
y = a*x.^b;

% b=100;
% x2 = linspace(20,10000);
% y2 = x2*0+b;

% x = pi*(0.01:0.01:10);
% 
% fun = @(x1) -cos(x1)./x1;
% ci2x = zeros(size(x));
% ci4x = zeros(size(x));
% for i=1:length(x)
%     ci2x(i) = integral(fun,2*x(i),Inf,'RelTol',1e-8,'AbsTol',1e-8);
% end
% for i=1:length(x)
%     ci4x(i) = integral(fun,4*x(i),Inf,'RelTol',1e-8,'AbsTol',1e-8);
% end
% 
% y = sqrt( 2/pi.* ( log(2) - (((sin(x)).^3)/(2.*(x.^2)) .*(sin(x)+4.*x.*cos(x))) + ...
%     ci2x - ci4x));

loglog(x,y)
hold on;
% loglog(x2,y2)
xline(sqrt(2))
xlabel('T')
ylabel('\sigma(T)')
text(10,10,'斜率 = 1')
grid on;



