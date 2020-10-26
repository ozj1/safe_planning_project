
%cost =  q1_road * exp(q2_road * (-6 - Xi(2)));
close all;
x= linspace(-100,20,100);
q1_road=1;
q2_road= linspace(10,20,100);
 cost1 =  q1_road .* exp(q2_road .* (-3 - 6));
 cost2 = cost1+ q1_road .* exp(q2_road .* (-6 - (-3)));
 plot(x,cost2)
%  hold on
%   plot(q2_road,cost2)



