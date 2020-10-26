function d = d2poly(P, vertices)
% d2poly: calculates the nearest distance from a vector of points
% to a polygon
%
% input
% =====
% p:        point that the distance is calculated from
%           size(p) = 2xk (typically 2x4)
% vertices: a set of vertices forming the polygon
%           size(vertices) = 2xN 
%           i.e. rectange -> 2x4 (repeat the first vertice not necessary)
% output
% ======
% dist:     nearest distance (vector)
%
% distance is positive if the point is outside the polygon
% if the intersection number by drawing a horizontal line is even, then the
% point is outside the polygon

k   = size(P, 2);
n   = size(vertices, 2);
dist = 10000;

for i = 1:k
    count = 0;
    p = P(:,i);
    for j = 1:n
       a = vertices(:,mod(j-1, n)+1);
       b = vertices(:,mod(j  , n)+1);
       
       if ( (p(2)-a(2))*(p(2)-b(2)) < 0 && ...
           p(1)<(b(1)-a(1))*(p(2)-a(2))/(b(2)-a(2))+a(1))
           count = count + 1;
       end
       d = pointToSegmentDist(p,a,b);
    end
    if mod(count,2) == 1 ... if the point is in the polygon dist is negative
        d = -d;
    end
    dist = min(dist, d);
end
end

function dist = pointToSegmentDist(p, a, b) 
% pointToSegmentDist: calculate the distance of P to a line segment AB

x1 = a(1); y1 = a(2);
x2 = b(1); y2 = b(2);
px = p(1); py = p(2);

AB = [x2 - x1; y2 - y1];
AP = [px - x1; py - y1];

AB_AP = AB' * AP;
distAB2 = AB' * AB;

D = a;

if distAB2 ~= 0
   t = AB_AP/distAB2;
   if (t > 1)
       D = b;
   elseif (t > 0)
       D = [x1 + AB(1)*t; y1 + AB(2) * t];
   else
       D = a;
   end
end

PD = [D(1) - px; D(2) - py];
    
dist = sqrt(PD'*PD);

end