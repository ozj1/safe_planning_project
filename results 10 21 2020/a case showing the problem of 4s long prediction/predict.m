function xhat = predict(f, A, x0, h)
xhat = x0;
xnew = x0;
for i = 1:h
    xnew = f(A, xnew);
    xhat = [xhat, xnew];
end
end
