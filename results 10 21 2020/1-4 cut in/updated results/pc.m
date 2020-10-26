function new_num = pc(num, n)%precision control
b = num*10^n;
aa = round(b);
new_num = aa/10^n;
end