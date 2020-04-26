function y= fcn(u)
fid = fopen('./resources/prog/temp.txt','w');
fprintf(fid,'%d\n',u);
fclose(fid);
system("python C:\Users\Dell\Desktop\BTP2\resources\prog\fcn.py");
fop = fopen('./resources/prog/angle.txt','r');
a = fscanf(fop,'%f\n');
fclose(fop);
y=a;




