function plotlake(Depth,fignr,water)
% Depth:     A topological map
% fignr:     Figure # to plot in
% water:     Watersurface on (1) or off (2)

figure(fignr)
clf(fignr)
x = 1:size(Depth,1);
y = 1:size(Depth,2);
if water
    W = 0.3*sin(0.2*x')*sin(.2*y) + 0.15*sin(0.5*x')*sin(.5*y) + 0.2*sin(0.7*x')*sin(.7*y);
    W = 0.2*W/(max(max(W)));
    figure(fignr)
    surfl(W);
    shading interp;
    colormap(gray);
    alpha(0.5)
    hold on
end
surfl(Depth);
shading interp;
colormap(gray);