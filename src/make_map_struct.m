function map = make_map_struct(Depth, xscale, yscale, harbour)
%MAKE_MAP_STRUCT  Pack bathymetry map into a struct.
% Depth: Ny x Nx, land=0, water<0
% xscale: 1 x Nx
% yscale: 1 x Ny
% harbour: 1 x 2 [x_home, y_home]

map.Depth   = double(Depth);
map.xscale  = double(xscale(:)');  % 1xNx
map.yscale  = double(yscale(:)');  % 1xNy
map.harbour = double(harbour(:)'); % 1x2

Ny = size(map.Depth,1);
Nx = size(map.Depth,2);
assert(numel(map.xscale)==Nx && numel(map.yscale)==Ny, 'Depth size mismatch with xscale/yscale.');

map.waterMask = (map.Depth < 0);
map.xmin = min(map.xscale); map.xmax = max(map.xscale);
map.ymin = min(map.yscale); map.ymax = max(map.yscale);

% grid step (assume near-uniform)
if Nx >= 2, map.dx = map.xscale(2) - map.xscale(1); else, map.dx = 0; end
if Ny >= 2, map.dy = map.yscale(2) - map.yscale(1); else, map.dy = 0; end
end
