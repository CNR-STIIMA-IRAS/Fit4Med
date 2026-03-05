
function make_cart_trj3_from_points(P, yamlOut, doPlot)
% make_cart_trj3_from_points
%
% - No YAML input file
% - Keeps EXACT YAML structure provided
% - Updates only:
%     end_config
%     max_velocity
%     total_time
%     cart_positions
%     time_from_start
% - 60 samples (30 go + 30 return)
% - Bell-shaped timing
% - Strictly increasing time vector
%
% Usage:
% P = [0 0 0;
%      0.02 0.10 0.25;
%      0.05 0.18 0.40;
%      0.0165 0.2599 0.5650];
%
% make_cart_trj3_from_points(P,'out.yaml',true);

arguments
    P (:,3) double
    yamlOut (1,:) char = 'out.yaml'
    doPlot (1,1) logical = false
end

if size(P,1) < 2
    error('At least 2 waypoints required.');
end

if norm(P(1,:) - [0 0 0]) > 1e-12
    error('First waypoint must be [0 0 0] to match template begin_config.');
end

%% PARAMETERS
nGo = 30;
nRet = 30;
nTot = nGo + nRet;
Ttot = 3.0;
Tgo  = Ttot/2;

%% ===== SPLINE WITH CHORD-LENGTH PARAM =====
d = sqrt(sum(diff(P,1,1).^2,2));
u_nodes = [0; cumsum(d)];
u_nodes = u_nodes / u_nodes(end);

ppX = spline(u_nodes, P(:,1));
ppY = spline(u_nodes, P(:,2));
ppZ = spline(u_nodes, P(:,3));

%% ===== ARC-LENGTH SAMPLING =====
uDense = linspace(0,1,2000)';
Xd = ppval(ppX,uDense);
Yd = ppval(ppY,uDense);
Zd = ppval(ppZ,uDense);
Pd = [Xd Yd Zd];

dseg = sqrt(sum(diff(Pd,1,1).^2,2));
L = [0; cumsum(dseg)];
L = L / L(end);

[L_unique, ia] = unique(L,'stable');
u_unique = uDense(ia);

fGo  = linspace(0,1,nGo)';
fRet = linspace(1,0,nRet+1)';
fRet = fRet(2:end);
fAll = [fGo; fRet];

uAll = interp1(L_unique,u_unique,fAll,'linear');

cart_positions = [ppval(ppX,uAll), ppval(ppY,uAll), ppval(ppZ,uAll)];

%% ===== BELL-SHAPED TIME LAW =====
inv_sigma = @(s) (Tgo/pi) * acos(max(-1,min(1, 1 - 2*s)));

tGo = arrayfun(inv_sigma, fGo);
prRet = linspace(0,1,nRet+1)';
prRet = prRet(2:end);
tRet = Tgo + arrayfun(inv_sigma, prRet);

tAll = [tGo; tRet];

if any(diff(tAll) <= 0)
    error('Time vector not strictly increasing.');
end

tAll(end) = Ttot;

%% ===== MAX VELOCITY =====
dseg2 = sqrt(sum(diff(cart_positions,1,1).^2,2));
L2 = [0; cumsum(dseg2)];
v = gradient(L2, tAll);
max_velocity = max(v);

end_config = P(end,:);

%% ===== WRITE YAML EXACT STRUCTURE =====
fid = fopen(yamlOut,'w');

fprintf(fid,'a_movement_definition:\n');
fprintf(fid,'  begin_config:\n');
fprintf(fid,'  - - 0.0\n');
fprintf(fid,'    - 0.0\n');
fprintf(fid,'    - 0.0\n');
fprintf(fid,'  begin_joint_config:\n');
fprintf(fid,'  - - 0.0\n');
fprintf(fid,'    - 0.0\n');
fprintf(fid,'    - 0.0\n');

fprintf(fid,'  end_config:\n');
fprintf(fid,'  - - %.15g\n', end_config(1));
fprintf(fid,'    - %.15g\n', end_config(2));
fprintf(fid,'    - %.15g\n', end_config(3));

fprintf(fid,'  max_velocity:\n');
fprintf(fid,'  - %.15g\n', max_velocity);

fprintf(fid,'  side:\n');
fprintf(fid,'  - 2\n');

fprintf(fid,'  total_time:\n');
fprintf(fid,'  - %.15g\n', Ttot);

fprintf(fid,'  type:\n');
fprintf(fid,'  - 2\n');

fprintf(fid,'  vel_profile:\n');
fprintf(fid,'  - 2\n');

fprintf(fid,'cart_trj3:\n');
fprintf(fid,'  cart_positions:\n');

for i=1:nTot
    fprintf(fid,'  - - %.15g\n', cart_positions(i,1));
    fprintf(fid,'    - %.15g\n', cart_positions(i,2));
    fprintf(fid,'    - %.15g\n', cart_positions(i,3));
end

fprintf(fid,'  joint_names:\n');
fprintf(fid,'  - joint_1\n');
fprintf(fid,'  - joint_2\n');
fprintf(fid,'  - joint_3\n');

fprintf(fid,'  time_from_start:\n');
for i=1:nTot
    fprintf(fid,'  - - %.15g\n', tAll(i));
end

fclose(fid);

if doPlot
    figure; plot3(cart_positions(:,1),cart_positions(:,2),cart_positions(:,3),'LineWidth',1.5);
    hold on; plot3(P(:,1),P(:,2),P(:,3),'ko');
    grid on; axis equal; view(3);
end

end
