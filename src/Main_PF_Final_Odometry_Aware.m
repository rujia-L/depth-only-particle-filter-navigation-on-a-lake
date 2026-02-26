%% Main_PF_Final_Odometry_Aware.m
% 1. Physics layer: look-ahead collision detection (prevent grounding / going on land)
% 2. Particle layer: use the *actual* rotation (odometry-aware) to fix the "bounce-turn but particles don't follow" issue
% 3. Particle layer: enforce wall/map constraints (map-aware) to prevent particles from crossing land
clc; clear; close all;
% === 1. Load Map ===
map_name = "Oresund"; 
load(map_name + ".mat")

map = make_map_struct(Depth, xscale, yscale, harbour);
if exist('harbour', 'var')
    sim.home = harbour; 
    if size(sim.home, 1) > 1, sim.home = sim.home'; end
else
    sim.home = [65, 100]; 
end
% === 2. Configuration ===
sim.T = 2500;           
sim.switch_step = 500;  
sim.docking_radius = 8.0; 
sim.snap_interval = 250; 
sim.water_threshold = -1.0; % depth shallower than -1.0m is treated as land
% Noise parameters
sim.sigma_xy_act = 0.05;         
sim.sigma_phi_act = deg2rad(1.0); 
sim.sensor_sigma_ratio = 0.10;   
% PF parameters
pf.N = 5000;
pf.sensorErrorSpec = 0.20;       
pf.sigma_xy = 0.20; 
pf.sigma_phi = deg2rad(3.0);
pf.resampleESS = 0.5;
% === 3. Initialization ===
fprintf('Initializing Start Position...\n');
while true
    tx = randi(round(max(map.xscale))); 
    ty = randi(round(max(map.yscale)));
    d_check = get_depth_strict(map, tx, ty);
    if d_check < -3.0 % must spawn in deep water
        sim.x0 = tx; sim.y0 = ty; break; 
    end
end
sim.phi0 = pi*(2*rand-1); 
true_pose = [sim.x0, sim.y0, sim.phi0];
parts = init_particles_engineering(map, pf.N, d_check, pf.sensorErrorSpec);
history.truth = zeros(sim.T, 3);
history.est   = zeros(sim.T, 2);
history.error = zeros(sim.T, 1);
snapshots = struct(); snap_idx = 1;
success_flag = false;
fprintf('Running Simulation (Odometry & Wall Aware)...\n');
% === 4. MAIN LOOP ===
for t = 1:sim.T
    if t == 1, est_pos = [sim.x0, sim.y0]; else, est_pos = [xhat, yhat]; end
    
    % --- A. Controller ---
    dx_est = sim.home(1) - est_pos(1); dy_est = sim.home(2) - est_pos(2);
    dist_est = sqrt(dx_est^2 + dy_est^2);
    true_dist = sqrt((sim.home(1)-true_pose(1))^2 + (sim.home(2)-true_pose(2))^2);
    
    if t <= sim.switch_step
        d_cmd = 0.6;
        if mod(t, 200) < 100, turn_cmd = deg2rad(5); else, turn_cmd = -deg2rad(5); end
    else
        target_phi = atan2(dy_est, dx_est);
        mean_sin = sum(sin(parts.phi) .* parts.w);
        mean_cos = sum(cos(parts.phi) .* parts.w);
        est_phi = atan2(mean_sin, mean_cos);
        phi_diff = wrapToPi_local(target_phi - est_phi);
        if dist_est < 20, d_cmd = 0.3; else, d_cmd = 0.6; end
        turn_cmd = max(-deg2rad(30), min(deg2rad(30), phi_diff));
        
        if dist_est < sim.docking_radius && true_dist < sim.docking_radius
            fprintf('SUCCESS at Step %d. Final Error: %.2fm\n', t, true_dist);
            success_flag = true;
        end
    end
    
    % --- B. Physics Engine (generate the true displacement) ---
    pose_before_move = true_pose; % store the pre-move pose
    
    % 1. Compute the intended motion
    phi_try = true_pose(3) + turn_cmd + sim.sigma_phi_act * randn;
    dist_try = d_cmd + sim.sigma_xy_act * randn;
    
    proposed_x = true_pose(1) + dist_try * cos(phi_try);
    proposed_y = true_pose(2) + dist_try * sin(phi_try);
    
    % 2. Physical collision check
    proposed_depth = get_depth_strict(map, proposed_x, proposed_y);
    is_valid_move = ~isnan(proposed_depth) && proposed_depth < sim.water_threshold;
    
    if is_valid_move
        true_pose(1) = proposed_x;
        true_pose(2) = proposed_y;
        true_pose(3) = wrapToPi_local(phi_try);
    else
        % Bounce logic: keep position, but force a large heading change (e.g., rebound)
        bounce_angle = sign(randn) * deg2rad(100) + randn*0.2; 
        true_pose(3) = wrapToPi_local(true_pose(3) + bounce_angle);
    end
    
    % --- C. Odometry Simulation (key correction) ---
    % The particles should not know turn_cmd; they should know how much the robot *actually* turned (IMU reading),
    % i.e., the new heading minus the old heading.
    odom_turn = wrapToPi_local(true_pose(3) - pose_before_move(3));
    
    % --- D. Particle Filter Update ---
    
    % 1. Measurement
    true_depth = get_depth_strict(map, true_pose(1), true_pose(2));
    z_meas = true_depth + max(0.5, abs(true_depth) * sim.sensor_sigma_ratio) * randn;
    
    % 2. Prediction (use odom_turn instead of turn_cmd)
    old_x = parts.x; old_y = parts.y;
    
    % Update heading: add particle-level noise here
    parts.phi = parts.phi + odom_turn + pf.sigma_phi * randn(pf.N, 1);
    
    % Update position: attempt to move
    move_dist = d_cmd + pf.sigma_xy * randn(pf.N, 1);
    proposed_px = parts.x + move_dist .* cos(parts.phi);
    proposed_py = parts.y + move_dist .* sin(parts.phi);
    
    % Particle collision check (wall constraint)
    part_depths = interp2(map.xscale, map.yscale, map.Depth, proposed_px, proposed_py, 'linear', NaN);
    bad_idx = isnan(part_depths) | (part_depths > sim.water_threshold);
    
    % Valid moves
    parts.x(~bad_idx) = proposed_px(~bad_idx);
    parts.y(~bad_idx) = proposed_py(~bad_idx);
    
    % Invalid moves: particle is blocked by a wall/shoreline (rollback position, keep new heading since IMU did turn)
    parts.x(bad_idx) = old_x(bad_idx);
    parts.y(bad_idx) = old_y(bad_idx);
    
    % 3. Weight update
    z_pred = interp2(map.xscale, map.yscale, map.Depth, parts.x, parts.y, 'linear', NaN);
    valid_mask = ~isnan(z_pred) & (z_pred < sim.water_threshold);
    loglik = -100 * ones(pf.N, 1);
    
    if any(valid_mask)
        pf_sd = max(0.5, abs(z_meas) * (pf.sensorErrorSpec/2));
        loglik(valid_mask) = -0.5 * ((z_meas - z_pred(valid_mask))./pf_sd).^2; 
    end
    
    % Resample Logic
    if max(loglik) < -50
       parts = init_particles_engineering(map, pf.N, z_meas, pf.sensorErrorSpec);
    else
       parts.w = exp(loglik - max(loglik)); 
       parts.w = parts.w / sum(parts.w);
    end
    
    if 1/sum(parts.w.^2) < pf.resampleESS * pf.N
        parts = systematic_resample(parts); 
    end
    
    [xhat, yhat] = weighted_mean(parts.x, parts.y, parts.w);
    
    % Record
    history.truth(t,:) = true_pose;
    history.est(t,:) = [xhat, yhat];
    history.error(t) = sqrt((xhat - true_pose(1))^2 + (yhat - true_pose(2))^2);
    
    if t == 1 || mod(t, sim.snap_interval) == 0 || success_flag
        snapshots(snap_idx).t = t;
        snapshots(snap_idx).parts = parts; 
        snapshots(snap_idx).true_pose = true_pose;
        snapshots(snap_idx).est_pos = [xhat, yhat];
        snapshots(snap_idx).truth_traj = history.truth(1:t, :);
        snap_idx = snap_idx + 1;
        if success_flag, break; end 
    end
end
T_end = min(t, sim.T);
history.truth = history.truth(1:T_end, :);
history.est = history.est(1:T_end, :);
history.error = history.error(1:T_end);
fprintf('Done. Generating Visuals...\n');
%% --- Visualization ---
figure(2); set(gcf, 'Color', 'w', 'Position', [50, 50, 1400, 700]); 
sgtitle(sprintf('PF Navigation on %s (Odometry & Wall Constraint)', map_name), 'FontSize', 15);

num_snaps = length(snapshots);
cols = 4; rows = ceil(num_snaps/cols);
for i = 1:num_snaps
    subplot(rows, cols, i);
    snap = snapshots(i);
    imagesc(map.xscale, map.yscale, map.Depth); axis xy equal; hold on;
    caxis([-15, 0]); colormap(parula);
    vis_idx = 1:10:pf.N; 
    
    h_part = plot(snap.parts.x(vis_idx), snap.parts.y(vis_idx), 'g.', 'MarkerSize', 1);
    h_traj = plot(snap.truth_traj(:,1), snap.truth_traj(:,2), 'r-', 'LineWidth', 1.5);
    h_home = plot(sim.home(1), sim.home(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8); 
    h_true = plot(snap.true_pose(1), snap.true_pose(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 5);
    h_est  = plot(snap.est_pos(1), snap.est_pos(2), 'p', 'MarkerFaceColor','y', 'MarkerEdgeColor','k', 'MarkerSize', 10);
    
    title(sprintf('Step %d', snap.t));
    set(gca, 'XTick', [], 'YTick', []); 
    
    
end

% Use the plot handles from the last subplot
lgd = legend([h_home, h_true, h_est, h_traj, h_part], ...
       {'Home (Blue Circle)', 'True Pos. (Red Circle)', 'Est. Pos. (Yellow Star)', 'True Trajectory', 'Particles'}, ...
       'Location', 'eastoutside', 'NumColumns', 1, 'FontSize', 10, 'Box', 'off');
% Slightly shift the legend closer to the plotting area
lgd.Position(1) = lgd.Position(1) - 0.03;

% Location Error plot stays unchanged
figure(3); set(gcf, 'Color', 'w', 'Position', [100, 100, 800, 400]);
plot(1:T_end, history.error, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time Step (t)');
ylabel('Location Error (meters)');
title(sprintf('Location Error over Time (%s)', map_name));
xlim([1, T_end]);
hold on;
plot(T_end, history.error(end), 'ro', 'MarkerFaceColor', 'r');
text(T_end, history.error(end), sprintf('  Final: %.2fm', history.error(end)), ...
    'VerticalAlignment', 'bottom', 'FontSize', 10);
%% --- Helpers ---
function d = get_depth_strict(map, x, y)
    if x < min(map.xscale) || x > max(map.xscale) || y < min(map.yscale) || y > max(map.yscale)
        d = NaN; 
    else
        d = interp2(map.xscale, map.yscale, map.Depth, x, y, 'linear', NaN); 
    end
end
function parts = init_particles_engineering(map, N, z_meas, err_spec)
    margin = 0.5 + abs(z_meas) * 0.2; 
    [iy, ix] = find(abs(map.Depth - z_meas) < margin & map.Depth < -0.5);
    if length(ix) < 100, [iy, ix] = find(map.Depth < -1.0); end
    if isempty(ix)
        parts.x = rand(N,1)*(max(map.xscale)-min(map.xscale)) + min(map.xscale);
        parts.y = rand(N,1)*(max(map.yscale)-min(map.yscale)) + min(map.yscale);
    else
        idx = randi(length(ix), N, 1);
        parts.x = map.xscale(ix(idx))' + (rand(N,1)-0.5)*2; 
        parts.y = map.yscale(iy(idx))' + (rand(N,1)-0.5)*2;
    end
    parts.phi = (rand(N,1)*2-1)*pi; 
    parts.w = ones(N,1)/N;
end
function parts = systematic_resample(parts)
    N = numel(parts.w); 
    edges = min([0; cumsum(parts.w(:))], 1); edges(end)=1;
    u = (rand + (0:N-1)') / N; [~, bins] = histc(u, edges);
    parts.x = parts.x(bins); parts.y = parts.y(bins); 
    parts.phi = parts.phi(bins); parts.w = ones(N,1)/N;
end
function [mx, my] = weighted_mean(x, y, w)
    sw = sum(w); if sw < 1e-9, mx=mean(x); my=mean(y); else, mx=sum(x.*w)/sw; my=sum(y.*w)/sw; end
end
function a = wrapToPi_local(a), a = mod(a + pi, 2*pi) - pi; end