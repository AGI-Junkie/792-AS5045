% make_figures.m - thesis figures from thesis/figdata/*.csv (written by export_figdata.py).
% Static print figures: Arial, 600 dpi PNG, validated categorical slots (blue, orange, aqua) each paired with its own marker,
% one y-axis per panel (two measures -> two panels), light grid, legend for >= 2 series.
function make_figures()
here = fileparts(mfilename('fullpath')); D = fullfile(here, 'figdata'); O = fullfile(here, 'figures');
if ~exist(O, 'dir'), mkdir(O); end
C = struct('blue', hex('#2a78d6'), 'orange', hex('#eb6834'), 'aqua', hex('#1baf7a'), 'ink', hex('#0b0b0b'), 'muted', hex('#52514e'), 'grid', hex('#d9d9d6'));
set(groot, 'defaultAxesFontName', 'Arial', 'defaultTextFontName', 'Arial', 'defaultAxesFontSize', 9, 'defaultLineLineWidth', 1.2);

%% E2 static encoder noise
T = readtable(fullfile(D, 'e2_static.csv'));
f = fig(16, 6); tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
names = {'arm1', 'arm2 (elbow)'}; vals = {T.arm1_lsb, T.arm2_lsb}; lsbdeg = [360/4096/40, 360/4096/51.06];
for k = 1:2
    ax = nexttile(tl); v = vals{k};
    histogram(ax, v, 'BinMethod', 'integers', 'FaceColor', C.blue, 'EdgeColor', 'w', 'FaceAlpha', 1);
    style(ax, C); xlabel(ax, 'Deviation from mean (encoder LSB)'); ylabel(ax, 'Samples');
    title(ax, sprintf('%s: \\sigma = %.2f LSB (%.4f°)', names{k}, std(v, 1), std(v, 1) * lsbdeg(k)), 'FontWeight', 'normal', 'FontSize', 9);
end
save_fig(f, O, 'fig_e2_static_noise');

%% E3 homing repeatability: origin deviation in output degrees
T = readtable(fullfile(D, 'e3_homing.csv'));
f = fig(16, 6.5); ax = axes(f); hold(ax, 'on');
d1 = (T.arm1_raw - mean(T.arm1_raw)) * 360/4096/40; d2 = (T.arm2_raw - mean(T.arm2_raw)) * 360/4096/51.06;
plot(ax, T.trial, d1, '-o', 'Color', C.blue, 'MarkerFaceColor', C.blue, 'MarkerSize', 5, 'DisplayName', 'arm1 (mechanical switch)');
plot(ax, T.trial, d2, '-s', 'Color', C.orange, 'MarkerFaceColor', C.orange, 'MarkerSize', 5, 'DisplayName', 'arm2 (NPN proximity sensor)');
style(ax, C); xlabel(ax, 'Homing run'); ylabel(ax, 'Origin deviation from mean (° at output)'); xlim(ax, [0.5, 10.5]); xticks(ax, 1:10);
legend(ax, 'Location', 'northeast', 'Box', 'off', 'TextColor', C.ink);
save_fig(f, O, 'fig_e3_homing');

%% E4-A tracking error against speed (two panels: RMSE, peak)
T = readtable(fullfile(D, 'e4a_speed.csv')); L = unique(T.level);
f = fig(16, 6.5); tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
pairs = {{'q1_rmse', 'elbow_rmse', 'RMS following error (°)'}, {'q1_peak', 'elbow_peak', 'Peak following error (°)'}};
for p = 1:2
    ax = nexttile(tl); hold(ax, 'on'); keys = pairs{p};
    for s = 1:2
        x = arrayfun(@(l) mean(T.peak1_usteps(T.level == l)), L); y = arrayfun(@(l) mean(T.(keys{s})(T.level == l)), L);
        lo = y - arrayfun(@(l) min(T.(keys{s})(T.level == l)), L); hi = arrayfun(@(l) max(T.(keys{s})(T.level == l)), L) - y;
        col = ifelse(s == 1, C.blue, C.orange); mk = ifelse(s == 1, 'o', 's');
        errorbar(ax, x, y, lo, hi, ['-' mk], 'Color', col, 'MarkerFaceColor', col, 'MarkerSize', 5, 'CapSize', 4, 'DisplayName', ifelse(s == 1, 'q1 (arm1)', 'elbow (arm2)'));
    end
    style(ax, C); xlabel(ax, 'Planned peak rate, motor 1 (microsteps/s)'); ylabel(ax, keys{3}); ylim(ax, [0, max(ylim(ax))]); xlim(ax, [700 2000]);
    if p == 1, legend(ax, 'Location', 'southeast', 'Box', 'off', 'TextColor', C.ink); end
end
save_fig(f, O, 'fig_e4a_speed');

%% E4 decomposition: error against motor-shaft angle, with the fitted once/twice-per-revolution model
T = readtable(fullfile(D, 'e4_phase.csv')); F = readtable(fullfile(D, 'e4_fit.csv'));
f = fig(16, 6.5); tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
spec = {{'motor1_deg', 'q1_err', 'q1'}, {'motor2_deg', 'elbow_err', 'Elbow'}};
for k = 1:2
    ax = nexttile(tl); hold(ax, 'on'); s = spec{k}; x = T.(s{1}); y = T.(s{2});
    scatter(ax, x, y, 4, C.blue, 'filled', 'MarkerFaceAlpha', 0.35, 'DisplayName', 'Telemetry samples');
    th = linspace(0, 360, 721); c = F{k, 2:6};
    model = c(1) + c(2) * cosd(th) + c(3) * sind(th) + c(4) * cosd(2 * th) + c(5) * sind(2 * th);
    plot(ax, th, model, '-', 'Color', C.orange, 'LineWidth', 2, 'DisplayName', 'Fitted model');
    resid = y - (c(1) + c(2) * cosd(x) + c(3) * sind(x) + c(4) * cosd(2 * x) + c(5) * sind(2 * x));
    style(ax, C); xlim(ax, [0 360]); xticks(ax, 0:90:360); xlabel(ax, 'Motor-shaft angle (°)'); ylabel(ax, 'Encoder − command (° at output)');
    title(ax, sprintf('%s: model explains %.0f%% of variance', s{3}, 100 * (1 - var(resid) / var(y))), 'FontWeight', 'normal', 'FontSize', 9);
    if k == 2, legend(ax, 'Location', 'south', 'Box', 'off', 'TextColor', C.ink); end
end
save_fig(f, O, 'fig_e4_decomposition');

%% E4-B shapes: RMSE before / after the per-revolution correction
T = readtable(fullfile(D, 'e4b_shapes.csv'), 'TextType', 'string');
f = fig(16, 6.5); tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
cols = {{'q1_before', 'q1_after', 'q1 (arm1)'}, {'elbow_before', 'elbow_after', 'elbow (arm2)'}};
for k = 1:2
    ax = nexttile(tl); s = cols{k};
    b = bar(ax, categorical(T.shape, T.shape), [T.(s{1}), T.(s{2})], 'grouped', 'BarWidth', 0.85, 'EdgeColor', 'w', 'LineWidth', 1);
    b(1).FaceColor = C.blue; b(2).FaceColor = C.orange; b(1).DisplayName = 'Measured'; b(2).DisplayName = 'After per-revolution correction';
    style(ax, C); ylabel(ax, 'RMS following error (°)'); title(ax, s{3}, 'FontWeight', 'normal', 'FontSize', 9);
    if k == 1, legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off', 'TextColor', C.ink); end
end
save_fig(f, O, 'fig_e4b_shapes');

%% E5 sheet-4 circles: paper and encoder diameter error against design diameter
T = readtable(fullfile(D, 'e5_circles.csv'), 'TextType', 'string');
f = fig(16, 7); ax = axes(f); hold(ax, 'on');
err = T.paper_mm - T.design_mm; P = polyfit(T.design_mm, T.paper_mm, 1);
scatter(ax, T.design_mm, err, 30, C.blue, 'o', 'filled', 'DisplayName', 'Paper (caliper, 4 directions)');
xx = [40 160]; plot(ax, xx, polyval(P, xx) - xx, '-', 'Color', C.blue, 'LineWidth', 1, 'DisplayName', sprintf('Linear fit: k = %+.2f%%, c = %+.2f mm', 100 * (P(1) - 1), P(2)));
[u, ia] = unique(T.design_mm); plot(ax, u, T.encoder_mm(ia) - u, '-s', 'Color', C.orange, 'MarkerFaceColor', C.orange, 'MarkerSize', 6, 'DisplayName', 'Encoder (motor side, forward kinematics)');
yline(ax, 0, '-', 'Color', C.muted, 'LineWidth', 0.8, 'HandleVisibility', 'off');
style(ax, C); xlim(ax, [40 160]); xticks(ax, [50 100 150]); xlabel(ax, 'Design diameter (mm)'); ylabel(ax, 'Diameter error (mm)');
legend(ax, 'Location', 'southeast', 'Box', 'off', 'TextColor', C.ink);
save_fig(f, O, 'fig_e5_circles');

%% E8 speed ladder (two panels: move time, peak following error)
T = readtable(fullfile(D, 'e8_ladder.csv'), 'TextType', 'string'); R = unique(T.rate_req);
f = fig(16, 6.5); tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl); hold(ax, 'on');
scatter(ax, T.rate_req, T.host_ms / 1000, 18, C.blue, 'o', 'filled', 'MarkerFaceAlpha', 0.6, 'HandleVisibility', 'off');
plot(ax, R, arrayfun(@(r) mean(T.host_ms(T.rate_req == r)) / 1000, R), '-', 'Color', C.blue);
style(ax, C); xlabel(ax, 'Requested rate (microsteps/s)'); ylabel(ax, 'Move time A↔B (s)'); ylim(ax, [0, max(ylim(ax))]); xlim(ax, [2400 5300]);
xline(ax, 5000, '--', 'Firmware cap', 'Color', C.muted, 'LabelVerticalAlignment', 'bottom', 'FontSize', 8);
ax = nexttile(tl); hold(ax, 'on');
plot(ax, R, arrayfun(@(r) max(T.q1_peak(T.rate_req == r)), R), '-o', 'Color', C.blue, 'MarkerFaceColor', C.blue, 'MarkerSize', 5, 'DisplayName', 'q1 (arm1)');
plot(ax, R, arrayfun(@(r) max(T.elbow_peak(T.rate_req == r)), R), '-s', 'Color', C.orange, 'MarkerFaceColor', C.orange, 'MarkerSize', 5, 'DisplayName', 'elbow (arm2)');
style(ax, C); xlabel(ax, 'Requested rate (microsteps/s)'); ylabel(ax, 'Peak following error (°)'); ylim(ax, [0, max(ylim(ax))]); xlim(ax, [2400 5300]);
xline(ax, 5000, '--', 'Color', C.muted, 'HandleVisibility', 'off'); legend(ax, 'Location', 'southeast', 'Box', 'off', 'TextColor', C.ink);
save_fig(f, O, 'fig_e8_speed');

%% E1 planning: path deviation against node spacing, three shape families
T = readtable(fullfile(D, 'e1_planning.csv'), 'TextType', 'string');
f = fig(16, 7); ax = axes(f); hold(ax, 'on');
fams = ["Circle", "Other curve", "Straight-edged"]; cc = {C.blue, C.orange, C.aqua}; mk = {'o', 's', '^'};
for k = 1:3
    m = T.family == fams(k);
    scatter(ax, T.node_gap_max(m), T.dev_max(m), 36, cc{k}, mk{k}, 'filled', 'DisplayName', fams(k));
end
yline(ax, max(T.node_dev_max), ':', sprintf('Step quantisation ≤ %.3f mm', max(T.node_dev_max)), 'Color', C.muted, 'LabelHorizontalAlignment', 'left', 'FontSize', 8, 'HandleVisibility', 'off');
style(ax, C); xlabel(ax, 'Largest node spacing (mm)'); ylabel(ax, 'Max deviation from ideal shape (mm)'); ylim(ax, [0, 0.16]);
legend(ax, 'Location', 'northwest', 'Box', 'off', 'TextColor', C.ink);
save_fig(f, O, 'fig_e1_planning');

%% E6 stops: arm-1 command and encoder around an operator stop and a heartbeat-loss stop
T = readtable(fullfile(D, 'e6_stops.csv'), 'TextType', 'string'); kinds = ["operator stop", "heartbeat loss"];
f = fig(16, 6.5); tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:2
    ax = nexttile(tl); hold(ax, 'on'); m = T.kind == kinds(k); t = T.t_ms(m); [t, i] = sort(t); c = T.q1_cmd_deg(m); e = T.q1_enc_deg(m);
    plot(ax, t, c(i), '-o', 'Color', C.blue, 'MarkerFaceColor', C.blue, 'MarkerSize', 3.5, 'DisplayName', 'Command');
    plot(ax, t, e(i), '--s', 'Color', C.orange, 'MarkerSize', 4, 'DisplayName', 'Encoder');
    xline(ax, 0, '-', 'Stop', 'Color', C.muted, 'LabelVerticalAlignment', 'top', 'FontSize', 8, 'HandleVisibility', 'off');
    style(ax, C); xlabel(ax, 'Time from firmware stop (ms)'); ylabel(ax, 'q1 (°)'); title(ax, char(kinds(k)), 'FontWeight', 'normal', 'FontSize', 9);
    if k == 1, legend(ax, 'Location', 'east', 'Box', 'off', 'TextColor', C.ink); end
end
save_fig(f, O, 'fig_e6_stops');
end

function f = fig(wcm, hcm)
f = figure('Visible', 'off', 'Color', 'w', 'Units', 'centimeters', 'Position', [2 2 wcm hcm]);
try, theme(f, 'light'); catch, end   % newer MATLAB defaults figures to the dark theme
end
function style(ax, C)
set(ax, 'Color', 'w', 'Box', 'off', 'TickDir', 'out', 'XColor', C.muted, 'YColor', C.muted, 'GridColor', C.grid, 'GridAlpha', 1, 'LineWidth', 0.6);
grid(ax, 'on'); ax.XLabel.Color = C.ink; if ~isempty(ax.Legend), ax.Legend.TextColor = C.ink; end; ax.YLabel.Color = C.ink; ax.Title.Color = C.ink;
end
function save_fig(f, O, name)
exportgraphics(f, fullfile(O, [name '.png']), 'Resolution', 600, 'BackgroundColor', 'white');
close(f); fprintf('%s\n', name);
end
function c = hex(h)
c = sscanf(h(2:end), '%2x%2x%2x', [1 3]) / 255;
end
function v = ifelse(cond, a, b)
if cond, v = a; else, v = b; end
end
