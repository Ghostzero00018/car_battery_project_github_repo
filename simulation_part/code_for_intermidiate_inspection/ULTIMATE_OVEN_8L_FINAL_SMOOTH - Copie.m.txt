function ULTIMATE_OVEN_8L_FINAL_SMOOTH
% Core = 65 °C in an 8 L box, with smoother, PID-based heater & airflow control.
%   • Air-loop PID (derivative filtering), anti-windup, slew-rate limited heater
%   • Core→air-setpoint PID (derivative filtering) + rate-limited setpoint
%   • Recirculation & purge flows are PI-driven + rate-limited
%   • Physics: half-thickness battery, wall node, radiation, UA leakage

close all; clc;

%% ------------ Physical Params (8 L box) ------------
cp_air    = 1005;                 % J/(kg*K)
rho_air   = 1.2;                  % kg/m^3
Tamb      = 25;                   % °C ambient
Tset_core = 65;                   % °C core target

V_box     = 0.008;                % m^3 (8 L)
m_air     = rho_air * V_box;      % kg air in box

% Inner wall area (≈ 0.2 m cube -> ~0.24 m^2).
A_wall    = 0.24;                 % m^2

% Battery exposed area & total slab thickness
A_batt       = 0.015;             % m^2 (exposed area)
thick_total  = 0.010;             % m (TOTAL thickness, both faces)
thick_half   = 0.5*thick_total;   % m (we model half-thickness to an adiabatic mid-plane)

% Battery properties (effective bulk)
rho_batt  = 2000;                 % kg/m^3
m_batt    = rho_batt * A_batt * thick_total;  % kg (use TOTAL thickness for mass)
cp_batt   = 900;                  % J/(kg*K)
k_batt    = 1.0;                  % W/(m*K)

% Radiation (battery surface to inner wall)
eps_batt  = 0.80;                 % emissivity
sigma     = 5.670374419e-8;       % W/(m^2*K^4)
A_rad     = A_batt;               % radiating area

% Wall lumped node (polyurethane/plastic)
h_airwall0= 5;                    % W/(m^2*K) baseline air->wall convection
m_wall    = 0.5;  cp_wall = 1400; % kg, J/(kg*K)
U_loss    = 1.0;                  % W/K leakage to ambient (through enclosure)

% Heater cap (hardware)
Qmax      = 60;                   % W

%% ------------ Discretization (half thickness) ------------
N  = 10;                           % number of battery layers (through half-thickness)
dx = thick_half / N;               % m

%% ------------ Control / Safety ------------
Tair_cap    = 75;      % °C hard cap
Tair_soft   = 73;      % °C soft threshold to start cooling
deadband_core = 0.3;   % °C deadband for outer I

% ===== Inner heater PID (air -> heater) =====
Kp_air = 0.35;   Ki_air = 0.018;  Kd_air = 0.08;   % gains
tau_d_air = 5.0;                     % s derivative filter const (1st order on T_air)
Kaw_air   = 1.5;                     % anti-windup back-calc gain

% ===== Outer PID (core -> air setpoint) =====
Kp_core = 2.5;   Ki_core = 0.04;  Kd_core = 0.03;
tau_d_core = 8.0;                    % s derivative filter on T_core
Kaw_core   = 1.0;

% ===== Actuator dynamics / rate limits =====
tau_Q     = 10;    % s 1st-order heater actuator
Q_slew    = 12;    % W/s maximum change of heater power

tau_flow  = 12;    % s 1st-order flow actuator
mrecirc_slew = 8e-4;  % kg/s^2 (≈ 0.8 g/s per second max change)
mext_slew    = 4e-5;  % kg/s^2 (≈ 0.04 g/s per second max change)

% Recirculation & purge targets (PI)
mrecirc_min = 3e-4;                % kg/s
mrecirc_max = 4e-3;                % kg/s
k_recirc_ff = 1.2e-4;              % kg/s per °C |core error| (feedforward)

mext_max  = 1e-4;                  % kg/s (0.1 g/s)
Kp_purge = 3e-5;    Ki_purge = 1e-6;  % purge PI gains

% Boost (push air near the cap, then regulate with PID)
Tair_sp_boost = min(Tair_cap, 75); % °C
t_boost_max   = 17*60;             % s
T_core_switch = 64;                % °C

% Measurement filter constants for derivative (noisy dT/dt is unsafe)
tau_meas_air  = 3.0;    % s  (filter on T_air for D term)
tau_meas_core = 5.0;    % s  (filter on T_core for D term)

%% ------------ Time / Solver ------------
t_end = 3600;                      % s (1 hour)
tspan = [0 t_end];
opts  = odeset('RelTol',1e-5,'AbsTol',1e-7,'MaxStep',0.5);

%% ------------ Initial Conditions ------------
T0_air    = Tamb;
T0_layers = ones(N,1)*Tamb;
T0_wall   = Tamb;

Iq_air = 0; Iq_core = 0;
Tair_sp0  = Tair_sp_boost;
m_recirc0 = mrecirc_min;
m_ext0    = 0;
Q0        = 0;

% Filtered measurements for derivative terms
Tair_f0  = T0_air;
Tcore_f0 = T0_layers(end);

x0 = [ T0_air; T0_layers; T0_wall; ...
       Iq_air; Iq_core; Tair_sp0; ...
       m_recirc0; m_ext0; Q0; ...
       Tair_f0; Tcore_f0 ];

%% ------------ Solve ------------
[t,X] = ode15s(@(t,x) ode_corefirst_pid_smooth(t,x, ...
    N,dx,A_batt,m_air,cp_air,Tamb,k_batt,rho_batt,cp_batt, ...
    Tset_core,Qmax, ...
    Kp_air,Ki_air,Kd_air,tau_d_air,Kaw_air, ...
    Kp_core,Ki_core,Kd_core,tau_d_core,Kaw_core, ...
    mrecirc_min,mrecirc_max,k_recirc_ff, ...
    Kp_purge,Ki_purge,mext_max, ...
    tau_Q,Q_slew, ...
    tau_flow,mrecirc_slew,mext_slew, ...
    Tair_sp_boost,t_boost_max,T_core_switch, ...
    Tair_cap,Tair_soft,deadband_core, ...
    eps_batt,sigma,A_rad, ...
    A_wall,h_airwall0,m_wall,cp_wall,U_loss, ...
    tau_meas_air,tau_meas_core), ...
    tspan,x0,opts);

%% ------------ Unpack / Derived ------------
T_air   = X(:,1);
Tb      = X(:,2:1+N);        % layers from surface (1) to mid-plane (N)
T_wall  = X(:,2+N);
T_core  = Tb(:,end);         % mid-plane (adiabatic)
T_surf  = Tb(:,1);           % surface

Tair_sp = X(:,end-5);
m_recirc= X(:,end-4);
m_ext   = X(:,end-3);
Q       = X(:,end-2);        % actuator state (already smoothed & rate-limited)
% filtered measurements:
Tair_f  = X(:,end-1); %#ok<NASGU>
Tcore_f = X(:,end);   %#ok<NASGU>

time_min= t/60;
depth   = linspace(0,thick_half,N);

%% ------------ Console Summary ------------
E_min_J  = m_batt*cp_batt*(Tset_core - Tamb);
fprintf('Box: V=%.3f m^3 (8 L), m_air=%.3f kg, A_wall=%.2f m^2\n', V_box, m_air, A_wall);
fprintf('Battery: rho=%.0f kg/m^3, m=%.3f kg, A=%.3f m^2, total t=%.3f m\n', rho_batt, m_batt, A_batt, thick_total);
fprintf('Ideal lower-bound heat-up at Qmax=%.0f W: %.1f min\n', Qmax, E_min_J/Qmax/60);

reach_idx = find(T_core>=Tset_core,1,'first');
if ~isempty(reach_idx)
    fprintf('Reached %g °C in %.2f min.\n', Tset_core, time_min(reach_idx));
else
    fprintf('Did NOT reach %g °C within 60 min.\n', Tset_core);
end
fprintf('Max T_air observed: %.1f °C (cap = %.1f °C)\n', max(T_air), Tair_cap);
fprintf('Final (60 min): Core=%.2f °C, Surface=%.2f °C, Air=%.2f °C, Wall=%.2f °C, Q=%.2f W\n', ...
    T_core(end), T_surf(end), T_air(end), T_wall(end), min(max(Q(end),0),Qmax));
fprintf('Final flows: m_recirc=%.1f g/s, m_ext=%.1f g/s\n', 1000*m_recirc(end), 1000*m_ext(end));

%% ------------ Plots ------------
% 1) Core (mid-plane) vs setpoint
figure; plot(time_min,T_core,'r','LineWidth',2); hold on;
yline(Tset_core,'--k','Target'); grid on;
xlabel('Time (min)'); ylabel('Core Temp (°C)');
title('Core Temperature (mid-plane, 8 L)');

% 2) Air vs setpoint and cap
figure; plot(time_min,T_air,'b','LineWidth',1.6); hold on;
plot(time_min,Tair_sp,'k--','LineWidth',1.2);
yline(Tair_cap,':r','T_{air,max}');
grid on; xlabel('Time (min)'); ylabel('Air Temp (°C)');
legend('T_{air}','T_{air,set}','Location','best');
title('Air Temperature Control (PID, capped)');

% 3) Flows
figure; plot(time_min,m_recirc*1000,'LineWidth',1.4); hold on;
plot(time_min,m_ext*1000,'--','LineWidth',1.4);
grid on; xlabel('Time (min)'); ylabel('Flow (g/s)');
legend('Recirculation (h)','External (purge)','Location','best');
title('Airflow Control (PI + slew limits)');

% 4) Heater power (rate-limited actuator)
figure; plot(time_min,min(max(Q,0),Qmax),'m','LineWidth',1.6); grid on;
xlabel('Time (min)'); ylabel('Heater Power Q (W)');
title('Heater Power vs Time (PID + slew)');

% 5) Through-thickness heatmap (surface -> mid-plane)
figure; imagesc(time_min,depth*1000,Tb'); set(gca,'YDir','normal');
colormap hot; colorbar; xlabel('Time (min)'); ylabel('Depth from surface (mm)');
title('Battery Temperature Through Half-Thickness (°C)');

%% ========== 3D OVEN VISUALIZATION & ANIMATION ==========
% Set to true if you want a video file too
makeVideo   = false;               % true -> writes oven_3D.mp4
frame_step_s = 5;                  % seconds per frame in the animation

% ---------- Geometry (8 L ~ 0.2 m cube) ----------
L = 0.20; W = 0.20; H = 0.20;     % oven inner box (m)
% Battery "tray" size & position (visual only)
Lb = 0.15; Wb = 0.11;             % planform (m)
x0v = (L - Lb)/2;                 % centered
y0v = (W - Wb)/2;
z_surf_v = 0.11;                  % height of battery surface (m)

% Create figure and scene
fig = figure('Color','w'); ax = axes(fig); hold(ax,'on');
axis(ax,[0 L 0 W 0 H]); axis(ax,'vis3d'); daspect(ax,[1 1 1]);
view(ax,145,22); grid(ax,'on'); box(ax,'on');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Oven Snapshot');

% Semi-transparent oven (6 faces)
drawOvenBox(L,W,H,0.12);

% Heater outlines (just visuals)
drawHeaterRect([0.03 0.03 0.015],[L-0.03 W-0.03 0.015],[1 0.1 0.1],2.0);   % bottom
drawHeaterRect([0.03 0.03 H-0.02],[L-0.03 W-0.03 H-0.02],[1 0.1 0.1],2.0);  % top

% Battery layers (surface->mid-plane): N thin plates colored by Tb(:,i)
dz = thick_half / N;
[xp,yp] = meshgrid([x0v x0v+Lb],[y0v y0v+Wb]);     % 2x2 plane per layer
layerSurf = gobjects(N,1);
for i = 1:N
    z = z_surf_v + (i-1)*dz;
    layerSurf(i) = surf(xp,yp, z*ones(2), ...
                        'EdgeColor','none','FaceAlpha',0.85);
end

% Battery top surface with a mild stripe pattern (illustrative flow)
[xg,yg] = meshgrid(linspace(x0v,x0v+Lb,80), linspace(y0v,y0v+Wb,28));
surfTop = surf(xg,yg, z_surf_v*ones(size(xg)), zeros(size(xg)), ...
               'EdgeColor','none','FaceAlpha',0.95);

% Color settings
colormap(jet);
clim([min(30, min(T_air)), max( max(T_core),  Tair_cap )]);
cb = colorbar; cb.Label.String = 'Temperature (°C)';

% Prepare animation
if makeVideo
    v = VideoWriter('oven_3D.mp4','MPEG-4'); v.FrameRate = 20; open(v);
end
idx = 1:frame_step_s: numel(time_min);  % sample every few seconds

for k = idx
    % Stripe field on top surface (purely visual, blends air & surface temps)
    stripes = 0.15 * cos( 2*pi*(xg - x0v)/Lb * 8 );          % 8 bands
    Tfield  = T_surf(k) + (T_air(k) - T_surf(k)) .* stripes;
    set(surfTop,'CData',Tfield);

    % Update layer colors from Tb (surface->mid-plane)
    for i = 1:N
        set(layerSurf(i),'CData', Tb(k,i) * ones(2));
    end

    title(ax, sprintf('3D Oven — t = %.1f min | Air = %.1f°C | Core = %.1f°C', ...
                      time_min(k), T_air(k), T_core(k)));
    drawnow;

    if makeVideo
        writeVideo(v, getframe(fig));
    end
end
if makeVideo, close(v); end
% --------- End 3D viz ----------

end % ===== end main =====


%% ================== ODE: PID + Smooth Actuators ==================
function dxdt = ode_corefirst_pid_smooth(t,x, ...
    N,dx,A_batt,m_air,cp_air,Tamb,k_batt,rho_batt,cp_batt, ...
    Tset_core,Qmax, ...
    Kp_air,Ki_air,Kd_air,~,Kaw_air, ...
    Kp_core,Ki_core,Kd_core,~,Kaw_core, ...
    mrecirc_min,mrecirc_max,k_recirc_ff, ...
    Kp_purge,~,mext_max, ...
    tau_Q,Q_slew, ...
    tau_flow,mrecirc_slew,mext_slew, ...
    Tair_sp_boost,t_boost_max,T_core_switch, ...
    Tair_cap,Tair_soft,deadband_core, ...
    eps_batt,sigma,A_rad, ...
    A_wall,h_airwall0,m_wall,cp_wall,U_loss, ...
    tau_meas_air,tau_meas_core)

% ---- Unpack state vector ----
% [ T_air; Tb(1..N); T_wall; I_air; I_core; Tair_sp; m_recirc; m_ext; Q; Tair_f; Tcore_f ]
i1 = 1;
i2 = (2 : 1+N);
iw = 2+N;
iIair = iw+1;
iIcore= iIair+1;
iSp   = iIcore+1;
imr   = iSp+1;
ime   = imr+1;
iQ    = ime+1;
iaf   = iQ+1;
icf   = iaf+1;

T_air   = x(i1);
Tb      = x(i2);
T_wall  = x(iw);
I_air   = x(iIair);
I_core  = x(iIcore);
Tair_sp = x(iSp);
m_recirc= x(imr);
m_ext   = x(ime);
Q       = x(iQ);
Tair_f  = x(iaf);
Tcore_f = x(icf);

T_core  = Tb(end);          % mid-plane (adiabatic)
T_surf  = Tb(1);            % surface

% ---- Phase logic ----
in_boost = (t < t_boost_max) && (T_core < T_core_switch);

% ---- Meas filters (for derivative-on-measurement) ----
dTair_f  = (T_air  - Tair_f)/tau_meas_air;
dTcore_f = (T_core - Tcore_f)/tau_meas_core;

% ---- Outer PID: core -> air setpoint (bounded) ----
e_core = Tset_core - T_core;
if abs(e_core) < deadband_core && ~in_boost
    I_core_dot = 0;
else
    I_core_dot = e_core;
end
D_core = -Kd_core * dTcore_f;
if in_boost
    Tair_sp_cmd = min(Tair_cap, Tair_sp_boost);
else
    Tair_sp_cmd_raw = (Kp_core*e_core + Ki_core*I_core + D_core);
    Tair_sp_cmd = Tair_soft + 1.5 + Tair_sp_cmd_raw;
    Tair_sp_cmd = min(max(40, Tair_sp_cmd), Tair_cap);
    I_core_dot = I_core_dot + Kaw_core*( (Tair_sp_cmd) - (Tair_soft + 1.5 + Tair_sp_cmd_raw) );
end

% ---- Recirculation feedforward (bounded) ----
mrecirc_ff = mrecirc_min + k_recirc_ff*abs(e_core);
m_recirc_cmd = min(mrecirc_max, max(mrecirc_min, mrecirc_ff));

% ---- Purge PI (on air-soft-cap error) ----
e_cap = max(T_air - Tair_soft, 0);
m_ext_cmd = min(mext_max, max(0, Kp_purge*e_cap)); % I-term can be added similarly if needed

% ---- Inner PID: air -> heater power (bounded & rate-limited) ----
e_air = Tair_sp - T_air;
I_air_dot = e_air;
D_air = -Kd_air * dTair_f;
Q_unsat = Kp_air*e_air + Ki_air*I_air + D_air;
Q_cmd_sat = min(max(Q_unsat, 0), Qmax);
I_air_dot = I_air_dot + Kaw_air*(Q_cmd_sat - Q_unsat);

% ---- Actuator dynamics + RATE LIMITS ----
dQ_des = (Q_cmd_sat - Q)/tau_Q;               % heater
dQ     = clamp(dQ_des, -Q_slew, Q_slew);

dmr_des = (m_recirc_cmd - m_recirc)/tau_flow; % recirc
dme_des = (m_ext_cmd   - m_ext)/tau_flow;     % purge
dm_recirc = clamp(dmr_des, -mrecirc_slew, mrecirc_slew);
dm_ext    = clamp(dme_des, -mext_slew,    mext_slew);

% ---- Heat transfer coeffs ----
h_batt = h_from_recirc(m_recirc);
h_aw   = h_airwall0 + 25*(m_recirc^(0.5));

% ---- Radiation (battery surface -> wall) ----
TsK = (T_surf + 273.15);  
TwK = (T_wall + 273.15);
q_rad = eps_batt * sigma * A_rad * (TsK^4 - TwK^4);   % W (positive = batt -> wall)

% ---- Air energy balance ----
Qeff = Q;
dTair = ( Qeff ...
        - h_batt*A_batt*(T_air - T_surf) ...
        - m_ext*cp_air*(T_air - Tamb) ...
        - h_aw*A_wall*(T_air - T_wall) ) / (m_air*cp_air);

% ---- Battery 1D conduction (half-thickness) ----
alpha      = k_batt/(rho_batt*cp_batt*dx^2);
conv_coeff = h_batt/(rho_batt*cp_batt*dx);
rad_coeff  = (q_rad/A_batt)/(rho_batt*cp_batt*dx);

dTb = zeros(N,1);
if N>=2
    dTb(1) = alpha*(Tb(2)-Tb(1)) + conv_coeff*(T_air - Tb(1)) - rad_coeff;
    for i = 2:N-1
        dTb(i) = alpha*(Tb(i+1)-2*Tb(i)+Tb(i-1));
    end
    dTb(N) = alpha*(Tb(N-1)-Tb(N)); % adiabatic back
else
    dTb(1) = conv_coeff*(T_air - Tb(1)) - rad_coeff;
end

% ---- Wall lumped mass ----
dTwall = ( q_rad + h_aw*A_wall*(T_air - T_wall) - U_loss*(T_wall - Tamb) ) / (m_wall*cp_wall);

% ---- Setpoint smoother ----
dTair_sp = (Tair_sp_cmd - Tair_sp)/max(tau_Q,1e-6);

% ---- Meas filter dynamics ----
dTair_f  = dTair_f;
dTcore_f = dTcore_f;

% ---- Collect derivatives ----
dxdt = [ dTair; dTb; dTwall; ...
         I_air_dot; I_core_dot; dTair_sp; ...
         dm_recirc; dm_ext; dQ; ...
         dTair_f; dTcore_f ];
end

%% ======= h from recirculation (tuned for ~20–80 W/m²K) =======
function h = h_from_recirc(m_recirc)
    h0 = 20;           % baseline ~natural convection
    c  = 2000;         % mixing gain (empirical)
    h  = h0 + c*m_recirc^0.8;
end

%% ======= Utility clamp =======
function y = clamp(x, lo, hi)
    if x < lo, y = lo; elseif x > hi, y = hi; else, y = x; end
end

%% ----- helpers for 3D viz -----
function drawOvenBox(L,W,H,alphaFace)
    V = [0 0 0;  L 0 0;  L W 0;  0 W 0;  0 0 H;  L 0 H;  L W H;  0 W H];
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch('Faces',F,'Vertices',V,'FaceColor',[0.6 0.6 0.65], ...
          'FaceAlpha',alphaFace,'EdgeColor','none');
end
function drawHeaterRect(pmin, pmax, color, lw)
    x = [pmin(1) pmax(1) pmax(1) pmin(1) pmin(1)];
    y = [pmin(2) pmin(2) pmax(2) pmax(2) pmin(2)];
    z = pmin(3) * ones(size(x));
    plot3(x,y,z,'Color',color,'LineWidth',lw);
end
