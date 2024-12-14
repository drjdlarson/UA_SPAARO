%%%% test_seldom %%%%

%% User Inputs
show_animation = false; % Takes longer to run
% Change velocity and angulare rate commands in seldom_sim/Pilot Input

%% Setup Model
if show_animation
    set_param('seldom_sim/UAVAnimation','Commented','off')
else
    set_param('seldom_sim/UAVAnimation','Commented','on')
end

%% Run Simulation
data_out = sim("seldom_sim.slx");

%% Plot
% Should match order in seldom/SignalLogging
data_idx.u_cmd = 1;
data_idx.v_cmd = 2;
data_idx.w_cmd = 3;
data_idx.u     = 4;
data_idx.v     = 5;
data_idx.w     = 6;
data_idx.u_err = 7;
data_idx.v_err = 8;
data_idx.w_err = 9;
data_idx.p_cmd = 10;
data_idx.q_cmd = 11;
data_idx.r_cmd = 12;
data_idx.p     = 13;
data_idx.q     = 14;
data_idx.r     = 15;
data_idx.p_err = 15;
data_idx.q_err = 17;
data_idx.r_err = 18;

t = data_out.vmsout.Time;
f = figure;

ax(1) = subplot(3,2,1); hold on;
plot(t,data_out.vmsout.Data(:,data_idx.u_cmd),'DisplayName','u cmd','LineWidth',2)
% plot(t,data_out.vmsout.Data(:,data_idx.u),'DisplayName','u')
plot(t,data_out.stateout.Body_Velocity__m_s_.Data(:,1),'DisplayName','u','LineWidth',2)
ylabel('m/s')
legend
axis([0.1 inf -inf inf])

ax(3) = subplot(3,2,3); hold on;
plot(t,data_out.vmsout.Data(:,data_idx.v_cmd),'DisplayName','v cmd','LineWidth',2)
% plot(t,data_out.vmsout.Data(:,data_idx.v),'DisplayName','v')
plot(t,data_out.stateout.Body_Velocity__m_s_.Data(:,2),'DisplayName','v','LineWidth',2)
ylabel('m/s')
legend
axis([0.1 inf -inf inf])

ax(5) = subplot(3,2,5); hold on;
plot(t,data_out.vmsout.Data(:,data_idx.w_cmd),'DisplayName','w cmd','LineWidth',2)
% plot(t,data_out.vmsout.Data(:,data_idx.w),'DisplayName','w')
plot(t,data_out.stateout.Body_Velocity__m_s_.Data(:,3),'DisplayName','w','LineWidth',2)
ylabel('m/s')
legend
axis([0.1 inf -inf inf])
xlabel('Time [s]')

ax(2) = subplot(3,2,2); hold on;
plot(t,data_out.vmsout.Data(:,data_idx.p_cmd)*180/pi,'DisplayName','p cmd','LineWidth',2)
% plot(t,data_out.vmsout.Data(:,data_idx.p)*180/pi,'DisplayName','p')
plot(t,data_out.stateout.Body_Rotational_Rate__rad_s_.Data(:,1)*180/pi,'DisplayName','p','LineWidth',2)
ylabel('deg/s')
legend
axis([0.1 inf -inf inf])

ax(4) = subplot(3,2,4); hold on;
plot(t,data_out.vmsout.Data(:,data_idx.q_cmd)*180/pi,'DisplayName','q cmd','LineWidth',2)
% plot(t,data_out.vmsout.Data(:,data_idx.q)*180/pi,'DisplayName','q')
plot(t,data_out.stateout.Body_Rotational_Rate__rad_s_.Data(:,2)*180/pi,'DisplayName','q','LineWidth',2)
ylabel('deg/s')
legend
axis([0.1 inf -inf inf])

ax(6) = subplot(3,2,6); hold on;
plot(t,data_out.vmsout.Data(:,data_idx.r_cmd)*180/pi,'DisplayName','r cmd','LineWidth',2)
% plot(t,data_out.vmsout.Data(:,data_idx.r)*180/pi,'DisplayName','r')
plot(t,data_out.stateout.Body_Rotational_Rate__rad_s_.Data(:,3)*180/pi,'DisplayName','r','LineWidth',2)
ylabel('deg/s')
legend
axis([0.1 inf -inf inf])
xlabel('Time [s]')

linkaxes(ax,'x')