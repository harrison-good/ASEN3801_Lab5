function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
    % extract variables from state array
    % assumes each row is a different variable
    xE = aircraft_state_array(:,1);
    yE = aircraft_state_array(:,2);
    zE = aircraft_state_array(:,3);
    roll = aircraft_state_array(:,4);
    pitch = aircraft_state_array(:,5);
    yaw = aircraft_state_array(:,6);
    u = aircraft_state_array(:,7);
    v = aircraft_state_array(:,8);
    w = aircraft_state_array(:,9);
    p = aircraft_state_array(:,10);
    q = aircraft_state_array(:,11);
    r = aircraft_state_array(:,12);
    de = rad2deg(control_input_array(:,1));
    da = rad2deg(control_input_array(:,2));
    dr = rad2deg(control_input_array(:,3));
    dt = control_input_array(:,4);

    outputFolder = 'OverleafPlots'; 
    if ~isfolder(outputFolder)
        mkdir(outputFolder); % Create folder if it doesn't exist
    end

    % plot inertial position assuming unit is meters
    figure(fig(1))
    subplot(3,1,1)
    plot(time, xE, col)
    title('Inertial X')
    xlabel('Time (s)')
    ylabel('X (m)')
    hold on
    subplot(3,1,2)
    plot(time, yE, col)
    title('Inertial Y')
    xlabel('Time (s)')
    ylabel("Y (m)")
    hold on
    subplot(3,1,3)
    plot(time, zE, col)
    title('Inertial Z')
    xlabel('Time (s)')
    ylabel("Z (m)")
    hold on
    filename = fullfile(outputFolder, strcat(fig(1)+".png"));
    saveas(fig(1), filename);

    % plot euler angles assuming unit is radians
    figure(fig(2))
    subplot(3,1,1)
    plot(time, roll, col)
    title('Euler Roll')
    xlabel('Time (s)')
    ylabel("Euler Roll (rad)")
    hold on
    subplot(3,1,2)
    plot(time, pitch, col)
    title('Euler Pitch')
    xlabel('Time (s)')
    ylabel("Euler Pitch (rad)")
    hold on
    subplot(3,1,3)
    plot(time, yaw, col)
    title('Euler Yaw')
    xlabel('Time (s)')
    ylabel("Euler Yaw (rad)")
    hold on
    filename = fullfile(outputFolder, strcat(fig(2)+".png"));
    saveas(fig(2), filename);

    % plot velocity assuming unit is m/s
    figure(fig(3))
    subplot(3,1,1)
    plot(time, u, col)
    title('Velocity Body x')
    xlabel('Time (s)')
    ylabel("Velocity Body x (m/s)")
    hold on
    subplot(3,1,2)
    plot(time, v, col)
    title('Velocity Body y')
    xlabel('Time (s)')
    ylabel("Velocity Body y (m/s)")
    hold on
    subplot(3,1,3)
    plot(time, w, col)
    title('Velocity Body z')
    xlabel('Time (s)')
    ylabel("Velocity Body z (m/s)")
    hold on
    filename = fullfile(outputFolder, strcat(fig(3)+".png"));
    saveas(fig(3), filename);

    % plot angular velocity assuming unit is rad/s
    figure(fig(4))
    subplot(3,1,1)
    plot(time, p, col)
    title('Roll Rate')
    xlabel('Time (s)')
    ylabel("Roll Rate (rad/s)")
    hold on
    subplot(3,1,2)
    plot(time, q, col)
    title('Pitch Rate')
    xlabel('Time (s)')
    ylabel("Pitch Rate (rad/s)")
    hold on
    subplot(3,1,3)
    plot(time, r, col)
    title('Yaw Rate')
    xlabel('Time (s)')
    ylabel("Yaw Rate (rad/s)")
    hold on
    filename = fullfile(outputFolder, strcat(fig(4)+".png"));
    saveas(fig(4), filename);

    % plot control inputs assuming units are N and Nm
    figure(fig(5))
    subplot(4,1,1)
    plot(time, de, col)
    title('Elevator Input')
    xlabel("Time (s)")
    ylabel('\delta_e (deg)')
    hold on
    subplot(4,1,2)
    plot(time, da, col)
    title('Aileron Input')
    xlabel("Time (s)")
    ylabel('\delta_a (deg)')
    hold on
    subplot(4,1,3)
    plot(time, dr, col)
    title("Rudder Input")
    xlabel("Time (s)")
    ylabel('\delta_r (deg)')
    hold on
    subplot(4,1,4)
    plot(time, dt, col)
    title('Thrust Input')
    xlabel("Time (s)")
    ylabel('\delta_t')
    ylim ([0 1]);
    hold on
    filename = fullfile(outputFolder, strcat(fig(5)+".png"));
    saveas(fig(5), filename);

    % plot the flight path with markers for start and end
    figure(fig(6))
    plot3(xE(1), yE(1), -zE(1), 'og', xE, yE, -zE, col, xE(end), yE(end), -zE(end), 'or')
    title('Aircraft Flight Path')
    xlabel('X (m)')
    ylabel('Y (m)')
    zlabel('Height (m)')
    legend('Start', '', 'End')
    hold on
    grid on
    grid minor
    filename = fullfile(outputFolder, strcat(fig(6)+".png"));
    saveas(fig(6), filename);
end