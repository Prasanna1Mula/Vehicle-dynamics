% Vehicle Dynamics Simulation

% Parameters
mass = 1200; % mass of the vehicle in kg
drag_coefficient = 0.3; % drag coefficient (dimensionless)
frontal_area = 2.2; % frontal area in m^2
air_density = 1.225; % air density in kg/m^3
engine_force = 4000; % engine force in N
time_step = 0.01; % time step for simulation in seconds
total_time = 20; % total simulation time in seconds

% Initialize variables
time = 0:time_step:total_time; % time vector
velocity = zeros(size(time)); % initialize velocity vector
position = zeros(size(time)); % initialize position vector

% Check that initial parameters are valid
if engine_force <= 0
    error('Engine force must be positive.');
end

% Simulation loop
for i = 1:length(time)-1
    % Calculate drag force
    drag_force = 0.5 * drag_coefficient * frontal_area * air_density * (velocity(i)^2);
    
    % Calculate net force
    net_force = engine_force - drag_force;
    
    % Calculate acceleration
    acceleration = net_force / mass;
    
    % Update velocity and position using Euler integration
    velocity(i+1) = velocity(i) + acceleration * time_step;
    position(i+1) = position(i) + velocity(i) * time_step;
end

% Plot results
figure;

% Plot velocity over time
subplot(2, 1, 1);
plot(time, velocity);
title('Velocity vs Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
grid on;

% Plot position over time
subplot(2, 1, 2);
plot(time, position);
title('Position vs Time');
xlabel('Time (s)');
ylabel('Position (m)');
grid on;

% Display final results
fprintf('Final Velocity: %.2f m/s\n', velocity(end));
fprintf('Total Distance Traveled: %.2f m\n', position(end));
