% Moon

% Clear Everything
clear;
clc;

% Declare Constants
moonRadius = 1737e3; %m
moonMass = 7.348e22; %kg
grav = 6.673e-11; %Gravitional Constant
dryMass = 4920; %kg
propellantMass = 10922; %kg
totalMass = propellantMass + dryMass;
parkAltitude = 62*1852; %Altitude of Parking Orbit
pdiAltitude = 50000*0.3048; %Altitude before Powered Descent
maxThrust = 10000*4.4482; %Max Thrust of Landing Module
breakBurnTime = 6*60 + 42;
breakBurn2Time = 1*60 + 44;
breakBurn2Angle = -127;
breakBurnAngle = -114.5;
approachBurnTime = 1*60 + 40;
approachBurnAngle = -170;
landingBurnTime = 19;

altitude = [];
time = [];
% Derived Constants

% Calculate Initial Orbital Velocity
parkRadius = parkAltitude + moonRadius;
parkVelocity = sqrt(grav*moonMass/parkRadius);

% Descent Orbit Insertion 
deltaV = sqrt(grav*moonMass/parkRadius)*(sqrt(2*(pdiAltitude + moonRadius)/(pdiAltitude + moonRadius + parkRadius))-1);

% Draw Moon
figure('units','normalized','outerposition',[0.2 0.2 0.8 0.8]);
hold on;
rectangle('Position',[-moonRadius,-moonRadius,2*moonRadius,2*moonRadius],...
  'Curvature',[1,1], 'FaceColor','black')

% Simulation Constants
parkTime = 100; %Time spent in parking orbit (seconds)
timeStep = 1; %Simulation Time between frames
displaySpeed = 0.001; %Real Time Simulation Spends on Each Frame (seconds)
viewSpeed = 1000;
totalFrames = 100000;
trailColour = 'r';

% Create Changing variables;
xAcc = 0;
yAcc = 0;
xVel = parkVelocity;
yVel = 0;
x = 0;
y = parkAltitude + moonRadius;
z = 0;
t = 0;
m = totalMass; %mass
thrust = 0;
angle = -pi/2;
stage = 'park';
alt = parkAltitude;
prevAlt = parkAltitude;
startTime = 0;
ticker = 0;

% Loop Through Positions
for i = 1:totalFrames
    
    % Calculate Graviational Force
    if ~strcmp(stage,'Landed')
        gravForce = grav*moonMass/(x^2 + y^2);
    else
        gravForce = 0;
    end
    
    % Calculate Velocity Magnitude
    vel = sqrt(xVel^2 + yVel^2);
    
    % Retrieve Polar Coordinates
    [theta,rho] = cart2pol(x,y);
    
    % Slow Sim During Burns
    if strcmp(stage,'DOI Burn')
        timeStep = 0.005;
    elseif strcmp(stage,'Breaking Burn') || strcmp(stage,'Breaking Burn 2') ...
            || strcmp(stage,'Approach Burn') || strcmp(stage,'Landing Burn') ...
            || strcmp(stage,'Insertion')
        timeStep = 0.05;
    elseif strcmp(stage,'Rise')
        timeStep = 0.01;
    elseif strcmp(stage,'Landing Burn')
        timeStep = 0.002;
    else
        timeStep = 1;
    end
    
    % Perform DOI Burn
    if vel > parkVelocity + deltaV && x < 0 && (strcmp(stage,'park') || strcmp(stage,'DOI Burn'))
        thrust = maxThrust;
        trailColour = 'b';
        stage = 'DOI Burn';
    elseif strcmp(stage,'DOI Burn')
        stage = 'DOI Cruise';
        thrust = 0;
        trailColour = 'g';
    end
    
    % Calculate Acceleration on Landing Module
    xAcc = -gravForce*cos(theta) - thrust*cos(theta + angle)/m;
    yAcc = -gravForce*sin(theta) - thrust*sin(theta + angle)/m;
    
    % Calculate Velocities and Positions and Time
    xVel = xVel + xAcc*timeStep;
    yVel = yVel + yAcc*timeStep;
    x = x + xVel*timeStep;
    y = y + yVel*timeStep;
    t = t + timeStep;
    ticker = ticker + timeStep;
    m = m - thrust*timeStep/(290*9.802);
    
    % Calculate Altitude
    alt = rho - moonRadius;
    % If at periapsis, display altitude
    if alt > prevAlt && strcmp(stage,'DOI Cruise') && y > 0
        stage = 'Breaking Burn';
        ticker = 0;
    end
    % Store Previous Altitude
    prevAlt = alt;
    
    % Perform Breaking Burn
    if strcmp(stage,'Breaking Burn')
        thrust = 0.94*maxThrust;
        startTime = startTime + timeStep;
        angle = angle + (deg2rad(breakBurnAngle) + pi/2)*timeStep/breakBurnTime;
    end
    
    % End Breaking Burn
    if startTime > breakBurnTime && strcmp(stage,'Breaking Burn')
        stage = 'Breaking Burn 2';
        startTime = 0;
    end
    
    % Perform Breaking Burn 2
    if strcmp(stage,'Breaking Burn 2')
        thrust = 0.6*maxThrust;
        startTime = startTime + timeStep;
        angle = angle + (deg2rad(breakBurn2Angle) - ...
            deg2rad(breakBurnAngle))*timeStep/breakBurn2Time;
    end
    
    % End Breaking Burn 2
    if startTime > breakBurn2Time && strcmp(stage,'Breaking Burn 2')
        stage = 'Approach Burn';
        startTime = 0;
    end
    
    % Perform Apprach Burn
    if strcmp(stage,'Approach Burn')
        thrust = thrust - (0.6-0.373)*maxThrust*timeStep/approachBurnTime;
        startTime = startTime + timeStep;
        angle = angle + (deg2rad(approachBurnAngle) - ...
            deg2rad(breakBurn2Angle))*timeStep/approachBurnTime;
    end
    
    % End Approach Burn
    if startTime > approachBurnTime && strcmp(stage,'Approach Burn')
        stage = 'Landing Burn';
        startTime = 0;
        thrust = 0.2*maxThrust;
    end
    
    % Perform Landing Burn
    if strcmp(stage,'Landing Burn')
        if thrust > 0
            thrust = thrust - (0.2*maxThrust - gravForce)*timeStep/landingBurnTime;
        end
        startTime = startTime + timeStep;
        angle = angle + (-pi - ...
            deg2rad(approachBurnAngle))*timeStep/landingBurnTime;
        if alt < 0
            stage = 'Landed';
            disp(m);
        end
    end
    
    % Restart when it lands and perform vertical rise
    if strcmp(stage,'Landed')
        disp(x);
        stage = 'Rise';
        xVel = 0;
        yVel = 0;
        xAcc = 0;
        yAcc = 0;
        angle = -180;
        thrust = maxThrust;
    end
    
    % End Vertical Rise 
    if strcmp(stage,'Rise') && alt > 76.2
        stage = 'Insertion';
    end
    
    % Begin Insertion
    if strcmp(stage,'Insertion')
        angle = pi/2 + deg2rad(90)*(0.65-alt/parkAltitude);
    end
    
    % End Insertion
    if strcmp(stage,'Insertion') && alt > parkAltitude
        disp(vel/parkVelocity);
        thrust = 0;
        xVel = parkVelocity*cos(theta - pi/2);
        yVel = parkVelocity*sin(theta - pi/2);
        stage = 'Done';
    end
    % Plot Spacecraft
    if mod(i,viewSpeed) == 0
        pTrail = plot(x,y,'or','MarkerSize',2,'MarkerFaceColor',trailColour);
        p = plot(x,y,'or','MarkerSize',5,'MarkerFaceColor',trailColour);
    end
    
    % Fix Axes
    axis([-1.5*moonRadius,1.5*moonRadius,-1.5*moonRadius,1.5*moonRadius]);
    axis square;
    
    % Display Position and Velocity Components
    % Textbox Position
    dim = [.65 .5 .4 .4];
    
    if mod(i,viewSpeed) == 0 && strcmp(stage,'Rise')
        alt = 0;
    end
    % Create Strings
    str3 = ['xvel = ', num2str(xVel,'%10.5e\n')];
    str4 = ['yvel = ', num2str(yVel,'%10.5e\n')];
    str5 = ['Altitude = ', num2str(alt,'%10.5e\n')];
    str6 = ['Distance to LS = ', num2str(abs(x-5.0069e5),'%10.5e\n')];
    str7 = ['Thrust = ', num2str(thrust,'%10.5e\n')];
    str8 = ['xacc = ', num2str(xAcc)];
    str9 = ['yacc = ',num2str(yAcc)];
    str10 = ['Mass = ',num2str((m-dryMass*0.5)*100/(totalMass-dryMass*0.5)),'%'];
    str = [str3 newline str4 newline str5 newline str6 newline str7 ...
        newline str8 newline str9 newline str10];
    if mod(i,viewSpeed) == 0
        %a = annotation('textbox',dim,'String',str,'FitBoxToText','on');
    end
    
    if (strcmp(stage,'Breaking Burn') || strcmp(stage,'Breaking Burn 2') ...
            || strcmp(stage,'Approach Burn') || strcmp(stage,'Landing Burn')) && ticker > 1
        ticker = ticker - 1;
        altitude(length(altitude)+1) = alt;
        time(length(time)+1) = t;
    end   
    
    if strcmp(stage,'Rise')
        hold off;
        plot(time,altitude);
        xlabel('Time (s)')
        ylabel('Altitude (m)')
        saveas(gcf,'Plot.png');
        break;
    end
    if mod(i,viewSpeed) == 0
        
        % Delay Next Plot
        pause(displaySpeed);
        
        delete(p);
        %delete(a);
    end
   
end


