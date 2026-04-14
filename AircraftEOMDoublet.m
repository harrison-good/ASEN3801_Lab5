function xdot = AircraftEOMDoublet(time, aircraft_state, aircraft_surfaces, doublet_size, doublet_time, wind_inertial, aircraft_parameters)

% doublet implementation
if time > 0 && time <= doublet_time
    aircraft_surfaces(1,1) = aircraft_surfaces(1,1) + doublet_size; 

elseif time > doublet_time && time <= (2 * doublet_time)
    aircraft_surfaces(1,1) = aircraft_surfaces(1,1) - doublet_size;

elseif time > (2 * doublet_time)
    aircraft_surfaces(1,1) = aircraft_surfaces(1,1);

end

density = atmosisa(-aircraft_state(3));
[aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);

g = aircraft_parameters.g;
m = aircraft_parameters.m;

% dissecting statevector
phi = aircraft_state(4);
theta = aircraft_state(5);
psi = aircraft_state(6);
uE = aircraft_state(7);
vE = aircraft_state(8);
wE = aircraft_state(9);
p = aircraft_state(10);
q = aircraft_state(11);
r = aircraft_state(12);

xdot = zeros(12,1);
DCM = [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
       cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
       -sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)];
xdot(1:3) = DCM*aircraft_state(7:9);

T = [1 sin(phi)*tan(theta) cos(phi)*tan(theta)
    0 cos(phi) -sin(phi)
    0 sin(phi)*sec(theta) cos(phi)*sec(theta)];
xdot(4:6) = T * aircraft_state(10:12);

xdot(7:9) = [r*vE-q*wE; p*wE-r*uE;q*uE-p*vE] +... 
    g*[-sin(theta);cos(theta)*sin(phi);cos(theta)*cos(phi)] +...
    (1/m)*(aero_forces);

a = aircraft_parameters;

Gamma = a.Ix*a.Iz - a.Ixz^2;
Gamma1 = a.Ixz*(a.Ix-a.Iy+a.Iz)/Gamma;
Gamma2 = (a.Iz*(a.Iz-a.Iy)+a.Ixz^2)/Gamma;
Gamma3 = a.Iz/Gamma;
Gamma4 = a.Ixz/Gamma;
Gamma5 = (a.Iz-a.Ix)/a.Iy;
Gamma6 = a.Ixz/a.Iy;
Gamma7 = (a.Ix*(a.Ix-a.Iy)+a.Ixz^2)/Gamma;
Gamma8 = a.Ix/Gamma;

L = aero_moments(1);
M = aero_moments(2);
N = aero_moments(3);

xdot(10:12) = [Gamma1*p*q - Gamma2*q*r; Gamma5*p*r-Gamma6*(p^2-r^2); Gamma7*p*q - Gamma1*q*r] +...
    [Gamma3*L + Gamma4*N; 1/a.Iy*M; Gamma4*L+Gamma8*N];


