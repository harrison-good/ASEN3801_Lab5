function v_out = TransformFromInertialToBody(v_in, euler_angles)

phi = euler_angles(1);
theta = euler_angles(2);
psi = euler_angles(3);


% Direction Cosine Matrix (Body → Inertial)
DCM = [ cos(theta)*cos(psi),  sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi),  cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
        cos(theta)*sin(psi),  sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi),  cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
       -sin(theta),           sin(phi)*cos(theta),                                 cos(phi)*cos(theta) ];

% Inertial → Body transformation
v_out = DCM' * v_in;

end