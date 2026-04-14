function [wind_body] = TransformFromInertialToBody(wind_inertial,angles321)

DCM = angle2dcm(angles321(1),angles321(2),angles321(3),'ZYX');
wind_body = DCM*wind_inertial';

end

