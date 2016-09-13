s = fusionTrack;
sn = s.devices;

fprintf ('\nDriver version %d\n\n', s.getInt32(sn, s.FTK_OPT_DRIVER_VER));
options = s.options(sn);
options(4) % 1001
fprintf ('Minimum value of option 1001: %f\n', s.getFloat32(sn, uint32(1001), s.FTK_MIN_VAL));
fprintf ('Value of option 1001:         %f\n',s.getFloat32(sn, uint32(1001), s.FTK_VALUE));
s.setFloat32(sn, uint32(1001), single(0.31415));
fprintf ('New value of option 1001:     %f\n',s.getFloat32(sn, uint32(1001), s.FTK_VALUE));

% test geometries
geom = struct( 'geometryId', uint32( 339 ), 'pointsCount', uint32( 4 ), ...
    'positions', zeros( 3, 3 ) );
geom.positions( 1, 1 ) = 0.0;
geom.positions( 2, 1 ) = 0.0;
geom.positions( 3, 1 ) = 0.0;
geom.positions( 1, 2 ) = 40.4933;
geom.positions( 2, 2 ) = 28.5717;
geom.positions( 3, 2 ) = 0;
geom.positions( 1, 3 ) = 87.5078;
geom.positions( 2, 3 ) = 2.85718e-007;
geom.positions( 3, 3 ) = 88.0;
geom.positions( 1, 4 ) = 0.0;
geom.positions( 2, 4 ) = -44.32;
geom.positions( 3, 4 ) = 40.45;

geoms = s.geometries (sn);
fprintf ('Number of predefined geometries: %d\n', size (geoms,1));
s.setGeometry (sn,geom)
fprintf ('Adding the removed one: %d\n\n', size (s.geometries (sn),1));
tic
for i = 1:100
    frame = s.getlastFrame(sn);    
    for j = 1: size (frame.markers,1)
        fprintf ('marker %d, trans = (%f\t%f\t%f)\t rms = %f\n', ...
                 frame.markers(j).geometryId, ...
                 frame.markers(j).translationMM(1), ...
                 frame.markers(j).translationMM(2), ...
                 frame.markers(j).translationMM(3), ...
                 frame.markers(j).registrationErrorMM);
    end
end
toc

s.delete
clear s
clear all
