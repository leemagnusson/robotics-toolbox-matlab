
if ( emulateMake( 'CPP_fusionTrack' ) )
	disp( 'Compiling CPP_fusionTrack.obj' )
	mex -g -c -I../include -I../include/interface -I../include/geometry CPP_fusionTrack.cpp
end
if ( emulateMake( 'CPP_geometry' ) )
	disp( 'Compiling CPP_geometry.obj' )
	mex -g -c -I../include -I../include/interface -I../include/geometry CPP_geometry.cpp
end
if ( emulateMake( 'CPP_frame' ) )
	disp( 'Compiling CPP_frame.obj' )
	mex -g -c -I../include -I../include/interface -I../include/geometry CPP_frame.cpp
end
if ( emulateMake( 'CPP_option' ) )
	disp( 'Compiling CPP_option.obj' )
	mex -g -c -I../include -I../include/interface -I../include/geometry CPP_option.cpp
end

disp( strcat ('Linking CPP_fusionTrack.', mexext));

if (strcmp (mexext,'mexw64'))
    mex -g CPP_fusionTrack.obj CPP_geometry.obj CPP_frame.obj CPP_option.obj ...
        "../lib/fusionTrack64.lib" C:\MATLAB\R2015b\sys\lcc64\lcc64\lib64\winmm.lib ...
        C:\MATLAB\R2015b\sys\lcc64\lcc64\lib64\ws2_32.lib ...
        -output CPP_fusionTrack
else
    mex -g CPP_fusionTrack.obj CPP_geometry.obj CPP_frame.obj CPP_option.obj ...
        "../lib/fusionTrack32.lib" winmm.lib ws2_32.lib ...
        -output CPP_fusionTrack
end
