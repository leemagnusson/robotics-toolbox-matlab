
function state = emulateMake( fileNameWithoutExt )

	allFiles = dir( strcat( fileNameWithoutExt, '.*' ) );

	objTimeStamp = 0;
	newTimeStamp = 0;

	for i = 1:length( allFiles )
		
		if ( ~isempty( strfind( allFiles( i ).name, mexext ) ) )
			continue;
		elseif ( ~isempty( strfind( allFiles( i ).name, '.obj' ) ) )
			objTimeStamp = allFiles( i ).datenum;
		elseif ( allFiles( i ).datenum > newTimeStamp )
			newTimeStamp = allFiles( i ).datenum;
		end
		
	end

	if ( objTimeStamp > newTimeStamp )
		state = 0;
	else
		state = 1;
	end

end