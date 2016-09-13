classdef fusionTrack < handle

    % Wrapper of the FusionTrack library
    % See fusionTrack C API for more detailed info
    % Note that most of the parameters are typed
    %
	% What is currently not wrapped:
	% - no get/set for data options
	% - no access to low-level information (left/right images/fiducials)
    %
    % Supported Matlab version: 
    % - Matlab R2009b 32 bits
    % - Matlab R2012b 64 bits
	
    properties (SetAccess = private, Hidden = true)
        % fusionTrack library handle.
        LibraryHandle;
    end
    
    properties (Constant)
        FTK_OPT_DRIVER_VER  = uint32(0004);
        FTK_MIN_VAL = uint8(0);
        FTK_MAX_VAL = uint8(1);
        FTK_DEF_VAL = uint8(2);
        FTK_VALUE   = uint8(3);
		version = 1; % Current version of the wrapper
    end
    
    methods
        
        % constructor
        % e.g: s = fusionTrack
        function this = fusionTrack(varargin)
            this.LibraryHandle = CPP_fusionTrack('new', varargin{:});
        end
        
        % destructor (required)
        % e.g: s.delete
		function delete(this)
            CPP_fusionTrack('delete', this.LibraryHandle);
        end
        
        % enumerate the devices (return a list of serial numbers)
        % e.g: sn = s.devices
		function varargout = devices(this, varargin)
			[varargout{1:nargout}] = CPP_fusionTrack('devices', this.LibraryHandle, varargin{:});
        end
        
        % enumerate the options 
        % e.g: options = s.options(sn)
		function varargout = options(this, varargin)
			[varargout{1:nargout}] = CPP_fusionTrack('options', this.LibraryHandle, varargin{:});
        end
        
        % get an int32 option
        % e.g: s.getInt32(sn, s.FTK_OPT_DRIVER_VER)
		function varargout = getInt32(this, varargin)
			[varargout{1:nargout}] = CPP_fusionTrack('getint32', this.LibraryHandle, varargin{:});
        end
        
        % set an int32 option
        % e.g: s.setInt32(sn, s.FTK_OPT_PREDEF_GEOM, int32(1))
		function varargout = setInt32(this, varargin)
			[varargout{1:nargout}] = CPP_fusionTrack('setint32', this.LibraryHandle, varargin{:});
        end
        
        % get an float32 option
        % e.g: s.getFloat32(sn, uint32(1001), s.FTK_MIN_VAL)
		function varargout = getFloat32(this, varargin)
			[varargout{1:nargout}] = CPP_fusionTrack('getfloat32', this.LibraryHandle, varargin{:});
        end
        
        % set an float32 option
        % e.g: s.setFloat32(sn, uint32(1001), single(0.31415))
		function varargout = setFloat32(this, varargin)
			[varargout{1:nargout}] = CPP_fusionTrack('setfloat32', this.LibraryHandle, varargin{:});
        end
        
        % get geometries
        % e.g: geoms = s.geometries (sn);
		function varargout = geometries(this, varargin)
			[varargout{1:nargout}] = CPP_fusionTrack('geometries', this.LibraryHandle, varargin{:});
        end
        
        % clear a geometry given its id
        % s.clearGeometry (sn,uint32(1))
		function varargout = clearGeometry(this, varargin)
			[varargout{1:nargout}] = CPP_fusionTrack('cleargeometry', this.LibraryHandle, varargin{:});
        end
        
        % set a geometry
        % g = geoms(1); s.setGeometry (sn,g)
		function varargout = setGeometry(this, varargin)
			[varargout{1:nargout}] = CPP_fusionTrack('setgeometry', this.LibraryHandle, varargin{:});
        end
        
        % get the lattest available frame
        % frame = s.getlastFrame(sn)
		function varargout = getlastFrame(this, varargin)
			[varargout{1:nargout}] = CPP_fusionTrack('getframe', this.LibraryHandle, varargin{:});
        end
    end      
    
end

