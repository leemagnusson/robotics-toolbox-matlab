function varargout = Arm_Setup_UI(varargin)
% ARM_SETUP_UI MATLAB code for Arm_Setup_UI.fig
%      ARM_SETUP_UI, by itself, creates a new ARM_SETUP_UI or raises the existing
%      singleton*.
%
%      H = ARM_SETUP_UI returns the handle to a new ARM_SETUP_UI or the handle to
%      the existing singleton*.
%
%      ARM_SETUP_UI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ARM_SETUP_UI.M with the given input arguments.
%
%      ARM_SETUP_UI('Property','Value',...) creates a new ARM_SETUP_UI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Arm_Setup_UI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Arm_Setup_UI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Arm_Setup_UI

% Last Modified by GUIDE v2.5 24-Mar-2016 17:18:03

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @Arm_Setup_UI_OpeningFcn, ...
    'gui_OutputFcn',  @Arm_Setup_UI_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Arm_Setup_UI is made visible.
function Arm_Setup_UI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Arm_Setup_UI (see VARARGIN)

% Choose default command line output for Arm_Setup_UI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Arm_Setup_UI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Arm_Setup_UI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in set_robot.
function set_robot_Callback(hObject, eventdata, handles)
% hObject    handle to set_robot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q_setup = get(handles.uitable_jnt, 'data') * pi / 180;
load('URDF_info.mat')
load('VertexData_origin.mat')
load('VertexData_Hernia_Body.mat');
ARM_setup_hernia;
axes(handles.disp_fig)
cla
hold on
axis equal
view(3)
view(str2double(get(handles.az_fig,'String')),str2double(get(handles.el_fig,'String')));
if get(handles.check_Frames,'Value') == 1
    draw_coordinate_system([0.1 0.1 0.1],eye(3),[0;0;0],'rgb','w')
    hold on
    draw_coordinate_system([0.1 0.1 0.1],R_Base1,Base1,'rgb','b1')
    hold on
    draw_coordinate_system([0.1 0.1 0.1],R_Base2,Base2,'rgb','b2')
    hold on
    draw_coordinate_system([0.1 0.1 0.1],R_Base3,Base3,'rgb','b3')
    hold on
    draw_coordinate_system([0.02 0.02 0.02],R_Trocar1,Trocar1,'rgb','t1')
    hold on
    draw_coordinate_system([0.02 0.02 0.02],R_Trocar2,Trocar2,'rgb','t2')
    hold on
    draw_coordinate_system([0.02 0.02 0.02],R_Trocar3,Trocar3,'rgb','t3')
    hold on
end
if get(handles.check_Body,'Value') == 1
    VertexData_hernia_tran = transformSTL(VertexData_Hernia_Body,R_Hernia_Body,Hernia_Body);
    rgba = [0 0 1 0.1];
    plotSTL_gui(VertexData_hernia_tran,rgba,handles.disp_fig)
    hold on
end
for index = 1:3
    q_rcm = convert2rcm(q_setup(:,index));
    Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm,base_T_setup(:,:,index));
    Draw_Robot_Arm_gui(Arm_Kinematics1,VertexData_origin,handles.disp_fig);
    index_jnt_limit = 1;
end
plot3(Hernia(1),Hernia(2),Hernia(3),'Marker','o','MarkerSize',10)
axis([-0.45 0.6 -0.5 0.8 -0.2 0.65])
light('Position',[1 3 2]);
light('Position',[-3 -1 -3]);
drawnow;

% --- Executes on button press in get_jnt.
function get_jnt_Callback(hObject, eventdata, handles)
% hObject    handle to get_jnt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% handles.jnt_file_name.String
load(handles.jnt_file_name.String);
load('jnt_limit_active.mat');
for index = 1:3
    for i = 1 :  length(jnt_limit_active)
        jnt_percentage(i,index) = (q_init_setup(i,index) - jnt_limit_active(1,i)) / (jnt_limit_active(2,i) - jnt_limit_active(1,i));
    end
end
q_setup(:,1:3) = q_init_setup * 180 / pi;
q_setup(:,4:6) = jnt_percentage * 100;
size_q = size(q_setup);
set(handles.uitable_jnt,'data',q_setup);




function jnt_file_name_Callback(hObject, eventdata, handles)
% hObject    handle to jnt_file_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of jnt_file_name as text
%        str2double(get(hObject,'String')) returns contents of jnt_file_name as a double


% --- Executes during object creation, after setting all properties.
function jnt_file_name_CreateFcn(hObject, eventdata, handles)
% hObject    handle to jnt_file_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in reset_jnt_value.
function reset_jnt_value_Callback(hObject, eventdata, handles)
% hObject    handle to reset_jnt_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q_init_setup = zeros(11,3);
load('jnt_limit_active.mat')
for index = 1:3
    for i = 1 :  length(jnt_limit_active)
        jnt_percentage(i,index) = (q_init_setup(i,index) - jnt_limit_active(1,i)) / (jnt_limit_active(2,i) - jnt_limit_active(1,i));
    end
end
q_setup(:,4:6) = jnt_percentage * 100;
set(handles.uitable_jnt,'data',q_setup);


% --- Executes on button press in zoom.
function zoom_Callback(hObject, eventdata, handles)
% hObject    handle to zoom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.disp_fig)
zoom on

% --- Executes on button press in Rotate.
function Rotate_Callback(hObject, eventdata, handles)
% hObject    handle to Rotate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.disp_fig)
h=rotate3d;
set(h,'Enable','on');


% --- Executes when entered data in editable cell(s) in uitable_jnt.
function uitable_jnt_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to uitable_jnt (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)
q_setup = get(handles.uitable_jnt, 'data') * pi / 180;
load('jnt_limit_active.mat')
for index = 1:3
    for i = 1 :  length(jnt_limit_active)
        jnt_percentage(i,index) = (q_setup(i,index) - jnt_limit_active(1,i)) / (jnt_limit_active(2,i) - jnt_limit_active(1,i));
    end
end
q_setup(:,4:6) = jnt_percentage * 100;
set(handles.uitable_jnt,'data',q_setup);


% --- Executes on button press in drag.
function drag_Callback(hObject, eventdata, handles)
% hObject    handle to drag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.disp_fig)
pan on;


% --- Executes on button press in check_Body.
function check_Body_Callback(hObject, eventdata, handles)
% hObject    handle to check_Body (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of check_Body


% --- Executes on button press in check_Frames.
function check_Frames_Callback(hObject, eventdata, handles)
% hObject    handle to check_Frames (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of check_Frames



function az_fig_Callback(hObject, eventdata, handles)
% hObject    handle to az_fig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of az_fig as text
%        str2double(get(hObject,'String')) returns contents of az_fig as a double


% --- Executes during object creation, after setting all properties.
function az_fig_CreateFcn(hObject, eventdata, handles)
% hObject    handle to az_fig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function el_fig_Callback(hObject, eventdata, handles)
% hObject    handle to el_fig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of el_fig as text
%        str2double(get(hObject,'String')) returns contents of el_fig as a double


% --- Executes during object creation, after setting all properties.
function el_fig_CreateFcn(hObject, eventdata, handles)
% hObject    handle to el_fig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
