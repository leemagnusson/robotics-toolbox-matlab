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

% Last Modified by GUIDE v2.5 19-Apr-2016 11:57:07

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
addpath(genpath([pwd, '../lib']));
addpath(genpath([pwd, '../data']));
addpath(genpath([pwd, '../export']));
addpath(genpath([pwd, '../init']));
addpath(genpath([pwd, '../projects']));
% Choose default command line output for Arm_Setup_UI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
% set(handles.arm1_drawer,'Value',2);
% set(handles.arm2_drawer,'Value',1);
% set(handles.arm3_drawer,'Value',5);

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
scale_deg_to_rad = pi/180;
num_arm = 4;
trocar_dis = 0.0377 + 0.05;
q_setup = get(handles.uitable_jnt, 'data');
q_setup([1:7 9:11],1:num_arm) = q_setup([1:7 9:11],1:num_arm) * scale_deg_to_rad;
load('urdf_info_1.0.mat')
load('vertex_arm_origin_1.0.mat')
load('arm_version_1.0.mat');
load('coupling_matrix.mat');
load('vertex_bed.mat');
if get(handles.table_adapter_version,'Value') == 1
    load('vertex_bed_adapter_new.mat');
else
    load('vertex_bed_adapter.mat');
end
switch get(handles.select_procedure,'Value')
    case 1
        InitHerniaSetup;
        load('vertex_patient_body.mat');
    case 2
        InitThoracicSetup;
        load('vertex_patient_body.mat');
    case 3
        InitHysterectomySetup;
        load('vertex_patient_body.mat');
    case 4
        InitGastricBypassSetup;
        load('vertex_patient_body.mat');
    case 5
        InitProstatectomySetup;
        load('vertex_patient_body.mat');
    case 6
        InitLarSetup;
        load('vertex_patient_body.mat');
    case 7
        InitOmentectomySetup;
        load('vertex_patient_body.mat');
    otherwise
        error('No such procedure');
end

robot_object1 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object2 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object3 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object4 = RobotClass(urdf_link_input,urdf_joint_input);
robot_arms = {robot_object1;robot_object2;robot_object3;robot_object4};

axes(handles.disp_fig)
q_bed_adapter = [str2double(get(handles.drawer11,'String'))*scale_deg_to_rad str2double(get(handles.drawer12,'String')) 0;str2double(get(handles.drawer21,'String'))*scale_deg_to_rad str2double(get(handles.drawer22,'String')) 0;str2double(get(handles.drawer31,'String'))*scale_deg_to_rad str2double(get(handles.drawer32,'String')) 0;str2double(get(handles.drawer41,'String'))*scale_deg_to_rad str2double(get(handles.drawer42,'String')) 0]';
selected_bed_adapter = [get(handles.arm1_drawer,'Value') get(handles.arm2_drawer,'Value') get(handles.arm3_drawer,'Value') get(handles.arm4_drawer,'Value')];
cla
hold on
axis equal
view(str2double(get(handles.az_fig,'String')),str2double(get(handles.el_fig,'String')));
if get(handles.check_Frames,'Value') == 1
    DrawCoordinateSystem([0.1 0.1 0.1],eye(3),[0;0;0],'rgb','w')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar1,'rgb','t1')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar2,'rgb','t2')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar3,'rgb','t3')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar4,'rgb','t3')
    hold on
    
end
if get(handles.check_Body,'Value') == 1
    vertex_patient_transformed = transformSTL(vertex_patient_body,rotation_patient,translation_patient);
    rgba = [0 0 1 0.1];
    PlotStl(vertex_patient_transformed,rgba,handles.disp_fig)
    hold on
end
for index_robot = 1:num_arm
    if selected_bed_adapter(index_robot) ~=10
        frames_bed_adapter = CalculateBedAdapterFK(q_bed_adapter(:,index_robot),frames_bed_adapter_base(:,:,selected_bed_adapter(index_robot)));
        DrawBedAdapter(frames_bed_adapter,vertex_bed_adapter,[1 0 0 1],handles.disp_fig)
        hold on
        robot_arms{index_robot}.transformation_base_ = frames_bed_adapter(:,:,end);
        robot_arms{index_robot}.CalculateFK(q_setup(1:11,index_robot));
        robot_arms{index_robot}.DrawRobotGUI(vertex_arm_origin,handles.disp_fig);
        if get(handles.check_Frames,'Value') == 1
            DrawCoordinateSystem([0.1 0.1 0.1],frames_bed_adapter(1:3,1:3,end),frames_bed_adapter(1:3,4,end),'rgb',strcat('b',num2str(index_robot)))
            hold on
            DrawCoordinateSystem([0.1 0.1 0.1],robot_arms{index_robot}.frames_(1:3,1:3,7),robot_arms{index_robot}.frames_(1:3,4,7),'rgb',6)
            hold on
        end
    end
end
plot3(translation_target(1),translation_target(2),translation_target(3),'Marker','o','MarkerSize',10)
% p1_tip = robot_arms{2}.frames_(1:3,4,18) + robot_arms{2}.frames_(1:3,3,18) * trocar_dis;
% p2_tip = robot_arms{3}.frames_(1:3,4,18) + robot_arms{3}.frames_(1:3,3,18) * trocar_dis;
% plot3(p1_tip(1),p1_tip(2),p1_tip(3),'Marker','o','MarkerSize',10,'MarkerFaceColor','r')
% plot3(p2_tip(1),p2_tip(2),p2_tip(3),'Marker','o','MarkerSize',10,'MarkerFaceColor','r')
DrawBed(vertex_bed,[0.0 0.7 0.0 1],handles.disp_fig)
hold on
axis([-1.2 1 -0.8 0.8 -0.2 2.8])
light('Position',[1 3 2]);
light('Position',[-3 -1 -3]);
if get(handles.grid_enable,'Value') == 1
    grid on
    grid minor
else
    grid off
end
drawnow;

% --- Executes on button press in get_jnt.
function get_jnt_Callback(hObject, eventdata, handles)
% hObject    handle to get_jnt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% handles.jnt_file_name.String
num_arm = 4;
load(handles.jnt_file_name.String);
load('joint_limit_active_1.0.mat');
q_setup(:,1:num_arm) = q_init_setup(:,1:num_arm);
q_setup([1:7  9:11],1:num_arm) = q_setup([1:7  9:11],1:num_arm) * 180 / pi;
for index = 1:num_arm
    for i = 1 :  length(jnt_limit_active)
        joint_percentage(i,index) = (q_init_setup(i,index) - jnt_limit_active(1,i)) / (jnt_limit_active(2,i) - jnt_limit_active(1,i));
    end
end

q_setup(:,num_arm+1:num_arm*2) = joint_percentage * 100;
set(handles.uitable_jnt,'data',q_setup);
for index2 = 1: num_arm
    if selected_bed_adapter(index2) == 0
        selected_bed_adapter(index2) = 10;
    end
end
set(handles.arm1_drawer,'Value',selected_bed_adapter(1));
set(handles.arm2_drawer,'Value',selected_bed_adapter(2));
set(handles.arm3_drawer,'Value',selected_bed_adapter(3));
set(handles.arm4_drawer,'Value',selected_bed_adapter(4));
set(handles.drawer11,'String',num2str(q_bed_adapter(1,1) * 180 / pi));
set(handles.drawer12,'String',num2str(q_bed_adapter(2,1)));
set(handles.drawer21,'String',num2str(q_bed_adapter(1,2) * 180 / pi));
set(handles.drawer22,'String',num2str(q_bed_adapter(2,2)));
set(handles.drawer31,'String',num2str(q_bed_adapter(1,3) * 180 / pi));
set(handles.drawer32,'String',num2str(q_bed_adapter(2,3)));
set(handles.drawer41,'String',num2str(q_bed_adapter(1,4) * 180 / pi));
set(handles.drawer42,'String',num2str(q_bed_adapter(2,4)));

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
num_arm = 4;
q_init_setup = zeros(11,num_arm);
load('joint_limit_active_1.0.mat')
for index = 1:num_arm
    for i = 1 :  length(jnt_limit_active)
        joint_percentage(i,index) = (q_init_setup(i,index) - jnt_limit_active(1,i)) / (jnt_limit_active(2,i) - jnt_limit_active(1,i));
    end
end
q_setup(:,num_arm+1:num_arm*2) = joint_percentage * 100;
% set(handles.arm1_drawer,'Value',1);
% set(handles.arm2_drawer,'Value',1);
% set(handles.arm3_drawer,'Value',1);
% set(handles.drawer11,'String',num2str(0));
% set(handles.drawer12,'String',num2str(0));
% set(handles.drawer21,'String',num2str(0));
% set(handles.drawer22,'String',num2str(0));
% set(handles.drawer31,'String',num2str(0));
% set(handles.drawer32,'String',num2str(0));
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
num_arm = 4;
q_setup = get(handles.uitable_jnt, 'data');
q_setup([1:7 9:11],1:num_arm) = q_setup([1:7 9:11],1:num_arm) * pi / 180;
load('joint_limit_active_1.0.mat')
for index = 1:num_arm
    for i = 1 :  length(jnt_limit_active)
        joint_percentage(i,index) = (q_setup(i,index) - jnt_limit_active(1,i)) / (jnt_limit_active(2,i) - jnt_limit_active(1,i));
    end
end
q_setup(:,num_arm+1:num_arm*2) = joint_percentage * 100;
q_setup([1:7 9:11],1:num_arm) = q_setup([1:7 9:11],1:num_arm) * 180 / pi;
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


% --- Executes on selection change in arm1_drawer.
function arm1_drawer_Callback(hObject, eventdata, handles)
% hObject    handle to arm1_drawer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns arm1_drawer contents as cell array
%        contents{get(hObject,'Value')} returns selected item from arm1_drawer


% --- Executes during object creation, after setting all properties.
function arm1_drawer_CreateFcn(hObject, eventdata, handles)
% hObject    handle to arm1_drawer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in arm2_drawer.
function arm2_drawer_Callback(hObject, eventdata, handles)
% hObject    handle to arm2_drawer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns arm2_drawer contents as cell array
%        contents{get(hObject,'Value')} returns selected item from arm2_drawer


% --- Executes during object creation, after setting all properties.
function arm2_drawer_CreateFcn(hObject, eventdata, handles)
% hObject    handle to arm2_drawer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in arm3_drawer.
function arm3_drawer_Callback(hObject, eventdata, handles)
% hObject    handle to arm3_drawer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns arm3_drawer contents as cell array
%        contents{get(hObject,'Value')} returns selected item from arm3_drawer


% --- Executes during object creation, after setting all properties.
function arm3_drawer_CreateFcn(hObject, eventdata, handles)
% hObject    handle to arm3_drawer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function drawer11_Callback(hObject, eventdata, handles)
% hObject    handle to drawer11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of drawer11 as text
%        str2double(get(hObject,'String')) returns contents of drawer11 as a double


% --- Executes during object creation, after setting all properties.
function drawer11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to drawer11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function drawer12_Callback(hObject, eventdata, handles)
% hObject    handle to drawer12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of drawer12 as text
%        str2double(get(hObject,'String')) returns contents of drawer12 as a double


% --- Executes during object creation, after setting all properties.
function drawer12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to drawer12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function drawer21_Callback(hObject, eventdata, handles)
% hObject    handle to drawer21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of drawer21 as text
%        str2double(get(hObject,'String')) returns contents of drawer21 as a double


% --- Executes during object creation, after setting all properties.
function drawer21_CreateFcn(hObject, eventdata, handles)
% hObject    handle to drawer21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function drawer22_Callback(hObject, eventdata, handles)
% hObject    handle to drawer22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of drawer22 as text
%        str2double(get(hObject,'String')) returns contents of drawer22 as a double


% --- Executes during object creation, after setting all properties.
function drawer22_CreateFcn(hObject, eventdata, handles)
% hObject    handle to drawer22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function drawer31_Callback(hObject, eventdata, handles)
% hObject    handle to drawer31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of drawer31 as text
%        str2double(get(hObject,'String')) returns contents of drawer31 as a double


% --- Executes during object creation, after setting all properties.
function drawer31_CreateFcn(hObject, eventdata, handles)
% hObject    handle to drawer31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function drawer32_Callback(hObject, eventdata, handles)
% hObject    handle to drawer32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of drawer32 as text
%        str2double(get(hObject,'String')) returns contents of drawer32 as a double


% --- Executes during object creation, after setting all properties.
function drawer32_CreateFcn(hObject, eventdata, handles)
% hObject    handle to drawer32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in save_jnt.
function save_jnt_Callback(hObject, eventdata, handles)
% hObject    handle to save_jnt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
num_arm = 4;
scale_deg_to_rad =  pi / 180;
q_setup = get(handles.uitable_jnt, 'data');
q_setup([1:7 9:11],1:num_arm) = q_setup([1:7 9:11],1:num_arm) * scale_deg_to_rad;
q_init_setup = q_setup(:,1:num_arm);
q_bed_adapter = [str2double(get(handles.drawer11,'String'))*scale_deg_to_rad str2double(get(handles.drawer12,'String')) 0;str2double(get(handles.drawer21,'String'))*scale_deg_to_rad str2double(get(handles.drawer22,'String')) 0;str2double(get(handles.drawer31,'String'))*scale_deg_to_rad str2double(get(handles.drawer32,'String')) 0;str2double(get(handles.drawer41,'String'))*scale_deg_to_rad str2double(get(handles.drawer42,'String')) 0]';
selected_bed_adapter = [get(handles.arm1_drawer,'Value') get(handles.arm2_drawer,'Value') get(handles.arm3_drawer,'Value') get(handles.arm4_drawer,'Value')];
for index2 = 1:num_arm
    if selected_bed_adapter(index2) == 10
        selected_bed_adapter(index2) = 0;
    end
end
save(strcat('../data/',handles.save_file_name.String),'q_init_setup','q_bed_adapter','selected_bed_adapter');


function save_file_name_Callback(hObject, eventdata, handles)
% hObject    handle to save_file_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of save_file_name as text
%        str2double(get(hObject,'String')) returns contents of save_file_name as a double


% --- Executes during object creation, after setting all properties.
function save_file_name_CreateFcn(hObject, eventdata, handles)
% hObject    handle to save_file_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in cal_robot.
function cal_robot_Callback(hObject, eventdata, handles)
% hObject    handle to cal_robot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
num_arm = 4;
load('joint_limit_active_1.0.mat');
scale_deg_to_rad = pi/180;
q_bed_adapter = [str2double(get(handles.drawer11,'String'))*scale_deg_to_rad str2double(get(handles.drawer12,'String')) 0;str2double(get(handles.drawer21,'String'))*scale_deg_to_rad str2double(get(handles.drawer22,'String')) 0;str2double(get(handles.drawer31,'String'))*scale_deg_to_rad str2double(get(handles.drawer32,'String')) 0;str2double(get(handles.drawer41,'String'))*scale_deg_to_rad str2double(get(handles.drawer42,'String')) 0]';
selected_bed_adapter = [get(handles.arm1_drawer,'Value') get(handles.arm2_drawer,'Value') get(handles.arm3_drawer,'Value') get(handles.arm4_drawer,'Value')];

for index2 = 1: num_arm
    if selected_bed_adapter(index2) == 10
        selected_bed_adapter(index2) = 0;
    end
end
procedure_num = get(handles.select_procedure,'Value');
q_setup = get(handles.uitable_jnt, 'data');
q_setup([1:7 9:11],1:num_arm) = q_setup([1:7 9:11],1:num_arm) * scale_deg_to_rad;
q_estimate = q_setup(:,1:num_arm);
[q_init_setup] = RobotProcedureSetup(q_estimate,q_bed_adapter,selected_bed_adapter,procedure_num,get(handles.table_adapter_version,'Value'));

q_setup(:,1:num_arm) = q_init_setup(:,1:num_arm);
q_setup([1:7  9:11],1:num_arm) = q_setup([1:7  9:11],1:num_arm) * 180 / pi;
for index = 1:num_arm
    for i = 1 :  length(jnt_limit_active)
        joint_percentage(i,index) = (q_init_setup(i,index) - jnt_limit_active(1,i)) / (jnt_limit_active(2,i) - jnt_limit_active(1,i));
    end
end

q_setup(:,num_arm+1:num_arm*2) = joint_percentage * 100;
set(handles.uitable_jnt,'data',q_setup);



% --- Executes on selection change in joint_state.
function joint_state_Callback(hObject, eventdata, handles)
% hObject    handle to joint_state (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns joint_state contents as cell array
%        contents{get(hObject,'Value')} returns selected item from joint_state
load('init_joint_state.mat');
num_arm = 4;
switch get(handles.joint_state,'Value')
    case 1
        q_init_setup = q_zero * ones(1,num_arm);
    case 2
        q_init_setup = q_draping * ones(1,num_arm);
    case 3
        q_init_setup = q_retracted * ones(1,num_arm);
    otherwise
        q_init_setup = q_zero * ones(1,num_arm);
end
load('joint_limit_active_1.0.mat')

q_setup(:,1:num_arm) = q_init_setup(:,1:num_arm);
q_setup([1:7  9:11],1:num_arm) = q_setup([1:7  9:11],1:num_arm) * 180 / pi;
for index = 1:num_arm
    for i = 1 :  length(jnt_limit_active)
        joint_percentage(i,index) = (q_init_setup(i,index) - jnt_limit_active(1,i)) / (jnt_limit_active(2,i) - jnt_limit_active(1,i));
    end
end

q_setup(:,num_arm+1:num_arm*2) = joint_percentage * 100;
set(handles.uitable_jnt,'data',q_setup);

% --- Executes during object creation, after setting all properties.
function joint_state_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint_state (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in select_procedure.
function select_procedure_Callback(hObject, eventdata, handles)
% hObject    handle to select_procedure (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns select_procedure contents as cell array
%        contents{get(hObject,'Value')} returns selected item from select_procedure


% --- Executes during object creation, after setting all properties.
function select_procedure_CreateFcn(hObject, eventdata, handles)
% hObject    handle to select_procedure (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function separation_angle_Callback(hObject, eventdata, handles)
% hObject    handle to separation_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of separation_angle as text
%        str2double(get(hObject,'String')) returns contents of separation_angle as a double


% --- Executes during object creation, after setting all properties.
function separation_angle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to separation_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in y_plus.
function y_plus_Callback(hObject, eventdata, handles)
% hObject    handle to y_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
num_arm = 4;
scale_deg_to_rad = pi/180;
separation_angle = str2double(get(handles.separation_angle,'String')) * scale_deg_to_rad;

q_setup = get(handles.uitable_jnt, 'data');
q_setup([1:7 9:11],1:num_arm) = q_setup([1:7 9:11],1:num_arm) * scale_deg_to_rad;
arm_index = get(handles.arm_selection,'Value');
q_setup(:,arm_index) = ArmRepositioning(q_setup(:,arm_index),separation_angle,'y');
load('urdf_info_1.0.mat')
load('vertex_arm_origin_1.0.mat');
load('arm_version_1.0.mat');
load('coupling_matrix.mat');
load('vertex_bed.mat');
robot_object1 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object2 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object3 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object4 = RobotClass(urdf_link_input,urdf_joint_input);
robot_arms = {robot_object1;robot_object2;robot_object3;robot_object4};
if get(handles.table_adapter_version,'Value') == 1
    load('vertex_bed_adapter_new.mat');
else
    load('vertex_bed_adapter.mat');
end
switch get(handles.select_procedure,'Value')
    case 1
        InitHerniaSetup;
        load('vertex_patient_body.mat');
    case 2
        InitThoracicSetup;
        load('vertex_patient_body.mat');
    case 3
        InitHysterectomySetup;
        load('vertex_patient_body.mat');
    case 4
        InitGastricBypassSetup;
        load('vertex_patient_body.mat');
    case 5
        InitProstatectomySetup;
        load('vertex_patient_body.mat');
    case 6
        InitLarSetup;
        load('vertex_patient_body.mat');
    case 7
        InitOmentectomySetup;
        load('vertex_patient_body.mat');
    otherwise
        error('No such procedure');
end

axes(handles.disp_fig)

q_bed_adapter = [str2double(get(handles.drawer11,'String'))*scale_deg_to_rad str2double(get(handles.drawer12,'String')) 0;str2double(get(handles.drawer21,'String'))*scale_deg_to_rad str2double(get(handles.drawer22,'String')) 0;str2double(get(handles.drawer31,'String'))*scale_deg_to_rad str2double(get(handles.drawer32,'String')) 0;str2double(get(handles.drawer41,'String'))*scale_deg_to_rad str2double(get(handles.drawer42,'String')) 0]';
selected_bed_adapter = [get(handles.arm1_drawer,'Value') get(handles.arm2_drawer,'Value') get(handles.arm3_drawer,'Value') get(handles.arm4_drawer,'Value')];
cla
hold on
axis equal
[az1,el1] = view;
view(az1,el1)
% view(str2double(get(handles.az_fig,'String')),str2double(get(handles.el_fig,'String')));
if get(handles.check_Frames,'Value') == 1
    DrawCoordinateSystem([0.1 0.1 0.1],eye(3),[0;0;0],'rgb','w')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar1,'rgb','t1')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar2,'rgb','t2')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar3,'rgb','t3')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar4,'rgb','t3')
    hold on
end
if get(handles.check_Body,'Value') == 1
    vertex_patient_transformed = transformSTL(vertex_patient_body,rotation_patient,translation_patient);
    rgba = [0 0 1 0.1];
    PlotStl(vertex_patient_transformed,rgba,handles.disp_fig)
    hold on
end
for index_robot = 1:num_arm
    if selected_bed_adapter(index_robot) ~=10
        frames_bed_adapter = CalculateBedAdapterFK( q_bed_adapter(:,index_robot),frames_bed_adapter_base(:,:,selected_bed_adapter(index_robot)));
        DrawBedAdapter(frames_bed_adapter,vertex_bed_adapter,[1 0 0 1],handles.disp_fig)
        hold on
        robot_arms{index_robot}.transformation_base_ = frames_bed_adapter(:,:,end);
        robot_arms{index_robot}.CalculateFK(q_setup(1:11,index_robot));
        robot_arms{index_robot}.DrawRobotGUI(vertex_arm_origin,handles.disp_fig);
        if get(handles.check_Frames,'Value') == 1
            DrawCoordinateSystem([0.1 0.1 0.1],frames_bed_adapter(1:3,1:3,end),frames_bed_adapter(1:3,4,end),'rgb',strcat('b',num2str(index_robot)))
            hold on
            DrawCoordinateSystem([0.1 0.1 0.1],robot_arms{index_robot}.frames_(1:3,1:3,7),robot_arms{index_robot}.frames_(1:3,4,7),'rgb',6)
            hold on
        end
    end
    
end
plot3(translation_target(1),translation_target(2),translation_target(3),'Marker','o','MarkerSize',10)
DrawBed(vertex_bed,[0.0 0.7 0.0 1],handles.disp_fig)
hold on
axis([-1.2 1 -0.8 0.8 -0.2 2.8])
light('Position',[1 3 2]);
light('Position',[-3 -1 -3]);
if get(handles.grid_enable,'Value') == 1
    grid on
    grid minor
else
    grid off
end
drawnow;

load('joint_limit_active_1.0.mat');
for index = 1:num_arm
    for i = 1 :  length(jnt_limit_active)
        joint_percentage(i,index) = (q_setup(i,index) - jnt_limit_active(1,i)) / (jnt_limit_active(2,i) - jnt_limit_active(1,i));
    end
end
q_setup([1:7  9:11],1:num_arm) = q_setup([1:7  9:11],1:num_arm) * 180 / pi;
q_setup(:,num_arm+1:num_arm*2) = joint_percentage * 100;
set(handles.uitable_jnt,'data',q_setup);

% --- Executes on button press in x_plus.
function x_plus_Callback(hObject, eventdata, handles)
% hObject    handle to x_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
num_arm = 4;
scale_deg_to_rad = pi/180;
separation_angle = str2double(get(handles.separation_angle,'String')) * scale_deg_to_rad;

q_setup = get(handles.uitable_jnt, 'data');
q_setup([1:7 9:11],1:num_arm) = q_setup([1:7 9:11],1:num_arm) * scale_deg_to_rad;
arm_index = get(handles.arm_selection,'Value');
q_setup(:,arm_index) = ArmRepositioning(q_setup(:,arm_index),separation_angle,'x');
load('urdf_info_1.0.mat')
load('vertex_arm_origin_1.0.mat');
load('arm_version_1.0.mat');
load('coupling_matrix.mat');
load('vertex_bed.mat');
robot_object1 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object2 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object3 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object4 = RobotClass(urdf_link_input,urdf_joint_input);
robot_arms = {robot_object1;robot_object2;robot_object3;robot_object4};
if get(handles.table_adapter_version,'Value') == 1
    load('vertex_bed_adapter_new.mat');
else
    load('vertex_bed_adapter.mat');
end
switch get(handles.select_procedure,'Value')
    case 1
        InitHerniaSetup;
        load('vertex_patient_body.mat');
    case 2
        InitThoracicSetup;
        load('vertex_patient_body.mat');
    case 3
        InitHysterectomySetup;
        load('vertex_patient_body.mat');
    case 4
        InitGastricBypassSetup;
        load('vertex_patient_body.mat');
    case 5
        InitProstatectomySetup;
        load('vertex_patient_body.mat');
    case 6
        InitLarSetup;
        load('vertex_patient_body.mat');
    case 7
        InitOmentectomySetup;
        load('vertex_patient_body.mat');
    otherwise
        error('No such procedure');
end

axes(handles.disp_fig)

q_bed_adapter = [str2double(get(handles.drawer11,'String'))*scale_deg_to_rad str2double(get(handles.drawer12,'String')) 0;str2double(get(handles.drawer21,'String'))*scale_deg_to_rad str2double(get(handles.drawer22,'String')) 0;str2double(get(handles.drawer31,'String'))*scale_deg_to_rad str2double(get(handles.drawer32,'String')) 0;str2double(get(handles.drawer41,'String'))*scale_deg_to_rad str2double(get(handles.drawer42,'String')) 0]';
selected_bed_adapter = [get(handles.arm1_drawer,'Value') get(handles.arm2_drawer,'Value') get(handles.arm3_drawer,'Value') get(handles.arm4_drawer,'Value')];
cla
hold on
axis equal
[az1,el1] = view;
view(az1,el1)
% view(str2double(get(handles.az_fig,'String')),str2double(get(handles.el_fig,'String')));
if get(handles.check_Frames,'Value') == 1
    DrawCoordinateSystem([0.1 0.1 0.1],eye(3),[0;0;0],'rgb','w')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar1,'rgb','t1')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar2,'rgb','t2')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar3,'rgb','t3')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar4,'rgb','t3')
    hold on
end
if get(handles.check_Body,'Value') == 1
    vertex_patient_transformed = transformSTL(vertex_patient_body,rotation_patient,translation_patient);
    rgba = [0 0 1 0.1];
    PlotStl(vertex_patient_transformed,rgba,handles.disp_fig)
    hold on
end
for index_robot = 1:num_arm
    if selected_bed_adapter(index_robot) ~=10
        frames_bed_adapter = CalculateBedAdapterFK( q_bed_adapter(:,index_robot),frames_bed_adapter_base(:,:,selected_bed_adapter(index_robot)));
        DrawBedAdapter(frames_bed_adapter,vertex_bed_adapter,[1 0 0 1],handles.disp_fig)
        hold on
        robot_arms{index_robot}.transformation_base_ = frames_bed_adapter(:,:,end);
        robot_arms{index_robot}.CalculateFK(q_setup(1:11,index_robot));
        robot_arms{index_robot}.DrawRobotGUI(vertex_arm_origin,handles.disp_fig);
        if get(handles.check_Frames,'Value') == 1
            DrawCoordinateSystem([0.1 0.1 0.1],frames_bed_adapter(1:3,1:3,end),frames_bed_adapter(1:3,4,end),'rgb',strcat('b',num2str(index_robot)))
            hold on
            DrawCoordinateSystem([0.1 0.1 0.1],robot_arms{index_robot}.frames_(1:3,1:3,7),robot_arms{index_robot}.frames_(1:3,4,7),'rgb',6)
            hold on
        end
    end
    
end
plot3(translation_target(1),translation_target(2),translation_target(3),'Marker','o','MarkerSize',10)
DrawBed(vertex_bed,[0.0 0.7 0.0 1],handles.disp_fig)
hold on
axis([-1.2 1 -0.8 0.8 -0.2 2.8])
light('Position',[1 3 2]);
light('Position',[-3 -1 -3]);
if get(handles.grid_enable,'Value') == 1
    grid on
    grid minor
else
    grid off
end
drawnow;

load('joint_limit_active_1.0.mat');
for index = 1:num_arm
    for i = 1 :  length(jnt_limit_active)
        joint_percentage(i,index) = (q_setup(i,index) - jnt_limit_active(1,i)) / (jnt_limit_active(2,i) - jnt_limit_active(1,i));
    end
end
q_setup([1:7  9:11],1:num_arm) = q_setup([1:7  9:11],1:num_arm) * 180 / pi;
q_setup(:,num_arm+1:num_arm*2) = joint_percentage * 100;
set(handles.uitable_jnt,'data',q_setup);


% --- Executes on button press in y_minus.
function y_minus_Callback(hObject, eventdata, handles)
% hObject    handle to y_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
num_arm = 4;
scale_deg_to_rad = pi/180;
separation_angle = str2double(get(handles.separation_angle,'String')) * scale_deg_to_rad;

q_setup = get(handles.uitable_jnt, 'data');
q_setup([1:7 9:11],1:num_arm) = q_setup([1:7 9:11],1:num_arm) * scale_deg_to_rad;
arm_index = get(handles.arm_selection,'Value');
q_setup(:,arm_index) = ArmRepositioning(q_setup(:,arm_index),-separation_angle,'y');
load('urdf_info_1.0.mat')
load('vertex_arm_origin_1.0.mat');
load('arm_version_1.0.mat');
load('coupling_matrix.mat');
load('vertex_bed.mat');
robot_object1 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object2 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object3 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object4 = RobotClass(urdf_link_input,urdf_joint_input);
robot_arms = {robot_object1;robot_object2;robot_object3;robot_object4};
if get(handles.table_adapter_version,'Value') == 1
    load('vertex_bed_adapter_new.mat');
else
    load('vertex_bed_adapter.mat');
end
switch get(handles.select_procedure,'Value')
    case 1
        InitHerniaSetup;
        load('vertex_patient_body.mat');
    case 2
        InitThoracicSetup;
        load('vertex_patient_body.mat');
    case 3
        InitHysterectomySetup;
        load('vertex_patient_body.mat');
    case 4
        InitGastricBypassSetup;
        load('vertex_patient_body.mat');
    case 5
        InitProstatectomySetup;
        load('vertex_patient_body.mat');
    case 6
        InitLarSetup;
        load('vertex_patient_body.mat');
    case 7
        InitOmentectomySetup;
        load('vertex_patient_body.mat');
    otherwise
        error('No such procedure');
end

axes(handles.disp_fig)

q_bed_adapter = [str2double(get(handles.drawer11,'String'))*scale_deg_to_rad str2double(get(handles.drawer12,'String')) 0;str2double(get(handles.drawer21,'String'))*scale_deg_to_rad str2double(get(handles.drawer22,'String')) 0;str2double(get(handles.drawer31,'String'))*scale_deg_to_rad str2double(get(handles.drawer32,'String')) 0;str2double(get(handles.drawer41,'String'))*scale_deg_to_rad str2double(get(handles.drawer42,'String')) 0]';
selected_bed_adapter = [get(handles.arm1_drawer,'Value') get(handles.arm2_drawer,'Value') get(handles.arm3_drawer,'Value') get(handles.arm4_drawer,'Value')];
cla
hold on
axis equal
[az1,el1] = view;
view(az1,el1)
% view(str2double(get(handles.az_fig,'String')),str2double(get(handles.el_fig,'String')));
if get(handles.check_Frames,'Value') == 1
    DrawCoordinateSystem([0.1 0.1 0.1],eye(3),[0;0;0],'rgb','w')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar1,'rgb','t1')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar2,'rgb','t2')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar3,'rgb','t3')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar4,'rgb','t3')
    hold on
end
if get(handles.check_Body,'Value') == 1
    vertex_patient_transformed = transformSTL(vertex_patient_body,rotation_patient,translation_patient);
    rgba = [0 0 1 0.1];
    PlotStl(vertex_patient_transformed,rgba,handles.disp_fig)
    hold on
end
for index_robot = 1:num_arm
    if selected_bed_adapter(index_robot) ~=10
        frames_bed_adapter = CalculateBedAdapterFK( q_bed_adapter(:,index_robot),frames_bed_adapter_base(:,:,selected_bed_adapter(index_robot)));
        DrawBedAdapter(frames_bed_adapter,vertex_bed_adapter,[1 0 0 1],handles.disp_fig)
        hold on
        robot_arms{index_robot}.transformation_base_ = frames_bed_adapter(:,:,end);
        robot_arms{index_robot}.CalculateFK(q_setup(1:11,index_robot));
        robot_arms{index_robot}.DrawRobotGUI(vertex_arm_origin,handles.disp_fig);
        if get(handles.check_Frames,'Value') == 1
            DrawCoordinateSystem([0.1 0.1 0.1],frames_bed_adapter(1:3,1:3,end),frames_bed_adapter(1:3,4,end),'rgb',strcat('b',num2str(index_robot)))
            hold on
            DrawCoordinateSystem([0.1 0.1 0.1],robot_arms{index_robot}.frames_(1:3,1:3,7),robot_arms{index_robot}.frames_(1:3,4,7),'rgb',6)
            hold on
        end
    end
    
end
plot3(translation_target(1),translation_target(2),translation_target(3),'Marker','o','MarkerSize',10)
DrawBed(vertex_bed,[0.0 0.7 0.0 1],handles.disp_fig)
hold on
axis([-1.2 1 -0.8 0.8 -0.2 2.8])
light('Position',[1 3 2]);
light('Position',[-3 -1 -3]);
if get(handles.grid_enable,'Value') == 1
    grid on
    grid minor
else
    grid off
end
drawnow;

load('joint_limit_active_1.0.mat');
for index = 1:num_arm
    for i = 1 :  length(jnt_limit_active)
        joint_percentage(i,index) = (q_setup(i,index) - jnt_limit_active(1,i)) / (jnt_limit_active(2,i) - jnt_limit_active(1,i));
    end
end
q_setup([1:7  9:11],1:num_arm) = q_setup([1:7  9:11],1:num_arm) * 180 / pi;
q_setup(:,num_arm+1:num_arm*2) = joint_percentage * 100;
set(handles.uitable_jnt,'data',q_setup);

% --- Executes on button press in x_minus.
function x_minus_Callback(hObject, eventdata, handles)
% hObject    handle to x_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
num_arm = 4;
scale_deg_to_rad = pi/180;
separation_angle = str2double(get(handles.separation_angle,'String')) * scale_deg_to_rad;

q_setup = get(handles.uitable_jnt, 'data');
q_setup([1:7 9:11],1:num_arm) = q_setup([1:7 9:11],1:num_arm) * scale_deg_to_rad;
arm_index = get(handles.arm_selection,'Value');
q_setup(:,arm_index) = ArmRepositioning(q_setup(:,arm_index),-separation_angle,'x');
load('urdf_info_1.0.mat')
load('vertex_arm_origin_1.0.mat');
load('arm_version_1.0.mat');
load('coupling_matrix.mat');
load('vertex_bed.mat');
robot_object1 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object2 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object3 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object4 = RobotClass(urdf_link_input,urdf_joint_input);
robot_arms = {robot_object1;robot_object2;robot_object3;robot_object4};
if get(handles.table_adapter_version,'Value') == 1
    load('vertex_bed_adapter_new.mat');
else
    load('vertex_bed_adapter.mat');
end
switch get(handles.select_procedure,'Value')
    case 1
        InitHerniaSetup;
        load('vertex_patient_body.mat');
    case 2
        InitThoracicSetup;
        load('vertex_patient_body.mat');
    case 3
        InitHysterectomySetup;
        load('vertex_patient_body.mat');
    case 4
        InitGastricBypassSetup;
        load('vertex_patient_body.mat');
    case 5
        InitProstatectomySetup;
        load('vertex_patient_body.mat');
    case 6
        InitLarSetup;
        load('vertex_patient_body.mat');
    case 7
        InitOmentectomySetup;
        load('vertex_patient_body.mat');
    otherwise
        error('No such procedure');
end

axes(handles.disp_fig)

q_bed_adapter = [str2double(get(handles.drawer11,'String'))*scale_deg_to_rad str2double(get(handles.drawer12,'String')) 0;str2double(get(handles.drawer21,'String'))*scale_deg_to_rad str2double(get(handles.drawer22,'String')) 0;str2double(get(handles.drawer31,'String'))*scale_deg_to_rad str2double(get(handles.drawer32,'String')) 0;str2double(get(handles.drawer41,'String'))*scale_deg_to_rad str2double(get(handles.drawer42,'String')) 0]';
selected_bed_adapter = [get(handles.arm1_drawer,'Value') get(handles.arm2_drawer,'Value') get(handles.arm3_drawer,'Value') get(handles.arm4_drawer,'Value')];
cla
hold on
axis equal
[az1,el1] = view;
view(az1,el1)
% view(str2double(get(handles.az_fig,'String')),str2double(get(handles.el_fig,'String')));
if get(handles.check_Frames,'Value') == 1
    DrawCoordinateSystem([0.1 0.1 0.1],eye(3),[0;0;0],'rgb','w')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar1,'rgb','t1')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar2,'rgb','t2')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar3,'rgb','t3')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],eye(3),translation_trocar4,'rgb','t3')
    hold on
end
if get(handles.check_Body,'Value') == 1
    vertex_patient_transformed = transformSTL(vertex_patient_body,rotation_patient,translation_patient);
    rgba = [0 0 1 0.1];
    PlotStl(vertex_patient_transformed,rgba,handles.disp_fig)
    hold on
end
for index_robot = 1:num_arm
    if selected_bed_adapter(index_robot) ~=10
        frames_bed_adapter = CalculateBedAdapterFK( q_bed_adapter(:,index_robot),frames_bed_adapter_base(:,:,selected_bed_adapter(index_robot)));
        DrawBedAdapter(frames_bed_adapter,vertex_bed_adapter,[1 0 0 1],handles.disp_fig)
        hold on
        robot_arms{index_robot}.transformation_base_ = frames_bed_adapter(:,:,end);
        robot_arms{index_robot}.CalculateFK(q_setup(1:11,index_robot));
        robot_arms{index_robot}.DrawRobotGUI(vertex_arm_origin,handles.disp_fig);
        if get(handles.check_Frames,'Value') == 1
            DrawCoordinateSystem([0.1 0.1 0.1],frames_bed_adapter(1:3,1:3,end),frames_bed_adapter(1:3,4,end),'rgb',strcat('b',num2str(index_robot)))
            hold on
            DrawCoordinateSystem([0.1 0.1 0.1],robot_arms{index_robot}.frames_(1:3,1:3,7),robot_arms{index_robot}.frames_(1:3,4,7),'rgb',6)
            hold on
        end
    end
    
end
plot3(translation_target(1),translation_target(2),translation_target(3),'Marker','o','MarkerSize',10)
DrawBed(vertex_bed,[0.0 0.7 0.0 1],handles.disp_fig)
hold on
axis([-1.2 1 -0.8 0.8 -0.2 2.8])
light('Position',[1 3 2]);
light('Position',[-3 -1 -3]);
if get(handles.grid_enable,'Value') == 1
    grid on
    grid minor
else
    grid off
end
drawnow;

load('joint_limit_active_1.0.mat');
for index = 1:num_arm
    for i = 1 :  length(jnt_limit_active)
        joint_percentage(i,index) = (q_setup(i,index) - jnt_limit_active(1,i)) / (jnt_limit_active(2,i) - jnt_limit_active(1,i));
    end
end
q_setup([1:7  9:11],1:num_arm) = q_setup([1:7  9:11],1:num_arm) * 180 / pi;
q_setup(:,num_arm+1:num_arm*2) = joint_percentage * 100;
set(handles.uitable_jnt,'data',q_setup);

% --- Executes on selection change in arm_selection.
function arm_selection_Callback(hObject, eventdata, handles)
% hObject    handle to arm_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns arm_selection contents as cell array
%        contents{get(hObject,'Value')} returns selected item from arm_selection


% --- Executes during object creation, after setting all properties.
function arm_selection_CreateFcn(hObject, eventdata, handles)
% hObject    handle to arm_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in arm4_drawer.
function arm4_drawer_Callback(hObject, eventdata, handles)
% hObject    handle to arm4_drawer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns arm4_drawer contents as cell array
%        contents{get(hObject,'Value')} returns selected item from arm4_drawer


% --- Executes during object creation, after setting all properties.
function arm4_drawer_CreateFcn(hObject, eventdata, handles)
% hObject    handle to arm4_drawer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function drawer41_Callback(hObject, eventdata, handles)
% hObject    handle to drawer41 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of drawer41 as text
%        str2double(get(hObject,'String')) returns contents of drawer41 as a double


% --- Executes during object creation, after setting all properties.
function drawer41_CreateFcn(hObject, eventdata, handles)
% hObject    handle to drawer41 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function drawer42_Callback(hObject, eventdata, handles)
% hObject    handle to drawer42 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of drawer42 as text
%        str2double(get(hObject,'String')) returns contents of drawer42 as a double


% --- Executes during object creation, after setting all properties.
function drawer42_CreateFcn(hObject, eventdata, handles)
% hObject    handle to drawer42 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in table_adapter_version.
function table_adapter_version_Callback(hObject, eventdata, handles)
% hObject    handle to table_adapter_version (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns table_adapter_version contents as cell array
%        contents{get(hObject,'Value')} returns selected item from table_adapter_version


% --- Executes during object creation, after setting all properties.
function table_adapter_version_CreateFcn(hObject, eventdata, handles)
% hObject    handle to table_adapter_version (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in grid_enable.
function grid_enable_Callback(hObject, eventdata, handles)
% hObject    handle to grid_enable (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of grid_enable


% --- Executes on button press in save_figure.
function save_figure_Callback(hObject, eventdata, handles)
% hObject    handle to save_figure (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file_name = strcat('figures\',get(handles.figure_name,'String'));
axes(handles.disp_fig);
saveas(gcf, file_name, 'jpg'); 



function figure_name_Callback(hObject, eventdata, handles)
% hObject    handle to figure_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of figure_name as text
%        str2double(get(hObject,'String')) returns contents of figure_name as a double


% --- Executes during object creation, after setting all properties.
function figure_name_CreateFcn(hObject, eventdata, handles)
% hObject    handle to figure_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
