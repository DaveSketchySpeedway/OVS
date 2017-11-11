function varargout = simulator(varargin)
% SIMULATOR MATLAB code for simulator.fig
%      SIMULATOR, by itself, creates a new SIMULATOR or raises the existing
%      singleton*.
%
%      H = SIMULATOR returns the handle to a new SIMULATOR or the handle to
%      the existing singleton*.
%
%      SIMULATOR('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SIMULATOR.M with the given input arguments.
%
%      SIMULATOR('Property','Value',...) creates a new SIMULATOR or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before simulator_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to simulator_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help simulator

% Last Modified by GUIDE v2.5 29-May-2017 19:32:36

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @simulator_OpeningFcn, ...
                   'gui_OutputFcn',  @simulator_OutputFcn, ...
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


% --- Outputs from this function are returned to the command line.
function varargout = simulator_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;









% --- Executes just before simulator is made visible.
function simulator_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to simulator (see VARARGIN)

% Choose default command line output for simulator
handles.output = hObject;

% init settings
handles.settings.simulator_time_step = 0.05;
handles.settings.world_time_step = 0.1;
handles.settings.gauge_time_step = 0.2;
handles.settings.robot_time_step = 0.1;
handles.settings.user_time_step = 0.1;
handles.settings.azimuth = -60;
handles.settings.zenith = 45;
handles.settings.dx = 30;
handles.settings.dy = 30;
handles.settings.dz = 20;

% init inputs
handles.inputs.cursor_x_max = 215;
handles.inputs.cursor_x_min = 178;
handles.inputs.cursor_y_max = 13;
handles.inputs.cursor_y_min = 01;
handles.inputs.throttle = 0;
handles.inputs.rudder = 0;
handles.inputs.joy_x = 0;
handles.inputs.joy_y = 0;

% init data
handles.data.time = [];
handles.data.body_angvel = [];
handles.data.body_sideslip = [];
handles.data.body_linacc = [];
handles.data.body_linvel = [];
handles.data.position = [];
handles.data.linvel = [];
handles.data.orientation = [];
handles.data.angvel = [];

% init timer
handles.counter = 0;
handles.timer_simulator = timer('ExecutionMode', 'fixedDelay',...
  'Period', handles.settings.simulator_time_step,...
  'TimerFcn',{@timer_simulator_update_fcn, hObject},...
  'BusyMode','drop');
handles.timer_world = timer('ExecutionMode', 'fixedDelay',...
  'Period', handles.settings.world_time_step,...
  'TimerFcn',{@timer_world_update_fcn, hObject},...
  'BusyMode','drop');
handles.timer_gauge = timer('ExecutionMode', 'fixedDelay',...
  'Period', handles.settings.gauge_time_step,...
  'TimerFcn',{@timer_gauge_update_fcn, hObject},...
  'BusyMode','drop');
handles.timer_robot = timer('ExecutionMode', 'fixedDelay',...
  'Period', handles.settings.robot_time_step,...
  'TimerFcn',{@timer_robot_update_fcn, hObject},...
  'BusyMode','drop');
handles.timer_user = timer('ExecutionMode', 'fixedDelay',...
  'Period', handles.settings.user_time_step,...
  'TimerFcn',{@timer_user_update_fcn, hObject},...
  'BusyMode','drop');

% instantiate model
handles.model = Model(handles.settings.simulator_time_step);
guidata(hObject, handles); % release
model_button_Callback(handles.model_button, eventdata, handles);
handles = guidata(hObject); % release

% instantiate solver 
handles.solver = Solver(handles.settings.simulator_time_step);
handles.solver.set_model(handles.model);

% instantiate trajectory generator
handles.trajgen = TrajectoryGenerator();
guidata(hObject, handles); % release
path_button_Callback(handles.path_button, eventdata, handles);
handles = guidata(hObject); % reacquire

% instantiate robot
handles.rob = Robot(handles.settings.robot_time_step);
handles.rob.set_trajectory(handles.trajgen);

% init world
handles.world.trajectory = animatedline(handles.world_axes,...
  'Color','r');
handles.world.trace = animatedline(handles.world_axes,...
  'Color', 'k');
handles.world.skin = patch(handles.world_axes,...
  'Faces', handles.model.skin_faces,...
  'Vertices', handles.model.skin_vertices,...
  'FaceVertexCData', handles.model.skin_colors,...
  'FaceColor','flat');
% handles.world.poserr = line(handles.world_axes,...
%   [0,100],...
%   [0,100],...
%   [0,100],...
%   'Color','m');
handles.world.ahead = line(handles.world_axes,...
  [0,-100],...
  [0,-100],...
  [0,100],...
  'Color','g');
guidata(hObject, handles); % release
reset_button_Callback(handles.reset_button, eventdata, handles);
handles = guidata(hObject); % reacquire

% init gauges
handles.gauges.bar_acc = bar(handles.body_linacc_axes, [0 0 0]);
ylim(handles.body_linacc_axes,[-10,10])
ylabel(handles.body_linacc_axes,'m/s2')
xlabel(handles.body_linacc_axes,'ACC')
set(handles.body_linacc_axes,'xtick',[])
handles.gauges.bar_vel = bar(handles.body_linvel_axes, [0 0 0]);
ylim(handles.body_linvel_axes,[-10,50])
ylabel(handles.body_linvel_axes,'m/2')
xlabel(handles.body_linvel_axes,'VEL')
set(handles.body_linvel_axes,'xtick',[])
handles.gauges.bar_slip = bar(handles.body_slip_axes, [0 0 0]);
ylim(handles.body_slip_axes,[-0.5,0.5])
ylabel(handles.body_slip_axes,'rad')
xlabel(handles.body_slip_axes,'SLIP')
set(handles.body_slip_axes,'xtick',[])

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes simulator wait for user response (see UIRESUME)
% uiwait(handles.main_figure);


function timer_simulator_update_fcn(hObject, eventdata, hFigure)
  handles = guidata(hFigure);
  handles.counter = handles.counter + 1;
  % disp(['Simulator instant period: ', ...
  %   num2str(get(handles.timer_simulator, 'InstantPeriod'))]);
  % step solver
  throttle = handles.inputs.throttle * 10; % TODO set gain in model
  refsteer = handles.inputs.joy_x * 0.5; % TODO set gain in model
  handles.model.set_input(throttle, refsteer);
  handles.solver.step();
  % record data
  handles.data.time = [handles.data.time;...
    handles.settings.simulator_time_step*handles.counter];
  handles.data.body_angvel = [handles.data.body_angvel;...
    handles.model.get_angvel];
  handles.data.body_sideslip = [handles.data.body_sideslip;...
    handles.model.get_sideslip()];
  handles.data.body_linacc = [handles.data.body_linacc;...
    handles.model.get_linacc()];
  handles.data.body_linvel = [handles.data.body_linvel;...
    handles.model.get_linvel()];
  handles.data.position = [handles.data.position;...
    handles.solver.get_position()];
  handles.data.linvel = [handles.data.linvel;...
    handles.solver.get_linvel];
  handles.data.orientation = [handles.data.orientation;...
    handles.solver.get_orientation()];
  handles.data.angvel = [handles.data.angvel;...
    handles.solver.get_angvel()];
  % release lock
  guidata(hFigure, handles);


function timer_world_update_fcn(hObject, eventdata, hFigure)
  handles = guidata(hFigure);
  % update world
  set(handles.world.skin,...
    'Vertices', handles.solver.render());
  pos = handles.solver.get_position();
  addpoints(handles.world.trace,...
    pos(1), pos(2), pos(3));
  axis(handles.world_axes,...
    [pos(1)-handles.settings.dx, pos(1)+handles.settings.dx,...
     pos(2)-handles.settings.dy, pos(2)+handles.settings.dy,...
     pos(3)-handles.settings.dz, pos(3)+handles.settings.dz]);
  view(handles.world_axes,...
    handles.settings.azimuth, handles.settings.zenith);
  drawnow 
  % release lock
  guidata(hFigure, handles);


function timer_gauge_update_fcn(hObject, eventdata, hFigure)
  handles = guidata(hFigure);
  % update gauges
  set(handles.gauges.bar_acc, 'YData',...
    handles.model.get_linacc());
  set(handles.gauges.bar_vel, 'YData',...
    handles.model.get_linvel());
  set(handles.gauges.bar_slip, 'YData',...
    handles.model.get_sideslip());
  drawnow
  % release lock
  guidata(hFigure, handles);


function timer_robot_update_fcn(hObject, eventdata, hFigure)
  handles = guidata(hFigure);
  % get reference and error
  pos = handles.solver.get_position();
  handles.rob.find_reference(pos);
  % poserr_vector = handles.rob.get_poserr_vector();
  % set(handles.world.poserr,...
  %   'XData', [pos(1) pos(1)+poserr_vector(1)],...
  %   'YData', [pos(2) pos(2)+poserr_vector(2)],...
  %   'ZData', [pos(3) pos(3)+poserr_vector(3)]);
  ahead_vector = handles.rob.get_ahead_vector();
  set(handles.world.ahead,...
    'XData', [pos(1) pos(1)+ahead_vector(1)],...
    'YData', [pos(2) pos(2)+ahead_vector(2)],...
    'ZData', [pos(3) pos(3)+ahead_vector(3)]);
  % close loop
  % body_poserr_vector = handles.solver.transform2body(poserr_vector);
  % handles.rob.set_body_poserr_vector(body_poserr_vector);
  body_ahead_vector = handles.solver.transform2body(ahead_vector);
  handles.rob.set_body_ahead_vector(body_ahead_vector);
  [throttle, joy_x] = handles.rob.get_command();
  handles.inputs.joy_x = joy_x;
  handles.inputs.throttle = throttle;
  % release lock
  guidata(hFigure, handles);

function timer_user_update_fcn(hObject, eventdata, hFigure)
  handles = guidata(hFigure);
  % get throttle and rudder
  handles.inputs.throttle = get(handles.throttle_slider, 'Value');
  handles.inputs.rudder = -1*get(handles.rudder_slider, 'Value'); % body y +ve to the left
  % get joystick 
  cursor = get(hFigure, 'CurrentPoint');
  [joy_x, joy_y] = cursor_joystick(cursor,...
    handles.inputs.cursor_x_max, handles.inputs.cursor_x_min,...
    handles.inputs.cursor_y_max, handles.inputs.cursor_y_min);
  handles.inputs.joy_x = -1*joy_x; % body y +ve to the left
  handles.inputs.joy_y = joy_y;
  % disp(cursor);
  % disp([num2str(joy_x), ' ', num2str(joy_y)]);
  % release lock
  guidata(hFigure, handles);















% --- Executes when user attempts to close main_figure.
function main_figure_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to main_figure (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% kill timers
  if strcmp(get(handles.timer_simulator, 'Running'), 'on')
    stop(handles.timer_simulator);
  end
  if strcmp(get(handles.timer_world, 'Running'), 'on')
    stop(handles.timer_world);
  end
  if strcmp(get(handles.timer_gauge, 'Running'), 'on')
    stop(handles.timer_gauge);
  end
  if strcmp(get(handles.timer_robot, 'Running'), 'on')
    stop(handles.timer_robot);
  end
  if strcmp(get(handles.timer_user, 'Running'), 'on')
    stop(handles.timer_user);
  end
delete(timerfind)
% plot results
figure
subplot(2,2,1)
plot(handles.data.time,handles.data.body_linvel)
title('body lin vel')
xlabel('time [s]')
ylabel('velocity [m/s]')
legend('fwd','left','up')
grid on
subplot(2,2,2)
plot(handles.data.time,handles.data.body_sideslip)
title('body side slip')
xlabel('time [s]')
ylabel('slip [rad]')
legend('refsteer','latacc','linvel')
grid on
subplot(2,2,3)
plot(handles.data.time,handles.data.linvel)
title('lin vel')
xlabel('time [s]')
ylabel('velocity [m/s]')
legend('x','y','z')
grid on
subplot(2,2,4)
plot(handles.data.time,handles.data.angvel)
title('ang vel')
xlabel('time [s]')
ylabel('spinrate [rad/s]')
legend('x','y','z')
grid on
% Hint: delete(hObject) closes the figure
delete(hObject);


% --- Executes on key press with focus on main_figure or any of its controls.
function main_figure_WindowKeyPressFcn(hObject, eventdata, handles)
% hObject    handle to main_figure (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
% Key: name of the key that was pressed, in lower case
% Character: character interpretation of the key(s) that was pressed
% Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)
key = eventdata.Key;
switch key
  case 'w'
    value = get(handles.throttle_slider, 'Value');
    set(handles.throttle_slider, 'Value', value + 0.1);
  case 's'
    value = get(handles.throttle_slider, 'Value');
    set(handles.throttle_slider, 'Value', value - 0.1);
  case 'a'
    value = get(handles.rudder_slider, 'Value');
    set(handles.rudder_slider, 'Value', value - 0.1);
  case 'd'
    value = get(handles.rudder_slider, 'Value');
    set(handles.rudder_slider, 'Value', value + 0.1);
  end

% --- Executes on mouse motion over figure - except title and menu.
function main_figure_WindowButtonMotionFcn(hObject, eventdata, handles)
% hObject    handle to main_figure (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)












% --- Executes on button press in simulate_toggle.
function simulate_toggle_Callback(hObject, eventdata, handles)
% hObject    handle to simulate_toggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of simulate_toggle
if get(hObject, 'Value') == 1
  if strcmp(get(handles.timer_simulator, 'Running'), 'off')
    start(handles.timer_simulator);
  end
  if strcmp(get(handles.timer_world, 'Running'), 'off')
      start(handles.timer_world);
  end
  if strcmp(get(handles.timer_gauge, 'Running'), 'off')
      start(handles.timer_gauge);
  end
  % if strcmp(get(handles.timer_robot, 'Running'), 'off')
  %     start(handles.timer_robot);
  % end
  if strcmp(get(handles.timer_user, 'Running'), 'off')
      start(handles.timer_user);
  end
else
  if strcmp(get(handles.timer_simulator, 'Running'), 'on')
      stop(handles.timer_simulator);
  end
  if strcmp(get(handles.timer_world, 'Running'), 'on')
      stop(handles.timer_world);
  end
  if strcmp(get(handles.timer_robot, 'Running'), 'on')
      stop(handles.timer_robot);
  end
  if strcmp(get(handles.timer_user, 'Running'), 'on')
      stop(handles.timer_user);
  end
end


% --- Executes on button press in control_toggle.
function control_toggle_Callback(hObject, eventdata, handles)
% hObject    handle to controller_toggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of controller_toggle
if get(hObject, 'Value') == 1
  if strcmp(get(handles.timer_user, 'Running'), 'on')
      stop(handles.timer_user);
  end
  if strcmp(get(handles.timer_robot, 'Running'), 'off')
      start(handles.timer_robot);
  end
else
  if strcmp(get(handles.timer_robot, 'Running'), 'on')
      stop(handles.timer_robot);
  end
  if strcmp(get(handles.timer_user, 'Running'), 'off')
    start(handles.timer_user);
  end
end
% release
guidata(hObject, handles);

% --- Executes on button press in reset_button.
function reset_button_Callback(hObject, eventdata, handles)
% hObject    handle to reset_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% init solver
handles.solver.init_position([0,0,0]);
handles.solver.init_linvel([50/3.6,0,0]);
handles.solver.init_orientation([1,0,0,0]);
% init model
handles.model.init();
% init robot
handles.rob.init();
% init world
set(handles.world.skin,...
    'Vertices', handles.solver.render());
clearpoints(handles.world.trace)
clearpoints(handles.world.trajectory)
addpoints(handles.world.trajectory,...
    handles.trajgen.get_splinepoints_x(),...
    handles.trajgen.get_splinepoints_y(),...
    handles.trajgen.get_splinepoints_z());
xlabel(handles.world_axes,'x [m]')
ylabel(handles.world_axes,'y [m]')
zlabel(handles.world_axes,'z [m]')
axis(handles.world_axes, [...
  -handles.settings.dx, handles.settings.dx,...
  -handles.settings.dy, handles.settings.dy,...
  -handles.settings.dz, handles.settings.dz]);
view(handles.world_axes,...
  handles.settings.azimuth,...
  handles.settings.zenith);
grid(handles.world_axes, 'on');
drawnow
% release lock
guidata(hObject, handles);


% --- Executes on button press in path_button.
function path_button_Callback(hObject, eventdata, handles)
% hObject    handle to path_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.trajgen.load_path(get(handles.path_edit, 'String'));
handles.trajgen.generate();
% release
guidata(hObject, handles);


% --- Executes on button press in model_button.
function model_button_Callback(hObject, eventdata, handles)
% hObject    handle to model_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.model.load_vehicle(get(handles.model_edit, 'String'));
% release
guidata(hObject, handles);


function path_edit_Callback(hObject, eventdata, handles)
% hObject    handle to path_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of path_edit as text
%        str2double(get(hObject,'String')) returns contents of path_edit as a double



function model_edit_Callback(hObject, eventdata, handles)
% hObject    handle to model_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of model_edit as text
%        str2double(get(hObject,'String')) returns contents of model_edit as a double


% --- Executes on slider movement.
function zenith_slider_Callback(hObject, eventdata, handles)
% hObject    handle to zenith_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.settings.zenith = get(hObject, 'Value');
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function zenith_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zenith_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function azimuth_slider_Callback(hObject, eventdata, handles)
% hObject    handle to azimuth_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.settings.azimuth = get(hObject, 'Value');
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function azimuth_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to azimuth_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function rudder_slider_Callback(hObject, eventdata, handles)
% hObject    handle to rudder_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function rudder_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rudder_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function throttle_slider_Callback(hObject, eventdata, handles)
% hObject    handle to throttle_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function throttle_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to throttle_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function dx_slider_Callback(hObject, eventdata, handles)
% hObject    handle to dx_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.settings.dx = get(hObject, 'Value');
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function dx_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dx_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes during object creation, after setting all properties.
function dy_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dy_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function dy_slider_Callback(hObject, eventdata, handles)
% hObject    handle to dy_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.settings.dy = get(hObject, 'Value');
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function dz_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dz_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function dz_slider_Callback(hObject, eventdata, handles)
% hObject    handle to dz_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.settings.dz = get(hObject, 'Value');
guidata(hObject, handles);