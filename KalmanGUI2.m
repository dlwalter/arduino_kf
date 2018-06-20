function varargout = KalmanGUI2(varargin)
% KalmanGUI2 MATLAB code for KalmanGUI2.fig
%      KalmanGUI2, by itself, creates a new KalmanGUI2 or raises the existing
%      singleton*.
%
%      H = KalmanGUI2 returns the handle to a new KalmanGUI2 or the handle to
%      the existing singleton*.
%
%      KalmanGUI2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in KalmanGUI2.M with the given input arguments.
%
%      KalmanGUI2('Property','Value',...) creates a new KalmanGUI2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before KalmanGUI2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to KalmanGUI2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help KalmanGUI2

% Last Modified by GUIDE v2.5 16-Mar-2014 13:48:08

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @KalmanGUI2_OpeningFcn, ...
                   'gui_OutputFcn',  @KalmanGUI2_OutputFcn, ...
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


% --- Executes just before KalmanGUI2 is made visible.
function KalmanGUI2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to KalmanGUI2 (see VARARGIN)

% Choose default command line output for KalmanGUI2
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
set(handles.axes, 'Xlim', [0, 5])
set(handles.axes, 'Ylim', [-35, 35]);
grid on;

% UIWAIT makes KalmanGUI2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = KalmanGUI2_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%Set MetaData

closeSerial();
formatSpec = '%+2.2f';
cla(handles.axes);

%Change button to "Turn Off" Button
set(hObject,'string', 'Turn Off', 'backgroundcolor', [1 0 0]);
set(hObject, 'callback', {@pushbutton1_Callback2, handles});
set(handles.status, 'string', 'Connecting...');
pause(0.01);
set(hObject, 'UserData', 1)


%Connect to Arduino IMU
[a.s f] = setupSerial(get(handles.arduinoAddress, 'string'));

%Once connected Calibrate Accelerometer
set(handles.status, 'string', 'Calibrating');
pause(0.01)
sum_roll = 0; sum_pitch = 0;
avgtot = 200;
for i = 1:avgtot
  [roll pitch] = getAccelAngles(a, [0,0]); 
  sum_roll = sum_roll + roll;
  sum_pitch = sum_pitch + pitch;
end
offset = [sum_roll/avgtot sum_pitch/avgtot];
%Calibration complete

standDisabled = 1;
encpos = 0;
if get(handles.disableStand, 'value') == 0
    standDisabled = 0;
    set(handles.status, 'string', 'Connecting Test Stand...');
    pause(0.01);
    minPWM = 40;  %Minimum value for motors due to internal friction

    %Connect to Test Stand
    s = arduino(get(handles.standAddress, 'string'));
    s.encoderAttach(0,3,2);  %Setup Encoder on int Pins 3 and 2
    encpos = 0;
    PWM1 = 6;   %Set Motor Controller Pins
    DIR1 = 7;
    s.pinMode(PWM1, 'output');
    s.pinMode(DIR1, 'output');

    %Test Stand Constants
    gearratio = 25;
    encpulses = 400;
    convFactor = 360/(encpulses*gearratio);
    PosMax = 30; %max amplitude in degrees
    timefactor = 12;

    %Command Signal waveforem;
    commandt = 1:1000*timefactor;
    commandPos = PosMax*sin(2*pi*1/timefactor*commandt/1000);
    commandVel = PosMax*2*pi*1/timefactor*cos(2*pi*1/timefactor*commandt/1000);

    %PID
    pidTerm = 0;
    pidError = 0;
    PWM = 0;
    DIR = 1;

    %Start Test Stand
    s.digitalWrite(DIR1, 1);
    s.analogWrite(PWM1, uint8(PWM));
end

set(handles.status, 'string', 'Connected');

%Prep Sampling Frequency / Looping Constants
samplingFreq = 15; %Hz
ind = 1;
nind = 1;
dt = 1.0/samplingFreq;

%Set Initial State
angle_roll = 0;
angle_pitch = 0;
bias_roll = 0;
bias_pitch = 0;

%Set Variance for Accelerometer and Gyro
% var_aroll = 0.0225324;   %From DFIT
% var_apitch = 0.0224521;
var_aroll = 1.5;    %From Realtime Analysis
var_apitch = 1.5;


var_groll = 0.0173011;   %From DFIT
var_gpitch = 0.0348563;

var_grollbias = 0.2;  %Out of my ass.
var_gpitchbias = 0.2;


%Measurement Variance Matrix
V = [var_aroll 0; 0 var_apitch];

%Procces Noise Variance Matrix
W = [var_groll 0; 0 var_gpitch];

%Observation Model
H = [1 0 0 0; 0 1 0 0];

%Initial A Priori State
x_hat = [0;0;0;0];

%Initial A Priori Covariance (Uncorrolated)
%M = [var_groll 0 0 0; 0 var_gpitch 0 0; 0 0 var_grollbias 0; 0 0 0 var_gpitchbias];
M = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

%Initial A Posteriori Covariance
P = (M^-1 + H'*V^-1*H)^-1;

%Data Storage
data = struct('accel', zeros(2,10000),...
              'xhat', zeros(4,10000),...
              'kpitch', zeros(1,10000),...
              'encoder', zeros(1,10000),...
              'time', zeros(1,10000),...
              'Pcovariance', zeros(4,4,10000),...
              'KalmanGain', zeros(4,2,10000),...
              'KalmanError', zeros(2,10000));


tstart = tic;
while get(hObject, 'UserData') == 1
    lstart = tic;
    if standDisabled == 0
        %Get Encoder Position in deg
        encposnew = s.encoderRead(0)*convFactor;
        ts = toc(tstart);
        ts(ts<0.001) = 0.001;
        %Use current time to establish index in Command Signal
        commandIndex = round(mod(ts, timefactor)*1000);
        tgtSpd = commandVel(commandIndex);

        %Use PID to control Motor Speed
        motorSpd = (encposnew - encpos)/dt;
        [PWM pidError] = updatePID(PWM, tgtSpd, motorSpd, pidError);
        %Determine Motor Direction
        if PWM < 0;
            DIR = 0;
        else
            DIR = 1;
        end
        
        s.digitalWrite(DIR1, DIR);
        s.analogWrite(PWM1, uint8(abs(PWM)));
        encpos = encposnew;
    else
        encpos = 0;
    end
    %Get current values from Accelerometer/Gyro
    [roll pitch gpitch groll gz] = readBoth(a, offset);
    %groll = -gpitch;  %Why did I have this here?
    %Run Kalman Filter Code Here
    theta_dot = [groll;gpitch];
    %Measurement
    z = [roll; pitch];
    %Transition Model
    PHI = [1 0 -dt 0; 0 1 0 -dt; 0 0 1 0; 0 0 0 1];
    GAMMA = [dt 0; 0 dt; 0 0; 0 0];
    THETA = [0 0; 0 0; 1 0; 0 1];
    
    B = [var_grollbias 0; 0 var_gpitchbias];
    %Calculate A Priori Estimate:
    x_bar = PHI*x_hat + GAMMA*theta_dot;
    %Calculate A Priori Covariance
    M = PHI*P*PHI' + GAMMA*W*GAMMA';% + THETA*B*THETA';
    %Calculate Kalman Gain
    K = M*H'*(H*M*H' + V)^-1;
    %Calculate Residual
    r = z - H*x_bar;
    %Calculate A Posteriori Estimate
    x_hat = x_bar + K*r;
    %Calculate A Posteriori Covariance
    P = (M^-1 + H'*V^-1*H)^-1;
           
    %Store Data for plotting/saving
    data.accel(:,ind) = z;
    data.xhat(:,ind) = x_hat;
    data.kpitch(:,ind) = x_hat(2);
    data.time(:,ind) = toc(tstart);
    data.Pcovariance(:,:,ind) = P;
    data.KalmanGain(:,:,ind) = K;
    data.encoder(:,ind) = encpos;
    
    %Purge zero entries
    data.accel(data.accel == 0) = NaN;
    data.kpitch(data.kpitch == 0) = NaN;
    data.encoder(data.encoder == 0) = NaN;
 
    
    select = get(handles.outputselect, 'Value');
    
    %Update display every n samples
    if mod(ind, 5) == 0 && get(hObject, 'UserData') == 1
        set(handles.samplingfreq, 'string',...
                    strcat(num2str(1/dt, '%50.1f'),' Hz'));
        set(handles.rollbias, 'string', num2str(x_hat(3), formatSpec));
        set(handles.pitchbias, 'string', num2str(x_hat(4), formatSpec));
        pause(0.0005);
        
                
        switch select
                % 1.Accelerometer [deg]             
                % 2.A Posteriori Estimate [deg]
                % 3.Encoder Position
                % 4.Error (Accelerometer - Command)    
                % 5.Error (A Posteriori - Command)  
            
            
            %Display Measurements from Accelerometer
            case 1
                set(handles.rollout, 'string', num2str(roll, formatSpec));
                set(handles.pitchout, 'string', num2str(pitch, formatSpec));
                pause(0.0005);  
                
            %Display Kalman Filter Estimate
            case 2
                set(handles.rollout, 'string', num2str(x_hat(1), formatSpec));
                set(handles.pitchout, 'string', num2str(x_hat(2), formatSpec));
                pause(0.0005);
                
            %Display  Encoder Position
            case 3
                set(handles.rollout, 'string', num2str(0.00, '%+50.4f'));
                set(handles.pitchout, 'string', num2str(encpos, '%+50.4f'));
                pause(0.0005);    
            %Display error between Accelerometer and Command
            case 4
                set(handles.rollout, 'string', num2str(0.00, '%+50.4f'));
                set(handles.pitchout, 'string', num2str(pitch-encpos, '%+50.4f'));
                pause(0.0005);
          
            %Display error between Filter and Command
            case 5
                set(handles.rollout, 'string', num2str(0.00, '%+50.4f'));
                set(handles.pitchout, 'string', num2str(x_hat(2)-encpos, '%+50.4f'));
                pause(0.0005);
        end
       
    end
    
    %Plot Data
    handles.data = data;
    guidata(hObject, handles);
    
    if ind <= 50
        leftind = 1;
        
    else
        leftind = ind-50;
    end
    
    xleft = data.time(leftind);
    xright = xleft + 5;
    
    switch select
        
        %Plot Current Position Data
        case {1,2,3}    
            plot(handles.axes,data.time(1,leftind:ind),data.encoder(1,leftind:ind), 'k',...
                data.time(1,leftind:ind),data.kpitch(1,leftind:ind), 'g',...
                data.time(1,leftind:ind),data.accel(2,leftind:ind), 'r');
            set(handles.axes, 'Xlim', [xleft, xright])
            set(handles.axes, 'Ylim', [-35, 35]);
            grid on;
            drawnow;
        %Plot Errors of Kalman and Accelerometer
        case {4,5}    
            plot(handles.axes,data.time(1,leftind:ind),data.encoder(1,leftind:ind)-data.encoder(1,leftind:ind), 'k',...
                data.time(1,leftind:ind),data.kpitch(1,leftind:ind)-data.encoder(1,leftind:ind), 'g',...
                data.time(1,leftind:ind),data.accel(2,leftind:ind)-data.encoder(1,leftind:ind), 'r');
            set(handles.axes, 'Xlim', [xleft, xright])
            set(handles.axes, 'Ylim', [-35, 35]);
            grid on;
            drawnow;
    end
    
    
    %Wait for next measurement to ensure fixed sampling frequency
    while(nind ==ind && get(hObject, 'UserData') == 1)
        nind = floor(toc(tstart)*samplingFreq)+1;
    end
    ind = nind;
    %Record Sample length of loop
    dt = toc(lstart);
end

%Return to level
if standDisabled == 0
    set(handles.status, 'string', 'Levelling...');
    while encpos ~= 0
        if encpos > 0
            DIR = 0;
        else
            DIR = 1;
        end
        
        encpos = s.encoderRead(0)*convFactor;
        set(handles.rollout, 'string', num2str(0.00, '%+50.2f'));
        set(handles.pitchout, 'string', num2str(encpos, '%+50.2f'));
        pause(0.0005);
        
        PWM = abs(255*encpos/40);
        PWM(PWM>255) = 255;
        PWM(PWM<minPWM) = minPWM;
        s.digitalWrite(DIR1, DIR);
        s.analogWrite(PWM1, uint8(PWM));
        
    end
    s.analogWrite(PWM1, 0);
    delete(s);
end
set(handles.status, 'string', 'Standby');
closeSerial()


function pushbutton1_Callback2(hObject, eventdata, handles)
set(hObject,'string', 'Turn On', 'backgroundcolor', [0.9294 0.9294 0.9294]);
set(hObject, 'callback', {@pushbutton1_Callback, handles});
set(handles.status, 'string', 'Standby');
set(hObject, 'UserData', 0)



% --- Executes on selection change in outputselect.
function outputselect_Callback(hObject, eventdata, handles)
% hObject    handle to outputselect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns outputselect contents as cell array
%        contents{get(hObject,'Value')} returns selected item from outputselect


% --- Executes during object creation, after setting all properties.
function outputselect_CreateFcn(hObject, eventdata, handles)
% hObject    handle to outputselect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function arduinoAddress_Callback(hObject, eventdata, handles)
% hObject    handle to arduinoAddress (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of arduinoAddress as text
%        str2double(get(hObject,'String')) returns contents of arduinoAddress as a double


% --- Executes during object creation, after setting all properties.
function arduinoAddress_CreateFcn(hObject, eventdata, handles)
% hObject    handle to arduinoAddress (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function standAddress_Callback(hObject, eventdata, handles)
% hObject    handle to standAddress (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of standAddress as text
%        str2double(get(hObject,'String')) returns contents of standAddress as a double


% --- Executes during object creation, after setting all properties.
function standAddress_CreateFcn(hObject, eventdata, handles)
% hObject    handle to standAddress (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in disableStand.
function disableStand_Callback(hObject, eventdata, handles)
% hObject    handle to disableStand (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of disableStand


% --- Executes on button press in saveData.
function saveData_Callback(hObject, eventdata, handles)
% hObject    handle to saveData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
data = handles.data;
uisave('data','KalmanData');
