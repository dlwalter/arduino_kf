function [] = KalmanGUI()


%variance in accelerometer X-axis
V_a = 0.03796^2;  %m^2/s^4

%variance in gyroscope
V_g = 455.26;     %deg^2/


S.fh = figure('units','pixels',...
              'position',[400 400 120 100],...
              'menubar','none',...
              'name','Kalman Filter',...
              'numbertitle','off',...
              'resize','off');
S.pb = uicontrol('style','push',...
              'unit','pix',...
              'position',[10 10 100 20],...
              'string','Turn On',...
              'tooltip','Push to Turn on Arduino',...
              'UserData', 0);
set(S.pb, 'callback', {@start_call,S});


           


      


function [] = start_call(varargin)

S = varargin{3};

set(S.pb, 'callback', {@stop_call,S});
set(S.pb, 'string', 'Turn Off', 'backgroundcolor', [1 0 0])
pause(0.01)
set(S.pb, 'UserData', 1)

while get(S.pb, 'UserData') == 1
    disp(datestr(now, 0));
    pause(0.25);
end
  
function [] = stop_call(varargin)

S = varargin{3};
set(S.pb, 'callback', {@start_call, S});
set(S.pb, 'string', 'Turn On', 'backgroundcolor', [.9294, .9292, .9294])
set(S.pb, 'UserData', 0);