classdef PoseViewerWithSwitches < HelperPoseViewer
    properties (Hidden)
        thefig
    end
    methods (Access = protected)
        function setupImpl(obj, varargin)
            obj.thefig.Visible = true;
            setupImpl@HelperPoseViewer(obj, varargin{:});
        end
        
        function createAppWindow(obj)
            fig = figure('Name', 'Pose Viewer', ...
                'NumberTitle', 'off', ...
                'DockControls','off', ...
                'Units', 'normalized', ...
                'OuterPosition', [0 0.25 0.5 0.5], ...
                'Visible', 'off', ...
                'HandleVisibility', 'on', ...
                'NextPlot', 'new', ...
                'IntegerHandle', 'off', ...
                'CloseRequestFcn', @(x,~)set(x,'Visible', 'off'));
            u1 = uipanel('Title', 'Pose', 'Parent', fig, 'Position', ...
                [0 0 0.75 1]);
            u2 = uipanel('Title', "Sensors", "Parent", fig, 'Position', ...
                [0.75 0 0.25 1]);
            obj.AppWindow = u1;
            obj.thefig = fig;
            
            ac = uicontrol(u2, 'Style', 'Checkbox', 'String', 'Accelerometer', ...
                'Units', 'normalized', 'Callback', @checkboxcb);
            ac.Position(2:3) = [0.8 0.5];
            ac.Value = true;
            af = uicontrol(u2, 'Style', 'popupmenu', 'String', ...
                ["100 Hz", "75 Hz", "50 Hz"], ...
                'Units', 'normalized', 'tag', 'AccelerometerSampleRate', ...
                'Callback', @pulldowncb);
            af.Position(1:2) = [0.6 0.8];
            
            gc = uicontrol(u2, 'Style', 'Checkbox', 'String', 'Gyroscope', ...
                'Units', 'normalized', 'Callback', @checkboxcb);
            gc.Position(2:3) = [0.7 0.5];
            gc.Value = true;
            gf = uicontrol(u2, 'Style', 'popupmenu', 'String', ...
                ["100 Hz", "75 Hz", "50 Hz"], ...
                'Units', 'normalized', 'tag', 'GyroscopeSampleRate', ...
                'Callback', @pulldowncb);
            gf.Position(1:2) = [0.6 0.7];
            
            mc = uicontrol(u2, 'Style', 'Checkbox', 'String', 'Magnetometer', ...
                'Units', 'normalized', 'Callback', @checkboxcb);
            mc.Position(2:3) = [0.6 0.5];
            mc.Value = true;
            mf = uicontrol(u2, 'Style', 'popupmenu', 'String', ["50 Hz", "25 Hz"], ...
                'Units', 'normalized', 'tag', 'MagnetometerSampleRate', ...
                'Callback', @pulldowncb);
            mf.Position(1:2) = [0.6 0.6];
            
            gpsc = uicontrol(u2, 'Style', 'Checkbox', 'String', 'GPS', ...
                'Units', 'normalized', 'Callback', @checkboxcb);
            gpsc.Position(2:3) = [0.5 0.5];
            gpsc.Value = true;
            gpsf = uicontrol(u2, 'Style', 'popupmenu', 'String', ...
                ["5 Hz", "1 Hz"], ...
                'Units', 'normalized', 'tag', 'GPSSampleRate', ...
                'Callback', @pulldowncb);
            gpsf.Position(1:2) = [0.6 0.5];
            
            % Defaults
            fig.UserData.Accelerometer = true;
            fig.UserData.Gyroscope = true;
            fig.UserData.Magnetometer = true;
            fig.UserData.GPS = true;
            fig.UserData.AccelerometerSampleRate = 100;
            fig.UserData.GyroscopeSampleRate = 100;
            fig.UserData.MagnetometerSampleRate = 50;
            fig.UserData.GPSSampleRate = 5;
        end
    end
    
end

function checkboxcb(src, ~)
[~,f] = gcbo;
f.UserData.(src.String) = src.Value;
end

function pulldowncb(src, ~)
[~,f] = gcbo;
str = strsplit(src.String{src.Value});
f.UserData.(src.Tag) = str2double(str{1});
end
