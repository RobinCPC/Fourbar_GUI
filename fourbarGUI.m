function varargout = fourbarGUI(varargin)
% FOURBARGUI M-file for fourbarGUI.fig
%      FOURBARGUI, by itself, creates a new FOURBARGUI or raises the existing
%      singleton*.
%
%      H = FOURBARGUI returns the handle to a new FOURBARGUI or the handle to
%      the existing singleton*.
%
%      FOURBARGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FOURBARGUI.M with the given input arguments.
%
%      FOURBARGUI('Property','Value',...) creates a new FOURBARGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before fourbarGUI_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to fourbarGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help fourbarGUI

% Last Modified by GUIDE v2.5 15-Jun-2009 10:23:41

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @fourbarGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @fourbarGUI_OutputFcn, ...
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


% --- Executes just before fourbarGUI is made visible.
function fourbarGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to fourbarGUI (see VARARGIN)

% Choose default command line output for fourbarGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes fourbarGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);
axes(handles.axes1);  %show figure on axes object of axes1
[x,y,z,v] = flow;
p = patch(isosurface(x,y,z,v,-3));
set(p,'facecolor','w','EdgeColor','b');
reducepatch(p,0.15); daspect([1,1,1]);
view(3);
set(gca,'visible','off') %hide the axes label of axes1


% --- Outputs from this function are returned to the command line.
function varargout = fourbarGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function Fralen_Callback(hObject, eventdata, handles)
% hObject    handle to Fralen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Fralen as text
%        str2double(get(hObject,'String')) returns contents of Fralen as a double


% --- Executes during object creation, after setting all properties.
function Fralen_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Fralen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cralen_Callback(hObject, eventdata, handles)
% hObject    handle to Cralen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cralen as text
%        str2double(get(hObject,'String')) returns contents of Cralen as a double


% --- Executes during object creation, after setting all properties.
function Cralen_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cralen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Coulen_Callback(hObject, eventdata, handles)
% hObject    handle to Coulen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Coulen as text
%        str2double(get(hObject,'String')) returns contents of Coulen as a double


% --- Executes during object creation, after setting all properties.
function Coulen_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Coulen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Roclen_Callback(hObject, eventdata, handles)
% hObject    handle to Roclen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Roclen as text
%        str2double(get(hObject,'String')) returns contents of Roclen as a double


% --- Executes during object creation, after setting all properties.
function Roclen_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Roclen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Courad_Callback(hObject, eventdata, handles)
% hObject    handle to Courad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Courad as text
%        str2double(get(hObject,'String')) returns contents of Courad as a double


% --- Executes during object creation, after setting all properties.
function Courad_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Courad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Fraang_Callback(hObject, eventdata, handles)
% hObject    handle to Fraang (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Fraang as text
%        str2double(get(hObject,'String')) returns contents of Fraang as a double


% --- Executes during object creation, after setting all properties.
function Fraang_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Fraang (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Betang_Callback(hObject, eventdata, handles)
% hObject    handle to Betang (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Betang as text
%        str2double(get(hObject,'String')) returns contents of Betang as a double


% --- Executes during object creation, after setting all properties.
function Betang_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Betang (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in AniPush.
function AniPush_Callback(hObject, eventdata, handles)
% hObject    handle to AniPush (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%below is the callback function of animate push
%get all of value of text edit, and transfer to number.
hfr = findobj('tag','Fralen'); r1 = str2double(get(hfr,'String'));
% r1 = str2double(handles.Fralen.String)  test new my to program
hcr = findobj('tag','Cralen'); r2 = str2double(get(hcr,'String'));
hcl = findobj('tag','Coulen'); r3 = str2double(get(hcl,'String'));
hrc = findobj('tag','Roclen'); r4 = str2double(get(hrc,'String'));
hcour = findobj('tag','Courad'); r6 = str2double(get(hcour,'String'));
hfa = findobj('tag','Fraang'); theta1 = str2double(get(hfa,'String'));
hba = findobj('tag','Betang'); beta = str2double(get(hba,'String'));
%get the value of assembly mode
% if get(handles.pos) == 1
%     handles.neg.value = 0
% else
%     handles.pos.value = 0
% end

%set the direction of rotate
nn=1;
a1 = 180:-nn:0;
a2 = -1:-nn:-179;
theta2=[a1 a2 a1 a2 a1 a2]; % three cycle
n = numel(theta2);

%decide assembly mode
hpos = findobj('tag','pos'); 
posvalue = get(hpos,'value');
    
% calculate loop equation

rm = r2; rj = r3;
thetam = theta2;
A = 2*r1*r4*cos(theta1*(pi/180))-2*rm*r4*cos(thetam.*(pi/180));
B = 2*r1*r4*sin(theta1*(pi/180))-2*rm*r4*sin(thetam.*(pi/180));
C = (r1^2)+(rm^2)+(r4^2)-(rj^2)-2*r1*rm*(cos(theta1*(pi/180))*cos(thetam.*(pi/180))+sin(theta1*(pi/180))*sin(thetam.*(pi/180)));

det = sqrt((B.^2)+(A.^2)-(C.^2));

if det > 0
    htext8 = findobj('tag','text8'); set(htext8,'String','Two distinct real roots!!');
    theta41 = 2* atan2((-B+det),(C-A))*(180/pi);
    for i = 1:n
        if (theta41(i) > 0)
            theta41(i) = theta41(i)-360;
        end
    end
    theta42 = 2* atan2((-B-det),(C-A))*(180/pi);
     for i = 1:n
        if (theta42(i) > 0)
            theta42(i) = theta42(i)-360;
        end
    end
    thetaj1 = atan2((r1*sin(theta1*(pi/180))+r4*sin(theta41*(pi/180))-rm*sin(thetam.*(pi/180))),...
        (r1*cos(theta1*(pi/180))+r4*cos(theta41*(pi/180))-rm*cos(thetam.*(pi/180))));
    
    thetaj2 = atan2((r1*sin(theta1*(pi/180))+r4*sin(theta42*(pi/180))-rm*sin(thetam.*(pi/180))),...
        (r1*cos(theta1*(pi/180))+r4*cos(theta42*(pi/180))-rm*cos(thetam.*(pi/180))));
    
    theta31 = thetaj1*(180/pi);
    
    theta32 = thetaj2*(180/pi);
    
elseif det == 0
    htext8 = findobj('tag','text8'); set(htext8,'String','One real root!!');
    theta4 = 2* atan2((-B),(C-A))*(180/pi);
    
    thetaj = atan2((r1*sin(theta1*(pi/180))+r4*sin(theta4*(pi/180))-rm*sin(thetam.*(pi/180))),...
        (r1*cos(theta1*(pi/180))+r4*cos(theta4*(pi/180))-rm*cos(thetam.*(pi/180))));
    
    theta3 = thetaj*(180/pi);
    
else 
    htext8 = findobj('tag','text8'); set(htext8,'String','No solution!!');
%     break         can't use break in GUI
end

%先找出P點的軌跡

X = zeros(1,n); Y = zeros(1,n); %for 儲存P點的位移
M = zeros(1,n); N = zeros(1,n); %for 儲存B點的位移

if posvalue == 1 %assembly mode is 1
    for i = 1:n
    JBx = r1*cos(theta1*(pi/180))+r4*cos(theta41(i)*(pi/180)); JBy = r1*sin(theta1*(pi/180))+r4*sin(theta41(i)*(pi/180));
    theta5 =atan2((r1*sin(theta1*(pi/180))+r4*sin(theta41(i)*(pi/180))-r2*sin(theta2(i)*(pi/180))),(r1*cos(theta1*(pi/180))+r4*cos(theta41(i)*(pi/180))-r2*cos(theta2(i)*(pi/180))))*(180/pi);
    theta6 = beta + theta5;
    JPx = r2*cos(theta2(i)*(pi/180))+r6*cos(theta6*(pi/180)); JPy = r2*sin(theta2(i)*(pi/180))+r6*sin(theta6*(pi/180));
        M(i)=JBx; N(i)=JBy;
        X(i)=JPx; Y(i)=JPy;
    end
    %加上four bar 的位置的運動圖  以及上一點的位移
    for i = 1:n
        JAx = r2*cos(theta2(i)*(pi/180)); JAy = r2*sin(theta2(i)*(pi/180));
        JBx = r1*cos(theta1*(pi/180))+r4*cos(theta41(i)*(pi/180)); JBy = r1*sin(theta1*(pi/180))+r4*sin(theta41(i)*(pi/180));
        JCx = r1*cos(theta1*(pi/180)); JCy = r1*sin(theta1*(pi/180));
        JOx = 0; JOy = 0;
        x1 = [JOx JAx JBx JCx]; y1 = [JOy JAy JBy JCy];

        theta5 =atan2((r1*sin(theta1*(pi/180))+r4*sin(theta41(i)*(pi/180))-r2*sin(theta2(i)*(pi/180))),(r1*cos(theta1*(pi/180))+r4*cos(theta41(i)*(pi/180))-r2*cos(theta2(i)*(pi/180))))*(180/pi);
        theta6 = beta + theta5;
        JPx = r2*cos(theta2(i)*(pi/180))+r6*cos(theta6*(pi/180)); JPy = r2*sin(theta2(i)*(pi/180))+r6*sin(theta6*(pi/180));

            x2 = JPx ; y2 = JPy;

        h = plot(x1, y1,x2,y2,'go', X, Y,'r:', M, N,'m:','erasemode', 'none');
        axis([-20,80,-20,60] , 'equal');
        drawnow
        
    end
else %assembly mode is -1
    for i = 1:n
    JBx = r1*cos(theta1*(pi/180))+r4*cos(theta42(i)*(pi/180)); JBy = r1*sin(theta1*(pi/180))+r4*sin(theta42(i)*(pi/180));
    theta5 =atan2((r1*sin(theta1*(pi/180))+r4*sin(theta42(i)*(pi/180))-r2*sin(theta2(i)*(pi/180))),(r1*cos(theta1*(pi/180))+r4*cos(theta42(i)*(pi/180))-r2*cos(theta2(i)*(pi/180))))*(180/pi);
    theta6 = beta + theta5;
    JPx = r2*cos(theta2(i)*(pi/180))+r6*cos(theta6*(pi/180)); JPy = r2*sin(theta2(i)*(pi/180))+r6*sin(theta6*(pi/180));
        M(i)=JBx; N(i)=JBy;
        X(i)=JPx; Y(i)=JPy;
    end
    %加上four bar 的位置的運動圖  以及上一點的位移
    for i = 1:n
        JAx = r2*cos(theta2(i)*(pi/180)); JAy = r2*sin(theta2(i)*(pi/180));
        JBx = r1*cos(theta1*(pi/180))+r4*cos(theta42(i)*(pi/180)); JBy = r1*sin(theta1*(pi/180))+r4*sin(theta42(i)*(pi/180));
        JCx = r1*cos(theta1*(pi/180)); JCy = r1*sin(theta1*(pi/180));
        JOx = 0; JOy = 0;
        x1 = [JOx JAx JBx JCx]; y1 = [JOy JAy JBy JCy];

        theta5 =atan2((r1*sin(theta1*(pi/180))+r4*sin(theta42(i)*(pi/180))-r2*sin(theta2(i)*(pi/180))),(r1*cos(theta1*(pi/180))+r4*cos(theta42(i)*(pi/180))-r2*cos(theta2(i)*(pi/180))))*(180/pi);
        theta6 = beta + theta5;
        JPx = r2*cos(theta2(i)*(pi/180))+r6*cos(theta6*(pi/180)); JPy = r2*sin(theta2(i)*(pi/180))+r6*sin(theta6*(pi/180));

            x2 = JPx ; y2 = JPy;

        h = plot(x1, y1,x2,y2,'go', X, Y,'r:', M, N,'m:','erasemode', 'none');
        axis([min(x1)-20,max(x1)+20,min(y1)-20,max(y1)+20],'equal');
        drawnow
        
    end
end
        

% --- Executes on button press in AnaPush.
function AnaPush_Callback(hObject, eventdata, handles)
% hObject    handle to AnaPush (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in InfPush.
function InfPush_Callback(hObject, eventdata, handles)
% hObject    handle to InfPush (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%show the info picture
figure('Name','Fourbar Info','NumberTitle','off','Menubar','none');
imshow('FourbarInfo.jpg');



% --- Executes on button press in pos.
function pos_Callback(hObject, eventdata, handles)
% hObject    handle to pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of pos


% --- Executes on button press in neg.
function neg_Callback(hObject, eventdata, handles)
% hObject    handle to neg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of neg


