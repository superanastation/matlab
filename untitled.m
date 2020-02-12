function varargout = untitled(varargin)
% UNTITLED MATLAB code for untitled.fig
%      UNTITLED, by itself, creates a new UNTITLED or raises the existing
%      singleton*.
%
%      H = UNTITLED returns the handle to a new UNTITLED or the handle to
%      the existing singleton*.
%
%      UNTITLED('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in UNTITLED.M with the given input arguments.
%
%      UNTITLED('Property','Value',...) creates a new UNTITLED or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before untitled_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to untitled_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help untitled

% Last Modified by GUIDE v2.5 21-Apr-2016 14:15:45

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @untitled_OpeningFcn, ...
                   'gui_OutputFcn',  @untitled_OutputFcn, ...
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
end
% End initialization code - DO NOT EDIT


% --- Executes just before untitled is made visible.
function untitled_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to untitled (see VARARGIN)

% Choose default command line output for untitled
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes untitled wait for user response (see UIRESUME)
% uiwait(handles.figure1);

end
% --- Outputs from this function are returned to the command line.
function varargout = untitled_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


end
function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double

end
% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


end
function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double

end
% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double

end
% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double

end
% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double

end
% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end
% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end
% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
L=get(handles.edit1,'string');
L=str2double(L);
I=get(handles.edit2,'string');
I=str2double(I);
I=I+1; %т.к. массивы здесь от 1 !!!
T=get(handles.edit3,'string');
T=str2double(T);
K=get(handles.edit4,'string');
K=str2double(K);
K=K+1;
D=get(handles.edit5,'string');
D=str2double(D);
H=get(handles.edit6,'string');
H=str2double(H);
Uc=get(handles.edit7,'string');
Uc=str2double(Uc);
% implicit(L,I,T,K,D,H,Uc);
func(L,I,T,K,D,H,Uc) 


end
function func(L,I,T,K,D,H,Uc)
hx=L/I;
ht=T/K;

x=linspace(0,L,I);
t = linspace(0, T, K);
g=ht*D/(hx*hx);
for i=1:I;
u(i,1)=Task(x(i));
end
for k=2:K;
    u(1,k)=2*g*(u(2,k-1)-u(1,k-1))+u(1,k-1);
    for i=2:I-1;        
        u(i,k)=g*(u(i+1,k-1)-2*u(i,k-1)+u(i-1,k-1))+u(i,k-1);
    end    
    u(I,k)=g*(2*hx*H*(Uc-u(I,k-1))+2*u(I-1,k-1)-2*u(I,k-1))+u(I,k-1);
end
k=1:K;
i=1:I;
% pcolor(handles.axes1,u);
% shading interp;
% mesh(t,x,u); colorbar;
% ylabel('координата') 
% xlabel('время') 
figure
plot(t,u(I,k))
hold on
plot(t,Solve(x(I),t,D),'r')
legend('Явная схема', 'Тестовое решение')
title('Конец цилиндра с мембраной')
ylabel('концентрация, г/см^3') 
xlabel('время, с') 
figure
plot(t,u(1,k))
hold on
plot(t,Solve(x(1),t,D),'r')
legend('Явная схема', 'Тестовое решение')
title('Закупоренный конец цилиндра')
ylabel('концентрация, г/см^3') 
xlabel('время, с') 
figure
plot(x,u(i,1))
hold on
plot(x,Solve(x,t(1),D),'r')
legend('Явная схема', 'Тестовое решение')
title('Концентрация в начальный момент времени')
ylabel('концентрация, г/см^3') 
xlabel('координата, см') 
figure
plot(x,u(i,K))
hold on
plot(x,Solve(x,t(K),D),'r')
legend('Явная схема', 'Тестовое решение')
title('Концентрация в конечный момент времени')
ylabel('концентрация, г/см^3') 
xlabel('координата, см') 
end
function implicit(L,I,T,K,D,H,Uc)
hx = L/I;
ht = T/K;

x = linspace(0,L,I);
t = linspace(0, T, K);
gamma = ht*D/(hx*hx);
for i = 1:I;
u(i,1)=Task(x(i));
end
for k = 2:K;
    delta(1) = 2*gamma/(1+2*gamma);
     lambda(1) = u(1,k-1)/(1+2*gamma);     
    for i = 2:I-1;
        delta(i) = gamma/(1+gamma*(2-delta(i-1)));
        lambda(i) = (u(i, k-1) + gamma*lambda(i-1))/(1 + gamma*(2-delta(i-1)));
    end;
    u(I,k) = (u(I, k-1) + 2*gamma*lambda(I-1))/(1 + 2*gamma*(1+hx*H - delta(I-1)));
    for i = I-1:(-1):2;
     u(i,k) = delta(i)*u(i+1,k)+lambda(i); 
    end;
u(1,k)=delta(1)*u(2,k)+lambda(1);
end;
k=1:K;
i=1:I;
% pcolor(handles.axes1,u);
% shading interp;
% mesh(t,x,u); colorbar;
% ylabel('координата') 
% xlabel('время') 
figure
plot(t,u(I,k))
hold on
plot(t,Solve(x(I),t,D),'r')
legend('Неявная схема', 'Тестовое решение')
title('Конец цилиндра с мембраной')
ylabel('концентрация, г/см^3') 
xlabel('время, с') 
figure
plot(t,u(1,k))
hold on
plot(t,Solve(x(1),t,D),'r')
legend('Неявная схема', 'Тестовое решение')
title('Закупоренный конец цилиндра')
ylabel('концентрация, г/см^3') 
xlabel('время, с') 
figure
plot(x,u(i,1))
hold on
plot(x,Solve(x,t(1),D),'r')
legend('Неявная схема', 'Тестовое решение')
title('Концентрация в начальный момент времени')
ylabel('концентрация, г/см^3') 
xlabel('координата, см') 
figure
plot(x,u(i,K))
hold on
plot(x,Solve(x,t(K),D),'r')
legend('Неявная схема', 'Тестовое решение')
title('Концентрация в конечный момент времени')
ylabel('концентрация, г/см^3') 
xlabel('координата, см') 
end

% u(I+1,k)=2*hx*H(Uc-u(I,k))+u(I-1,k);
function [s]=Task(x)
    s = cos(0.07757031*x);
    end
function [v]= Solve(x,t,D)
        lam=0.07757031;
        v=exp(-D*t*lam^2)*cos(lam*x);
        end
        
    

