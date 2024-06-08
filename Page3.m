function varargout = Page3(varargin)
% PAGE2 MATLAB code for Page2.fig
%      PAGE2, by itself, creates a new PAGE2 or raises the existing
%      singleton*.
%
%      H = PAGE2 returns the handle to a new PAGE2 or the handle to
%      the existing singleton*.
%
%      PAGE2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PAGE2.M with the given input arguments.
%
%      PAGE2('Property','Value',...) creates a new PAGE2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Page2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Page2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Page2

% Last Modified by GUIDE v2.5 28-May-2024 01:05:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Page3_OpeningFcn, ...
                   'gui_OutputFcn',  @Page3_OutputFcn, ...
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


% --- Executes just before Page2 is made visible.
function Page3_OpeningFcn(hObject, eventdata, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to Page3 (see VARARGIN)

    % Choose default command line output for Page3
    handles.output = hObject;

    % Check if 'region' control is properly created
    if ~isfield(handles, 'region')
        handles.region = uicontrol('Style', 'edit', 'String', '', 'Tag', 'region', 'Position', [100, 100, 100, 30]);
    end

    % Update handles structure
    guidata(hObject, handles);

    % UIWAIT makes Page3 wait for user response (see UIRESUME)
    % uiwait(handles.figure1);

% UIWAIT makes Page2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Page3_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double

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

% --- Executes during object creation, after setting all properties.
function axes3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes3


% --- Executes on slider movement.
function slider_Callback(hObject, eventdata, handles)
% hObject    handle to slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% --- Executes during object creation, after setting all properties.
function slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


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
 

% --- Executes on button press in button5.
function button5_Callback(hObject, eventdata, handles)
% hObject    handle to button5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
a=getappdata(0,'a');
if size(a, 3) == 3
  grayImage = rgb2gray(a);
else
  grayImage = a;
end

% Apply tag edge detection
sobelEdgeImage = zeros(size(grayImage));
sobelThreshold = str2double(get(handles.sobel,'string'));

for i = 2:size(grayImage, 1)-1
    for j = 2:size(grayImage, 2)-1
     
        Gx = grayImage(i-1,j-1) + 2*grayImage(i,j-1) + grayImage(i+1,j-1) ...
            - grayImage(i-1,j+1) - 2*grayImage(i,j+1) - grayImage(i+1,j+1);

        Gy = grayImage(i-1,j-1) + 2*grayImage(i-1,j) + grayImage(i-1,j+1) ...
            - grayImage(i+1,j-1) - 2*grayImage(i+1,j) - grayImage(i+1,j+1);

        sobelEdgeImage(i,j) = sqrt(double(Gx)^2 + double(Gy)^2); 
    end
end

sobelEdgeImage = sobelEdgeImage > sobelThreshold;

segmentedImage = imfill(sobelEdgeImage, 'holes'); % Fill holes in edges
segmentedImage = imclearborder(segmentedImage); % Clear border objects

setappdata(0,'filename',segmentedImage);
axes(handles.axes6);
imshow(segmentedImage);


% --- Executes on button press in button1.

% --- Executes on button press in button1.
function button1_Callback(hObject, eventdata, handles)
% hObject    handle to button1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    [filename, pathname] = uigetfile({'*.png;*.jpg;*.jpeg;*.bmp','Image Files (*.png, *.jpg, *.jpeg, *.bmp)'}, 'Select an Image File');
    if isequal(filename,0)
        disp('User selected Cancel');
        return;
    else
        fullpath = fullfile(pathname, filename);
        a = imread(fullpath);
        setappdata(0, 'a', a);
        axes(handles.axes2);
        imshow(a);
        title('Original Image');
    end

    % Get the image from appdata
    a = getappdata(0, 'a');
    if isempty(a)
        errordlg('No image found. Please load an image first.', 'Image Error');
        return;
    end


% --- Executes on button press in button4.
function button4_Callback(hObject, eventdata, handles)
% hObject    handle to button4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

a=getappdata(0,'a');
if size(a, 3) == 3
  grayImage = rgb2gray(a);
else
  grayImage = a;
end
regionSize=str2double(get(handles.region,'string'));

threshold = graythresh(grayImage); 
binaryImage = imbinarize(grayImage, threshold);

binaryImage = bwareaopen(binaryImage, regionSize);

distanceTransform = bwdist(~binaryImage);

watershedMask = watershed(-distanceTransform);

segmentedImg = a;
segmentedImg(watershedMask == 0) = 0;

setappdata(0,'filename', segmentedImg);
axes(handles.axes3);
imshow(segmentedImg);


% --- Executes on key press with focus on button1 and none of its controls.
function button1_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to button1 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)



function region_Callback(hObject, eventdata, handles)
% hObject    handle to region (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of region as text
%        str2double(get(hObject,'String')) returns contents of region as a double


% --- Executes during object creation, after setting all properties.
function region_CreateFcn(hObject, eventdata, handles)
% hObject    handle to region (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function sobel_Callback(hObject, eventdata, handles)
% hObject    handle to tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tag as text
%        str2double(get(hObject,'String')) returns contents of tag as a double


% --- Executes during object creation, after setting all properties.
function tag_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function seedy_Callback(hObject, eventdata, handles)
% hObject    handle to seedy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of seedy as text
%        str2double(get(hObject,'String')) returns contents of seedy as a double


% --- Executes during object creation, after setting all properties.
function seedy_CreateFcn(hObject, eventdata, handles)
% hObject    handle to seedy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function regmax_Callback(hObject, eventdata, handles)
% hObject    handle to regmax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of regmax as text
%        str2double(get(hObject,'String')) returns contents of regmax as a double


% --- Executes during object creation, after setting all properties.
function regmax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to regmax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in nxt.
function nxt_Callback(hObject, eventdata, handles)
% hObject    handle to nxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in back.
function back_Callback(hObject, eventdata, handles)
% hObject    handle to back (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close(gcf);


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Close the current figure
close(gcf); 

% Open page3.fig
open('page2.fig');


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    [filename, pathname] = uigetfile({'*.png;*.jpg;*.jpeg;*.bmp','Image Files (*.png, *.jpg, *.jpeg, *.bmp)'}, 'Select an Image File');
    if isequal(filename,0)
        disp('User selected Cancel');
        return;
    else
        fullpath = fullfile(pathname, filename);
        a = imread(fullpath);
        setappdata(0, 'a', a);
        axes(handles.axes8);
        imshow(a);
        title('Original Image');
    end

    % Get the image from appdata
    a = getappdata(0, 'a');
    if isempty(a)
        errordlg('No image found. Please load an image first.', 'Image Error');
        return;
    end