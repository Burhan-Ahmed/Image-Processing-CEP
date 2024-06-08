function varargout = Page2(varargin)
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

% Last Modified by GUIDE v2.5 25-May-2024 19:02:07

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Page2_OpeningFcn, ...
                   'gui_OutputFcn',  @Page2_OutputFcn, ...
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
function Page2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Page2 (see VARARGIN)

% Choose default command line output for Page2
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Page2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Page2_OutputFcn(hObject, eventdata, handles) 
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
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1


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


% --- Executes on button press in button2.
function button2_Callback(hObject, eventdata, handles)
% hObject    handle to button2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
a=getappdata(0,'a');
if size(a, 3) == 3
  img_gray = rgb2gray(a);
else
  img_gray = a;
end
val=str2num(get(handles.Threshinput,'string'));
% Apply simple thresholding 
simple_threshold = val; 
binary_img_simple = img_gray > simple_threshold;

[rows, cols] = size(img_gray);

histogram = zeros(256, 1);
for i = 1:rows
  for j = 1:cols
    pixel_value = img_gray(i, j);
    histogram(pixel_value + 1) = histogram(pixel_value + 1) + 1;
  end
end

total_pixels = sum(histogram);

probability = histogram / total_pixels;

mean_intensity = 0;
for i = 1:256
  mean_intensity = mean_intensity + i * probability(i);
end

max_variance = 0;
optimal_threshold = 0;

for threshold = 1:255
  weight_background = sum(probability(1:threshold));
  weight_foreground = sum(probability(threshold + 1:end));
  
  if weight_background == 0 || weight_foreground == 0
    continue;
  end
  
  mean_background = 0;
  for i = 1:threshold
    mean_background = mean_background + i * probability(i);
  end
  mean_background = mean_background / weight_background;
  
  mean_foreground = 0;
  for i = threshold + 1:256
    mean_foreground = mean_foreground + i * probability(i);
  end
  mean_foreground = mean_foreground / weight_foreground;
  
  variance = weight_background * weight_foreground * (mean_background - mean_foreground).^2;
  
  if variance > max_variance
    max_variance = variance;
    optimal_threshold = threshold;
  end
end

% Apply Otsu's threshold
binary_img_otsu = img_gray > optimal_threshold;

setappdata(0,'filename',binary_img_simple)
axes(handles.axes2);
imshow(binary_img_simple);
title('Simple Threshold');

setappdata(0,'filename',binary_img_otsu)
axes(handles.axes5);
imshow(binary_img_otsu);
title('Otsu''s Threshold (Custom)');


% --- Executes on button press in button3.
function button3_Callback(hObject, eventdata, handles)
% hObject    handle to button3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

a=getappdata(0,'a');

if size(a, 3) == 3
  img_gray = rgb2gray(a);
else
  img_gray = a;
end

seed_x=str2double(get(handles.seedx,'string'));
seed_y=str2double(get(handles.seedy,'string'));
reg_maxdist=str2double(get(handles.regmax,'string'));

% Initialize binary mask (all zeros)
    mask = false(size(img_gray));
    mask(seed_y, seed_x) = 1;
    
    % neighborhood offsets (8-connectivity)
    dx = [-1, -1, 0, 1, 1, 1, 0, -1];
    dy = [0, 1, 1, 1, 0, -1, -1, -1];
   
    queue = [seed_x, seed_y];
    
    while ~isempty(queue)
        % Get current pixel coordinates
        current_x = queue(1);
        current_y = queue(2);
        queue(1:2) = [];
        
        for k = 1:8
            neighbor_x = current_x + dx(k);
            neighbor_y = current_y + dy(k);
            
            % Check if neighbor is within image bounds
            if neighbor_x >= 1 && neighbor_x <= size(img_gray, 2) && ...
               neighbor_y >= 1 && neighbor_y <= size(img_gray, 1)
                
                % Calculate intensity difference
                intensity_diff = abs(double(img_gray(current_y, current_x)) - double(img_gray(neighbor_y, neighbor_x)));
                
                % Add neighbor to region if within similarity threshold
                if intensity_diff <= reg_maxdist && ~mask(neighbor_y, neighbor_x)
                    mask(neighbor_y, neighbor_x) = 1;
                    queue(end+1:end+2) = [neighbor_x, neighbor_y]; % Enqueue
                end
            end
        end
    end
    
    segmented_image = uint8(mask) * 255;

setappdata(0,'filename',segmented_image);
axes(handles.axes4);
imshow(segmented_image);
 

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

% Apply Sobel edge detection
sobelEdgeImage = zeros(size(grayImage));
sobelThreshold = 0.1; 

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
        axes(handles.axes1);
        imshow(a);
        title('Original Image');
    end

    % Get the image from appdata
    a = getappdata(0, 'a');
    if isempty(a)
        errordlg('No image found. Please load an image first.', 'Image Error');
        return;
    end




function button4_Callback(hObject, eventdata, handles)
% hObject    handle to button4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%a=getappdata(0,'a');
%if size(a, 3) == 3
%  grayImage = rgb2gray(a);
%else
%  grayImage = a;
%end
%regionSize=str2double(get(handles.region,'string'));

%threshold = graythresh(grayImage); 
%binaryImage = imbinarize(grayImage, threshold);

%binaryImage = bwareaopen(binaryImage, regionSize);

%distanceTransform = bwdist(~binaryImage);

%watershedMask = watershed(-distanceTransform);

%segmentedImg = a;
%segmentedImg(watershedMask == 0) = 0;

%setappdata(0,'filename', segmentedImg);
%axes(handles.axes3);
%imshow(segmentedImg);


% --- Executes on key press with focus on button1 and none of its controls.
function button1_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to button1 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)



function Threshinput_Callback(hObject, eventdata, handles)
% hObject    handle to Threshinput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Threshinput as text
%        str2double(get(hObject,'String')) returns contents of Threshinput as a double


% --- Executes during object creation, after setting all properties.
function Threshinput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Threshinput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function seedx_Callback(hObject, eventdata, handles)
% hObject    handle to seedx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of seedx as text
%        str2double(get(hObject,'String')) returns contents of seedx as a double


% --- Executes during object creation, after setting all properties.
function seedx_CreateFcn(hObject, eventdata, handles)
% hObject    handle to seedx (see GCBO)
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
  openfig('Page3.fig');


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on key press with focus on pushbutton10 and none of its controls.
function pushbutton10_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in next.
function next_Callback(hObject, eventdata, handles)
% hObject    handle to next (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Close the current figure
close(gcf); 
% Open page3.fig
Page3
