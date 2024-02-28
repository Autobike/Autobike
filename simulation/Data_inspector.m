
%%%%%%%%%%%% DATA INSPECTOR %%%%%%%%%%%%%%%
% In order to run and select data, Main_sim needs to be ran manually after
% selecting the data.
clc
clear
dataInspector
function dataInspector
fig = uifigure('Name','Data Inspector','WindowState','maximized');
g = uigridlayout(fig, [6 4]);
g.RowHeight = {'fit','1x','1x','1x','1x','fit'};
g.ColumnWidth = {'1x'};


ax1 = uiaxes(g);
ax1.Layout.Row = [2 5];
ax1.Layout.Column = [1 2];

ax2 = uiaxes(g);
ax2.Layout.Row = [2 3];
ax2.Layout.Column = [3 4];

ax3 = uiaxes(g);
ax3.Layout.Row = [4 5];
ax3.Layout.Column = [3 4];

sld = uislider(g);
sld.Layout.Row = 6;
sld.Layout.Column = [1 2];
sld.Limits = [0 1];


dd = uidropdown(g);
dd.Items = {'Select test session'};
dd.Layout.Row = 1;
dd.Layout.Column = [1 3];
dd.ValueChangedFcn = @(src,event) ChangePlots(ax1,ax2,ax3,sld,src);

sld.ValueChangingFcn = @(src,event) UpdateSld(event,dd);

b = uibutton(g, ...
    'Text','Open', ...
    'ButtonPushedFcn', @(src,event) pressButtonActivaton(dd));
b.Layout.Row = 1;
b.Layout.Column = 4;

b2 = uibutton(g, ...
    'Text', 'Simulation', ...
    'ButtonPushedFcn', @(src,event) SimButtonActivation(dd));
b2.Layout.Row = 6;
b2.Layout.Column = 3;

b3 = uibutton(g, ...
    'Text','Section', ...
    'ButtonPushedFcn', @(src,event) SectionButtonActivation(ax1,ax2,ax3,sld,dd));
b3.Layout.Row = 6;
b3.Layout.Column = 4;

end

function SimButtonActivation(dd)
    [file,path] = uigetfile('*.csv','Choose trajectory file','Traj_ref_test\');
    traj = readtable([path,file]);
    assignin('base','traj',traj);
    init = 1;
    assignin('base','init',init)
    Data_sim = readtable(dd.Value);
    assignin('base','Data_sim',Data_sim);
    %Main_sim           Note: Can't run directly from this for some reason.
    %                         Just run Main_sim from the file directly.
end

function SectionButtonActivation(ax1,ax2,ax3,sld,dd)
    index = evalin('base','index');
    Data = evalin('base','Data');
    cla(ax1);
    cla(ax2);
    cla(ax3);
    [r,~] = size(Data);
   
    if index < r/20
        low = 1;
    else
        low = index-r/20;
    end
    if r/20 > r-index
        high = r;
    else
        high = index+r/20;
    end
    Data = Data(round(low):round(high),:);


    plot(ax1,Data.StateEstimateX_m_,Data.StateEstimateY_m_);
    hold(ax1,'on');
    point = plot(ax1,Data.StateEstimateX_m_(1),Data.StateEstimateY_m_(1),'o','MarkerFaceColor','red');
    %plot(ax1,traj.Var2,traj.Var1);
    hold(ax2,"on");
    hold(ax3,"on");
    legend(ax1,'Est. Pos')
    
    sld.Limits = [1 numel(Data.StateEstimateX_m_)];
    sld.MajorTicks = [1 numel(Data.StateEstimateX_m_)/2 numel(Data.StateEstimateX_m_)];
    sld.MajorTickLabels = {'start','','end'};
    sld.MinorTicks = [];
    sld.Value = 1;

    plot(ax2,Data.Error1);
    point2 = plot(ax2,Data.Error1(1),'o','MarkerFaceColor','red');
    plot(ax3,Data.Error2);
    lineE1_1 = xline(ax2,point2.XData+numel(Data.Error1)/20,'--r');
    lineE1_2 = xline(ax2,point2.XData,'--r');
    point3 = plot(ax3,Data.Error2(1),'o','MarkerFaceColor','red');
    lineE2_1 = xline(ax3,point3.XData+numel(Data.Error2)/20,'--r');
    lineE2_2 = xline(ax3,point3.XData,'--r');
    legend(ax2,'Error1')
    legend(ax3,'Error2')
    assignin('base','point',point);
    assignin('base','point2',point2);
    assignin('base','point3',point3);
    assignin('base','lineE1_1',lineE1_1);
    assignin('base','lineE1_2',lineE1_2);
    assignin('base','lineE2_1',lineE2_1);
    assignin('base','lineE2_2',lineE2_2);
    assignin('base','Data',Data);
%     plot(ax1,X,Y);


end

function UpdateSld(event,dd)
    Data = evalin('base','Data');
    point = evalin('base','point');
    point2 = evalin('base','point2');
    point3 = evalin('base','point3');
    lineE1_1 = evalin('base','lineE1_1');
    lineE1_2 = evalin('base','lineE1_2');
    lineE2_1 = evalin('base','lineE2_1');
    lineE2_2 = evalin('base','lineE2_2');
    point.XData = Data.StateEstimateX_m_(round(event.Value));
    point.YData = Data.StateEstimateY_m_(round(event.Value));
    point2.YData = Data.Error1(round(event.Value));
    point2.XData = round(event.Value);
    point3.YData = Data.Error2(round(event.Value));
    point3.XData = round(event.Value);
    lineE1_1.Value = point2.XData+numel(Data.Error1)/20;
    if point2.XData > numel(Data.Error1)/20
        lineE1_2.Value = point2.XData-numel(Data.Error1)/20;
    end
    lineE2_1.Value = point3.XData+numel(Data.Error2)/20;
    if point3.XData > numel(Data.Error2)/20
        lineE2_2.Value = point3.XData-numel(Data.Error2)/20;
    end
    drawnow
    assignin('base','point',point);
    assignin('base','point2',point2);
    assignin('base','point3',point3);
    assignin('base','index',round(event.Value));
end

function ChangePlots(ax1,ax2,ax3,sld,src)
    cla(ax1);
    cla(ax2);
    cla(ax3);
    Data = readtable(src.Value);
    reset_traj = find(Data.ResetTraj==1,1,'last');
    Data(1:reset_traj,:) = [];


    plot(ax1,Data.StateEstimateX_m_,Data.StateEstimateY_m_);
    hold(ax1,'on');
    point = plot(ax1,Data.StateEstimateX_m_(1),Data.StateEstimateY_m_(1),'o','MarkerFaceColor','red');
    hold(ax2,"on");
    hold(ax3,"on");
    legend(ax1,'Est. Pos')
    
    sld.Limits = [1 numel(Data.StateEstimateX_m_)];
    sld.MajorTicks = [1 numel(Data.StateEstimateX_m_)/2 numel(Data.StateEstimateX_m_)];
    sld.MajorTickLabels = {'start','','end'};
    sld.MinorTicks = [];
    sld.Value = 1;

    plot(ax2,Data.Error1);
    point2 = plot(ax2,Data.Error1(1),'o','MarkerFaceColor','red');
    plot(ax3,Data.Error2);
    lineE1_1 = xline(ax2,point2.XData+numel(Data.Error1)/20,'--r');
    lineE1_2 = xline(ax2,point2.XData,'--r');
    point3 = plot(ax3,Data.Error2(1),'o','MarkerFaceColor','red');
    lineE2_1 = xline(ax3,point3.XData+numel(Data.Error2)/20,'--r');
    lineE2_2 = xline(ax3,point3.XData,'--r');
    legend(ax2,'Error1')
    legend(ax3,'Error2')
    assignin('base','point',point);
    assignin('base','point2',point2);
    assignin('base','point3',point3);
    assignin('base','lineE1_1',lineE1_1);
    assignin('base','lineE1_2',lineE1_2);
    assignin('base','lineE2_1',lineE2_1);
    assignin('base','lineE2_2',lineE2_2);
    assignin('base','Data',Data);

end

function pressButtonActivaton(dd)
    [file, path] = uigetfile('*.csv','MultiSelect','on');
    [p,~] = size(file);
    if p < 2
        dd.Items = [dd.Items [path,file]];
    else
        for i = 1:p     
            dd.Items = [dd.Items [path,file{i}]];
        end
    end

end