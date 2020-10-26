function [x_flow, y_flow] = extract_flowpipes(time_step, sampling_time, model)
    x_flow = [];
    y_flow = [];
%     line_num = 0;

    line_num = 1;
    if strcmp(model, 'linear')
        fid = fopen('linear_model/outputs/RC_bicycle.m');
    else
        fid = fopen('nonlinear_model/outputs/RC_bicycle.m');
    end
    while ~feof(fid)
        tline = fgetl(fid);
        left = strfind(tline, '[');
        right = strfind(tline, ']');
        if size(left,2) > 0
            %line_num = line_num+1;
            if mod(line_num, sampling_time/time_step)==0 
                x_string = tline(left(1,1)+1:right(1,1)-1);
                x_cell = strsplit(x_string, ',');
                %x_m = cell2mat(x)
                y_string = tline(left(1,2)+1:right(1,2)-1);
                y_cell = strsplit(y_string, ',');
                x_flow_single = zeros(5, 1);
                y_flow_single = zeros(5, 1);
                for i = 1:5
                    x_single_flow(i, 1) = str2num(x_cell{i});
                    y_single_flow(i, 1) = str2num(y_cell{i});
                end
                x_flow = [x_flow, x_single_flow];
                y_flow = [y_flow, y_single_flow];
            end
            line_num = line_num+1;
        end
    end
    fclose(fid);
end
