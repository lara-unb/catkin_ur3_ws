function [time, input_pose,  input_vel, output_pose, output_vel, output_effort] = export_rosbags(file_name)

    path = strcat('/home/ur3/catkin_ur3_ws/bags/',file_name, '.bag');
    bag = rosbag(path);
    
    output_sel = select(bag,'Topic','/ur3/arm');
    input_sel_pose = select(bag,'Topic','/ur3/ref_pos');
    input_sel_vel = select(bag,'Topic','/ur3/ref_vel');

    outpu_structs = readMessages(output_sel,'DataFormat','struct');
    
    input_pose_structs = readMessages(input_sel_pose,'DataFormat','struct');
    
    input_vel_structs = readMessages(input_sel_vel,'DataFormat','struct');
    
    time = zeros(output_sel.NumMessages,1);
    
    input_pose = zeros(input_sel_pose.NumMessages,6);
    input_vel = zeros(input_sel_pose.NumMessages,6);
    
    output_pose = zeros(input_sel_pose.NumMessages,6);
    output_vel = zeros(input_sel_pose.NumMessages,6);
    output_effort = zeros(input_sel_pose.NumMessages,6);
    
    for idx = 1:input_sel_pose.NumMessages
        
        time(idx) = output_sel.MessageList.Time(idx) - output_sel.StartTime;
        
        output_pose(idx,:) = outpu_structs{idx}.Position';
        output_vel(idx,:) = outpu_structs{idx}.Velocity';
        output_effort(idx,:) = outpu_structs{idx}.Effort'; 
        
        input_vel(idx,:) = input_vel_structs{idx}.Data';
        
        input_pose(idx,:) = input_pose_structs{idx}.Data';
       
    end
    
end
