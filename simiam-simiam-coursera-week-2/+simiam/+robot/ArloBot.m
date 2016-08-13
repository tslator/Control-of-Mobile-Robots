classdef ArloBot < simiam.robot.Robot

% Copyright (C) 2013, Georgia Tech Research Corporation
% see the LICENSE file included with this software
    
    properties
        wheel_radius
        wheel_base_length
        ticks_per_rev
        speed_factor
        
        encoders = simiam.robot.sensor.WheelEncoder.empty(1,0);
        ir_array = simiam.robot.sensor.ProximitySensor.empty(1,0);
        
        dynamics
    end
    
    properties (SetAccess = private)
        right_wheel_speed
        left_wheel_speed
    end
    
    methods
        function obj = ArloBot(parent, pose)
           obj = obj@simiam.robot.Robot(parent, pose);
           
           % Add surfaces: ArloBot in top-down 2D view
           
           ab_base_plate = [
                    %    x       y      ?
                     0        0.22545   1;
                     0.08628  0.20829   1;
                     0.15942  0.15942   1;
                     0.20829  0.08628   1;
                     0.22545  0         1;
                     0.20829 -0.08628   1;
                     0.15942 -0.15942   1;
                     0.08628 -0.20829   1;
                     0       -0.22545   1;
                    -0.08628 -0.20829   1;
                    -0.15942 -0.15942   1;
                    -0.20829 -0.08628   1;
                    -0.22545  0         1;
                    -0.20829  0.08628   1;
                    -0.15942  0.15942   1;
                    -0.08628  0.20829   1;
                     0        0.22545   1;
           
                    ];
                
            qb_bbb =  [ -0.0914 -0.0406 1;
                        -0.0944 -0.0376 1;
                        -0.0944  0.0376 1;
                        -0.0914  0.0406 1;
                        -0.0429  0.0406 1;
                        -0.0399  0.0376 1;
                        -0.0399 -0.0376 1;
                        -0.0429 -0.0406 1];
                    
            qb_bbb_rail_l = [ -0.0429 -0.0356 1;
                              -0.0429  0.0233 1;
                              -0.0479  0.0233 1;
                              -0.0479 -0.0356 1;];
                          
            qb_bbb_rail_r = [ -0.0914 -0.0356 1;
                              -0.0914  0.0233 1;
                              -0.0864  0.0233 1;
                              -0.0864 -0.0356 1;
                              ];
                          
            qb_bbb_eth = [ -0.0579 0.0436 1;
                           -0.0579 0.0226 1;
                           -0.0739 0.0226 1;
                           -0.0739 0.0436 1;];
                       
            qb_left_wheel = [ 0.0775 0.2015    1;
                               0.0775 0.1715    1;
                              -0.0775 0.1715    1;
                              -0.0775 0.2015    1;  ];
                          
            qb_left_wheel_ol = [ 0.0775 0.2015    1;
                               0.0775 0.1715    1;
                              -0.0775 0.1715    1;
                              -0.0775 0.2015    1;  ];
            
            qb_right_wheel_ol = [ 0.0775 -0.2015    1;
                               0.0775 -0.1715    1;
                              -0.0775 -0.1715    1;
                              -0.0775 -0.2015    1;  ];
                         
            qb_right_wheel = [ 0.0775 -0.2015    1;
                               0.0775 -0.1715    1;
                              -0.0775 -0.1715    1;
                              -0.0775 -0.2015    1;  ];
                          
            % There are five IR sensors on the front and first IR sensors
            % on the back.  They are placed at the following angles:
            %
            %   -60, -30, 0, 30, 60
            pos_60 = [cos(deg2rad(60)) -sin(deg2rad(60)); sin(deg2rad(60)) cos(deg2rad(60));];
            pos_30 = [cos(deg2rad(30)) -sin(deg2rad(30)); sin(deg2rad(30)) cos(deg2rad(30));];
            zero_0 = [cos(deg2rad(0)) -sin(deg2rad(0)); sin(deg2rad(0)) cos(deg2rad(0));];
            neg_30 = [cos(deg2rad(-30)) -sin(deg2rad(-30)); sin(deg2rad(-30)) cos(deg2rad(-30));];
            neg_60 = [cos(deg2rad(-60)) -sin(deg2rad(-60)); sin(deg2rad(-60)) cos(deg2rad(-60));];
            
            platter_radius = 0.22545;
            front_ir_pt1 = [(platter_radius + 0.01/2) 0.03/2]';
            front_ir_pt2 = [(platter_radius + 0.01/2) -0.03/2]';
            front_ir_pt3 = [(platter_radius + -0.01/2) -0.03/2]';
            front_ir_pt4 = [(platter_radius + -0.01/2) 0.03/2]';
            
            pt1 = pos_60 * front_ir_pt1;
            pt2 = pos_60 * front_ir_pt2;
            pt3 = pos_60 * front_ir_pt3;
            pt4 = pos_60 * front_ir_pt4;
            qb_ir_1 = [ pt1(1) pt1(2)  1;
                        pt2(1) pt2(2)  1;
                        pt3(1) pt3(2)  1;
                        pt4(1) pt4(2)  1];
                    
            pt1 = pos_30 * front_ir_pt1;
            pt2 = pos_30 * front_ir_pt2;
            pt3 = pos_30 * front_ir_pt3;
            pt4 = pos_30 * front_ir_pt4;
            qb_ir_2 = [ pt1(1) pt1(2)  1;
                        pt2(1) pt2(2)  1;
                        pt3(1) pt3(2)  1;
                        pt4(1) pt4(2)  1];

            pt1 = zero_0 * front_ir_pt1;
            pt2 = zero_0 * front_ir_pt2;
            pt3 = zero_0 * front_ir_pt3;
            pt4 = zero_0 * front_ir_pt4;
            qb_ir_3 = [ pt1(1) pt1(2)  1;
                        pt2(1) pt2(2)  1;
                        pt3(1) pt3(2)  1;
                        pt4(1) pt4(2)  1
                        ];
                    
            pt1 = neg_30 * front_ir_pt1;
            pt2 = neg_30 * front_ir_pt2;
            pt3 = neg_30 * front_ir_pt3;
            pt4 = neg_30 * front_ir_pt4;
            qb_ir_4 = [ pt1(1) pt1(2)  1;
                        pt2(1) pt2(2)  1;
                        pt3(1) pt3(2)  1;
                        pt4(1) pt4(2)  1];
                    
            pt1 = neg_60 * front_ir_pt1;
            pt2 = neg_60 * front_ir_pt2;
            pt3 = neg_60 * front_ir_pt3;
            pt4 = neg_60 * front_ir_pt4;
            qb_ir_5 = [ pt1(1) pt1(2)  1;
                        pt2(1) pt2(2)  1;
                        pt3(1) pt3(2)  1;
                        pt4(1) pt4(2)  1];                    

            rear_ir_pt1 = [(-platter_radius + 0.01/2) 0.03/2]';
            rear_ir_pt2 = [(-platter_radius + 0.01/2) -0.03/2]';
            rear_ir_pt3 = [(-platter_radius + -0.01/2) -0.03/2]';
            rear_ir_pt4 = [(-platter_radius + -0.01/2) 0.03/2]';
            
            pt1 = pos_60 * rear_ir_pt1;
            pt2 = pos_60 * rear_ir_pt2;
            pt3 = pos_60 * rear_ir_pt3;
            pt4 = pos_60 * rear_ir_pt4;
            qb_ir_6 = [ pt1(1) pt1(2)  1;
                        pt2(1) pt2(2)  1;
                        pt3(1) pt3(2)  1;
                        pt4(1) pt4(2)  1];
                    
            pt1 = pos_30 * rear_ir_pt1;
            pt2 = pos_30 * rear_ir_pt2;
            pt3 = pos_30 * rear_ir_pt3;
            pt4 = pos_30 * rear_ir_pt4;
            qb_ir_7 = [ pt1(1) pt1(2)  1;
                        pt2(1) pt2(2)  1;
                        pt3(1) pt3(2)  1;
                        pt4(1) pt4(2)  1];

            pt1 = zero_0 * rear_ir_pt1;
            pt2 = zero_0 * rear_ir_pt2;
            pt3 = zero_0 * rear_ir_pt3;
            pt4 = zero_0 * rear_ir_pt4;
            qb_ir_8 = [ pt1(1) pt1(2)  1;
                        pt2(1) pt2(2)  1;
                        pt3(1) pt3(2)  1;
                        pt4(1) pt4(2)  1
                        ];
                    
            pt1 = neg_30 * rear_ir_pt1;
            pt2 = neg_30 * rear_ir_pt2;
            pt3 = neg_30 * rear_ir_pt3;
            pt4 = neg_30 * rear_ir_pt4;
            qb_ir_9 = [ pt1(1) pt1(2)  1;
                        pt2(1) pt2(2)  1;
                        pt3(1) pt3(2)  1;
                        pt4(1) pt4(2)  1];
                    
            pt1 = neg_60 * rear_ir_pt1;
            pt2 = neg_60 * rear_ir_pt2;
            pt3 = neg_60 * rear_ir_pt3;
            pt4 = neg_60 * rear_ir_pt4;
            qb_ir_10 = [ pt1(1) pt1(2)  1;
                        pt2(1) pt2(2)  1;
                        pt3(1) pt3(2)  1;
                        pt4(1) pt4(2)  1];                    
                    
            qb_bbb_usb = [ -0.0824 -0.0418 1;
                           -0.0694 -0.0418 1;
                           -0.0694 -0.0278 1;
                           -0.0824 -0.0278 1];
                        
                       
            obj.add_surface_with_depth(ab_base_plate, [ 226 0 2 ]/255, 1.3);
            obj.add_surface(qb_right_wheel, [ 0.15 0.15 0.15 ]);
            s = obj.add_surface_with_depth(qb_right_wheel_ol, [0.15 0.15 0.15], 1.5);
            
            set(s.handle_, 'LineStyle', '--');
            set(s.handle_, 'FaceColor', 'none');
            
            s = obj.add_surface_with_depth(qb_left_wheel_ol, [0.15 0.15 0.15], 1.5);
            
            set(s.handle_, 'LineStyle', '--');
            set(s.handle_, 'FaceColor', 'none');
%             obj.add_surface_with_alpha(qb_axle, [0.15 0.15 0.15], 0.5);
            obj.add_surface(qb_left_wheel, [ 0.15 0.15 0.15 ]);
            
            obj.add_surface_with_depth(qb_ir_1, [0.1 0.1 0.1], 1.2);
            obj.add_surface_with_depth(qb_ir_2, [0.1 0.1 0.1], 1.2);
            obj.add_surface_with_depth(qb_ir_3, [0.1 0.1 0.1], 1.2);
            obj.add_surface_with_depth(qb_ir_4, [0.1 0.1 0.1], 1.2);
            obj.add_surface_with_depth(qb_ir_5, [0.1 0.1 0.1], 1.2);
            obj.add_surface_with_depth(qb_ir_6, [0.1 0.1 0.1], 1.2);
            obj.add_surface_with_depth(qb_ir_7, [0.1 0.1 0.1], 1.2);
            obj.add_surface_with_depth(qb_ir_8, [0.1 0.1 0.1], 1.2);
            obj.add_surface_with_depth(qb_ir_9, [0.1 0.1 0.1], 1.2);
            obj.add_surface_with_depth(qb_ir_10, [0.1 0.1 0.1], 1.2);
                       
                       
            obj.add_surface_with_depth(qb_bbb, [0.2 0.2 0.2], 1.4);
            obj.add_surface_with_depth(qb_bbb_rail_l, [0 0 0], 1.5);
            obj.add_surface_with_depth(qb_bbb_rail_r, [0 0 0], 1.5);
            obj.add_surface_with_depth(qb_bbb_eth, [0.7 0.7 0.7], 1.5);
            obj.add_surface_with_depth(qb_bbb_usb, [0.7 0.7 0.7], 1.5);
            
            
            

            
            % Add sensors: wheel encoders and IR proximity sensors
            %obj.wheel_radius = 0.0325;           % 65.0mm in diameter
            obj.wheel_radius = 0.0775;
            %obj.wheel_base_length = 0.09925;     % 99.25mm
            obj.wheel_base_length = 0.4030;
            %obj.ticks_per_rev = 16;
            obj.ticks_per_rev = 144;
            obj.speed_factor = 0;
            
            obj.encoders(1) = simiam.robot.sensor.WheelEncoder('right_wheel', obj.wheel_radius, obj.wheel_base_length, obj.ticks_per_rev);
            obj.encoders(2) = simiam.robot.sensor.WheelEncoder('left_wheel', obj.wheel_radius, obj.wheel_base_length, obj.ticks_per_rev);
            
            import simiam.robot.sensor.ProximitySensor;
            import simiam.robot.QuickBot;
            import simiam.ui.Pose2D;
                        
            ir_pose = Pose2D(platter_radius*cos(deg2rad(60)), platter_radius*sin(deg2rad(60)), Pose2D.deg2rad(60));
            obj.ir_array(1) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.04, 0.3, Pose2D.deg2rad(6), 'simiam.robot.QuickBot.ir_distance_to_raw');
            
            ir_pose = Pose2D(platter_radius*cos(deg2rad(30)), platter_radius*sin(deg2rad(30)), Pose2D.deg2rad(30));
            obj.ir_array(2) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.04, 0.3, Pose2D.deg2rad(6), 'simiam.robot.QuickBot.ir_distance_to_raw');
            
            ir_pose = Pose2D(platter_radius*cos(deg2rad(0)), platter_radius*sin(deg2rad(0)), Pose2D.deg2rad(0));
            obj.ir_array(3) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.04, 0.3, Pose2D.deg2rad(6), 'simiam.robot.QuickBot.ir_distance_to_raw');
            
            ir_pose = Pose2D(platter_radius*cos(deg2rad(-30)), platter_radius*sin(deg2rad(-30)), Pose2D.deg2rad(-30));
            obj.ir_array(4) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.04, 0.3, Pose2D.deg2rad(6), 'simiam.robot.QuickBot.ir_distance_to_raw');
            
            ir_pose = Pose2D(platter_radius*cos(deg2rad(-60)), platter_radius*sin(deg2rad(-60)), Pose2D.deg2rad(-60));
            obj.ir_array(5) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.04, 0.3, Pose2D.deg2rad(6), 'simiam.robot.QuickBot.ir_distance_to_raw');
            
            ir_pose = Pose2D(platter_radius*cos(deg2rad(60+180)), platter_radius*sin(deg2rad(60+180)), Pose2D.deg2rad(60+180));
            obj.ir_array(6) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.04, 0.3, Pose2D.deg2rad(6), 'simiam.robot.QuickBot.ir_distance_to_raw');
            
            ir_pose = Pose2D(platter_radius*cos(deg2rad(30+180)), platter_radius*sin(deg2rad(30+180)), Pose2D.deg2rad(30+180));
            obj.ir_array(7) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.04, 0.3, Pose2D.deg2rad(6), 'simiam.robot.QuickBot.ir_distance_to_raw');
            
            ir_pose = Pose2D(platter_radius*cos(deg2rad(0+180)), platter_radius*sin(deg2rad(0+180)), Pose2D.deg2rad(0+180));
            obj.ir_array(8) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.04, 0.3, Pose2D.deg2rad(6), 'simiam.robot.QuickBot.ir_distance_to_raw');
            
            ir_pose = Pose2D(platter_radius*cos(deg2rad(-30+180)), platter_radius*sin(deg2rad(-30+180)), Pose2D.deg2rad(-30+180));
            obj.ir_array(9) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.04, 0.3, Pose2D.deg2rad(6), 'simiam.robot.QuickBot.ir_distance_to_raw');
            
            ir_pose = Pose2D(platter_radius*cos(deg2rad(-60+180)), platter_radius*sin(deg2rad(-60+180)), Pose2D.deg2rad(-60+180));
            obj.ir_array(10) = ProximitySensor(parent, 'IR', pose, ir_pose, 0.04, 0.3, Pose2D.deg2rad(6), 'simiam.robot.QuickBot.ir_distance_to_raw');
            
            % Add dynamics: two-wheel differential drive
            obj.dynamics = simiam.robot.dynamics.DifferentialDrive(obj.wheel_radius, obj.wheel_base_length);
            
            obj.right_wheel_speed = 0;
            obj.left_wheel_speed = 0;
        end
        
        function ir_distances = get_ir_distances(obj)
            ir_array_values = obj.ir_array.get_range();
            
            ir_voltages = ir_array_values*2/1000;
            coeff = [-0.0182    0.1690   -0.6264    1.1853   -1.2104    0.6293];
            
            ir_distances = polyval(coeff, ir_voltages);
        end
        
        
        function pose = update_state(obj, pose, dt)
            R = obj.wheel_radius;
            
            vel_r = obj.right_wheel_speed;     % mm/s
            vel_l = obj.left_wheel_speed;      % mm/s
            
            pose = obj.dynamics.apply_dynamics(pose, dt, vel_r, vel_l);
            obj.update_pose(pose);
            
            for k=1:length(obj.ir_array)
                obj.ir_array(k).update_pose(pose);
            end
            
            % update wheel encoders
            
            vel_r = obj.right_wheel_speed; %% mm/s
            vel_l = obj.left_wheel_speed; %% mm/s
            
            obj.encoders(1).update_ticks(vel_r, dt);
            obj.encoders(2).update_ticks(vel_l, dt);
        end
        
        function set_wheel_speeds(obj, vel_r, vel_l)
            [vel_r, vel_l] = obj.limit_speeds(vel_r, vel_l);
            R = obj.wheel_radius;
            
            obj.right_wheel_speed = vel_r;
            obj.left_wheel_speed = vel_l;
        end
        
        function [vel_r, vel_l] = limit_speeds(obj, vel_r, vel_l)
            % actuator hardware limits            
            max_rpm = 80;
            max_vel = max_rpm*2*pi/60;
            
            vel_r = max(min(vel_r, max_vel), -max_vel);
            vel_l = max(min(vel_l, max_vel), -max_vel);
        end
    end
    
    methods (Static)
        function raw = ir_distance_to_raw(varargin)
            distance = cell2mat(varargin);
            coeff = [-5.3245, 5.4518, -2.2089, 0.4511, -0.0491, 0.0027]*10^6;
            raw = min(max(round(polyval(coeff, distance)), 200), 1375);
        end
    end
    
end

