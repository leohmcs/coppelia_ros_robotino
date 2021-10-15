function publish_joint_state(jointHandle, jointName, publisher)
    jointPos = sim.getJointPosition(jointHandle)
    jointVel = sim.getJointTargetVelocity(jointHandle, -1)
    jointEffort = sim.getJointForce(jointHandle)
    
    msg = {
        header = {
            stamp = simROS.getTime(),
            frame_id = jointName
        },
        name = {jointName},
        position = {jointPos},
        velocity = {jointVel},
        effort = {jointEffort}
    }
    
    simROS.publish(publisher, msg)
end


function publish_gt_pose()
    p = sim.getObjectPosition(robotHandle, -1)
    o = sim.getObjectQuaternion(robotHandle, -1)
    
    pose_stamped = {
        header = {
            stamp= simROS.getTime(),
            frame_id= "/" .. robotPrefix .. '_tf/odom'
        },
        pose = {
            position = {x = p[1], y = p[2], z = p[3]},
            orientation = {x = o[1], y = o[2], z = o[3], w = o[4]}
        }
    }
    
    simROS.publish(publisher_gt_pose, pose_stamped)
end


function publish_gt_relative_pose()
    p = sim.getObjectPosition(robotHandle, -1)
    o = sim.getObjectQuaternion(robotHandle, -1)
    
    initial_p = initial_pose["position"]
    initial_o = initial_pose["orientation"]
        
    pose_stamped = {
        header = {
            stamp= simROS.getTime(),
            frame_id= "/" .. robotPrefix .. '_tf/odom'
        },
        pose = {
            position = {x = p[1] - initial_p[1], y = p[2] - initial_p[2], z = p[3] - initial_p[3]},
            orientation = {x = o[1], y = o[2], z = o[3], w = o[4]}
        }
    }
    
    simROS.publish(publisher_gt_relative_pose, pose_stamped)
end

function velocity_callback(msg)
    dx = msg.linear.x
    dy = msg.linear.y
    dt = msg.angular.z
    
    move(dx, dy, dt)
end


function move(dx, dy, dt)
    q = Vector3{dx, dy, dt}
    v = Minv * q
    
    sim.setJointTargetVelocity(motor0Handle, v[1])
    sim.setJointTargetVelocity(motor1Handle, v[2])
    sim.setJointTargetVelocity(motor2Handle, v[3])
end


function euler_to_quaternion(quat)
    -- TODO
end

function sysCall_init()
    require 'matrix'
    
    robotHandle = sim.getObjectHandle(sim.handle_self)
    robotName = sim.getObjectName(robotHandle)
    robotPrefix = robotName:gsub("#","")  -- Remove #
    
    motor0Handle = sim.getObjectHandle("wheel0_joint")
    motor1Handle = sim.getObjectHandle("wheel1_joint")
    motor2Handle = sim.getObjectHandle("wheel2_joint")
    
    local pos = sim.getObjectPosition(robotHandle, -1)
    local ori = sim.getObjectOrientation(robotHandle, -1)
    initial_pose = {
        position = pos,
        orientation = ori
    }
    
    Rz = Matrix(3, 3, {math.cos(ori[3]), math.sin(ori[3]), 0.0, math.cos(ori[3]), -math.sin(ori[3]), 0.0, 0.0, 0.0, 1.0})
        
    -- Kinematic model
    L = 0.135   -- Meters
    r = 0.040   -- Meters
    
    Minv = Matrix(3, 3, {-math.sqrt(3)/2, 0.5, L, 0.0, -1.0, L, math.sqrt(3)/2, 0.5, L})
    Minv = Minv * (1/r)
    
    if simROS then
        sim.addLog(sim.verbosity_scriptinfos, "ROS interface was found.")
        
        -- Prepare GT pose
        publisher_gt_relative_pose = simROS.advertise("/" .. robotPrefix .. '/gt_relative_pose', 'geometry_msgs/PoseStamped')
        publisher_gt_pose = simROS.advertise("/" .. robotPrefix .. '/gt_pose', 'geometry_msgs/PoseStamped')
        
        publisher_joint0_state = simROS.advertise("/" .. robotPrefix .. "/joint0_state", "sensor_msgs/JointState")
        publisher_joint1_state = simROS.advertise("/" .. robotPrefix .. "/joint1_state", "sensor_msgs/JointState")
        publisher_joint2_state = simROS.advertise("/" .. robotPrefix .. "/joint2_state", "sensor_msgs/JointState")
        
        subscriber_velocity = simROS.subscribe("/" .. robotPrefix .. "/cmd_vel", "geometry_msgs/Twist", "velocity_callback")
        
        simROS.setParamDouble("/" .. robotPrefix .. "/map_merge/init_pose_x", initial_pose["position"][1])
        simROS.setParamDouble("/" .. robotPrefix .. "/map_merge/init_pose_y", initial_pose["position"][2])
        simROS.setParamDouble("/" .. robotPrefix .. "/map_merge/init_pose_z", 0)
        simROS.setParamDouble("/" .. robotPrefix .. "/map_merge/init_pose_yaw", initial_pose["orientation"][3])
    else
        sim.addLog(sim.verbosity_scripterrors, "ROS interface was not found. Cannot run.")    
    end
end


function sysCall_actuation()
    publish_gt_pose()
    publish_gt_relative_pose()
    
    joints_handlers = {motor0Handle, motor1Handle, motor2Handle}
    joints_frames = {"wheel0_joint", "wheel1_joint", "wheel2_joint"}
    joints_publishers = {publisher_joint0_state, publisher_joint1_state, publisher_joint2_state}
    for i=1,3,1 do  
        publish_joint_state(joints_handlers[i], joints_frames[i], joints_publishers[i])
    end
end