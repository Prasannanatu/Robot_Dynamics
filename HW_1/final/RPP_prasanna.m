dhparams = [0   	0	    1   	pi/5;
            0	   -pi/2    1       0
            0	    0	    1	    pi/2];
robot = rigidBodyTree;


bodies = cell(3,1);
joints = cell(3,1);
for i = 1:3
    bodies{i} = rigidBody(['body' num2str(i)]);
    
    if i == 1  
        joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
        setFixedTransform(joints{i},dhparams(i,:),"dh");
        bodies{i}.Joint = joints{i};
        addBody(robot,bodies{i},"base")
     
    else 
        bodies{i} = rigidBody(['body' num2str(i)]);
        joints{i} = rigidBodyJoint(['jnt' num2str(i)],"prismatic");
        setFixedTransform(joints{i},dhparams(i,:),"dh");
        bodies{i}.Joint = joints{i};
        addBody(robot,bodies{i},bodies{i-1}.Name)
    end

end
showdetails(robot)

figure(Name="RPP robot Model")
show(robot);

figure(Name="Interactive GUI")
gui = interactiveRigidBodyTree(robot)