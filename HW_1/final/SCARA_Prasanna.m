dhparams = [0   	0	    0   	pi/3;
            1	    pi      0       pi/6
            0	    0	    1	    0;
            0   	0	    1	    pi/5];
            
robot = rigidBodyTree;

bodies = cell(4,1);
joints = cell(4,1);
for i = 1:4
    bodies{i} = rigidBody(['body' num2str(i)]);
    if (i==3)
        joints{i} = rigidBodyJoint(['joint' num2str(i)],"prismatic");
    else
        joints{i} = rigidBodyJoint(['joint' num2str(i)],"revolute");
    end
    setFixedTransform(joints{i},dhparams(i,:),"dh");
    bodies{i}.Joint = joints{i};


    if i == 1 % Add first body to base
        addBody(robot,bodies{i},"base")
       
    else % Add current body to previous body by name
        addBody(robot,bodies{i},bodies{i-1}.Name)
    end
end

showdetails(robot)

figure(Name="SCARA robot Model")
show(robot);
figure(Name="Interactive GUI")
gui = interactiveRigidBodyTree(robot,MarkerScaleFactor=0.5);