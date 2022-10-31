% rosinit
for k = 1:size(open,1)
    poseMsg.Position.X = open(k,1);
    poseMsg.Position.Y = open(k,2);
    poseMsg.Position.Z = open(k,3);
    posearray.Poses(k) = poseMsg; 
end

send(pub_fit, posearray);

