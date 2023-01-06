xyz = [0 0 0];

aa = [2 3 6];
bb = [8 5 4];
cc = [7 4 2];

aa = [1.7089   -0.6639   -0.4121];
bb = [1.7509   -0.6192   -0.3843];
cc = [1.7528   -0.6183   -0.3841];

merge = [aa;bb;cc];

origMoveAngle    = acos((bb-aa) / norm(bb-aa))
origMoveDistance = norm(bb-aa)
debrisAngle      = acos((cc-aa) / norm(cc-aa))

avoidAngle = 5;
signs = sign(origMoveAngle-debrisAngle);
ang = origMoveAngle + signs*avoidAngle;

te = bb-aa;
rotaZYX = rotz(ang(3))*roty(ang(2))*rotx(ang(1))*te' + aa'

merge = [aa;bb;cc];
merge1 = [aa;bb;cc;rotaZYX'];
figure(1)
scatter3(merge(:, 1), merge(:, 2), merge(:, 3))
figure(2)
scatter3(merge1(:, 1), merge1(:, 2), merge1(:, 3))


