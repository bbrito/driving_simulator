filepath = 'ICRA.bag';
bag = rosbag(filepath);

belief = select(bag, 'Topic', '/belief');
ts = timeseries(belief, '.belief'); 