function msgs = read_introspection_bag(bag_name)
 bag = ros.Bag.load(bag_name);
 topic1 = '/data';
 msgs = bag.readAll(topic1);
end
