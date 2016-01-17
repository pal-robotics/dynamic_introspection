function dipsplay_variable_names(bag_name)

msgs = read_introspection_bag(bag_name);
value = read_introspection_variable(msgs, 'Torques', 'vectors');


end