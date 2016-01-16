function variable = read_introspection_variable(msgs, variable_name, variable_type)
 if(strcmp(variable_type, 'double'))
  index = get_introspection_varible_index(msgs, variable_name, variable_type);
  number_msgs = size(msgs, 2);
  variable = zeros(number_msgs, 1);
  for i = 1:number_msgs
    variable(i) = msgs{i}.doubles(index).value;
  end
  
 elseif (strcmp(variable_type, 'vectors'))
  index = get_introspection_varible_index(msgs, variable_name, variable_type);
  number_msgs = size(msgs, 2);
  vector_size = size(msgs{1}.vectors(index).value, 1);
  variable = zeros(number_msgs, vector_size);
  for i = 1:number_msgs
    variable(i, :) = msgs{i}.vectors(index).value;
  end
  
 else
  display('ERROR: Type not supported')
 end
end

