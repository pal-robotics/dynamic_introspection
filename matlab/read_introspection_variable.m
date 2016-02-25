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
 
 elseif (strcmp(variable_type, 'vectors3d'))
  index = get_introspection_varible_index(msgs, variable_name, variable_type);
  number_msgs = size(msgs, 2);
  vector_size = size(msgs{1}.vectors3d(index).value, 1);
  variable = zeros(number_msgs, vector_size);
  for i = 1:number_msgs
    variable(i, :) = msgs{i}.vectors3d(index).value;
  end  
  
  elseif (strcmp(variable_type, 'matrixs3d'))
    index = get_introspection_varible_index(msgs, variable_name, variable_type);
    number_msgs = size(msgs, 2);
    matrix_rows = msgs{1}.matrixs3d(index).rows;
    matrix_cols = msgs{1}.matrixs3d(index).cols;
    variable = cell(number_msgs, 1);
  for i = 1:number_msgs
    M = zeros(matrix_rows, matrix_cols);
    for m = 1:matrix_rows
       for n = 1:matrix_cols
          M(m, n) = msgs{i}.matrixs3d(index).value((m - 1)*matrix_rows + n);
       end
    end
    variable{i} = M;
  end  
  
 else
  display('ERROR: Type not supported')
 end
end

