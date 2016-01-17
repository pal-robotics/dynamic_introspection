function index = get_introspection_varible_index(msgs, variable_name, type)
 if(strcmp(type, 'double'))
   number_variables = size(msgs{1}.doubles, 2);
   for i = 1:number_variables
     if strcmp(msgs{1}.doubles(i).name, variable_name)
       index = i;
       break;
     end
   end
   
 elseif(strcmp(type, 'vectors'))
   number_variables = size(msgs{1}.vectors, 2);
   for i = 1:number_variables
     if strcmp(msgs{1}.vectors(i).name, variable_name)
       index = i;
       break;
     end
   end
 
  elseif(strcmp(type, 'vectors3d'))
   number_variables = size(msgs{1}.vectors3d, 2);
   found = 0;
   for i = 1:number_variables
     if strcmp(msgs{1}.vectors3d(i).name, variable_name)
       index = i;
       found = 1;
       break;
     end
   end
   if found == 0
    display('Variable not found')
   end
   
 else
   display('ERROR: Type not supported')
 end
 
end
