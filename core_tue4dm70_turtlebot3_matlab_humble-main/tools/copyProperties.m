% Copies the writable properties from an (original) object to another 
% (clone) object of the same class.

function copyProperties(objOriginal, objClone)

    if not(isa(objOriginal, class(objClone)))
        error('Object class mismatch!');
    end
    
    propertyList = properties(objOriginal);
   
    for i = 1:length(propertyList)
        if isempty(properties(objOriginal.(propertyList{i})))
            try
                objClone.(propertyList{i}) = objOriginal.(propertyList{i});
            catch
                % Do nothing
                % disp(propertyList{i})
            end    
        else
            copyProperties(objOriginal.(propertyList{i}), objClone.(propertyList{i}));
        end
    end
    
end