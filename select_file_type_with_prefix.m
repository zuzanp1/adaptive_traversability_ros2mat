function newdata = select_file_type_with_prefix(data, type, prefix)

valid = false(1,length(data));
for i = 1:length(data)
    try
        c = cellfun(@(x) strcmpi(data(i).name(end-length(x)+1:end),x),type, 'UniformOutput', 0);
        d = cellfun(@(x) strcmpi(data(i).name(1:length(x)), x), prefix, 'uni', 0);
        valid(i) = any([c{:}]) & any([d{:}]);
        %valid(i) = strcmpi(data(i).name(end-2:end),type);
    catch
        continue;
    end
end
newdata = data(valid);

end