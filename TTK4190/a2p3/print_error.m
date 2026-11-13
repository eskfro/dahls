function print_error(error_msg, should_abort)

% Repeated error message
for i=1:10, disp(error_msg); end

% Force quit
if should_abort, error(error_msg);

end