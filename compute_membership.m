% membership function computation 
function o = compute_membership(in, type)
    % check type of memership function 
    switch(type)
        case 'f11'
            o = min([max([0, (in-80)/-160]), 1]);
        case 'f12'
            o = 1 - min([max([0, (in-80)/-160]), 1]);
        case 'f21'
            o = (-abs(in)+180)/180;
        case 'f22'
            o = 1 - (-abs(in)+180)/180;
    end
end