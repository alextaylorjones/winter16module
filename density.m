function r = density(x,y)
    global density_type;
    global density_params;
    if (density_type == 'gaussian')
        %Sum gaussians
        r = 0
        for i=1:size(density_params,1)
            r = r + exp(-((x-density_params(i,1))^2) - (y-density_params(i,2)^2))
        end
    end
    
    r = exp(-(x-10)*(x-10)-(y-15)*(y-15));
    
    
function setup_density_function(type,params)
    global density_type;
    global density_params;
    density_type = type;
    density_params = params;
    
    