function r = density(x,y)
    global density_type;
    global density_params;
    if (strcmp(density_type ,'gaussian')  )
        %Sum gaussians
        r = 0;
        for i=1:size(density_params,1)
            r = r + exp(-((x-density_params(i,1))^2) - (y-density_params(i,2))^2);
        end
    end
    if (strcmp(density_type ,'uniform') == 1)
       r = 1;
    end
    if (strcmp(density_type ,'ellipse') == 1)
        %entry 1 = xc
        %entry 2 = yc
        %entry 3 = a (x stretch)
        %entry 4 = b (y stretch)
        %entry 5 = k (intensity)
        %entry 6 = r (radius)
        r =0;
        for i=1:size(density_params,1)
            xc = (density(i,1));
            yc =density(i,2);
            a = density(i,3);
            b = density(i,4);
            k = density(i,5);
            r = density(i,6);
            r = r + exp(-k*(a*(x-xc)^2+b*(y-yc)^2-r^2)^2);
            
        end
    end
    if (strcmp(density_type ,'disk') == 1)
       assert(false);
    end
    %Uniform intensity outside of possible rectanges
    %Each rectangle has uniform multiple of coverage cost 
    % Format for params:
    % first row - [relative_weight,DC;
    % other rows- [xmax,xmin,xmax,xmin] for values of  rectangle
    if strcmp(density_type ,'multi_rect')==1
      %check if x,y in rect
      relative_weight = density_params(1,1);
      multi_rect = density_params(2:size(density_params,1),:); 
      
      %
      r=0;
      
      for i=1:size(multi_rect,1)
        north = multi_rect(i,1);
        south = multi_rect(i,2);
        east = multi_rect(i,3);
        west = multi_rect(i,4);
        if y <= north
          if y >= south
            if x <= east
              if x >= west
              %In rectange 
              r = r+relative_weight;
              r = r +exp(-((x-((east+west)/2))^2+((y-(north+south)/2)^2)));
              return;
              end
            end
          end
        end
        
        %boundary ramp
        height = north-south;
        width = east-west;
        if y < north+(height/10)
            if y > south - (height/10)
                if x > west - width/10
                    if x < east + width/10
                        min_x_d = min(x - (west-(width/10)),x - (east+(width/10)));
                        min_y_d = min(y - (north+(height/10)),y - (south-(height/10)));
                        r = relative_weight - ((relative_width-1)/(height+width))*(min_x_d+min_y_d);
                        return;
                    end
                end
            end
        end
        
      end
      
      r = 1;
    end
    
function setup_density_function(type,params)
    global density_type;
    global density_params;
    density_type = type;
    density_params = params;
    
    
