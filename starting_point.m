function [Px,Py] = starting_point(obstacles,crs,startingLoc,num_agents)

if strcmp(startingLoc{1},'random') == 1
  for i = 1:num_agents
      valid_location = 0;
      while (valid_location == 0)
          %Setup location as valid (hypothesis)
          Px(i) = rand()*max(crs(:,1)); 
          Py(i) = rand()*max(crs(:,2));
          
          valid_location = 1;
          %Test for all obstacles
          for ob =1:size(obstacles,1)
              if (inpolygon(Px(i),Py(i),obstacles(ob,:,1), obstacles(ob,:,2)))
                  valid_location = 0;
                  break;
              end
          end

      end
  end
elseif strcmp(startingLoc{1},'fixed') == 1
  % Extract starting point matrix
  start_mat = startingLoc{2};
  sx = start_mat(1);
  sy = start_mat(2);
  width = ceil(sqrt(num_agents));
  height = ceil(sqrt(num_agents));
  success_flag = 0;
  %find bounding box which fits in the region
  %starting from width of 5 down to 1
  for w=flip(1:5)
       if (inpolygon(sx-w,sy-w,crs(:,1),crs(:,2)) && (inpolygon(sx-w,sy+w,crs(:,1),crs(:,2))) && (inpolygon(sx+w,sy-w,crs(:,1),crs(:,2))) && (inpolygon(sx+w,sy+w,crs(:,1),crs(:,2))))
              %if bounding box corners lie in the region (crs)
              success_flag = 1;
      
       end

    for ob =1:size(obstacles,1)
      %check for intersection of bounding box og agents and the obstacles
      if ~isempty((polybool('intersection',[sx-w,sx+w,sx+w,sx-w],[sy-w,sy-w,sy+w,sy+w], obstacles(ob,:,1),obstacles(ob,:,2))))
       success_flag = 0;
       break;
     end
   end %end for
     if (success_flag ==1)
       break
     end
  end
  %place agents in small box surrounding the central point
 
  %num agents divided by width gives step size
  st_w = (2*w) / width;
  st_h = (2*w) / height;
  c = 1;
  for i=1:width
    for j=1:height
      if (c > num_agents)
          break
      end
      Px( (j) + (i-1)*height) = sx - (w) + (j-1)*st_w;
      Py( (j) + (i-1)*height) = sy - (w) + (i-1)*st_h;
      c = c + 1;
      
    end
    if (c > num_agents)
          break
    end
  end

end
