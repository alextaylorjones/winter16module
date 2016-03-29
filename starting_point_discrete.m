function [Px,Py] = starting_point_discrete(obstacles,startingLoc,num_agents,grid_mat)

if strcmp(startingLoc{1},'random') == 1

Px = zeros(num_agents,1);
Py = zeros(num_agents,1);
xrange = size(grid_mat,2)
yrange = size(grid_mat,1)
%Place robots randomly on grid
  for i = 1:numel(Px)  
      Px(i) = randi(xrange);
      Py(i) = randi(yrange);

      while (grid_mat(Px(i),Py(i)) ~= 0)
          Px(i) = randi(xrange);
          Py(i) = randi(yrange);
      end

      grid_mat(Px(i),Py(i)) = i;
  end
elseif strcmp(startingLoc{1},'fixed') == 1
  % Extract starting point matrix
  start_mat = startingLoc{2};
  sx = ceil(start_mat(1));
  sy = ceil(start_mat(2));
  width = ceil(sqrt(num_agents))+1;
  height = ceil(sqrt(num_agents))+1;

  %find bounding box which fits in the region
  %place agents in small box surrounding the central point

  c = 0;
  for i=1:width
    for j=1:height
      if (c >= num_agents)
          break
      end
      success_flag = 1;
      for ob=1:size(obstacles)

        if ( sx - (j-1) < 1 || sy - (i-1) < 1 || grid_mat(sx - (j-1),sy-(i-1)) ~= 0)
          success_flag = 0;
          break;
        end
      end
      if success_flag == 1
          c = c + 1;
          Px( c) = sx - (j-1);
          Py( c) = sy - (i-1);
      end

    end %end for
  end %end for

   
  assert(c == num_agents); %if this assert fails, then location to start agents is infeasible
end
