%obstacle configuration - numerical id for obstacle configurations described in get_obstacle_set
% density_function_type - see density.m,
% 'gaussian','uniform','ellipse','disk','multi_rect
%density_function_params - UNIMPLEMENTED
% test
%num_agents - as described, integer input
%num_iterations - number of iterations to compute for each simulation
%simulation type [string,param1,param2,...]
   %1. metric-all : runs all algorithms on all algorithms 
       %param 1 -control gain for lloyd
       %param 2 - control gain for ladybug
       %param 3 control gain for combined
       %param 4- loop gain
   % EXAMPLES
   % multi_sim_comparison(obstacle_configuration, simulation_type, 'gaussian',density_function_params, 10,100,0)
% startingLoc - {'type',x,y}
%   %type = 'fixed','random'
%   % fixed - start agents surrounding x y point
%   % random - start agents randomly (does not use 2nd and 3rd elements of
%   cell)
function agent_loc = multi_sim_comparison(obstacle_configuration, simulation_type, density_function_type,density_function_params, num_agents, num_iterations, startingLoc,agentLoc)

%Num samples runs of each algorithm
num_trials = 1;

%random seed
seed = 19;
    
%For now show plots of algorithm runs
show_plot = true;

%setup density function and obstacle global vars
global density_params;
global density_type;
global obstacle_config;
density_type = density_function_type;
density_params = density_function_params;
obstacle_config = obstacle_configuration;

%Setup obstacles
global obstacles;
obstacles = [];
get_obstacle_set();


%Split based on simulation type
if strcmp(simulation_type{1},'param-vary') == 1
    NUM_SAMPLES = 5000;
    sz = size(simulation_type);
    for i=2:size(sz,2)
       algorithm_name = simulation_type{i};
       %Run the algorithm specified against reasonable parameter ranges
       %Once there is convergence (change in displacement over iteration is within 5% for three runs in a row), 
       %then we assess the metrics of concern
       %after sweeping through all parameters, visualize change of metrics at convergence for specified algorithm
       agent_loc = zeros(num_trials,1,num_iterations,num_agents,2);
       
       if strcmp(algorithm_name,'lloyd')== 1
          
           i = 1;
           %Modify this to alter randge of parameters to sweep
           param_sweep = 0.05:0.05:0.9;
           
           
           cost_vec = zeros(numel(param_sweep),num_trials);
           disp_vec = zeros(numel(param_sweep),num_trials);
           kEnergy_vec = zeros(numel(param_sweep),num_trials);

           for control_gain=param_sweep
             for trial=1:num_trials
                agent_loc(trial,i,1:num_iterations,1:num_agents,1:2) = Non_adaptive_ladybug_coverage(num_iterations,show_plot,num_agents,obstacles,seed,control_gain,0,startingLoc); 
                cost_vec(i,trial) = cost_vec(i) + get_final_cost(agent_loc(trial,i,1:num_iterations,1:num_agents,1:2), obstacles,NUM_SAMPLES);
                kEnergy_vec(i,trial) = kEnergy_vec(i) + get_final_kEnergy(agent_loc(trial,i,1:num_iterations,1:num_agents,1:2));
                disp_vec(i,trial) = disp_vec(i) + get_final_displacement(agent_loc(trial,i,1:num_iterations,1:num_agents,1:2));
                
              end
              i = i + 1;
           end
        %chart lloyd;
          %close all;
          figure;
          plot(param_sweep,sum(cost_vec,2));
          title('Lloyd Algorithm Final Cost vs Control Gain');
          figure;
          plot(param_sweep,sum(disp_vec,2));
          title('Lloyd Algorithm Total Displacement vs Control Gain');
          figure;
          plot(param_sweep,sum(kEnergy_vec,2));
          title('Lloyd Algorithm Total kEnergy vs Control Gain');
               
       elseif strcmp(algorithm_name,'ladybug')== 1
           %Modify these to alter range of parameters to sweep
         param_sweep_exp = 0.05:0.05:0.50;
         param_sweep_ctl = 0.05:0.05:0.5;
         i = 1;
          cost_vec = zeros(numel(param_sweep_exp)*numel(param_sweep_ctl),1);
           disp_vec = zeros(numel(param_sweep_exp)*numel(param_sweep_ctl),1);
           kEnergy_vec = zeros(numel(param_sweep_exp)*numel(param_sweep_ctl),1);


          for exploration_gain = param_sweep_exp
            for control_gain=param_sweep_ctl
              for trial=1:num_trials
                agent_loc(trial,i,1:num_iterations,1:num_agents,1:2) = Non_adaptive_ladybug_coverage(num_iterations,show_plot,num_agents,obstacles,seed,control_gain,exploration_gain,startingLoc); 

                cost_vec(i) = cost_vec(i) + get_final_cost(agent_loc(trial,i,1:num_iterations,1:num_agents,1:2), obstacles,NUM_SAMPLES);
                kEnergy_vec(i) = kEnergy_vec(i) + get_final_kEnergy(agent_loc(trial,i,1:num_iterations,1:num_agents,1:2));
                disp_vec(i) = disp_vec(i) + get_final_displacement(agent_loc(trial,i,1:num_iterations,1:num_agents,1:2));

              end

             i = i + 1; 
            end
          end


          %chart ladybug
          %close all;
          figure ;
          hold on;
          h = zeros(1,numel(param_sweep_exp));
          for i=1:numel(param_sweep_exp)
            col(i) = (i/numel(param_sweep_exp))* [1,1,1];
          end
          for i = 1:numel(param_sweep_exp)
            h(i) = plot(param_sweep_ctl,sum(cost_vec((i-1)*numel(param_sweep_ctl) + 1:i*numel(param_sweep_ctl)),2), 'DisplayName',sprintf('exp-gain%2f',param_sweep_exp(i)),'Color',col(i));
          end
          title('Nonadaptive Ladybug Algorithm Final Cost vs Control Gain');
          legend(h);
          hold off;
          
          figure ;
          hold on;
          h = zeros(1,numel(param_sweep_exp));
          for i = 1:numel(param_sweep_exp)
            h(i) = plot(param_sweep_ctl,sum(disp_vec((i-1)*numel(param_sweep_ctl) + 1:i*numel(param_sweep_ctl)),2), 'DisplayName',sprintf('exp-gain%2f',param_sweep_exp(i)),'Color',col(i));
          end
          title('Nonadaptive Ladybug Algorithm Total Displacement vs Control Gain');
          legend(h);
          hold off;

          figure ;
          hold on;
          h = zeros(1,numel(param_sweep_exp));
          for i = 1:numel(param_sweep_exp)
            h(i) = plot(param_sweep_ctl,sum(cost_vec((i-1)*numel(param_sweep_ctl) + 1:i*numel(param_sweep_ctl)),2), 'DisplayName',sprintf('exp-gain%2f',param_sweep_exp(i)),'Color',col(i));
          end
          title('Nonadaptive Ladybug Algorithm Total kEnergy vs Control Gain');
          legend(h)
          hold off;

       elseif strcmp(algorithm_name,'combined')== 1
         %control gain for virtual generators
         i = 1;
         max_step = 0;
         
         %Modify this to change parameter range
           param_sweep = 0.85;
           
           cost_vec = zeros(numel(param_sweep),1);
           disp_vec = zeros(numel(param_sweep),1);
           kEnergy_vec = zeros(numel(param_sweep),1);


          for control_gain=param_sweep
            for trial=1:num_trials
                agent_loc(trial,i,1:num_iterations,1:num_agents,1:2) = combined(num_iterations,show_plot,num_agents,obstacles,seed,control_gain,0,max_step,startingLoc);          
                cost_vec(i) = cost_vec(i) + get_final_cost(agent_loc(trial,i,1:num_iterations,1:num_agents,1:2), obstacles,NUM_SAMPLES);
                kEnergy_vec(i) = kEnergy_vec(i) + get_final_kEnergy_avoid_obstacles(agent_loc(trial,i,1:num_iterations,1:num_agents,1:2));
                disp_vec(i) = disp_vec(i) + get_final_displacement_avoid_obstacles(agent_loc(trial,i,1:num_iterations,1:num_agents,1:2));

            end
            i = i + 1;
          end
          %chart combined metrics
          %close all;
          figure;
          plot(param_sweep,sum(cost_vec,2));
          title('Combined Algorithm Final Cost vs Control Gain');
          figure;
          plot(param_sweep,sum(disp_vec,2));
          title('Combined Algorithm Total Displacement vs Control Gain');
          figure;
          plot(param_sweep,sum(kEnergy_vec,2));
          title('Combined Algorithm Total kEnergy vs Control Gain');
          
       end
       
    end
    
    
end

if (strcmp(simulation_type{1},'metric-all') == 1)
    control_gain_lloyd = simulation_type{2};
    control_gain_lb = simulation_type{3};
    control_gain_combined = simulation_type{4};
    loop_gain = simulation_type{5};
    exploration_gain = simulation_type{6};
    %Number of algorithms tried
    num_algs = 5;
    
    agent_loc = zeros(num_trials,num_algs,num_iterations,num_agents,2);
    for cur_trial=1:num_trials
        %Run the algorithms using params specified in function input


        %Run lloyd style algorithm (use degenerate
        %non_adaptive_ladybug_coverage
        
        agent_loc(cur_trial,1,:,:,:) = Non_adaptive_ladybug_coverage(num_iterations,show_plot,num_agents,obstacles,seed,control_gain_lloyd,0,startingLoc);
      
        %Run approximation via search based grid algorithm
       agent_loc(cur_trial,2,:,:,:) = approximation_discrete_nonconvex_coverage(num_iterations,show_plot,num_agents,obstacles,seed,startingLoc);

        %Run combined tangentbug and lloyd
        max_step = 0.25;
        B = combined(num_iterations,show_plot,num_agents,obstacles,seed,control_gain_combined,loop_gain,max_step,startingLoc);
        agent_loc(cur_trial,3,:,:,:) = B;
        %Run optimal annealing, algorithm
        %A = optimal_coverage_grid(num_iterations,show_plot,num_agents,obstacles,seed,startingLoc);
        %agent_loc(cur_trial,4,:,:,:)= A;
        %Run non adaptive ladybug algorithm

        agent_loc(cur_trial,5,:,:,:) = Non_adaptive_ladybug_coverage(num_iterations,show_plot,num_agents,obstacles,seed,control_gain_lb, exploration_gain,startingLoc);

    end
        %Plot metric results
    close all;

    NUM_SAMPLES = 5000;
    [time_lloyd, cost_lloyd ] = get_cost_timeline_scaled(agent_loc(:,1,:,:,:),obstacles,NUM_SAMPLES);
    [time_approx, cost_approx ] = get_cost_timeline_scaled(agent_loc(:,2,:,:,:),obstacles,NUM_SAMPLES);
    [time_combined,cost_combined] = get_cost_timeline_scaled(agent_loc(:,3,:,:,:),obstacles,NUM_SAMPLES);
    [time_optimal, cost_optimal ] = get_cost_timeline_scaled(agent_loc(:,4,:,:,:),obstacles,NUM_SAMPLES);
    [time_ladybug, cost_ladybug ] = get_cost_timeline_scaled(agent_loc(:,5,:,:,:),obstacles,NUM_SAMPLES);

    figure(1);
    col(1,1:3) = [1,0,0];
    col(2,1:3) = [1,1,0];
    col(3,1:3) = [0,0,1];
    col(4,1:3) = [0,1,0];
    col(5,1:3) = [0,1,1];
    
%     for i =1:num_algs
%         col(i,1:3) = (num_algs- i)/num_algs * ones(3,1);
%     end
    hold on;
    for trial = 1:num_trials
             h((trial-1)*num_algs + 1) =  plot(time_lloyd(:,trial),cost_lloyd(:,trial), 'DisplayName',sprintf('lloyd-trial(%d)',trial),'Color',col(1,:));
             h((trial-1)*num_algs + 2) =  plot(time_approx(:,trial),cost_approx(:,trial), 'DisplayName',sprintf('approx-trial(%d)',trial),'Color',col(2,:));

             h((trial-1)*num_algs + 3) =  plot(time_combined(:,trial),cost_combined(:,trial), 'DisplayName',sprintf('combined-trial(%d)',trial),'Color',col(3,:));
             h((trial-1)*num_algs + 4) =  plot(time_optimal(:,trial),cost_optimal(:,trial), 'DisplayName',sprintf('optimal-trial(%d)',trial),'Color',col(4,:));
             h((trial-1)*num_algs + 5) =  plot(time_ladybug(:,trial),cost_ladybug(:,trial), 'DisplayName',sprintf('ladybug-trial(%d)',trial),'Color',col(5,:));
    end
    legend(h);
    title('Coverage Control Sampled (5000) Cost');
    xlabel('Scaled Time');
    ylabel('Cost');
    hold off;

    disp_lloyd = get_displacement_vec(agent_loc(:,1,:,:,:));
    disp_approx = get_displacement_vec(agent_loc(:,2,:,:,:));
    disp_combined = get_displacement_vec_avoid_obstacles(agent_loc(:,3,:,:,:));
    disp_optimal = get_displacement_vec(agent_loc(:,4,:,:,:));
    disp_ladybug = get_displacement_vec(agent_loc(:,5,:,:,:));


    figure(2);
    hold on;
    for trial = 1:num_trials
             h((trial-1)*num_algs + 1) =  plot(time_lloyd(:,trial),disp_lloyd(:,trial), 'DisplayName',sprintf('lloyd-trial(%d)',trial),'Color',col(1,:));
             h((trial-1)*num_algs + 2) =  plot(time_approx(:,trial),disp_approx(:,trial), 'DisplayName',sprintf('approx-trial(%d)',trial),'Color',col(2,:));

             h((trial-1)*num_algs + 3) =  plot(time_combined(:,trial),disp_combined(:,trial), 'DisplayName',sprintf('combined-trial(%d)',trial),'Color',col(3,:));
             h((trial-1)*num_algs + 4) =  plot(time_optimal(:,trial),disp_optimal(:,trial), 'DisplayName',sprintf('optimal-trial(%d)',trial),'Color',col(4,:));
             h((trial-1)*num_algs + 5) =  plot(time_ladybug(:,trial),disp_ladybug(:,trial), 'DisplayName',sprintf('ladybug-trial(%d)',trial),'Color',col(5,:));
    end
    legend(h);
    title('Coverage Control Displacement');
    xlabel('Scaled Time');
    ylabel('Total Displacement');
    hold off;
    
    kEnergy_lloyd  = get_kEnergy_timeline(agent_loc(:,1,:,:,:), time_lloyd);
    kEnergy_approx  = get_kEnergy_timeline(agent_loc(:,2,:,:,:), time_approx);
    kEnergy_combined = get_kEnergy_timeline_avoid_obstacles(agent_loc(:,3,:,:,:),time_combined);
    kEnergy_optimal  = get_kEnergy_timeline(agent_loc(:,4,:,:,:),time_optimal);
    kEnergy_ladybug  = get_kEnergy_timeline(agent_loc(:,5,:,:,:),time_ladybug);


    figure(3);
    hold on;
    for trial = 1:num_trials
             h((trial-1)*num_algs + 1) =  plot(time_lloyd(:,trial),kEnergy_lloyd(:,trial), 'DisplayName',sprintf('lloyd-trial(%d)',trial),'Color',col(1,:));
             h((trial-1)*num_algs + 2) =  plot(time_approx(:,trial),kEnergy_approx(:,trial), 'DisplayName',sprintf('approx-trial(%d)',trial),'Color',col(2,:));
             h((trial-1)*num_algs + 3) =  plot(time_combined(:,trial),kEnergy_combined(:,trial), 'DisplayName',sprintf('combined-trial(%d)',trial),'Color',col(3,:));
             h((trial-1)*num_algs + 4) =  plot(time_optimal(:,trial),kEnergy_optimal(:,trial), 'DisplayName',sprintf('optimal-trial(%d)',trial),'Color',col(4,:));
             h((trial-1)*num_algs + 5) =  plot(time_ladybug(:,trial),kEnergy_ladybug(:,trial), 'DisplayName',sprintf('ladybug-trial(%d)',trial),'Color',col(5,:));
    end
    legend(h);
    title('Kinetic Energy');
    xlabel('Scaled Time');
    ylabel('Cost');
    hold off;
    


    
end

end

%Setup terrain matrix
function TM = get_terrain_matrix(obstacles)
    rows = 50;
    cols = 50;
    xrange = 30;
    yrange = 30;
    
    dx = cols/xrange;
    dy = rows/yrange;
    
    
    TM = zeros(rows,cols);
    
    for i=1:rows-1
        for j=1:cols-1
            for ob =1:size(obstacles,1)
               X = polybool('intersection',[(j)*dx,j*dx,(j-1)*dx,(j-1)*dx],[(i-1)*dy,(i)*dy,i*dy,(i-1)*dy],obstacles(ob,:,1),obstacles(ob,:,2));
               if (~isempty(X))
                    TM(i,j) = 1;
               end
            end
        end
    end
end

%Add an elseif clause to add new obstacles
%Format of obstacles are M x N x N, whre M is number of obstacles, N is
%vertices of obstacles.
%We assume boundary is 30x30
function get_obstacle_set()
    global obstacles;
    global obstacle_config;
    ob_config = obstacle_config;
    if (ob_config == 1)
        obstacles = [];
    end
    if mod(ob_config,2) == 0
        %one saw shape osbtacle in bottom left corner
        obstacles(size(obstacles,1)+1,:,:) = [0,0;10,0;10,10;2,7;7,4;0,0];
    end
    if mod(ob_config,3) == 0

    obstacles(size(obstacles,1)+1,:,:) = [0,0;20,0;20,10;3,10;3,6;7,6;7,4;3,4;0,0];
    end
    if mod(ob_config,5) == 0
        obstacles(size(obstacles,1)+1,:,:) = [3,25;25,3;15,10;5,25;5,25;5,25;5,25;5,25;5,25];
    end
    if mod(ob_config,7) == 0
        obstacles(size(obstacles,1)+1,:,:) = [15,5;20,10;20,20;15,25;10,25;5,20;5,15;10,10;15,15;15,20;10,20;10,17;13,14;10,12;7,15;10,23;15,23;18,18;18,10;15,5 ];
    end
    if mod(ob_config,11) == 0
        %obstacles(size(obstacles,1)+1,:,:) = (0.034)*[0,0; 60,0; 60,35; 30,35; 30,45; 70,45; 70,5; 100,5; 70, 100; 70,55; 30, 55; 30, 65; 60,65; 60,95; 5,95; 5,65; 20,65; 20,55; 5,55; 5,45; 20,45; 20,35; 5,35; 0,0 ]
        obstacles(size(obstacles,1)+1,:,:) = 0.3*[0,0; 60,0; 60,35; 30,35; 30,45; 70,45; 70,5; 95,5; 70, 95; 70,55; 30, 55; 30, 65; 60,65; 60,95; 5,95; 5,65; 20,65; 20,55; 5,55; 5,45; 20,45; 20,35; 5,35; 0,0 ]
    end
    %simple spiral
    if mod(ob_config,13) == 0
      %number of spiral loops
      loopcnt = 3;
      xrange = 30;
      yrange = 30;
      dx = floor(xrange/(2*loopcnt));
      dy = (yrange/(2*loopcnt - 1));
      xleft = 0;
      xright = xrange;
      ydown =0;
      yup = yrange;
      spiral = [];
        spiral(size(spiral,1)+1,1) = xleft + dx;
        spiral(size(spiral,1),2) = ydown;
        
      for l =1:loopcnt-1


        spiral(size(spiral,1)+1,1) = xleft + dx;
        spiral(size(spiral,1),2) = yup-dy;

        spiral(size(spiral,1)+1,1) = xright-dx;
        spiral(size(spiral,1),2) = yup-dy;

        spiral(size(spiral,1)+1,1) = xright-dx;
        spiral(size(spiral,1),2) = ydown+dy;

        spiral(size(spiral,1)+1,1) = xleft + 2*dx;
        spiral(size(spiral,1),2) = ydown+dy;

        xleft = xleft+dx;
        xright= xright-dx;
        yup = yup -dy;
        ydown = ydown+dy;
      end
      
      %offset
      for i =1:size(spiral,1)
        if mod(i,4) == 1
            sgx = 1;
            sgy = 1;
        elseif mod(i,4) == 2
            sgx = 1;
            sgy = -1;
        elseif mod(i,4) == 3
            sgx = -1;
            sgy = -1;
        elseif mod(i,4) == 0
            sgx = -1;
            sgy = 1;
        end
        spiral_ccw(i,:) = spiral(i,:) + [sgx*dx/3,sgy*dy/3];
      end
      
      

      obstacles(size(obstacles,1)+1,:,:) = ([spiral;fliplr(spiral_ccw')']);
        
    end
%     if mod(ob_config,17)==0 %Office Plan
%         obstacles(size(obstacles,1)+1,:,:) = 0.3*[60,0; 60,35; 30,35; 30,45; 70,45; 70,0; 60,0];
%         obstacles(size(obstacles,1)+1,:,:) = 0.3*[60,100; 60,65; 30,65; 30,55; 70,55; 70,100; 60,100];
%         obstacles(size(obstacles,1)+1,:,:) = 0.3*[0,65; 20,65; 20,55; 0,55; 0,65;0,65;0,65 ];
%         obstacles(size(obstacles,1)+1,:,:) = 0.3*[0,45; 20,45; 20,35; 0,35; 0,45;0,65;0,65];
%     end
    if mod(ob_config,19)==0 %Street
        for i=0:2
            for j=0:2
                obstacles(size(obstacles,1)+1,:,:) = [4+8*i,4+8*j; 10+8*i,4+8*j; 10+8*i,10+8*j; 4+8*i,10+8*j; 4+8*i,4+8*j];
            end
        end
    end
    
    if mod(ob_config,23)==0
        obstacles(size(obstacles,1)+1,:,:) = [17.5,5; 25,7.5; 20,12.5; 17.5,12; 12,17.5; 12.5,20; 7.5,25; 5,20; 17.5,5];
    end
    if mod(ob_config,29)==0 %Random Rectangles
        x=[];
        y=[];
        cx=[];
        cy=[];
        obstacles(size(obstacles,1)+1,:,:)=[]
%         x(size(x,1)+1,:) = round(rand(1)*30)/4;
%         y(size(x,1)+1,:) = round(rand(1)*30)/4;
%         cx(size(x,1)+1,:) = round(rand(1)*30);
%         cy(size(x,1)+1,:) = round(rand(1)*30);
%         obstacles(size(obstacles,1)+1,:,:) = [cx-x,cy-y; cx+x,cy-y; cx+x,cy+y; cx-x,cy+y];
        
        for i=1:5%round(rand(1)*30)/4
            x(i) = round(rand(1)*30)/4;
            y(i) = round(rand(1)*30)/4;
            cx(i) =round(rand(1)*30);
            cy(i) =round(rand(1)*30);
%             if ~(polybool('intersection', x(i+1), y(i), x(i-1), y(i-1)))
                 obstacles(size(obstacles,1)+1,:,:) = [cx(i)-x(i),cy(i)-y(i); cx(i)+x(i),cy(i)-y(i); cx(i)+x(i),cy(i)+y(i); cx(i)-x(i),cy(i)+y(i)];         
%             end
        end
    end
end

function [time_scaled,cost_vec] = get_cost_timeline_scaled(agent_locations,obstacles,NUM_SAMPLES)
    

    for trial=1:size(agent_locations,1)
        
        cost_vec(:,trial) = get_cost_timeline(agent_locations,obstacles,NUM_SAMPLES);
    end
    cur = 1;
    time_scaled = [];
    for i=2:size(cost_vec,1)
        for trial=1:size(agent_locations,1)
            time_scaled(i-1,trial) = cur;

            time_scaled(i,trial) = cur + max_distance_travelled(trial,i,agent_locations,obstacles);
            cur = time_scaled(i);
        end
    end
end

function d_max = max_distance_travelled(trial,counter,agent_locations,obstacles)
    d_max = -Inf;
    for ag_num = 1:size(agent_locations,4) %num agents
        d_cur = wall_hugging_path_length(agent_locations(trial,1,counter,ag_num,1),agent_locations(trial,1,counter,ag_num,2),...
                                agent_locations(trial,1,counter-1,ag_num,1),agent_locations(trial,1,counter-1,ag_num,2));
        if d_cur > d_max
            d_max = d_cur;
        end
    end
end

%Use sampling to detesrmine global cost
function cost_vec = get_cost_timeline(agent_locations,obstacles, NUM_SAMPLES)
	xrange = 30;
	yrange = 30;
    
    if min(agent_locations == 0) == 1
        cost_vec = zeros(size(agent_locations,3),1);%num iterations
        return;
    end
        
	sample_points(:,1) = rand(NUM_SAMPLES,1)*xrange;
	sample_points(:,2) = rand(NUM_SAMPLES,1)*yrange;
	sample_points(:,3) = zeros(NUM_SAMPLES,1);
    used_samples = NUM_SAMPLES;
	for i =1:NUM_SAMPLES
		for ob = 1:size(obstacles,1)

			if inpolygon(sample_points(i,1), sample_points(i,2),obstacles(ob,:,1),obstacles(ob,:,2))
				sample_points(i,3) = -1;
                used_samples = used_samples - 1;
				break;
			end
		end
	end

  %num trials, repeat and average cost
  cost_vec = zeros(size(agent_locations,3),size(agent_locations,1));%num iterations
  for trial = 1:size(agent_locations,1)

    for counter =1:size(agent_locations,3)%num iterations
          for i=1:NUM_SAMPLES
              if (sample_points(i,3) == 0)
              min_dist = Inf;
              min_agent = -1;
                  for ag = 1:size(agent_locations,4) %num agents
                      dist = sqrt(sum( ([agent_locations(trial,1,counter,ag,1), agent_locations(trial,1,counter,ag,2)] - sample_points(i,1:2)) .^ 2));% * density(sample_points(i,1:2));
                      if dist < min_dist
                          min_agent = ag;
                          min_dist = dist;
                      end
                  end
                  assert(min_agent ~= -1);
                  
                  cost_vec(counter,trial) = cost_vec(counter,trial) + min_dist * density(sample_points(i,1),sample_points(i,2));
              end
          end
      end
      
     
  end
    cost_vec = sum(cost_vec,2) ./ (used_samples * size(agent_locations,1));
end

%Assuming that agents moved from location i to i+1 in shortest path,
%avoiding obstacles as they went.
function kEnergy_vec = get_kEnergy_timeline_avoid_obstacles(agent_locations, time_vec)
    %start at second position and calc distance moved
    kEnergy_vec = zeros(size(agent_locations,3),size(agent_locations,1));% num iterations x num_trials

    for trial=1:size(agent_locations,1)
      for counter =2:size(agent_locations,3)%num iterations
          if counter > 1
            kEnergy_vec(counter,trial) = kEnergy_vec(counter-1,trial);
          end
          for agent_num=1:size(agent_locations,4) %num agents
              diff = wall_hugging_path_length (agent_locations(trial,1,counter,agent_num,1),(agent_locations(trial,1,counter,agent_num,2)), (agent_locations(trial,1,counter-1,agent_num,1)),(agent_locations(trial,1,counter-1,agent_num,2)));
              
              kEnergy_vec(counter,trial) = kEnergy_vec(counter,trial) + (diff/(time_vec(counter,trial) - time_vec(counter-1,trial)))^2;
          end
      end
  end
  %average across trials
  kEnergy_vec = sum(kEnergy_vec,2) ./ size(agent_locations,1);
end


function kEnergy_vec = get_kEnergy_timeline(agent_locations, time_vec)
    %start at second position and calc distance moved
    kEnergy_vec = zeros(size(agent_locations,3),size(agent_locations,1));% num iterations x num_trials

    for trial=1:size(agent_locations,1)
      for counter =2:size(agent_locations,3)%num iterations
        if counter > 1
        
          kEnergy_vec(counter,trial) = kEnergy_vec(counter-1,trial);
        end
          for agent_num=1:size(agent_locations,4) %num agents
              %Add euc distance moved ^2
              diff = agent_locations(trial,1,counter,agent_num,1:2) - agent_locations(trial,1,counter-1,agent_num,1:2);
              diff = sqrt(diff(1)^2 + diff(2)^2);
              kEnergy_vec(counter,trial) = kEnergy_vec(counter,trial) +  (diff/(time_vec(counter,trial) - time_vec(counter-1,trial)))^2;
          end
      end
  end
  %average across trials
  kEnergy_vec = sum(kEnergy_vec,2) ./ size(agent_locations,1);
end

function movement_vec = get_displacement_vec_avoid_obstacles(agent_locations)
    %start at second position and calc distance moved
    movement_vec = zeros(size(agent_locations,3),size(agent_locations,1));% num iterations x num_trials
    global obstacles;
    for trial=1:size(agent_locations,1)
      for counter =2:size(agent_locations,3)%num iterations
          %
          movement_vec(counter,trial) = movement_vec(counter-1,trial);
          for agent_num=1:size(agent_locations,4) %num agents
              %Add euc distance moved along shortest obstacle avoiding path
              diff = (wall_hugging_path_length((agent_locations(trial,1,counter,agent_num,1)) , (agent_locations(trial,1,counter,agent_num,2)),(agent_locations(trial,1,counter-1,agent_num,1)), (agent_locations(trial,1,counter-1,agent_num,2))));
    
              movement_vec(counter,trial) = movement_vec(counter,trial) + diff;
          end
      end
  end
  %average across trials
  movement_vec = sum(movement_vec,2) ./ size(agent_locations,   1);
end


function total = wall_hugging_path_length(cx,cy,Px,Py)
    global obstacles;
    int_points = [];
    for ob =1:size(obstacles,1)
        for vert =1:size(obstacles,2)
            vstart = obstacles(ob,vert,1:2);
            if vert == size(obstacles,2)
                vend = obstacles(ob,1,1:2);
            else
                vend = obstacles(ob,vert+1,1:2);
            end
            %Find the intersection point along trajectory of
            %agent i to its destination
            [int_x int_y] = polyxpoly([Px cx],[Py cy],[vstart(1) vend(1)],[vstart(2) vend(2)]);
            if (~isempty(int_x) || ~isempty(int_y) )
                
                int_points(1:4,size(int_points,2)+1) = [int_x(1),int_y(1),ob,vert];
                 
            end

        end
    end
    %If more than one intersection point, then the straight line distance moves through an
    %obstacle, so calc the path around the boundary


    total = 0;
    if (size(int_points,2) < 2)         
          % if no occlusion found, return geodesic distance
     total = sqrt((Px - cx)^2 + (Py-cy)^2);
     return;
    end
    for j=0:2:(size(int_points,2))-1 %compute for each pair of intersections
           
            %num of obstacle
            ob = int_points(3,1+j);
            assert(ob == int_points(3,2+j)); %assert that obstacle matches for start and end

            %%%%%%%%%CCW
            start_vert = int_points(4,1+j) + 1;
            if (start_vert > size(obstacles,2) )
              start_vert = 1;
            end

            end_vert = int_points(4,2+j);
            
            if (start_vert <= end_vert)
              vert_array = start_vert:end_vert;
            else
              vert_array = [end_vert:size(obstacles,2),1:start_vert];
            end

            %calc distances along obstacle wall from start vert to end vert
            %add dist from intersection to next vertex
            df_ccw = sqrt((obstacles(ob,start_vert,2) - int_points(2,1+j) ) ^ 2 + (obstacles(ob,start_vert,1) - int_points(1,1+j) ) ^ 2) + ...
                     sqrt((obstacles(ob,end_vert,2) - int_points(2,2+j) ) ^ 2 + (obstacles(ob,end_vert,1) - int_points(1,2+j) ) ^ 2);
            i = 1;
            for v=vert_array
              if i == 1
                prev = v;
                i = 2;
                continue;
              end
              df_ccw = df_ccw + euc_dist(obstacles(ob,v,:),obstacles(ob,prev,:));
              prev = v;
            end



            %%%%%%%%%CW
            start_vert = int_points(4,1+j);
            end_vert = int_points(4,2+j) + 1;
            if (end_vert > size(obstacles,2))
              end_vert = 1;
            end

            if (start_vert >= end_vert)
              vert_array = fliplr(end_vert:start_vert);
            else
              vert_array = fliplr([end_vert:size(obstacles,2),1:start_vert]);
            end

            %calc distances along obstacle wall from start vert to end vert
            %add dist from intersection to next vertex
            df_cw = sqrt((obstacles(ob,start_vert,2) - int_points(2,1+j) ) ^ 2 + (obstacles(ob,start_vert,1) - int_points(1,1+j) ) ^ 2) + ...
                     sqrt((obstacles(ob,end_vert,2) - int_points(2,2+j) ) ^ 2 + (obstacles(ob,end_vert,1) - int_points(1,2+j) ) ^ 2);
            i = 1;
            for v=vert_array
              if i == 1
                prev = v;
                i = 2;
                continue;
              end
              df_cw = df_cw + euc_dist(obstacles(ob,v,:),obstacles(ob,prev,:));
              prev = v;
            end
    
        %Get min
        if j ==0
            startx = Px;
            starty = Py;
            endx = int_points(1,2);
            endy = int_points(2,2);
        else
            startx = int_points(1,j); %from last intersection from previous obstacle
            starty = int_points(2,j); %from last intersection of prev obstacle
            endx = int_points(1,2+j);
            endy = int_points(2,2+j);   
        end
            if (j == max(0:2:(size(int_points,2)/2))) %if this is the final obstacle, then use the target as the final point
                total = total + min([df_cw,df_ccw]) + euc_dist([startx,starty]',int_points(1:2,1+j)) + euc_dist([cx,cy]',int_points(1:2,2+j));
                return;
            else
                total = total + min([df_cw,df_ccw]) + euc_dist([startx,starty]',int_points(1:2,1+j)) + euc_dist([endx,endy]',int_points(1:2,2+j));
            end
        end
        
    
      
end

function d = euc_dist(p1,p2)
    d = sqrt(sum( (p1 - p2) .^ 2) );
end

function movement_vec = get_displacement_vec(agent_locations)
    %start at second position and calc distance moved
    movement_vec = zeros(size(agent_locations,3),size(agent_locations,1));% num iterations x num_trials

    for trial=1:size(agent_locations,1)
      for counter =2:size(agent_locations,3)%num iterations
          %
          movement_vec(counter,trial) = movement_vec(counter-1,trial);
          for agent_num=1:size(agent_locations,4) %num agents
              %Add euc distance moved
              
              diff = agent_locations(trial,1,counter,agent_num,1:2) - agent_locations(trial,1,counter-1,agent_num,1:2);
              movement_vec(counter,trial) = movement_vec(counter,trial) + sqrt(diff(1)^2 + diff(2)^2);
          end
      end
  end
  %average across trials
  movement_vec = sum(movement_vec,2) ./ size(agent_locations,1);
end

function final_cost = get_final_cost(agent_locations,obstacles,NUM_SAMPLES)
%arg: 1 x 1 x num_iterations x num_agents x 2
	xrange = 30;
	yrange = 30;
    
  if min(agent_locations == 0) == 1
      final_cost = 0;
      return;
  end
        
	sample_points(:,1) = rand(NUM_SAMPLES,1)*xrange;
	sample_points(:,2) = rand(NUM_SAMPLES,1)*yrange;
	sample_points(:,3) = zeros(NUM_SAMPLES,1);
    used_samples = NUM_SAMPLES;
	for i =1:NUM_SAMPLES
		for ob = 1:size(obstacles,1)

			if inpolygon(sample_points(i,1), sample_points(i,2),obstacles(ob,:,1),obstacles(ob,:,2))
				sample_points(i,3) = -1;
        used_samples = used_samples - 1;
				break;
			end
		end
	end

  %num trials, repeat and average cost
  final_cost=0;
  for trial = 1:size(agent_locations,1)

    for i=1:NUM_SAMPLES
        if (sample_points(i,3) == 0) %if non zero,then random point was in obstacle - dont sample
        min_dist = Inf;
        min_agent = -1;
            for ag = 1:size(agent_locations,4) %num agents
                dist = sqrt(sum( ([agent_locations(trial,1,size(agent_locations,3),ag,1), agent_locations(trial,1,size(agent_locations,3),ag,2)] - sample_points(i,1:2)) .^ 2));% * density(sample_points(i,1:2));
                if dist < min_dist
                    min_agent = ag;
                    min_dist = dist;
                end
            end
            
            final_cost = final_cost + min_dist * density(sample_points(i,1),sample_points(i,2));
         end
    end
     
  end
end

function kE= get_final_kEnergy(agent_loc)
%arg: 1 x 1 x num_iterations x num_agents x 2
  kv = get_kEnergy_timeline(agent_loc);
  kE = kv(numel(kv))
end
function disp = get_final_displacement(agent_loc)
%arg: 1 x 1 x num_iterations x num_agents x 2
    disp_vec = get_displacement_vec(agent_loc);
    disp = disp_vec(numel(disp_vec));
end
function kE= get_final_kEnergy_avoid_obstacles(agent_loc)
%arg: 1 x 1 x num_iterations x num_agents x 2
  kv = get_kEnergy_timeline_avoid_obstacles(agent_loc);
  kE = kv(numel(kv))
end
function disp = get_final_displacement_avoid_obstacles(agent_loc)
%arg: 1 x 1 x num_iterations x num_agents x 2
    disp_vec = get_displacement_vec_avoid_obstacles(agent_loc);
    disp = disp_vec(numel(disp_vec));
end
