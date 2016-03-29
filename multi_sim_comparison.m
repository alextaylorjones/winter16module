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
function agent_loc = multi_sim_comparison(obstacle_configuration, simulation_type, density_function_type,density_function_params, num_agents, num_iterations, startingLoc)

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
obstacles = get_obstacle_set();

%Split based on simulation type
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
         A = optimal_coverage_grid(num_iterations,show_plot,num_agents,obstacles,seed,startingLoc);
        agent_loc(cur_trial,4,:,:,:)= A;
        %Run non adaptive ladybug algorithm

        agent_loc(cur_trial,5,:,:,:) = Non_adaptive_ladybug_coverage(num_iterations,show_plot,num_agents,obstacles,seed,control_gain_lb, exploration_gain,startingLoc);

    end
        %Plot metric results
    
    v = 1:num_iterations;
    for trail=1:trials
      print('computing stats on trial');
      trial
      NUM_SAMPLES = 5000;
      cost_lloyd  = get_cost_timeline(agent_loc(trial,1,:,:,:),obstacles,NUM_SAMPLES);
      cost_approx  = get_cost_timeline(agent_loc(trial,2,:,:,:),obstacles,NUM_SAMPLES);
      cost_combined = get_cost_timeline(agent_loc(trial,3,:,:,:),obstacles,NUM_SAMPLES);
      cost_optimal  = get_cost_timeline(agent_loc(trial,4,:,:,:),obstacles,NUM_SAMPLES);
      cost_ladybug  = get_cost_timeline(agent_loc(trial,5,:,:,:),obstacles,NUM_SAMPLES);

      figure(1);
      plot(v,cost_lloyd,'o',v,cost_approx,'+',v, cost_combined,'x',v,cost_optimal,'^',v,cost_ladybug,'s');
      legend('Lloyd','Discrete Apx.','Local Path','Annealing','Ladybug')
      title('Coverage Control Sampled (5000) Cost');
      xlabel('Iterations');
      ylabel('Cost');

      disp_lloyd = get_displacement_vec(agent_loc(trial,1,:,:,:));
      disp_approx = get_displacement_vec(agent_loc(trial,2,:,:,:));
      disp_combined = get_displacement_vec(agent_loc(trial,3,:,:,:));
      disp_optimal = get_displacement_vec(agent_loc(trial,4,:,:,:));
      disp_ladybug = get_displacement_vec(agent_loc(trial,5,:,:,:));


      figure(2);
      plot(v,disp_lloyd,'o',v,disp_approx,'+',v, disp_combined,'x',v,disp_optimal,'^',v,disp_ladybug,'s');
      legend('Lloyd','Discrete Apx.','Local Path','Annealing','Ladybug')
      title('Coverage Control Displacement');
      xlabel('Iterations');
      ylabel('Total Displacement');

      kEnergy_lloyd  = get_kEnergy_timeline(agent_loc(trial,1,:,:,:));
      kEnergy_approx  = get_kEnergy_timeline(agent_loc(trial,2,:,:,:));
      kEnergy_combined = get_kEnergy_timeline(agent_loc(trial,3,:,:,:));
      kEnergy_optimal  = get_kEnergy_timeline(agent_loc(trial,4,:,:,:));
      kEnergy_ladybug  = get_kEnergy_timeline(agent_loc(trial,5,:,:,:));


      figure(3);
      plot(v,kEnergy_lloyd,'o',v,kEnergy_approx,'+',v, kEnergy_combined,'x',v,kEnergy_optimal,'^',v,kEnergy_ladybug,'s');
      legend('Lloyd','Discrete Apx.','Local Path','Annealing','Ladybug')
      title('Coverage Control Sampled (1000) Kinetic Energy');
      xlabel('Iterations');
      ylabel('Kinetic Energy');
    end

end

end

%Add an elseif clause to add new obstacles
%Format of obstacles are M x N x N, whre M is number of obstacles, N is
%vertices of obstacles.
%We assume boundary is 30x30
function obstacles = get_obstacle_set()
    obstacles = [];
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
        obstacles(size(obstacles,1)+1,:,:) = [5,25;25,5;15,10;5,25;5,25;5,25;5,25;5,25;5,25];
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
      spiral_ccw = (fliplr(spiral'))';
      %offset
      for i =1:size(spiral_ccw)
        spiral_ccw(i,:) = spiral_ccw(i,:) + [dx/3,-dy/3];
      end

      obstacles(size(obstacles,1)+1,:,:) = [spiral;spiral_ccw];
        
    end
    
end

%Use sampling to detesrmine global cost
function cost_vec = get_cost_timeline(agent_locations,obstacles, NUM_SAMPLES)
	xrange = 30;
	yrange = 30;
    
    if min(agent_locations == 0) == 1
        cost_vec = zeros(size(agent_locations,3));%num iterations
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

	cost_vec = zeros(size(agent_locations,3),1);%num iterations
	for counter =1:size(agent_locations,3)%num iterations
        for i=1:NUM_SAMPLES
            if (sample_points(i,3) == 0)
            min_dist = Inf;
            min_agent = -1;
                for ag = 1:size(agent_locations,4) %num agents
                    dist = sqrt(sum( ([agent_locations(1,1,counter,ag,1), agent_locations(1,1,counter,ag,2)] - sample_points(i,1:2)) .^ 2));% * density(sample_points(i,1:2));
                    if dist < min_dist
                        min_agent = ag;
                        min_dist = dist;
                    end
                end
                assert(min_agent ~= -1);
                
                cost_vec(counter,1) = cost_vec(counter,1) + min_dist * density(sample_points(i,1),sample_points(i,2));
            end
        end
    end
    
    cost_vec = cost_vec ./ used_samples;

end

function kEnergy_vec = get_kEnergy_timeline(agent_locations)
    %start at second position and calc distance moved
    kEnergy_vec = zeros(size(agent_locations,3),1);%num iterations

    for counter =2:size(agent_locations,3)
        %
        kEnergy_vec(counter,1) = kEnergy_vec(counter-1,1);
        for agent_num=1:size(agent_locations,4) %num agents
            %Add euc distance moved
            kEnergy_vec(counter,1) = kEnergy_vec(counter,1) + sum((agent_locations(1,1,counter,agent_num,1:2) - agent_locations(1,1,counter-1,agent_num,1:2)).^ 2 );
        end
    end
end


function movement_vec = get_displacement_vec(agent_locations)
    %start at second position and calc distance moved
    movement_vec = zeros(size(agent_locations,3),1);% num iterations

    for counter =2:size(agent_locations,3)%num iterations
        %
        movement_vec(counter,1) = movement_vec(counter-1,1);
        for agent_num=1:size(agent_locations,4) %num agents
            %Add euc distance moved
            movement_vec(counter,1) = movement_vec(counter,1) + sqrt(sum(agent_locations(1,1,counter,agent_num,1:2) - agent_locations(1,1,counter-1,agent_num,1:2).^ 2 ));
        end
    end
end

