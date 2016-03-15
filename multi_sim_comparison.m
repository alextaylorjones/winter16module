%obstacle configuration - numerical id for obstacle configurations described in get_obstacle_set
% density_function_type - UNIMPLEMENTED
%density_function_params - UNIMPLEMENTED
% test
%num_agents - as described, integer input
%num_iterations - number of iterations to compute for each simulation
function agent_loc = multi_sim_comparison(obstacle_configuration, density_function_type,density_function_params, num_agents,num_iterations, agent_loc)

%random seed
seed = 16;

%Setup obstacles
obstacles = get_obstacle_set(obstacle_configuration);

%For now show plots of algorithm runs
show_plot = true;

%setup density function
global density_params ;
global density_type;
density_type = density_function_type;
density_params = density_function_params;


%If agent_loc is not passed as argument
if nargin < 6
    
    %Run the 4 algorithms using params specified in function input
    agent_loc = zeros(4,num_iterations,num_agents,2);

    %Set control gain
    control_gain = 0.5;

    %Run lloyd style algorithm (use degenerate
    %non_adaptive_ladybug_coverage
    % agent_loc(1,:,:,:) = loydsAlgorithm_nonuniform(num_iterations,show_plot,num_agents,obstacles,seed,control_gain,0);

    agent_loc(1,:,:,:) = Non_adaptive_ladybug_coverage(num_iterations,show_plot,num_agents,obstacles,seed,control_gain,0);

    %Run approximation via search based grid algorithm
    agent_loc(2,:,:,:) = approximation_discrete_nonconvex_coverage(num_iterations,show_plot,num_agents,obstacles,seed);

    %Run combined tangentbug and lloyd
    loop_gain = 3;
    max_step = 0.25;
    %agent_loc(3,:,:,:) = combined(num_iterations,show_plot,num_agents,obstacles,seed,control_gain,loop_gain,max_step);

    %Run optimal annealing, algorithm
    %agent_loc(4,:,:,:) = optimal_coverage_grid(num_iterations,show_plot,num_agents,obstacles,seed);
    
    %Run non adaptive ladybug algorithm
    control_gain_lb = 0.5;
    exploration_gain = 0.3;
    agent_loc(5,:,:,:) = Non_adaptive_ladybug_coverage(num_iterations,show_plot,num_agents,obstacles,seed,control_gain_lb, exploration_gain);
    
    
end

%Plot cost functions

v = 1:num_iterations;
NUM_SAMPLES = 1000;
cost_lloyd  = get_cost_timeline(agent_loc(1,:,:,:),obstacles,NUM_SAMPLES);
cost_approx  = get_cost_timeline(agent_loc(2,:,:,:),obstacles,NUM_SAMPLES);
cost_combined = get_cost_timeline(agent_loc(3,:,:,:),obstacles,NUM_SAMPLES);
cost_optimal  = get_cost_timeline(agent_loc(4,:,:,:),obstacles,NUM_SAMPLES);
cost_ladybug  = get_cost_timeline(agent_loc(5,:,:,:),obstacles,NUM_SAMPLES);

NUM_SAMPLES = 1000;
kEnergy_lloyd  = get_kEnergy_timeline(agent_loc(1,:,:,:));
kEnergy_approx  = get_kEnergy_timeline(agent_loc(2,:,:,:));
kEnergy_combined = get_kEnergy_timeline(agent_loc(3,:,:,:));
kEnergy_optimal  = get_kEnergy_timeline(agent_loc(4,:,:,:));
kEnergy_ladybug  = get_kEnergy_timeline(agent_loc(5,:,:,:));


figure(2);
plot(v,cost_lloyd,'o',v,cost_approx,'+',v, cost_combined,'x',v,cost_optimal,'^',v,cost_ladybug,'s');
legend('Lloyd','Discrete Apx.','Local Path','Annealing','Ladybug')
title('Coverage Control Sampled (1000) Cost');
xlabel('Iterations');
ylabel('Cost');


NUM_SAMPLES = 5000;

cost_lloyd = get_cost_timeline(agent_loc(1,:,:,:),obstacles,NUM_SAMPLES);
cost_approx = get_cost_timeline(agent_loc(2,:,:,:),obstacles,NUM_SAMPLES);
cost_combined = get_cost_timeline(agent_loc(3,:,:,:),obstacles,NUM_SAMPLES);
cost_optimal  = get_cost_timeline(agent_loc(4,:,:,:),obstacles,NUM_SAMPLES);
cost_ladybug  = get_cost_timeline(agent_loc(5,:,:,:),obstacles,NUM_SAMPLES);



figure(3);
plot(v,cost_lloyd,'o',v,cost_approx,'+',v, cost_combined,'x',v,cost_optimal,'^',v,cost_ladybug,'s');
legend('Lloyd','Discrete Apx.','Local Path','Annealing','Ladybug')
title('Coverage Control Sampled (5000) Cost');
xlabel('Iterations');
ylabel('Cost');

disp_lloyd = get_displacement_vec(agent_loc(1,:,:,:));
disp_approx = get_displacement_vec(agent_loc(2,:,:,:));
disp_combined = get_displacement_vec(agent_loc(3,:,:,:));
disp_optimal = get_displacement_vec(agent_loc(4,:,:,:));
disp_ladybug = get_displacement_vec(agent_loc(5,:,:,:));


figure(4);
plot(v,disp_lloyd,'o',v,disp_approx,'+',v, disp_combined,'x',v,disp_optimal,'^',v,disp_ladybug,'s');
legend('Lloyd','Discrete Apx.','Local Path','Annealing','Ladybug')
title('Coverage Control Displacement');
xlabel('Iterations');
ylabel('Total Displacement');

figure(5);
plot(v,kEnergy_lloyd,'o',v,kEnergy_approx,'+',v, kEnergy_combined,'x',v,kEnergy_optimal,'^',v,kEnergy_ladybug,'s');
legend('Lloyd','Discrete Apx.','Local Path','Annealing','Ladybug')
title('Coverage Control Sampled (1000) Kinetic Energy');
xlabel('Iterations');
ylabel('Kinetic Energy');
end

%Add an elseif clause to add new obstacles
%Format of obstacles are M x N x N, whre M is number of obstacles, N is
%vertices of obstacles.
%We assume boundary is 30x30
function obstacles = get_obstacle_set(ob_config)
    obstacles = [];
    if (ob_config == 1)
        obstacles = [];
    end
    if mod(ob_config,2) == 0
        %one saw shape osbtacle in bottom left corner
        obstacles(size(obstacles,1)+1,:,:) = [0,0;10,0;10,10;2,7;7,4;0,0];
    end
    if mod(ob_config,3) == 0
        % 
          obstacles(size(obstacles,1)+1,:,:) = [0,0;20,0;20,10;3,10;3,6;7,6;7,4;3,4;0,0];
    end
    if mod(ob_config,5) == 0
        obstacles(size(obstacles,1)+1,:,:) = [5,25;25,5;15,10;5,25;5,25;5,25;5,25;5,25;5,25];
    end
    if mod(ob_config,7) == 0
        obstacles(size(obstacles,1)+1,:,:) = [15,5;20,10;20,20;15,25;10,25;5,20;5,15;10,10;15,15;15,20;10,20;10,17;13,14;10,12;7,15;10,23;15,23;18,18;18,10;15,5 ];
    end
    if mode(ob_config,11) == 0
        %obstacles(size(obstacles,1)+1,:,:) = (0.034)*[0,0; 60,0; 60,35; 30,35; 30,45; 70,45; 70,5; 100,5; 70, 100; 70,55; 30, 55; 30, 65; 60,65; 60,95; 5,95; 5,65; 20,65; 20,55; 5,55; 5,45; 20,45; 20,35; 5,35; 0,0 ]
        obstacles(size(obstacles,1)+1,:,:) = 0.3*[0,0; 60,0; 60,35; 30,35; 30,45; 70,45; 70,5; 95,5; 70, 95; 70,55; 30, 55; 30, 65; 60,65; 60,95; 5,95; 5,65; 20,65; 20,55; 5,55; 5,45; 20,45; 20,35; 5,35; 0,0 ]
    end
    
end

%Use sampling to detesrmine global cost
function cost_vec = get_cost_timeline(agent_locations,obstacles, NUM_SAMPLES)
	xrange = 30;
	yrange = 30;
    
    if min(agent_locations == 0) == 1
        cost_vec = zeros(size(agent_locations,2));
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

	cost_vec = zeros(size(agent_locations,2),1);
	for counter =1:size(agent_locations,2)
        for i=1:NUM_SAMPLES
            if (sample_points(i,3) == 0)
            min_dist = Inf;
            min_agent = -1;
                for ag = 1:size(agent_locations,3)
                    dist = sqrt(sum( ([agent_locations(1,counter,ag,1) agent_locations(1,counter,ag,2)] - sample_points(i,1:2)) .^ 2));% * density(sample_points(i,1:2));
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
    kEnergy_vec = zeros(size(agent_locations,2),1);

    for counter =2:size(agent_locations,2)
        %
        kEnergy_vec(counter,1) = kEnergy_vec(counter-1,1);
        for agent_num=1:size(agent_locations,3) %num agents
            %Add euc distance moved
            kEnergy_vec(counter,1) = (kEnergy_vec(counter,1) + sqrt(sum((agent_locations(1,counter,agent_num,1:2) - agent_locations(1,counter-1,agent_num,1:2)).^ 2 )))^2;
        end
    end
end

function movement_vec = get_displacement_vec(agent_locations)
    %start at second position and calc distance moved
    movement_vec = zeros(size(agent_locations,2),1);

    for counter =2:size(agent_locations,2)
        %
        movement_vec(counter,1) = movement_vec(counter-1,1);
        for agent_num=1:size(agent_locations,3) %num agents
            %Add euc distance moved
            movement_vec(counter,1) = movement_vec(counter,1) + sqrt(sum((agent_locations(1,counter,agent_num,1:2) - agent_locations(1,counter-1,agent_num,1:2)).^ 2 ));
        end
    end
end

