function triangular_matrix_solve(REP)

% N, avg time Eigen, GFLOPS Eigen, avg time MKL, GFLPS MKL
action = 'triangular_solve'
data            = load([action, '.txt']);
data_MKL      	= load([action, '_MKL.txt']);           % Eigen uses MKL
data_MKL_OpenMP = load([action, '_MKL_OpenMP.txt']);    % Eigen uses MKL, repeats are parallelized with OpenMP
data_OpenMP     = load([action, '_OpenMP.txt']);        % repeats are parallelized with OpenMP

num_test = size(data, 1);
data_Matlab = zeros(num_test, 2);

% bench for Matlab
for test = 1:num_test
    % size
    N = data(test, 1);

    % FLOP
    FLOP = 0;
    for i = 0:N-1
        FLOP = FLOP + 2*i+1;
    end
    FLOP = FLOP * REP;

    % initialization
    L = rand(N, N);
    b = rand(N, 1);
    x = zeros(N, 1);
    opts.UT = true;

    %tic;
    time = cputime;
    for i = 1:REP
        x = linsolve(L,b,opts);
    end
    elapsed_time = cputime - time;
    %elapsed_time = toc;
    
    % time
    data_Matlab(test, 1) = elapsed_time/REP;
    data_Matlab(test, 2) = 10^-9*FLOP/elapsed_time;
end

comparison(action, data, data_MKL, data_MKL_OpenMP, data_OpenMP, data_Matlab);