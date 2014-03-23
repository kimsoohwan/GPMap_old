function comparison(action, data, data_MKL, data_MKL_OpenMP, data_OpenMP, data_Matlab)

h = figure('Name', action, 'Position', [100, 100, 1400, 600]);
subplot(2, 4, 1);   title('Average Time');                  draw(data,              'AvgTime');
subplot(2, 4, 2);   title('Average Time (MKL)');            draw(data_MKL,          'AvgTime');
subplot(2, 4, 3);   title('Average Time (MKL, OpenMP)');    draw(data_MKL_OpenMP,   'AvgTime');
subplot(2, 4, 4);   title('Average Time (OpenMP)');         draw(data_OpenMP,       'AvgTime');
subplot(2, 4, 5);   title('GFLPS');                         draw(data,              'GFLPS');
subplot(2, 4, 6);   title('GFLPS (MKL)');                   draw(data_MKL,          'GFLPS');
subplot(2, 4, 7);   title('GFLPS (MKL, OpenMP)');           draw(data_MKL_OpenMP,   'GFLPS');
subplot(2, 4, 8);   title('GFLPS (OpenMP)');                draw(data_OpenMP,       'GFLPS');
%print(h, '-dpng',  [action, '.png']);
%print(h, '-depsc', [action, '.eps']);
saveas(h, [action, '.fig']);

    function draw(data_Action, value)
        hold on
        if strcmp(value, 'AvgTime')
            ylabel('sec');
            plot(data_Action(:, 1), data_Action(:, 2), '-*b', 'LineWidth', 3); % Eigen
            plot(data_Action(:, 1), data_Action(:, 4), '-*r', 'LineWidth', 2); % MKL
            plot(data_Action(:, 1), data_Matlab(:, 1), '-*g', 'LineWidth', 1); % Matlab
        else
            ylabel('GFLPS');
            plot(data_Action(:, 1), data_Action(:, 3), '-*b', 'LineWidth', 3); % Eigen
            plot(data_Action(:, 1), data_Action(:, 5), '-*r', 'LineWidth', 2); % MKL
            plot(data_Action(:, 1), data_Matlab(:, 2), '-*g', 'LineWidth', 1); % Matlab    
        end   
        xlabel('N');
        legend('Eigen', 'MKL', 'Matlab', 'Location', 'NorthWest');
        set(gca,'XScale','log') 
    end

end