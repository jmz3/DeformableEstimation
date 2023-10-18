x = 1:10;
y = [1, 3, 5, 7, 9, 11, 13, 15, 17, 19];

% Plot each line
figure;
hold on;
for i = 1:10
    plot(x, y(i)*ones(size(x)));
end

% Add legend for fourth line
legend('test Line');