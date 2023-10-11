figure;

d = zeros(1,length(splineXY)-1);

for k = 2:1:length(splineXY)
    d(k-1) = sqrt( (x(k)-x(k-1))^2 + (y(k)-y(k-1))^2 ) ;
end

total_length = sum(d);
segments = 15;
L = total_length/segments;
eps = 4;
marker = zeros(2,segments);
j = 1;
sum_prev = 0;
marker_idx = [1];
for i = 1:1:length(d)
    
    temp_distance = sum(d(marker_idx(j):i));
    if abs(temp_distance - L) < 0.01
        marker(:,j) = splineXY(:,i+1);
        marker_idx = [marker_idx , i+1];
        j = j+1;
        
        disp("find marker, the distance is " + num2str(temp_distance,5))
    end
end
% disp(marker)
marker = [ [splineXY(:,1)] marker];
plot(marker(1, :), marker(2, :), 'ro',...
    splineXY(1, 1:marker_idx(j)), splineXY(2, 1:marker_idx(j)), 'b', 'LineWidth', 2, 'MarkerSize', 6);
axis equal
% figure
% plot(marker(1, :), marker(2, :), 'ro')
% axis equal