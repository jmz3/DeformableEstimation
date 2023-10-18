clear;
l = 1000;
for n = 1:1:4
    if n == 1
        theta = linspace(0,pi/2,l);
    elseif n==2
        theta = linspace(pi/2,pi,l);
    elseif n==3
        theta = linspace(pi,3*pi/2,l);
    elseif n==4
        theta = linspace(-pi/2,0,l);
    end
    L = 10;
    rho = L * linspace(0,5,length(theta));

    mean_rho = 2*L*cos(theta)./(pi - 2*theta);
    % if theta = pi/2 mean_rho will become inf
    % replace it by the actual length
    x = zeros(length(theta));
    y = zeros(length(theta));
    prob = zeros(length(theta));
    for i=1:1:length(theta)
        for j = 1:1:length(theta)
            x(i,j) = cos(theta(i))*rho(j);
            y(i,j) = sin(theta(i))*rho(j);
            if mean_rho(i)==inf
                mean_rho(i) = L;
            end
            prob(i,j) = normpdf(rho(j),mean_rho(i),2);
        end
    end


    %
    % for i=1:1:size(x,2)
    %     temp = normpdf(rho,mean_rho(i),1);
    %     prob(:,i) = temp;
    % end
% 
    s = surface(x,y,prob);
    s.EdgeColor = "none";
 
%     contourf(x,y,prob)
%     hold on
end
hold on
xlim([-15 15])
ylim([-7.5 15])
ax = gca;
ax.FontSize = 13;
xlabel('x (mm)');
ylabel('y (mm)');


c = winter;
colormap(jet)
colorbar

%%
% previous segment
ax = gca;
R = 12.5;
L = 10;
prev_x = zeros(1,10);
prev_y = linspace(-10,0,10);
plot(ax, prev_x, prev_y, 'LineWidth',8,'Color',"#F7903D",...
    'Marker','.',...
    'MarkerIndices',10,'MarkerSize',50,...
    'MarkerEdgeColor',"#F7903D" ,'MarkerFaceColor',"#F7903D")
hold(ax,'on')

% reference data point
theta = (-pi/2+0.7):0.7:pi/2;
rho = 2*L*cos(theta)./(pi - 2*theta);
node_x = rho.*cos(theta);
node_y = rho.*sin(theta);
plot(ax,node_x,node_y,'LineStyle','none','Marker','.','MarkerSize',50,'Color',"#F7903D")

node_x = - rho.*cos(theta);
node_y = rho.*sin(theta);
plot(ax,node_x,node_y,'LineStyle','none','Marker','.','MarkerSize',50,'Color',"#F7903D")


% Plot reference curve
for i = 1:1:length(rho)
    alpha = (2*theta(i)):0.01:pi;
    r = rho(i) / (2*cos(theta(i)));
    seg_i_x = r * cos(alpha) + r;
    seg_i_y = r * sin(alpha);
    plot(ax,seg_i_x,seg_i_y,'LineStyle','-.','LineWidth',4,...
        'Color',"#59A95A")

    seg_i_x = -seg_i_x;
    plot(ax,seg_i_x,seg_i_y,'LineStyle','-.','LineWidth',4,...
        'Color',"#59A95A")
end

% ax.FontSize = 12;
% ax.LineWidth = 1;
% xlabel(ax,'x (mm)')
% ylabel(ax,'y (mm)')
axis(ax,'equal')
ax.CameraViewAngle 

