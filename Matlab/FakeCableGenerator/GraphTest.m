A = zeros(16);

for i=1:1:16
    for j=i:1:16
        if norm(marker(:,i)-marker(:,j)) <= 1.5*L
            A(i,j) = norm(marker(:,i)-marker(:,j));
            A(j,i) = A(i,j);
        else
%             A(i,j) = inf;
%             A(j,i) = inf;
        end

    end
end

g = graph(A);
ax_1 = subplot(1,3,3);
ax_2 = subplot(1,3,2);
ax_3 = subplot(1,3,1);


hold(ax_1);
hold(ax_2);



box(ax_1,"on")
box(ax_2,"on")
% ax_1.XAxis.Visible='off';
% ax_2.XAxis.Visible='off';
% ax_1.YAxis.Visible='off';
% ax_2.YAxis.Visible='off';



plot(ax_1, marker(1, :), marker(2, :), 'bo','MarkerSize',15, 'DisplayName','none',"LineWidth",2)
plot(ax_1, splineXY(1, 1:marker_idx(j)), splineXY(2, 1:marker_idx(j)),...
    'r', 'LineWidth', 2, 'MarkerSize', 15,...
    'DisplayName','Original Cable');
% legend(ax_1,"show")

plot(ax_2, g,'XData',marker(1,:),'YData',marker(2,:),...
    'MarkerSize', 15, 'LineWidth', 2, ...
    'EdgeColor', 'b', 'DisplayName','Graph Edges',...
    "NodeFontSize",18);
% plot(ax_2, x, y,...
%     'r', 'LineWidth',0.5, 'Marker','none',...
%     'DisplayName', 'True Shape of the Object');
% title("2D Graph Visualization of the Captured Cable")

plot(ax_3, marker(1, :), marker(2, :), 'bo','DisplayName','Retro-reflective Markers','MarkerSize',15,...
    "LineWidth",2)

xlim(ax_1,[0 10]);
xlim(ax_2,[0 10]);
xlim(ax_3,[0 10]);
ylim(ax_1,[0 12]);
ylim(ax_2,[0 12]);
ylim(ax_3,[0 12]);

for ax = [ax_1, ax_2, ax_3]
    xlim(ax,[0 10]);
    ylim(ax,[0 12]);
    ax.FontSize = 18;
    ax.LineWidth = 2;
    box(ax,"off")
    axis(ax,"off")
%     legend(ax, 'Box','off')
end


% legend(ax_2,"show")
% legend(ax_1,['Original Shape of LDO' , ''])
% ax_1.Legend.String = ['', 'Original Shape of LDO' ];


