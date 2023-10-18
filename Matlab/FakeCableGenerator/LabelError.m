score = [2.0 4.8 9.5 10;
         3.3 5.2 9.9 10;
         1.0 3.3 10.0 10;
         4.3 4.0 9.1 10;
         1.5 7.8 10.0 10;
         2.9 6.6 9.7 10;
         1.0 5.5 8.9 10;
         2.8 8.9 8.5 10;
         0.0 2.0 9.2 10;
         3.5 8.1 9.9 10];
score = score/10;
score_static = [5 8 10;
                5 9 9;
                7 9 10;
                4 8 10;
                6 8 9;
                5 9 9;
                5 7 8;
                6 8 9;];
score_static = score_static/10;
t = linspace(0,20,200);

ax = axes;
lw = 3;
spacing = [1 2 3];
mean_move = mean(score(:,1:3));
std_move = std(score(:,1:3));

mean_static = mean(score_static);
std_static = std(score_static);


errorbar(ax, spacing, mean_static, std_static,'LineStyle',"-", ...
    'Color',"#F7903D",...
        'Marker','o',...
        "LineWidth",lw,...
        "MarkerSize",10,...
    "MarkerEdgeColor","#F7903D","MarkerFaceColor",[0.95 0.65 0.60])

hold(ax,"on")
errorbar(ax, spacing, mean_move, std_move,"-s", ...
    'Color','#4D85BD',...
    "LineWidth",lw,...
    "MarkerSize",10,...
    "MarkerEdgeColor",'#4D85BD',"MarkerFaceColor",[0.65 0.85 0.90])
% axis(ax,"padded")



ax.XLim = [0.5 3.5];
ax.XTick = [1 2 3];
ax.XTickLabel = ["No Labeling", "Naive Labeling", "PCLG Labeling"];
ax.LineWidth = 1;
ax.FontSize = 16;
ylabel(ax,"Normalized Match Score");
legend(ax,["Static","Moving"],'Box','off','FontSize',18,'Location','southeast')

% ax.YTick = [];
hold(ax,"off")