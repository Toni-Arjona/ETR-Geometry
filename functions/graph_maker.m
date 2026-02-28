function graph_maker(xvals,yvals,name,x_name,y_name,color)
    f = figure("Name",name);
    hold on

    title(name);
    plot(xvals, yvals, color);
    xlabel(x_name);
    ylabel(y_name);
    grid on
end

