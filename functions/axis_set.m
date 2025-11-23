function axis_set(ax)
    axes(ax);
    hold(ax, "on");
    grid(ax, "on");
    xlabel(ax, "X");
    ylabel(ax, "Y");
    zlabel(ax, "Z");
    axis(ax, "equal");
    axis(ax, [-500,2000,-800,800,-50,700]);
    view(ax, 3);
    datacursormode(ax, "on");
    rotate3d(ax, "on");
end

