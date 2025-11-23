function plot3dline(ax, p1,p2, color)
    plot3(ax, [p1.x p2.x], [p1.y p2.y], [p1.z p2.z], color, 'LineWidth',2);
    p1.plot3d(ax, color);
    p2.plot3d(ax, color);
end

