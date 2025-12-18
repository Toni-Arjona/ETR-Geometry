function graph_maker_double(xvals,yvals1,yvals2,given_title,name1,name2,x_name,y_name,color1,color2)
    f = figure("Name",given_title);
    hold on

    title(given_title);
    h1 = plot(xvals, yvals1,color1, 'DisplayName',name1);
    h2 = plot(xvals, yvals2,color2, 'DisplayName',name2);
    xlabel(x_name);
    ylabel(y_name);
    legend([h1,h2], 'Location','best');
    grid on

    hold off
end

