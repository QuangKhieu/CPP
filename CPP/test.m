load landmark
plot(landmark_x2, landmark_y2, 'x')
hold on
grid on
plot(landmark_x1, landmark_y1, 'x');

x_plot =[];
y_plot = [];
for i=1:2:20
     
   x_plot = [x_plot, landmark_x1(i+17)];
   y_plot = [y_plot, landmark_y1(i+17)];
   
   x_plot = [x_plot, landmark_x2(i+5)];
   y_plot = [y_plot, landmark_y2(i+5)];
   
   x_plot = [x_plot, landmark_x2(i+1+5)];
   y_plot = [y_plot, landmark_y2(i+1+5)];
   
   x_plot = [x_plot, landmark_x1(i+1+17)];
   y_plot = [y_plot, landmark_y1(i+1+17)];
   
   
   
end

plot(x_plot, y_plot)