close all;

magx = 6047;
magy = 8751;
magz = -31158;

mx = 47449;
my = 44803;
mz = 5682;

figure;
quiver3(0, 0, 0, 1, 0, 0, 'r', 'LineWidth', 3); hold on;
quiver3(0, 0, 0, 0, 1, 0, 'g', 'LineWidth', 3); hold on;
quiver3(0, 0, 0, 0, 0, 1, 'b', 'LineWidth', 3); hold on;

quiver3(0, 0, 0, magx/65535, magy/65535, magz/65535, 'm'); hold on;
quiver3(0, 0, 0, mx/65535, my/65535, mz/65535, 'c'); hold on;

xlim([-1 1]);
ylim([-1 1]);
zlim([-1 1]);
axis square;