classdef Drone3DAnimator
    % Drone3DAnimator - 3D Visualization for drone trajectory
    % Usage:
    %   anim = visual.Drone3DAnimator();
    %   anim.animate(t, x, y, z);

    properties
        fig
        bodyLength = 0.3;   % chiều dài thân drone
        armLength = 0.25;   % chiều dài cánh
        trailLength = 200;  % số điểm hiển thị trong quỹ đạo
    end

    methods
        function obj = Drone3DAnimator()
            obj.fig = figure('Name','Drone 3D Visualization','Color','w');
            axis equal
            grid on
            xlabel('X (m)');
            ylabel('Y (m)');
            zlabel('Z (m)');
            view(45,25);
            hold on;
            title('Drone Flight Animation');
        end

        function animate(obj, t, x, y, z)
            % Khởi tạo drone
            hBody = plot3(NaN,NaN,NaN,'ko','MarkerFaceColor','b','MarkerSize',6);
            hArms = plot3(NaN,NaN,NaN,'r-','LineWidth',2);
            hTrail = plot3(NaN,NaN,NaN,'b--','LineWidth',1);

            for k = 1:length(t)
                % Cập nhật vị trí thân
                set(hBody,'XData',x(k),'YData',y(k),'ZData',z(k));

                % Cập nhật cánh tay drone (vẽ dấu + hoặc x)
                armX = [x(k)-obj.armLength, x(k)+obj.armLength, NaN, x(k), x(k), NaN];
                armY = [y(k), y(k), NaN, y(k)-obj.armLength, y(k)+obj.armLength, NaN];
                armZ = [z(k), z(k), NaN, z(k), z(k), NaN];
                set(hArms, 'XData', armX, 'YData', armY, 'ZData', armZ);

                % Hiển thị quỹ đạo bay
                idx0 = max(1, k - obj.trailLength);
                set(hTrail, 'XData', x(idx0:k), 'YData', y(idx0:k), 'ZData', z(idx0:k));

                % Giới hạn trục cho đẹp
                axis([min(x)-1 max(x)+1 min(y)-1 max(y)+1 min(z)-1 max(z)+1]);

                drawnow limitrate nocallbacks;
            end

            disp('✅ Drone animation complete.');
        end
    end
end
