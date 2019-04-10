classdef RadarStation
    properties
        pos
        range_std
        elev_angle_std
    end
    methods
        function obj = RadarStation(pos,range_std,elev_angle_std)
            obj.pos = pos;
            obj.range_std = range_std;
            obj.elev_angle_std = elev_angle_std;
        end
            
        function [rng, brg] = reading_of(obj, ac_pos)
            % Returns (range, elevation angle) to aircraft.
            % Elevation angle is in radians.
            diff = minus(ac_pos, obj.pos);
            rng = norm(diff);
            brg = atan2(diff(2), diff(1));
        end

        function [rng, brg] = noisy_reading(obj, ac_pos)
            % Compute range and elevation angle to aircraft with
            % simulated noise
            [rng, brg] = obj.reading_of(ac_pos);
            rng = rng + randn() * obj.range_std;
            brg = brg + randn() * obj.elev_angle_std;
        end
    end
end