function rval = MIT_CAOS_gen (scale, hilb) 
dialect = mavlinkdialect("common.xml");


plan = struct("fileType", "Plan", ...
              "version", 1, ...
              "groundStation", "MIT CAOS generator");

plan.geoFence = struct("circles", [], "polygons", [], "version", 2);
plan.rallyPoints = struct("points", [], "version", 2);
          
plan.mission = struct("groundSpeed", 15, ...
                      "hoverSpeed", 5, ...
                      "firmwareType", 12, ...
                      "vehicleType", 2, ...
                      "version", 2);

home_coords = [47.397995, 8.5453033, 20];

n_segments = 180;

if hilb > 0
    % Hilbert? 
    a = 1 + 1i;
    b = 1 - 1i;

    z = 0;
    order = hilb;
    for k = 1:order
        w = 1i*conj(z);
        z = [w-a; z-b; z+a; b-w]/2;
    end
    
    n_segments = size(z, 1) - 1;
end

% End Hilbert



plan.mission.items(n_segments+3) = cmd_return(); 



for k = n_segments:-1:0
    rel = [sin(k/n_segments * 2 * pi) cos(k/n_segments * 2 * pi)];
    
    % enable for Hilbert. 
    if hilb
        rel = [real(z(k+1)) imag(z(k+1))];
    end
    % end Hilbert
    
    % WGS coordinate coorection.. 
    rel = rel * [scale*0.01/2215 0 0; 0 scale*0.01084/sin(home_coords(1)*2*pi/360)/2215 0]; 
    plan.mission.items(k+2) = cmd_waypoint(NaN, home_coords+rel); 
end

plan.mission.plannedHomePosition = plan.mission.items(2).params(5:7) + [0 0 481]; 

plan.mission.items(1) = cmd_takeoff(15, NaN, plan.mission.items(2).params(5:7)); 


%  cmd_waypoint(NaN, home_coords) cmd_return()];
             
nval = num2cell(1:size(plan.mission.items,2));
[plan.mission.items.doJumpId] =  nval{:};

jsonencode(plan)

rval = plan;


function item = cmd_takeoff(pitch, yaw, lla) 

item = struct("command", enum2num(dialect,"MAV_CMD","MAV_CMD_NAV_TAKEOFF"), ...
              "frame", enum2num(dialect, "MAV_FRAME", "MAV_FRAME_GLOBAL_RELATIVE_ALT"), ...
              "autoContinue", true, ...
              "type", "SimpleItem", ...
              "AMSLAltAboveTerrain", NaN, ...
              "Altitude", lla(3), ...
              "AltitudeMode", 1, ... 
              "params", [pitch 0 0 yaw lla] ...
    );
end

function item = cmd_waypoint(yaw, lla) 

item = struct("command", enum2num(dialect,"MAV_CMD","MAV_CMD_NAV_WAYPOINT"), ...
              "frame", enum2num(dialect, "MAV_FRAME", "MAV_FRAME_GLOBAL_RELATIVE_ALT"), ...
              "autoContinue", true, ...
              "type", "SimpleItem", ...
              "AMSLAltAboveTerrain", NaN, ...
              "Altitude", lla(3), ...
              "AltitudeMode", 1, ... 
              "params", [0 0 0 yaw lla] ...
    );
end

function item = cmd_return() 
item = struct("command", enum2num(dialect,"MAV_CMD","MAV_CMD_NAV_RETURN_TO_LAUNCH"), ...
              "frame", enum2num(dialect, "MAV_FRAME", "MAV_FRAME_MISSION"), ...
              "autoContinue", true, ...
              "type", "SimpleItem", ...
              "AMSLAltAboveTerrain", NaN, ...
              "Altitude", NaN, ...
              "AltitudeMode", NaN, ... 
              "params", [0 0 0 0 0 0 0] ...
    );
end

end