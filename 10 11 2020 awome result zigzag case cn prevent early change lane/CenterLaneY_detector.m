%this func detects the center lane y coordinate of lane that the ego vehicle is moving on (Omid)
function CenterLaneY = CenterLaneY_detector(egoY)

%  Inputs
% ======
% egoY     : y coordinate of the ego vehicle
global road_up_lim road_low_lim Lane_size
m=0;
Num_Lanes= (road_up_lim-road_low_lim)/Lane_size;

        for i = 1:Num_Lanes
            
            if road_low_lim+(i-1)*Lane_size<=egoY & egoY<road_low_lim+i*Lane_size

                CenterLaneY= road_low_lim+(2*i-1)*Lane_size/2;
                break;
            else
                m=m+1;
            end
                  if m== Num_Lanes
                     CenterLaneY= 0;
                  end
        end
end
