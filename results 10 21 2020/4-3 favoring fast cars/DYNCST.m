function [cx,cu,cxx,cux,cuu,fx,fu] = DYNCST(X, U, ref_traj, obstacle, ...
                                        use_prediction, stopping, adaptive)
% DYNCST: calculate all necessary first and second derivatives
%
% all derivatives are matrices
%
% input
% ======
% X:        state, size(X) = XDIM * (N+1)
% U:        control sequence, size(U) = UDIM * (N)
% ref_traj: reference trajectory
% obstacle: obstacle class
% use_prediction: 0 to use deterministic model, and 1 to use prediction
%
% output
% =====
% cx:       first derivative of cost fcn w.r.t state X
% cu:       first derivative of cost fcn w.r.t control u
% cxx:      second derivative of cost fcn w.r.t state X
% cux:      second derivative of cost fcn w.r.t control u and state X
% cuu:      second derivative of cost fcn w.r.t control u
% fx:       first derivative of car dynamics w.r.t state X
% fu:       first derivative of car dynamics w.r.t state u

% load global parameters
global X_DIM U_DIM NUM_CTRL L dt
global a_dot_max a_dot_min v_dot_max v_dot_min delta_max delta_min

% load cost_weights
global w_ref w_vel w_jerk w_acc w_del w_end_ref w_end_acc w_end_vel
global q1_back q2_back
global q1_front q2_front
global q1_stop q2_stop
global q1_road q2_road
global q1_CenterLane q2_CenterLane %added by Omid
global v_min q1_min_vel q2_min_vel

global q1_jerk q2_jerk q1_acc q2_acc q1_del q2_del
w_ref = zeros(NUM_CTRL,1);
w_vel = zeros(NUM_CTRL,1);
w_acc = zeros(NUM_CTRL,1);
w_jerk = zeros(NUM_CTRL,1);
w_del = zeros(NUM_CTRL,1);
if (adaptive == 0)
    global w_ref w_vel
else
    global a_vel b_vel a_ref b_ref
end

% stop sign location
global x_stop

% road upper and lower limits
global road_up_lim road_low_lim Lane_size
global vref aref vref_road

% required for weight calculation in DYNCST 
    global y_final y_temp_final
    global param

 % for obstacle overtaking until the road is clear 
 global EgoPolicy Phase
%% vehicle kinematcs differential terms
fx = zeros(X_DIM, X_DIM, NUM_CTRL);
fu = zeros(X_DIM, U_DIM, NUM_CTRL);

for i = 1:1:NUM_CTRL
    acc     = X(3,i);
    v       = X(4,i);
    theta   = X(5,i);
    jerk     = U(1,i);
    delta   = U(2,i);
    
    k       = tan(delta)/L;...              curvature
    l       = v*dt + 0.5*acc*dt^2+(1/6.)*jerk*dt^3;...       distance traveled % jerk added by Omid
    phi     = theta + k*l;

    % df_dx
    if (k == 0)
        fx(:,:,i) = eye(X_DIM) + ...
                    [0, 0, 0.5*(dt^2)*cos(phi), dt*cos(phi), -l*sin(theta);
                     0, 0, 0.5*(dt^2)*sin(phi), dt*sin(phi), l*cos(theta);
                     0, 0, 0, 0,           0;
                     0, 0,dt, 0,           0;
                     0, 0,0.5*(dt^2)*k, dt*k,        0];
    else
        fx(:,:,i) = eye(X_DIM) + ...
                    [0, 0, 0.5*(dt^2)*cos(phi), dt*cos(phi), 1/k*(cos(phi) - cos(theta));
                     0, 0, 0.5*(dt^2)*sin(phi), dt*sin(phi), 1/k*(sin(phi) - sin(theta));
                     0, 0, 0, 0,           0;
                     0, 0, dt, 0,          0;
                     0, 0, 0.5*(dt^2)*k, dt*k,        0];%updated for jerk min x=[px,py,a,v,tetha], Omid
    end
    
    
    % df_du
    dk = (sec(delta))^2/L;
    if (k == 0)
        fu(:,:,i) = [cos(phi)*(1/6.)*dt^3,  0;
                     sin(phi)*(1/6.)*dt^3,  0;
                     dt,                 0;
                     0,                 0;
                     0,                  l * dk];
    else
        fu(:,:,i) = [cos(phi)*(1/6.)*dt^3, (l/k*cos(phi) - 1/k^2*(sin(phi)-sin(theta)))*dk;
                     sin(phi)*(1/6.)*dt^3, (l/k*sin(phi) + 1/k^2*(cos(phi)-cos(theta)))*dk;
                     dt,                 0;
                     0,                 0;
                     k*(1/6.)*dt^3,         l * dk];
    end


end

%% cost function and constraints
cx  = zeros(X_DIM, 1, NUM_CTRL + 1);
cxx = zeros(X_DIM, X_DIM, NUM_CTRL + 1);
cu  = zeros(U_DIM, 1, NUM_CTRL);
cuu = zeros(U_DIM, U_DIM, NUM_CTRL);
cux = zeros(U_DIM, X_DIM, NUM_CTRL + 1);...     du then dx

%parameters required for weight calculations 
Imag_targetCar_dis=1000;%this distance has been used to for an Imaginary target Car farawa from ego vehicle in a same lane with lane refrence speed so it can be used in adptive eight tning furmula in case no car was in front of ego vehicle and its lane   
for i = 1:1:NUM_CTRL
    Xi = X(:,i);
    Ui = U(:,i);
    % adaptive weight calculation
    if (adaptive == 1)

%         if length(obstacle)==0
%            w_jerk   =0.05.* ones(NUM_CTRL,1);
%            w_acc       = 10.* ones(NUM_CTRL,1);
%            w_del       = 20.* ones(NUM_CTRL,1);
%            w_ref       = 30.* ones(NUM_CTRL,1);... 3
%            w_vel       = 30.* ones(NUM_CTRL,1);
%         else

       PDM1=0.;EgoPolicy1=0.* ones(20,1);PDM2=1;EgoPolicy2=0.0001.* ones(20,1);counter=0;%initialization meaning lane keeping by default 
       NLUA=0;NLGP=0;VNLB=0.89*vref_road;VNLF=1.01*vref_road;VELB=0.89*vref_road;VELF=1.01*vref_road;%default values

       front_index=20;back_index=20;backELC=20;frontELC=20;
       %EgoPolicy2=1.* ones(20,1); b default for checking 20 cars in the other lane which is more than enough
       % the default value for EgoPolicy2 is 1 which will mk equal to 1 if y_temp_final is not the first or the last lane of the road 

       %if there wasn't an car near than distance of 300 from our car, we spcify the index as a requirment for following eqs  
       ref_dist=10000.;dangerousCar=0.;closest_car=ref_dist; closest_back_car_dist=-ref_dist;closest_front_car_dist=ref_dist;%initalization large distance
       backNLposition=-ref_dist; frontNLposition=ref_dist;
       backELCdist=-ref_dist;frontELCdist=ref_dist;
       ccie=0;%closest_car_in_egolane=0;
       frontTCV=vref_road;%initial value for front target ca velocity
       index=-1;Vindex=-1;%initalization index, we need to evaluate each time the nvironment to find the correct trget vehicle and if there's no car available in our lane index should be NaN 
        for k = 1:length(obstacle)
              EgoLaneY = CenterLaneY_detector(Xi(2));
              tgtLaneY = CenterLaneY_detector(obstacle(k).traj(2,i));
%         dx_wei = sqrt((Xi(1) - obstacle(1).traj(1,i,k))^2 + (Xi(2) - obstacle(1).traj(2,i,k))^2);
%         w_ref = abs(obstacle(1).traj(3,i))/a_ref / exp(b_ref * dx_wei);
%         w_vel = a_vel/abs(obstacle(1).traj(3,i)) * exp(b_vel * dx_wei);
%         disp(length(obstacle))
%         disp(obstacle(k).traj(1,i))
        % adaptive weight calculations
%          dx_wei_tags(k) = sqrt((Xi(1) - obstacle(k).traj(1,i))^2 + (Xi(2) - obstacle(k).traj(2,i))^2);% (k) is added by Omid to see all the targets

%               dx_wei_tags(k) =obstacle(k).traj(1,i)-Xi(1);% (k) is added by Omid to see all the targets

         %          if EgoLaneY~=tgtLaneY%In the velocity reference  we only account for the vehicles
%              dx_wei_tags(k)=1000;
%          end 
          if EgoLaneY==tgtLaneY%In the velocity reference  we only account for the vehicles
              deltaX=obstacle(k).traj(1,i)-Xi(1);
              % EgoPolicy can be lane changing for EgoPolicy<0
              %finding the index of target vhicle with min distance if it's in the same lane with ego car   
%               trajV=obstacle(k).traj(4,i);
% 
%               if deltaV==0
%                       deltaV=0.01; % just a small number instead of 0  
%               elseif deltaX==0
%                       deltaX=0.01;
%               elseif trajV==0
%                   trajV=0.01;%by having this if a car is stopped at behind EgoPolicy1(k) will be <0 so will trigger an index and if it's in front EgoPolicy1(k)>0 so it can replace dangerousCar down here so no index=-1 happens incorrectly
%               end

              if deltaX>0%using the 1.1 coefficient front target vehicles have more influence on egopolicy value than the back vehicles 

                  deltaV=obstacle(k).traj(4,i)-0.9*vref_road;%Vtraffic is either Vref_road or speed of th car with lowest speed
                  EgoPolicy1(k) =1-exp(-(deltaV));%trajV*(deltaV/abs(deltaV))/deltaX;  
                  
                  %EgoPolicy1(k) =1.01*trajV*(deltaV/abs(deltaV))/deltaX;%deltaV/deltaX;
                  if deltaX<frontELCdist
                  frontELCdist=deltaX;
                  frontELC=k;
                  %the index will be used in W eqs will be the index of the closest front car 
                  index=k;
                  Vindex=k;% if there's a dangerous car behind then Vindex will be updated 
                  VELF=obstacle(k).traj(4,i);%VELB=ego lane front velocity

                  mainTCV=obstacle(k).traj(4,i);%mainTCV = main Target Car Velocity
                  end
              elseif deltaX==0%to prevent infinity values we consider egopolicy2= 0 for deltaX=0
                  EgoPolicy1(k) =0;  
              elseif deltaX<0
                  
                  deltaV=obstacle(k).traj(4,i)-vref_road;%Vtraffic is either Vref_road or speed of th car with lowest speed
                  EgoPolicy1(k) =1-1/(exp(-(deltaV)));%trajV*(deltaV/abs(deltaV))/deltaX;  
                  
                  if deltaX>backELCdist
                  backELCdist=deltaX;
                  backELC=k;
                  VELB=obstacle(k).traj(4,i);%VELB=ego lane back velocity

                  end
              end
              
              
              if abs(EgoPolicy1(k))>abs(dangerousCar)
              dangerousCar=abs(EgoPolicy1(k));
%               if closest_car>0

              Vindex=k;%Vindex velocity index ill b sed to update v_ref
              
              
                  
%                   if deltaV==0
%                       deltaV=0.01; % just a small number instead of 0  
%                   elseif deltaX==0
%                       deltaX=0.01;
%                   elseif  mainTCV==0
%                       mainTCV=0.01;
%                   end 
%                       EgoPolicy2(k) =tanh((mainTCV*(deltaV/abs(deltaV))/deltaX));%tanh((deltaV/deltaX)*abs(deltaX/deltaV));
                  
%               end
              end
         else   % as we need to make sure to not to double count the tgt vehicles, for 2 lanes roads if ytempfinal is linear and  tgtLaneY==y_temp_final then there's a chance to overcount the tagt vehicle here, that's h we need this else here   
%          if y_temp_final==(road_up_lim-Lane_size/2.) || y_temp_final==(road_low_lim+Lane_size/2.)%in here we check if the temporary destination is on the frst or the last line of the road 
%           if tgtLaneY==y_temp_final%In the velocity reference  we only account for the vehicles
          if ((tgtLaneY==(road_up_lim-Lane_size/2.)) || (tgtLaneY==(road_low_lim+Lane_size/2.))) 
          if abs(EgoLaneY-tgtLaneY)==Lane_size
%               sprintf('%0.55f',abs(tgtLaneY-y_temp_final))
%               sprintf('%0.55f',Lane_size)
%           if abs(abs(tgtLaneY-y_temp_final)-Lane_size) <= 1e-15 || abs(abs(tgtLaneY-y_temp_final)-0) <= 1e-15  
          if ((roundn(abs(tgtLaneY-y_temp_final),2)==roundn(Lane_size,2)) || (roundn(abs(tgtLaneY-y_temp_final),2)==roundn(0,2))) 

              deltaX=obstacle(k).traj(1,i)-Xi(1);

%               if counter==0%we use a counter to not to add the default value of EgoPolicy2=1 here
              
                  if deltaX>0%using the 1.1 coefficient front target vehicles have more influence on egopolicy value than the back vehicles 
                     deltaV=obstacle(k).traj(4,i)-0.9*vref_road;%Vtraffic is either Vref_road or speed of th car with lowest speed
                     if deltaV==0
                      deltaV=0.01; % just a small number instead of 0  
                     end
                     EgoPolicy2(k) =1-exp(-(deltaV));%trajV*(deltaV/abs(deltaV))/deltaX; 
                      %tanh(1-exp(-(deltaV)));
                      
                      if deltaX<closest_front_car_dist
                      closest_front_car_dist=deltaX;
                      front_index=k;
                      frontNLposition=obstacle(k).traj(1,i);
                      VNLF=obstacle(k).traj(4,i);%VELB=neigboring lane front velocity

                     end
%                   frontTCV=obstacle(k).traj(4,i);% frontTCV front target car velocity 
%                   elseif deltaX==0
%                       deltaX=0.01;
%                   elseif  frontTCV==0
%                       frontTCV=0.01;
%                   end 
%                       EgoPolicy2(k) =1.01*tanh((frontTCV*(deltaV/abs(deltaV))/deltaX));
                  
                  
                  elseif deltaX==0%to prevent infinity values we consider egopolicy2= 0 for deltaX=0
                  EgoPolicy2(k) =0;  
                  elseif deltaX<0
                       deltaV=obstacle(k).traj(4,i)-vref_road;%Vtraffic is either Vref_road or speed of th car with lowest speed
                      if deltaV==0
                      deltaV=-0.01; % just a small ngative number instead of 0  
                      end
                       EgoPolicy2(k) =1-1/(exp(-3.*(deltaV)));%trajV*(deltaV/abs(deltaV))/deltaX;  
                       %tanh(1-1/(exp(-(deltaV))));
                      if deltaX>closest_back_car_dist
                      closest_back_car_dist=deltaX;
                      back_index=k;
                      backNLposition=obstacle(k).traj(1,i);
                      VNLB=obstacle(k).traj(4,i);%VELB=ego lane back velocity

                      end
%                   trajV=obstacle(k).traj(4,i);
%                   if deltaV==0
%                       deltaV=0.01; % just a small number instead of 0  
%                   elseif deltaX==0
%                       deltaX=0.01;
%                   elseif  trajV==0
%                       trajV=0.01;
%                   end
%                   
%                       EgoPolicy2(k) =tanh((trajV*(deltaV/abs(deltaV))/deltaX)); 
                  
                  end
%                   counter=counter+1;
%               else
%               if deltaX>0%using the 1.1 coefficient front target vehicles have more influence on egopolicy value than the back vehicles 
%                   EgoPolicy2 =EgoPolicy2+1.1*tanh((deltaV/deltaX)*abs(deltaX/deltaV));
%               elseif deltaX==0%to prevent infinity values we consider egopolicy2= 0 for deltaX=0
%                   EgoPolicy2 =EgoPolicy2+0;  
%               elseif deltaX<0
%                   EgoPolicy2 =EgoPolicy2+tanh((deltaV/deltaX)*abs(deltaX/deltaV));
%               end
%               end
          end    
          end
           end 
%          end
           end
        end
        if index~=-1 %&& back_index~=1.5
%         if abs(obstacle(index).traj(1,i)-Xi(1))<abs(obstacle(back_index).traj(1,i)-Xi(1))
        if abs(obstacle(Vindex).traj(1,i)-Xi(1))<4*param.len% bug we should use Vindex here instead of index 
        ccie=1;
        end
        end
        
       if  i<5% as we don't want an early dciion making. for example short term prediction part can see the next 4s upfront and if we  
        %this part is addd to only account for closest front and back cars in calculating EgoPolicy2tot for the first and last road lanes  
              %ccie*EgoPolicy2(index)
              
%        if index~=-1
%               if closest_back_car_dist<ref_dist && closest_front_car_dist<ref_dist
%               EgoPolicy2tot=EgoPolicy2(back_index)-0.5*ccie*EgoPolicy2(index)+EgoPolicy2(front_index);
%               elseif closest_back_car_dist<ref_dist && roundn(closest_front_car_dist,2)==roundn(ref_dist,2)
%               EgoPolicy2tot=EgoPolicy2(back_index)-0.5*ccie*EgoPolicy2(index);
%               elseif closest_front_car_dist<ref_dist && roundn(closest_back_car_dist,2)==roundn(ref_dist,2)
%               EgoPolicy2tot=EgoPolicy2(front_index)-0.5*ccie*EgoPolicy2(index);
%               end
%        else

              if backELCdist>-ref_dist && frontELCdist<ref_dist
             PDM1=EgoPolicy1(backELC)+EgoPolicy1(frontELC);
              elseif backELCdist>-ref_dist && roundn(frontELCdist,2)==roundn(ref_dist,2)
             PDM1=EgoPolicy1(backELC);
              elseif frontELCdist<ref_dist && roundn(backELCdist,2)==roundn(-ref_dist,2)
             PDM1=EgoPolicy1(frontELC);
              end

%               old PDM2 approach
%               if closest_back_car_dist>-ref_dist && closest_front_car_dist<ref_dist
%               PDM2=EgoPolicy2(back_index)+EgoPolicy2(front_index)-ccie*1.1*tanh(PDM1);
%               elseif closest_back_car_dist>-ref_dist && roundn(closest_front_car_dist,2)==roundn(ref_dist,2)
%               PDM2=EgoPolicy2(back_index)-ccie*1.1*tanh(PDM1);
%               elseif closest_front_car_dist<ref_dist && roundn(closest_back_car_dist,2)==roundn(-ref_dist,2)
%               PDM2=EgoPolicy2(front_index)-ccie*1.1*tanh(PDM1);
%               end

%               new PDM2 approach
%               if closest_back_car_dist>-ref_dist && closest_front_car_dist<ref_dist
%               PDM2=EgoPolicy2(back_index)+EgoPolicy2(front_index)-ccie*1.01*tanh(PDM1);
%               elseif closest_back_car_dist>-ref_dist && roundn(closest_front_car_dist,2)==roundn(ref_dist,2)
%               PDM2=EgoPolicy2(back_index)-ccie*1.01*tanh(PDM1);
%               elseif closest_front_car_dist<ref_dist && roundn(closest_back_car_dist,2)==roundn(-ref_dist,2)
%               PDM2=EgoPolicy2(front_index)-ccie*1.01*tanh(PDM1);
%               end
              PDM2=VNLB*EgoPolicy2(back_index)+VNLF*EgoPolicy2(front_index)-ccie*1.01*(VELB*EgoPolicy1(backELC)+VELF*EgoPolicy1(frontELC));

%        end


        deltaXeb=Xi(1)-backNLposition;
        deltaXfb=frontNLposition-backNLposition;
        deltaXfe=frontNLposition-Xi(1);
        GDM=(1-exp(-(deltaXeb-1.5*param.len)))+(1-exp(-(deltaXfb-5.*param.len)))+(1-exp(-(deltaXfe-3.*param.len)));
        
        NLUA=(PDM2+abs(PDM2))/(2);
        NLGP=(GDM+abs(GDM))/2.;
        if NLGP~=0
%         disp('ghgh');    
        end
        EgoPolicy=NLGP*NLUA*PDM1;
        if EgoPolicy<0
% %              disp('mee');
        end
       % phase determination
         %if NLGP>0 && NLUA>0 && (index==-1 || mainTCV>=vref_road) && VNLF>=0.9*vref_road  % if mk=0 it means egopolicy2<0 meaning frst or last lane is not clear and safe  % && ((Xi(1)-closest_back_car_dist)>1*param.len) && ((closest_front_car_dist-closest_back_car_dist)>8*param.len)
         if NLGP>0 && NLUA>0 && PDM1>=0  % if mk=0 it means egopolicy2<0 meaning frst or last lane is not clear and safe  % && ((Xi(1)-closest_back_car_dist)>1*param.len) && ((closest_front_car_dist-closest_back_car_dist)>8*param.len)
       % bug having mk>0 && index==-1  instead of will covers more phase  transmission
         Phase=2;
         else
         Phase=1;
         end
       
        
       end

       
%          [dx_wei,index]=min(dx_wei_tags);% addaptive weight func modified by Omid
%         ggg=round(Xi(2));
%         if ggg~=4
%         disp(index);
%         end
%         
%         if index==4
%          disp(index);
%         end

         coef=0.02;

         if index==-1 && Vindex~=-1

         dx_1=Imag_targetCar_dis;
         dx_2=abs(Xi(2)-Xi(2));
         %vref_old=vref;
         vref=floor(obstacle(Vindex).traj(4,i)+abs((vref_road+0.1-obstacle(Vindex).traj(4,i))*tanh(coef*dx_1+0.2*dx_2)));%+0.2*dx_2         
         
         B=dx_1+vref_road-0.6*vref-2*param.len;
         C=dx_1+vref_road-vref-10*param.len;

         elseif index==-1 && Vindex==-1
             dx_1=Imag_targetCar_dis;
             dx_2=abs(Xi(2)-Xi(2));
             vref=floor(vref_road+abs((vref_road+0.1-vref_road)*tanh(coef*dx_1+0.2*dx_2)));
             
             B=dx_1+vref_road-0.6*vref-2*param.len;
             C=dx_1+vref_road-vref-10*param.len;
             
         else
             dx_1=obstacle(index).traj(1,i)-Xi(1);%dx_1=deltax(1) tp remove the influnce of back cars on ego vehicle speed we don't use abs distance value here 
             dx_2=abs(Xi(2)-obstacle(index).traj(2,i));%dx_2=deltax(2)

            %         if y_temp_final ~= y_final
            vref=floor(obstacle(Vindex).traj(4,i)+abs((vref_road+0.1-obstacle(Vindex).traj(4,i))*tanh(coef*dx_1+0.2*dx_2)));%+0.2*dx_2
            %vref=15.;
            %10 6 2020 if tgt was faster than vref_road then it is important to diminish the vlocoty of vref to vref_road
            B=dx_1+obstacle(index).traj(4,i)-0.6*vref-2*param.len;
            C=dx_1+obstacle(index).traj(4,i)-vref-10*param.len;
         end

if vref==10
%     disp('fff');
end

            if vref>vref_road
                vref=vref_road;
            elseif vref<0.2
                vref=0;
            end
%             disp(vref);
            %             vref=obstacle(index).traj(4,i);
%         elseif y_temp_final == y_final
%             vref=vref_road;
%         end
%         if y_temp_final<0 && y_temp_final>-1
%             disp('jhj')
%         end


        y_temp=y_temp_final;
        dy_f=abs(y_temp_final-Xi(2));%deltay_final
%         dy_f=abs(y_final-Xi(2));%deltay_final
        dy_t=abs(y_temp-Xi(2));%dy_t=deltay_temp is at max equal to the distance between the curent ego vehicle center lane with the other lane center if we want to go there as a step to reach th y_final
        
        
        E=dx_2-dy_t;
        mean_acc=0;sigma_acc=3;mean_jerk=0;sigma_jerk=0.1;
        V=(abs(B)+B)/2.;W=(abs(C)+C)/2.;S=(abs(E)+E)/2.;
        D=abs(Xi(4)-vref)+abs(-log(1.-exp(-0.01*((10^-0)+W))));
        F=abs(Xi(4)-vref)+abs(Xi(3)-aref)+abs(-log(1.-exp(-0.01*((10^-0)+W))));
        sss=(1+2*tanh(0.2*(dy_f+dx_2+V)));
        sss=(-log(1.00004-exp(-0.01*dy_t)));
        sss=1.057^(30*tanh(0.09*V));
        %w_ref(i)=(1+2*tanh(0.2*(dy_f+dx_2+V)));
%         const1=-2.*atan(-30)+2.*atan(100*EgoPolicy);%acotd( X )
        const1=-1.*atan(-30);%acotd( X )
%         const1=2*acotd(-30);
%abs(const1+1.*atan(100*EgoPolicy))*
% if i==1
         w_ref(i)=((-log(1.-exp(-0.01*((10^-3)+dy_t)))))*1.057^(30*tanh(0.09*V));
%          w_ref(i)=(1+2*tanh(0.2*(dy_f+dx_2+V)))*(-log(1.00004-exp(-0.01*dy_t)))*1.057^(30*tanh(0.09*V));
         w_vel(i)=(1+5*tanh(0.1*W))*((-log(1.-exp(-0.01*((10^-1)+dy_t)))));
%          w_vel(i)=(0.1+5*tanh(0.1*W+0.24*S))*(-log(1.00673-exp(-0.01*dy_t)));
         w_acc(i)=(37.59*(1/(sigma_acc*(2*pi)^0.5))*(exp(-((D-mean_acc)^2)/(2*sigma_acc^2))))*((-log(1.-exp(-0.01*((10^-1)+dy_t)))));
%          w_acc(i)=(37.59*(1/(sigma_acc*(2*pi)^0.5))*(exp(-((D-mean_acc)^2)/(2*sigma_acc^2))))*(1+2*tanh(0.2*(dy_f+dx_2+V)))*(-log(1.00673-exp(-0.01*dy_t)));
         w_del(i)=(2+5*tanh(0.1*W))*((-log(1.-exp(-0.01*((10^-3)+dy_t)))));
         w_jerk(i)=10+2.5*(1/(sigma_jerk*(2*pi)^0.5))*(exp(-((F-mean_jerk)^2)/(2*sigma_jerk^2)));
%0.00005+
% if w_jerk(i)>0
% disp('ddd');
% end

% else
%          w_ref(i)=w_ref(i-1);
%         w_vel(i)=w_vel(i-1);
%         w_acc(i)=w_acc(i-1);
%         w_del(i)=w_del(i-1);
%         w_jerk(i)=w_jerk(i-1);
% end



%w_ref(i) = 30.*(obstacle(index).traj(4,i)/vref) / a_ref * exp(b_ref * dx_wei);
        %         w_vel(i) = a_vel / (obstacle(index).traj(4,i)/vref) / exp(b_vel * dx_wei);
%        w_vel(i) =  a_vel*exp(b_vel * dx_wei.^1.)/(obstacle(index).traj(4,i)/vref) ;
%         w_acc(i) =w_ref(i)/2. ;
 %       w_acc(i) = 1.*(obstacle(index).traj(4,i)/vref)/ exp(b_ref * dx_wei.^0.25);
 %       w_jerk(i) = 0.05;
 %       w_del(i) = 0.07;
%         w_jerk(i) = 0.05*(obstacle(index).traj(4,i)/vref)/ exp(b_ref * dx_wei.^0.25);
%         w_del(i) = (obstacle(index).traj(4,i)/vref)/ exp(b_ref * dx_wei.^0.25);
%         xxx = [w_ref,'     ',w_vel];
% disp(w_ref)
% disp(w_vel)
% disp(index)
% disp('     ')

%         disp(num2str(w_ref),'   ',num2str());
        
%         end
    end
    
    % reference trajectory
    [~, index] = min(sum((ref_traj' - Xi).^2));
    Xref = transpose(ref_traj(index,:));
    cx(:,:,i) = cx(:,:,i) + w_ref(i) * diag([2 2 0 0 0]) * (Xi - Xref);...   4x1
    cxx(:,:,i) = cxx(:,:,i) + w_ref(i) * diag([2 2 0 0 0]);...               4x4
    
    % acceleration
    cx(:,:,i) = cx(:,:,i) + w_acc(i) * diag([0 0 2 0 0]) * (Xi - Xref);...   4x1
    cxx(:,:,i) = cxx(:,:,i) + w_acc(i)* diag([0 0 2 0 0]);...               4x4
    
    % velocity
    cx(:,:,i) = cx(:,:,i) + w_vel(i) * diag([0 0 0 2 0]) * (Xi - Xref);...   4x1
    cxx(:,:,i) = cxx(:,:,i) + w_vel(i) * diag([0 0 0 2 0]);...               4x4
    
    % control
    cu(:,:,i) = cu(:,:,i) + diag([w_jerk(i) w_del(i)]) * diag([2 2]) * (Ui);...              2x1
    cuu(:,:,i) = cuu(:,:,i) + diag([w_jerk(i) w_del(i)]) * diag([2 2]);...                   2x2
    
    % obstacle term
    for j = 1:length(obstacle)
        % only calculates one step derivative cost instead of a sequence
        [bx, bxx] = calc_bx(Xi, obstacle(j), i, ...
            q1_back, q2_back, q1_front, q2_front, use_prediction);
        cx(:,:,i) = cx(:,:,i) + bx;...                                  4x1
        cxx(:,:,i) = cxx(:,:,i) + bxx;...                               4x4
    end

    % ==================== road limits ====================
    % upper bound
    % road upper and lower limits
    g   = Xi(2) - road_up_lim;
    dg 	= [0;1;0;0;0];
    db  = q1_road * q2_road * exp(q2_road * g) * (dg);              % scalar * (4x1) 
    ddb = q1_road * q2_road ^2 * exp(q2_road * g) * (dg) * (dg');   % scalar * (4x1) * (1x4)

    cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
    cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4
    
    % lower bound
    g   = road_low_lim - Xi(2);
    dg 	= [0;1;0;0;0];
    db  = q1_road * q2_road * exp(q2_road * g) * (dg);              % scalar * (4x1) 
    ddb = q1_road * q2_road ^2 * exp(q2_road * g) * (dg) * (dg');   % scalar * (4x1) * (1x4)

    cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
    cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4
    
    % ====================  penalizing distanc from the lane center (Omid)  ====================
%     CenterLaneY = CenterLaneY_detector(Xi(2));
%     g   = abs(Xi(2)-CenterLaneY) - 1;
%     dg 	= [0;1;0;0;0];
%     db  = q1_CenterLane * q2_CenterLane * exp(q2_CenterLane * g) * (dg);              % scalar * (4x1) 
%     ddb = q1_CenterLane * q2_CenterLane ^2 * exp(q2_CenterLane * g) * (dg) * (dg');   % scalar * (4x1) * (1x4)
% 
%     cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
%     cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4
    
     % ====================acceleration limit ====================

    %min velocity constraint
    g   =v_min-Xi(4);
    dg 	= [0;0;0;1;0];
    db  = q1_min_vel * q2_min_vel * exp(q2_min_vel * g) * (dg);              % scalar * (2x1) 
    ddb = q1_min_vel * q2_min_vel ^2 * exp(q2_min_vel * g) * (dg) * (dg');   % scalar * (2x1) * (1x2)

    cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
    cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4
    
    % acceleration
    % upper bound
    g   = Xi(3) - v_dot_max;
    dg 	= [0;0;1;0;0];
    db  = q1_acc * q2_acc * exp(q2_acc * g) * (dg);              % scalar * (2x1) 
    ddb = q1_acc * q2_acc ^2 * exp(q2_acc * g) * (dg) * (dg');   % scalar * (2x1) * (1x2)

    cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
    cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4
    
    % lower bound
    g   = v_dot_min - Xi(3);%Omid this must be Xi(3) for acceleration
    dg 	= [0;0;1;0;0];
    db  = q1_acc * q2_acc * exp(q2_acc * g) * (dg);              % scalar * (2x1) 
    ddb = q1_acc * q2_acc ^2 * exp(q2_acc * g) * (dg) * (dg');   % scalar * (2x1) * (1x2)

    cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
    cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4
    % ==================== ctrl limits ====================
    % jerk
    % upper bound
    g   = Ui(1)  - a_dot_max;
    dg 	= [1;0];
    db  = q1_jerk * q2_jerk * exp(q2_jerk * g) * (dg);              % scalar * (2x1) 
    ddb = q1_jerk * q2_jerk ^2 * exp(q2_jerk * g) * (dg) * (dg');   % scalar * (2x1) * (1x2)

    cu(:,:,i)   = cu(:,:,i) + db;...                                 2x1
    cuu(:,:,i)  = cuu(:,:,i) + ddb;...                               2x2
    
    % lower bound
    g   = a_dot_min - Ui(1) ;%Omid this must be Xi(3) for acceleration
    dg 	= [1;0];
    db  = q1_jerk * q2_jerk * exp(q2_jerk * g) * (dg);              % scalar * (2x1) 
    ddb = q1_jerk * q2_jerk ^2 * exp(q2_jerk * g) * (dg) * (dg');   % scalar * (2x1) * (1x2)

    cu(:,:,i)   = cu(:,:,i) + db;...                                 2x1
    cuu(:,:,i)  = cuu(:,:,i) + ddb;...                               2x2
    
    % steering
    % upper bound
    g   = Ui(2) - delta_max;
    dg 	= [0;1];
    db  = q1_del * q2_del * exp(q2_del * g) * (dg);              % scalar * (2x1) 
    ddb = q1_del * q2_del ^2 * exp(q2_del * g) * (dg) * (dg');   % scalar * (2x1) * (1x2)

    cu(:,:,i)   = cu(:,:,i) + db;...                                 2x1
    cuu(:,:,i)  = cuu(:,:,i) + ddb;...                               2x2
    
    % lower bound 
    g   = delta_min - Ui(2);
    dg 	= [0;1];
    db  = q1_del * q2_del * exp(q2_del * g) * (dg);              % scalar * (2x1) 
    ddb = q1_del * q2_del ^2 * exp(q2_del * g) * (dg) * (dg');   % scalar * (2x1) * (1x2)

    cu(:,:,i)   = cu(:,:,i) + db;...                                 2x1
    cuu(:,:,i)  = cuu(:,:,i) + ddb;...                               2x2
    
    % ==================== stop sign ====================
    if (stopping == 1)
        % stop sign at 75 meters
        g   = Xi(1) - x_stop;
        dg 	= [1;0;0;0;0];
        db  = q1_stop * q2_stop * exp(q2_stop * g) * (dg);              % scalar * (4x1) 
        ddb = q1_stop * q2_stop ^2 * exp(q2_stop * g) * (dg) * (dg');   % scalar * (4x1) * (1x4)
        
        cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
        cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4


%         cx(:,:,i)   = cx(:,:,i) + 2 * diag([w_end_ref 0 w_end_vel 0]) * ...
%             [Xi(1) - x_stop; 0; Xi(3); 0];...               4x1
%         cxx(:,:,i)  = cxx(:,:,i) + 2 * ...
%             diag([w_end_ref 0 w_end_vel 0]);...  	4x4
    end
end
if cxx(2,2,31)>500
% disp('ghhg');    
end
%% end state
i  = NUM_CTRL + 1;
Xi = X(:,i);

% reference trajectory and velocity cost
[~, index] = min(sum((ref_traj' - Xi).^2));
Xref = transpose(ref_traj(index,:));
cx(:,:,i) = cx(:,:,i) + 2 * diag([w_end_ref w_end_ref w_end_acc w_end_vel 0]) *  (Xi - Xref);...	4x1
cxx(:,:,i) = cxx(:,:,i) + 2 * diag([w_end_ref w_end_ref w_end_acc w_end_vel 0]);...              	4x4

% added by Omid to get w_vel w_ref fo i=41
%  if (adaptive == 1)
%         for k = 1:length(obstacle)
% 
%         % adaptive weight calculations
%         dx_wei_tags(k) = sqrt((Xi(1) - obstacle(k).traj(1,i))^2 + (Xi(2) - obstacle(k).traj(2,i))^2);% (k) is added by Omid to see all the targets
% 
%         end
%         [dx_wei,index]=min(dx_wei_tags);% addaptive weight func modified by Omid
%         w_ref(i) = obstacle(index).traj(4,i) / a_ref * exp(b_ref * dx_wei);
%         w_vel(i) = a_vel / obstacle(index).traj(4,i) / exp(b_vel * dx_wei);
%     end


% road limits
% upper bound
g   = Xi(2) - road_up_lim;
dg 	= [0;1;0;0;0];
db  = q1_road * q2_road * exp(q2_road * g) * (dg);              % scalar * (4x1) 
ddb = q1_road * q2_road ^2 * exp(q2_road * g) * (dg) * (dg');   % scalar * (4x1) * (1x4)

cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4

% lower bound
g   = road_low_lim - Xi(2);
dg 	= [0;1;0;0;0];
db  = q1_road * q2_road * exp(q2_road * g) * (dg);              % scalar * (4x1) 
ddb = q1_road * q2_road ^2 * exp(q2_road * g) * (dg) * (dg');   % scalar * (4x1) * (1x4)

cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4

% ==================== penalizing distanc from the lane center (Omid) ====================
%     CenterLaneY = CenterLaneY_detector(Xi(2));
%     g   = abs(Xi(2)-CenterLaneY) - 1;
%     dg 	= [0;1;0;0;0];
%     db  = q1_CenterLane * q2_CenterLane * exp(q2_CenterLane * g) * (dg);              % scalar * (4x1) 
%     ddb = q1_CenterLane * q2_CenterLane ^2 * exp(q2_CenterLane * g) * (dg) * (dg');   % scalar * (4x1) * (1x4)
% 
%     cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
%     cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4


% obstacle term
if (~isempty(obstacle))
    for j = 1:length(obstacle)
        % only calculates one step derivative cost instead of a sequence
        [bx, bxx] = calc_bx(Xi, obstacle(j), i, q1_back, q2_back, q1_front, q2_front, use_prediction);
        cx(:,:,i) = cx(:,:,i) + bx;...                                      4x1
        cxx(:,:,i) = cxx(:,:,i) + bxx;...                                   4x4
    end
end

% stop sign
if (stopping == 1)
    % stop sign at 75 meters
    g   = Xi(1) - x_stop;
    dg 	= [1;0;0;0;0];
    db  = q1_stop * q2_stop * exp(q2_stop * g) * (dg);              % scalar * (4x1) 
    ddb = q1_stop * q2_stop ^2 * exp(q2_stop * g) * (dg) * (dg');   % scalar * (4x1) * (1x4)

    cx(:,:,i) = cx(:,:,i) + db;...                                  4x1
    cxx(:,:,i) = cxx(:,:,i) + ddb;...                               4x4

%     cx(:,:,i)   = cx(:,:,i) + 2 * diag([w_end_ref 0 w_end_vel 0]) * ...
%         [Xi(1) - x_stop; 0; Xi(3); 0];...               4x1
%     cxx(:,:,i)  = cxx(:,:,i) + 2 * ...
%         diag([w_end_ref 0 w_end_vel 0]);...  	4x4
end

end
