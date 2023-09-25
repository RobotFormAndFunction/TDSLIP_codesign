classdef motor_param
    %   motor_param a class with the following fields, only needed if motor
    %               is being used in the simulation
    %       - Ra, motor resistance Ohms
    %
    %       - La, motot inductance Henry's
    %
    %       - kt, motor torque constant, Nm/A
    %
    %       - kb, motor back emf constant, V/rad/sec
    %
    %       - J, inertial force SEEN BY MOTOR, kg m^2
    %
    %       - mech_c, mechanical system damping term N-m-s
    %
    %       - omega0, optional, initial motor speed defaults to 0 if not provieded, rad/s
    %
    %       - curr0, optional, initial current defaults to 0 if not provieded, A
    %
    %       - gearR, optional gear reduction constant if 18:1 provide as 18,
    %         if not provided assumed to be 1:1
    %
    %       - gearJ, interia of gearbox as seen by the motor, defaults to 0, kg m^2
    %
    %       - connect_torque, boolean defaults to 0.  If 0, will disconnect motor
    %         during stance phase set to 1 to connect to motor
    %
    %       - connect_voltage, boolean defaults to 0.  If 0, input voltage=0,
    %         set to 1 to provide a driving voltage
    %
    %       - omega_drive, boolean defaults to 0.  If 0, will disconnect motor
    %         during flight phase and speed is controlled by rad_freq term, set
    %         to 1 to connect to motor
    %
    %       - stanceV, two column matrix where first column is time and
    %         second column is voltage at that time.  When using motor
    %         simulation during stance will interpolate this to find v at 
    %         time t, if you want a specific function will have to rewrite the motor
    %         simulator
    %
    %       - flightV, same as stanceV but for flight, use an annymous
    %       function to update params between cycles
    %
    %       - interpType, interpolation format for stanceV,flightV,
    %       defaults to linear
    %
    % Public Methods:
    %       - Unpack, returns [Ra, La, kt, kb, J, mech_c, omega0, curr0, gearR]
    %
    %       - no_load_speed: assumes input load is zero and motor is subjected 
    %                           to a constant voltage [V], returns motor speed 
    %                           [rad/s] for a given voltage [V]
    %           Inputs: [voltage[V], type(optional) [0,1]] 
    %                   type=0 if w=rad/s, type=1 if w=rpm, defaults to 0
    %                   if not specified
    %           Ouputs: [motor speed, motor speed after gearbox]
    %
    %       - no_load_c: assumes input load is zero and motor is subjected 
    %                           to a constant voltage [V], returns motor current 
    %                           [A] for a given voltage [V]
    %           Inputs: voltage[V]
    %
    %       - ss_speed: for a given constant torque and voltage [V] returns motor speed [rad/s] 
    %           Inputs: [voltage [V], load torque[Nm], type(optional) [0,1]]]
    %                   type=0 if w=rad/s, type=1 if w=rpm, defaults to 0
    %                   if not specified
    %           Ouputs: [motor speed, motor speed after gearbox]
    %
    %       - ss_current: for a given constant torque and voltage [V] returns motor current[A]
    %           Inputs: [voltage [V], load torque[Nm]]
    %
    %       - stall_current: calculates motor stall current [A] for agiven voltage [V]
    %           Inputs: voltage [V]
    %
    %       - stall_torq: calculates motor stall torque [Nm] for a given voltage [V]
    %           Inputs: voltage [V]
    %
    %       - v_for_omega: calculates voltage [V] needed to obtain a
    %       specific motor shaft speed, can be given as rad/s or rpm
    %           Inputs: [w, type(optional) [0,1]] 
    %                   type=0 if w=rad/s, type=1 if w=rpm, defaults to 0
    %                   if not specified
    %
    %       - stall_torq: calculates motor stall torque [Nm] for agiven voltage [V]
    %           Inputs: v, tl [voltage [V]]
    %
    %       - v_for_omega_with_load: calculates voltage [V] needed to obtain a
    %       specific motor shaft speed with a load, can be given as rad/s or rpm
    %           Inputs: [w, tl[Nm],  type(optional) [0,1]] 
    %                   type=0 if w=rad/s, type=1 if w=rpm, defaults to 0
    %                   if not specified
    %                   w and tl can be given as vectors
    %
    %       - speed_at_time_with_IC: motor speed after time T when subjected to a
    %                                constant voltage with no load. 
    %           Inputs: [t, v,  omega0(optional)] 
    %                   t is time to evaluate at
    %                   v is constant voltage value
    %                   omega is initial speed of gearbox output shaft in
    %                   rad/sec
    %
    %       - v_for_rot_in_time_T: calculates voltage [V] needed to obtain a
    %       specific motor rotation in a specific amount of time.  Use
    %       pi+2*phi
    %           Inputs: [T, phi,  omega0(optional)] 
    %                   t is time to complete rotation
    %                   phi is how much the motor should rotate
    %                   omega is initial speed of gearbox output shaft in
    %                   rad/sec
    %    
    %       - rot_for_v_in_T: Calculates the amount of shaft roation for a
    %       given voltage in time T with no load
    %           Inputs: [ v,  omega0(optional)] 
    %                   t is time to complete rotation
    %                   v voltage
    %                   omega is initial speed of gearbox output shaft in
    %                   rad/sec
    %    
    %       - T_for_rot_cont_v: time voltage v should be on to complete x
    %       rotation.  Used if you want max speed in the air
    %           Inputs: [v, tstep, v, phitot, omega0] 
    %                   tstep is the simulation step size
    %                   v voltage
    %                   phitot total amount the gearbox shaft needs
    %                   to rotate
    %                   omega is initial speed of gearbox output shaft in
    %                   rad/sec
    %           Outputs: [time, err]
    %                   err: err between desired and actual rotation
    
    properties
        Ra     = 0
        La     = 0
        kt     = 0
        kb     = 0
        J      = 0
        mech_c = 0
        omega0 = 0
        curr0  = 0
        gearR  = 1
        gearJ  = 0
        connect_torque  = 0
        connect_voltage = 0
        omega_drive     = 0
        PN=0;
        GearPN=0;
        flightV= [linspace(0,0.5,100)' zeros(100,1)];
        stanceV=[linspace(0,0.5,100)' zeros(100,1)];
        interpType='linear'
        
    end
    methods
        function [Ra, La, kt, kb, J, mech_c, omega0, curr0, gearR]=unpack(obj)
            Ra     = obj.Ra;
            La     = obj.La;
            kt     = obj.kt;
            kb     = obj.kb;
            J      = obj.J+obj.gearJ;
            mech_c = obj.mech_c;
            omega0 = obj.omega0;
            curr0  = obj.curr0;
            gearR  = obj.gearR;
            
        end
        
        function [connect_torque, connect_voltage, omega_drive]=unpack_connections(obj)
            connect_torque  = obj.connect_torque;
            connect_voltage = obj.connect_voltage;
            omega_drive     = obj.omega_drive;
            
        end
        
        function [w,wgear]=no_load_speed(obj, V,type)
            if ~exist('type','var'), type=0; end
            
            w=(obj.kt*V)/(obj.mech_c*obj.Ra+obj.kt*obj.kb);
            wgear=w/obj.gearR;
            
            if type==1
                w=w*9.5493;
                wgear=wgear*9.5493;
            end
        end
        
        function [i_no_load]=no_load_c(obj, V)
            i_no_load=(obj.mech_c*V)/(obj.mech_c*obj.Ra+obj.kt*obj.kb);
        end
        
        function [w_ss,w_ssG]=ss_speed(obj, V, TL,type)
            T=TL/obj.gearR; %decrase torque on motor by gear ratio
            if ~exist('type','var'), type=0; end
            
            w_ss=(obj.kt*V-obj.Ra*T)./(obj.mech_c*obj.Ra+obj.kt*obj.kb);
            w_ssG=w_ss/obj.gearR;
            
             if type==1
                w_ss=w_ss*9.5493;
                w_ssG=w_ssG*9.5493;
             end
            
        end
        
        function [i_ss]=ss_current(obj, V, TL)
            T=TL/obj.gearR; %decrase torque on motor by gear ratio
            i_ss=(obj.mech_c*V+obj.kt*T)/(obj.mech_c*obj.Ra+obj.kt*obj.kb);
        end
        
        function [i_stall]=stall_current(obj, V)
            i_stall=(obj.mech_c+(obj.kt*obj.kb)/obj.Ra*V)/(obj.mech_c*obj.Ra+obj.kt*obj.kb);
        end
        
        function [torq_stall]=stall_torq(obj, V)
            torq_stall=(obj.kt*V)/obj.Ra;
        end
        
        function [v]=v_for_omega(obj, w, type)
            if ~exist('type', 'var')
                type=0;
            end
            if type==1
                w=w*0.10472;
            end
            
            w=w*obj.gearR;
            v=w*(obj.mech_c*obj.Ra+obj.kb*obj.kt)/obj.kt;
        end
        
        function [v]=v_for_omega_with_load(obj, w, TL, type)
            T=TL/obj.gearR; %decrease torque on motor by gear ratio
            if ~exist('type', 'var')
                type=0;
            end
            if type==1
                w=w*0.10472;
            end
            
            w=w*obj.gearR;
            v=(w*(obj.mech_c*obj.Ra+obj.kb*obj.kt)+obj.Ra*T)./obj.kt;

        end
        
        function [w, i_no_load, w_ss, i_ss, i_stall,torq_stall]=motor_performance(obj, V, TL)
            w=(obj.kt*V)/(obj.mech_c*obj.Ra+obj.kt*obj.kb);
            i_no_load=(obj.mech_c*V)/(obj.mech_c*obj.Ra+obj.kt*obj.kb);
            
            w_ss=(obj.kt*V-obj.Ra*TL)/(obj.mech_c*obj.Ra+obj.kt*obj.kb);
            i_ss=(obj.mech_c*V+obj.kt*TL)/(obj.mech_c*obj.Ra+obj.kt*obj.kb);
            
            i_stall=(obj.mech_c+(obj.kt*obj.kb)/obj.Ra*V)/(obj.mech_c*obj.Ra+obj.kt*obj.kb);
            torq_stall=(obj.kt*V)/obj.Ra;
        end
        
        function [omega]=speed_at_time_with_IC(obj, t, V,omega0)
            
            if ~exist('omega0', 'var')
                omega0=0;
            end
            
            omega0=omega0*obj.gearR;
            
            a=(obj.kt*obj.kb+obj.Ra*obj.mech_c)/(obj.Ra*obj.J);
            
            b=obj.kt/(obj.kt*obj.kb+obj.Ra*obj.mech_c);     
            
            omega=b*V+(omega0-b*V)*exp(-a*t);
   
        end
        
        function [v]=v_for_rot_in_time_T(obj, T, phi, omega0)
            if ~exist('omega0', 'var')
                omega0=0;
            end
            
            phi=phi*obj.gearR;
            omega0=omega0*obj.gearR;
            
            a=(obj.kt*obj.kb+obj.Ra*obj.mech_c)/(obj.Ra*obj.J);
            
            b=obj.kt/(obj.kt*obj.kb+obj.Ra*obj.mech_c);
            
            num=phi *a-omega0*(1-exp(-a*T));
            
            den=-b+b*a*T+b*exp(-a*T);
            
            %(den*v+omega0*(1-exp(-a*T)))/(obj.gearR*a)           
            v=num/den;
            
        end
        
        function [phi]=rot_for_v_in_T(obj, T, v, omega0)
            if ~exist('omega0', 'var')
                omega0=0;
            end
            
            omega0=omega0*obj.gearR;
            
            a=(obj.kt*obj.kb+obj.Ra*obj.mech_c)/(obj.Ra*obj.J);
            
            b=obj.kt/(obj.kt*obj.kb+obj.Ra*obj.mech_c);
            
            den=-b+b*a*T+b*exp(-a*T);
            
            phi=(den*v+omega0*(1-exp(-a*T)))/(obj.gearR*a);        
            
        end
        
        function [time, err ]=T_for_rot_cont_v(obj, tstep, v, phitot, omega0)
            if ~exist('omega0', 'var')
                omega0=0;
            end
            
            omega0=omega0*obj.gearR;
            phitot=phitot*obj.gearR;
            
            a=(obj.Ra*obj.J+obj.La*obj.mech_c)/(obj.La*obj.J);
            
            b=(obj.kt*obj.kb+obj.Ra*obj.mech_c)/(obj.La*obj.J);
            
            B=[0;0; obj.kt/(obj.La*obj.J)];
            
            A=[0 1 0;
                0 0 1;
                0 -b -a];
            tend=0.5;
            options=optimoptions(@fmincon,'FiniteDifferenceStepSize', tstep, 'Display','off');
            [time,err]=fmincon(@obj.min_time_cost,0.05,[],[],[],[],0,tend,[],options,...
                A,B,omega0,tend,tstep,v,phitot);
            

        end
        
        function a=test(obj, a)
            
        end
        
    end
    
    methods (Access = private)
        
        function err=min_time_cost(obj,x,A,B,omega0,tend,tstep,max_v,phi_goal)
            %Make system
            sys=ss(A,B,[1 0 0],[]);
            
            %create time and control vectors
            T=linspace(0,tend,tend/tstep+1)';
            U=zeros(size(T));
            
            %Set control to max until time x
            [~,I]=min(abs(x-T));
            U(1:I)=max_v;
            
            %simulate system, assume domega0=0
            phi=lsim(sys,U,T,[0 omega0 0]);
            phi_end=phi(end);
            err=abs(phi_end-phi_goal)/phi_goal;
            
            %For debugging
%             figure(1)
%             plot(T,phi,T,U)

        end
        
    end
        
end

