





classdef Link < handle
    properties
        % kinematic parameters
        theta % kinematic: link angle
        d     % kinematic: link offset
        alpha % kinematic: link twist
        a     % kinematic: link length
        jointtype  % revolute='R', prismatic='P' -- should be an enum
        offset % joint coordinate offset
        name   % joint coordinate name
        flip   % joint moves in opposite direction
        
        % %         % dynamic parameters
        % %         m  % dynamic: link mass
        % %         r  % dynamic: position of COM with respect to link frame (3x1)
        % %         I  % dynamic: inertia of link with respect to COM (3x3)
        % %         Jm % dynamic: motor inertia
        % %         B  % dynamic: motor viscous friction (1x1 or 2x1)
        % %
        % %         Tc % dynamic: motor Coulomb friction (1x2 or 2x1)
        % %         G  % dynamic: gear ratio
        % %         qlim % joint coordinate limits (2x1)
    end
    
    methods
        
        function l = Link(varargin)
            l.name = 'No name';
            l.jointtype = 0;
            l.theta = 0;
            l.d = 0;
            l.alpha = 0;
            l.a = 0;
            l.offset = 0;
            l.flip = false;
            
            %                 %% dynamic parameters
            %                 % these parameters must be set by the user if dynamics is used
            %                 l.m = 0;
            %                 l.r = [0 0 0];
            %                 l.I = zeros(3,3);
            %
            %                 % dynamic params with default (zero friction)
            %                 l.Jm = 0;
            %                 l.G = 1;
            %                 l.B = 0;
            %                 l.Tc = [0 0];
            
            vect_num_pro=linspace(1,8,8);
            InLengArgin=length(varargin);
            ItARgin=1;
            while (~isempty(varargin)) && (ItARgin<InLengArgin)
                if ischar(varargin{ItARgin})
                    switch varargin{ItARgin}
                        case 'name'
                            l.name = varargin(ItARgin+1);
                            varargin(ItARgin:ItARgin+1)=[];
                            vect_num_pro((vect_num_pro==1))=[];
                            InLengArgin=InLengArgin-2;
                        case 'revolute'
                            l.jointtype = 0;
                            varargin(ItARgin)=[];
                            vect_num_pro((vect_num_pro==2))=[];
                            InLengArgin=InLengArgin-1;
                        case 'prismatic'
                            l.jointtype = 1;
                            varargin(ItARgin)=[];
                            vect_num_pro((vect_num_pro==2))=[];
                            InLengArgin=InLengArgin-1;
                        case 'theta'
                            l.theta=varargin{ItARgin+1};
                            varargin(ItARgin:ItARgin+1)=[];
                            vect_num_pro((vect_num_pro==3))=[];
                            InLengArgin=InLengArgin-2;
                        case 'd'
                            l.d=varargin{ItARgin+1};
                            varargin(ItARgin:ItARgin+1)=[];
                            vect_num_pro((vect_num_pro==4))=[];
                            InLengArgin=InLengArgin-2;
                        case 'alpha'
                            l.alpha=varargin{ItARgin+1};
                            varargin(ItARgin:ItARgin+1)=[];
                            vect_num_pro((vect_num_pro==5))=[];
                            InLengArgin=InLengArgin-2;
                        case 'a'
                            l.a=varargin{ItARgin+1};
                            varargin(ItARgin:ItARgin+1)=[];
                            vect_num_pro((vect_num_pro==6))=[];
                            InLengArgin=InLengArgin-2;
                        case 'offset'
                            l.offset=varargin{ItARgin+1};
                            varargin(ItARgin:ItARgin+1)=[];
                            vect_num_pro((vect_num_pro==7))=[];
                            InLengArgin=InLengArgin-2;
                        case 'flip'
                            l.flip=varargin{ItARgin+1};
                            varargin(ItARgin:ItARgin+1)=[];
                            vect_num_pro((vect_num_pro==8))=[];
                            InLengArgin=InLengArgin-2;
                        otherwise
                            if find(vect_num_pro==1)~=0
                                l.name = varargin{ItARgin};
                                varargin(ItARgin)=[];
                                vect_num_pro((vect_num_pro==1))=[];
                                InLengArgin=InLengArgin-1;
                            else
                                disp('The name of the property is not defined')
                            end
                    end
                else
                    ItARgin=ItARgin+1;
                end
            end         
            while (~isempty(varargin)) && (~isempty(vect_num_pro))
                switch vect_num_pro(1)
                    case 2
                        l.jointtype = varargin{1};
                        varargin(1)=[];
                        vect_num_pro(1)=[];
                    case 3
                        l.theta = varargin{1};
                        varargin(1)=[];
                        vect_num_pro(1)=[];
                    case 4
                        l.d = varargin{1};
                        varargin(1)=[];
                        vect_num_pro(1)=[];
                    case 5
                        l.alpha = varargin{1};
                        varargin(1)=[];
                        vect_num_pro(1)=[];
                    case 6
                        l.a = varargin{1};
                        varargin(1)=[];
                        vect_num_pro(1)=[];
                    case 7
                        l.offset = varargin{1};
                        varargin(1)=[];
                        vect_num_pro(1)=[];
                    case 8
                        l.flip = varargin{1};
                        varargin(1)=[];
                        vect_num_pro(1)=[];
                end
            end          
        end
        
        
 
        
        
    
        
    end
end
    
    
    
    
    
    
    
    
    
    
    
