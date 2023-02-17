function [XYout,map]=linexlines2D(XY1,varargin)
%Given a reference 2D line or line segment, find its intersections with either 
%a polyshape boundary or with a batch of (not necessarily connected) line 
%segments.
%
%SYNTAXES:
%
%   xyBoundary=linexlines2D(poly, [x1,y1],[x2,y2]) finds the intersections
%   of the line segment with end points [x1,y1] and [x2,y2] with the
%   boundary of the polyshape object poly.
%
%   xyBoundary=linexlines2D(poly,[a,b,c]) finds the intersections of the
%   infinite 2D line, given by the equation a*x+b*y+c=0, with the boundary
%   of the polyshape object poly.
%
%   [XYout, map]=linexlines2D(XY1,XY2, [x1,y1],[x2,y2]), where XY1 and XY2
%   are 2xN matrices, finds the intersection of the reference line segment,
%   having end points [x1,y1] and [x2,y2], with each of N line segments
%   specified by XY1 and XY2. The i-th line segment is that connecting
%   XY1(:,i) and XY2(:,i). The output XYout is a 2xN matrix such that
%   XYout(:,i) is the i-th intersection or [NaN;NaN] if there is no
%   intersection. The second output argument map is a binary vector with
%   map(i)=1 indicating that an intersection with the i-th line exists.
%
%   [XYout, map]=linexlines2D(XY1,XY2, [a,b,c]) is similar to the previous
%   syntax, except that the reference line is the infinite line satisfying
%   the equation a*x+b*y+c=0.
%
%   [XYout, map]=linexlines2D(E, [a,b,c]) with E a 3xN matrix is similar to
%   the previous syntax, except that intersections between the reference
%   line a*x+b*y+c=0 and a batch of N infinite lines are computed. The
%   equation for the i-th line in the batch is given by the 3-vector
%   E(:,i).
%
%   [XYout, map]=linexlines2D(E, [x1,y1],[x2,y2]) is similar to the
%   previous syntax, except that the intersections are with the reference
%   line segment with end points [x1,y1] and [x2,y2].

    if isa(XY1,'polyshape') %special input syntax

        if nargout>1
            error 'When a polyshape object is the first input, only 1 output can be requested'
        end



        poly=XY1;

        if ~isscalar(poly), error 'Polyshape input must be scalar'; end

        [xb,yb]=poly.boundary;

        Vertices=[xb,yb].';

        XY1=Vertices(:,1:end-1);
        XY2=Vertices(:,2:end);

        discard=any(isnan(XY1)|isnan(XY2),1);
        XY1(:,discard)=[];
        XY2(:,discard)=[];

        [XYout,map]=linexlines2D(XY1,XY2,varargin{:});

        XYout=XYout(:,map);

        XYout=unique(XYout.','rows','stable').';

        return
    elseif size(XY1,1)==3

        E=XY1; clear XY1
        mode="E"; %Batch of query lines are infinite, given by equations
        
    elseif size(XY1,1)==2
        
        XY2=varargin{1}; varargin(1)=[];
        mode="XY1,XY2"; %Batch of query lines are finite line segments
        
    else
        error 'Unrecognized syntax'
    end

    %%%%%%%%%%%%% THE MAIN ROUTINE %%%%%%%%%%%%%%%%

    nv=numel(varargin);
    if nv==1

        equline=varargin{1};

        finiteLine=false;

    elseif nv==2


        [xy1,xy2]=deal(varargin{:});
        xy1=[xy1(:);1] ; xy2=[xy2(:);1];
        equline=cross( xy1 , xy2 );
        finiteLine=true;

    else
        error 'Unrecognized input syntax'
    end

    switch mode

        case "XY1,XY2"
            
            %Move into homogeneous coordinates
            XY1(3,:)=1;
            XY2(3,:)=1;

            E=cross(XY1,XY2);

            XYout=xprodmat(equline)*E;

            %Revert to normal coordiantes

            XY1(3,:)=[];
            XY2(3,:)=[];
            XYout=XYout(1:2,:)./XYout(3,:);


            %Check whether unconstrained line intersections fall within line
            %segment boundaries

            delta=XY2-XY1;
            t=dot(XYout-XY1,delta)./sum(delta.^2,1);

            map=(t>=0 & t<=1);


            if finiteLine %test for intersection with the finite line

                xy1(end)=[]; xy2(end)=[];

                map=map| all(XY1==xy1) | all(XY2==xy1) | all(XY1==xy2) | all(XY2==xy2);

                delta=xy2-xy1;
                t=delta.'*(XYout-xy1)./sum(delta.^2,1);

                map=map & (t>=0 & t<=1);

            end

            XYout(:,~map)=nan;

        case "E"
            
             XYout=xprodmat(equline)*E;
             XYout=XYout(1:2,:)./XYout(3,:);  
             
             map=all(isfinite(XYout),1);
            
             XYout(:,~map)=nan;

    end

function A=xprodmat(a)
%Matrix representation of a cross product
%
%  A=xprodmat(a)
%
%in:
%
% a: 3D vector
%
%out:
%
% A: a matrix such that A*b=cross(a,b)



    if length(a)<3, error 'Input must be a vector of length 3'; end

    ax=a(1);
    ay=a(2);
    az=a(3);

    A=zeros(3);

    A(2,1)=az;  A(1,2)=-az;
    A(3,1)=-ay; A(1,3)=ay;
    A(3,2)=ax;  A(2,3)=-ax;