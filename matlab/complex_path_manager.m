function [path_segment, path_type]  = complex_path_manager(p,  ParamFixComplex, current_path_segments)

persistent path_segment2;
if isempty(path_segment2)
        path_segment2 = 1;
end

path_segment = path_segment2;

number_of_segments = size(ParamFixComplex.paths);
number_of_segments = number_of_segments(2);



proximity_radius = ParamFixComplex.proximity_to_next_path ;

if ~(path_segment == number_of_segments)
         
   % c_n = ParamFixComplex.c0(1, path_segment );    
   % c_e = ParamFixComplex.c0(2,path_segment );
   % c_d = ParamFixComplex.c0(3,path_segment); 
%else 
    
    c_n = ParamFixComplex.c0(1, path_segment + 1);   
    c_e = ParamFixComplex.c0(2,path_segment + 1);
    c_d = ParamFixComplex.c0(3,path_segment + 1);
    
    path_type_next = ParamFixComplex.paths(path_segment + 1);
    
    distance_to_next_path = sqrt((c_n - p(1))^2 + (c_e - p(2))^2 + (c_d - p(3))^2);


    if path_type_next == 0
        if distance_to_next_path < proximity_radius
            path_segment = path_segment + 1 ;

        end
    else
        if distance_to_next_path < (proximity_radius + ParamFixComplex.Rh(path_segment + 1))  %if it's close to any point from the inside of the helix
            path_segment = path_segment + 1 ;
        end
    end

    

end




path_segment2 = path_segment;
path_type = ParamFixComplex.paths(path_segment);

current_path_segments= horzcat(current_path_segments, path_segment);

end
