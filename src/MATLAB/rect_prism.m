%% rect_prism.m
%
% Description:
%   Generates a triangular mesh representation of a rectangular prism with
%   specified length, width, and height, and number of faces along each
%   axis.
%
% Inputs:
%   lx: side length of the rectangular prism in the x-direction
%   ly: side length of the rectangular prism in the y-direction
%   lz: side length of the rectangular prism in the z-direction
%   N:  number of faces along each axis
%
% Outputs:
%   TR: triangulation object consisting of vertices (points) and edges
%   (connectivity list)
%
% References:
%   https://www.mathworks.com/matlabcentral/answers/1628390-how-do-i-plot-a-3d-cube-in-a-3d-array

function TR = rect_prism(lx,ly,lz,N)
    % create a cube of side length 1 centered at the origin:
    a = linspace(-0.5,0.5,N);
    [X,Y,Z] = meshgrid(a);
    
    % scale the cube:
    X = X.*lx;
    Y = Y.*ly;
    Z = Z.*lz;

    % convert data to Triangulation Object and extract vertices and edges:
    k = boundary([X(:),Y(:),Z(:)]);
    TR = triangulation(k,X(:),Y(:),Z(:));
end