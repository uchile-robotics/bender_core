
% Matlab script... aplica threshold a mapas .pgm que estan difuminados

img = imread('map.pgm');
img_converted = img;

white = uint8(254);
grey  = uint8(205);
black = uint8( 0 );

for k = 1:size(img,1)    
    for  j = 1:size(img,2)

        
        if img(k,j) == white
            img_converted(k,j) = white;
            
        elseif img(k,j) > uint8(198)
            img_converted(k,j) = grey;
        else
            img_converted(k,j) = uint8(0);
        end
        
        %img_converted(k,j) = uint8(0);
        
    end
end


imwrite(img_converted,'map_converted.pgm','pgm');