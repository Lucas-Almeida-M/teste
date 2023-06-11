function phi = ajustaAngulo(theta)
    phi = mod(theta,2*pi);
    if (phi > pi)
       phi = phi - 2*pi; 
    end
end