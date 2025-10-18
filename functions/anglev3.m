function angle_rad = anglev3(vector_1,vector_2)
    arguments
        vector_1 (1,1) v3
        vector_2 (1,1) v3
    end
    vector_1 = vector_1';
    vector_2 = vector_2';

    rotation_complex = acos((vector_1*vector_2)/(vector_1.modulus()*vector_2.modulus()));
    angle_rad = real(rotation_complex);
end

