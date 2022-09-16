function   [x_new,P_new] = update_step_mag(x,P,z)

    Rmag = 0.09;
    R_mag = eye(3)*Rmag;
    
    h_x = mag_measurement_model(x);

    H_mag = mag_measurement_jacobian(x);
    
    [x_new,P_new]  = update_step(x,P,z,h_x,H_mag,R_mag);
end
