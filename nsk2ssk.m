function M = nsk2ssk(gamma, psi, theta)
    sin_gamma = sin(gamma);
    cos_gamma = cos(gamma);

    sin_psi = sin(psi);
    cos_psi = cos(psi);

    sin_theta = sin(theta);
    cos_theta = cos(theta);

    M = [cos_psi*cos_theta                                 sin_theta            -sin_psi*cos_theta
        -cos_psi*sin_theta*cos_gamma + sin_psi*sin_gamma   cos_theta*cos_gamma   cos_psi*sin_gamma + sin_psi*sin_theta*cos_gamma
         cos_psi*sin_theta*sin_gamma + sin_psi*cos_gamma  -cos_theta*sin_gamma   cos_psi*cos_gamma - sin_psi*sin_theta*sin_gamma];
end