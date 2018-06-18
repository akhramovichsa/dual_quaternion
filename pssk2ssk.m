function M = pssk2ssk(alpha, betta)
    % http://lektsii.org/3-22923.html
    sa = sin(alpha);
    ca = cos(alpha);
    sb = sin(betta);
    cb = cos(betta);

    M = [ ca*cb sa -ca*sb;
         -sa*cb ca  sa*sb;
          sb     0  cb];
end
