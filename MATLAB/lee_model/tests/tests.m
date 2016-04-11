

try
    rmdir tests s
end
mkdir tests
%%
urdf_test
%%
Tj = r.gravload(q0)
%%
gravity_test
%%
jinv_test