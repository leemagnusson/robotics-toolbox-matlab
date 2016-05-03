function link = link_add_mass_properties(link,filename,transmission)

[m,r,I] = import_solidworks_mass_properties(filename);
link.m = m;
link.r = r;
link.I = I;

trs = loadjson('harmonic_drive.json');
tr = trs.(transmission);

link.Jm = tr.Jt;
link.G = tr.G;

end