%for r = 5:5:100
%    fid = fopen(sprintf('genPlan-t0-circ-%03dm.plan', r), 'wt');
%    fprintf(fid,'%s',jsonencode(MIT_CAOS_gen(r, 0)));
%    fclose(fid);
%end

for h = 1:4
    for r = 100:50:1000
        fid = fopen(sprintf('genPlan-t1-hilb-%d-%03dm.plan', h, r), 'wt');
        fprintf(fid,'%s',jsonencode(MIT_CAOS_gen(r, h)));
        fclose(fid);
    end
end

