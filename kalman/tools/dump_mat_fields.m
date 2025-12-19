function dump_mat_fields(mat_path, out_path)
% Dump selected fields from a .mat into a small text file for external inspection
S = load(mat_path);
fid = fopen(out_path,'w');
if fid==-1, error('Cannot open output file'); end
fprintf(fid,'Loaded file: %s\n', mat_path);
fn = fieldnames(S); fprintf(fid,'Top-level fields:\n');
for i=1:length(fn), fprintf(fid,'  %s\n', fn{i}); end
if isfield(S,'eskf_state')
    st = S.eskf_state;
    fprintf(fid,'\neskf_state.p = [%g %g %g]\n', st.p(:));
    try
        P = st.P;
        fprintf(fid,'eskf_state.P11_33 =\n');
        for r=1:3, fprintf(fid,' %g %g %g\n', P(r,1), P(r,2), P(r,3)); end
    catch
    end
end
if isfield(S,'k'), fprintf(fid,'\nk = %d\n', S.k); end
if isfield(S,'new_state')
    ns = S.new_state;
    try fprintf(fid,'\nnew_state.p = [%g %g %g]\n', ns.position(:)); catch, end
end
fclose(fid);
end
